"""Reactive replanning executor (report Table VI "Replan the trajectory";
Table XVII Case 3: obstacle changes position dynamically during operation).

Unlike trajectory_planner_node (Phase 1: plans the whole raster once against
a static obstacle), this node executes the coating raster ROW BY ROW. Before
each row it re-checks that row's Cartesian path against the *current*
planning scene:

  - If clear (fraction >= fraction_threshold): execute it directly, exactly
    like trajectory_planner_node does for the whole path.
  - If blocked: attempt to replan -- ask move_group for a full joint-space
    (OMPL) plan to the row's end pose, expressed as position/orientation
    goal constraints (not a pre-solved joint goal: OMPL samples/solves IK
    itself as part of planning, unlike straight-line Cartesian interpolation,
    it can route around the obstacle). If that also fails, the row is
    unreachable: report it as a failed trajectory and stop (Table XVII
    Case 4), matching Sec. VI-B's "safe robot stop".

Between rows the node pauses for `segment_pause_s` seconds, specifically so
you can trigger a "controlled change in obstacle position" during that
window (Table IX: "Replanning type: Discrete/reactive") and watch the next
row react to it:

    ros2 param set /scene_setup_node obstacle.position "[X, Y, Z]"
    ros2 service call /scene_setup_node/refresh_scene std_srvs/srv/Trigger

    ros2 run irb2600_coating_cell replanning_executor_node --ros-args -p execute:=true
"""

import time

import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, MoveItErrorCodes, OrientationConstraint, PositionConstraint
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import SetBool

from irb2600_coating_cell.raster_path import generate_raster_rows


class ReplanningExecutorNode(Node):

    def __init__(self):
        super().__init__("replanning_executor_node")

        self.declare_parameter("target_structure.frame_id", "world")
        self.declare_parameter("target_structure.position", [1.0, 0.0, 1.0])
        self.declare_parameter("target_structure.orientation_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("target_structure.size", [0.02, 1.0, 0.6])
        self.declare_parameter("target_structure.local_normal", [-1.0, 0.0, 0.0])

        self.declare_parameter("d_standoff", 0.20)
        self.declare_parameter("edge_margin", 0.05)
        self.declare_parameter("row_pitch", 0.10)
        self.declare_parameter("max_step", 0.01)
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("tcp_link", "nozzle_tip")
        self.declare_parameter("execute", False)

        # Minimum compute_cartesian_path fraction to accept a row as clear.
        self.declare_parameter("fraction_threshold", 0.99)
        # Pause between rows -- the window to manually move the obstacle to
        # exercise Case 3 (see module docstring).
        self.declare_parameter("segment_pause_s", 3.0)
        self.declare_parameter("replanning_time_s", 2.0)
        self.declare_parameter("replanning_attempts", 5)

        self._cartesian_path_client = self.create_client(
            GetCartesianPath, "compute_cartesian_path"
        )
        self._move_group_client = ActionClient(self, MoveGroup, "move_action")
        self._execute_client = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self._spray_client = self.create_client(
            SetBool, "/spray_controller_node/set_spray_on"
        )

        self.get_logger().info("Waiting for /compute_cartesian_path (move_group)...")
        self._cartesian_path_client.wait_for_service()

        self._run()

    # -- top-level control loop ----------------------------------------------

    def _run(self):
        p = self.get_parameter
        rows, _normal_world = generate_raster_rows(
            position=p("target_structure.position").value,
            rpy=p("target_structure.orientation_rpy").value,
            size=p("target_structure.size").value,
            local_normal=p("target_structure.local_normal").value,
            d_standoff=p("d_standoff").value,
            edge_margin=p("edge_margin").value,
            row_pitch=p("row_pitch").value,
        )

        metrics = {"direct": 0, "replanned": 0, "failed": 0, "total_replan_time_s": 0.0}
        segment_pause_s = float(p("segment_pause_s").value)
        n_rows = len(rows)

        for idx, row in enumerate(rows):
            self.get_logger().info(f"--- Row {idx + 1}/{n_rows} ---")
            ok = self._process_row(row, idx, metrics)
            if not ok:
                self.get_logger().error(
                    f"Row {idx + 1} could not be reached even after replanning "
                    "(Table XVII Case 4: failed-trajectory report, safe robot "
                    "stop). Aborting remaining rows."
                )
                self._stop_spray_sync()
                break

            if idx < n_rows - 1:
                self.get_logger().info(
                    f"Row {idx + 1} done. Pausing {segment_pause_s:.1f}s before "
                    "the next row -- move the obstacle now to test Case 3, e.g.:\n"
                    '  ros2 param set /scene_setup_node obstacle.position "[0.79, 0.0, 1.0]"\n'
                    "  ros2 service call /scene_setup_node/refresh_scene std_srvs/srv/Trigger"
                )
                time.sleep(segment_pause_s)

        self.get_logger().info(
            "Summary: {direct} row(s) direct, {replanned} row(s) replanned, "
            "{failed} row(s) failed, total replanning time {total_replan_time_s:.3f} s"
            .format(**metrics)
        )

    def _process_row(self, row, idx, metrics):
        t0 = time.time()
        fraction, trajectory = self._compute_cartesian_segment(row)
        t_plan = time.time() - t0

        if fraction >= float(self.get_parameter("fraction_threshold").value):
            self.get_logger().info(
                f"Row {idx + 1}: direct Cartesian path OK "
                f"(fraction={fraction:.3f}, t_plan={t_plan:.3f} s)."
            )
            metrics["direct"] += 1
            return self._execute_with_spray(trajectory)

        self.get_logger().warn(
            f"Row {idx + 1} blocked (fraction={fraction:.3f}, checked in "
            f"{t_plan:.3f} s). Replanning around it (Table VI 'Replan the "
            "trajectory')..."
        )
        t0 = time.time()
        replanned_trajectory = self._replan_row(row, idx)
        t_replan = time.time() - t0
        metrics["total_replan_time_s"] += t_replan

        if replanned_trajectory is None:
            metrics["failed"] += 1
            return False

        self.get_logger().info(
            f"Row {idx + 1}: replanned successfully (t_replan={t_replan:.3f} s)."
        )
        metrics["replanned"] += 1
        return self._execute_with_spray(replanned_trajectory)

    # -- direct Cartesian segment ---------------------------------------------

    def _compute_cartesian_segment(self, row):
        request = GetCartesianPath.Request()
        request.header.frame_id = self.get_parameter("target_structure.frame_id").value
        request.group_name = self.get_parameter("group_name").value
        request.link_name = self.get_parameter("tcp_link").value
        request.waypoints = row
        request.max_step = float(self.get_parameter("max_step").value)
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.start_state.is_diff = True

        future = self._cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.fraction, response.solution

    # -- replanning fallback: full joint-space (OMPL) plan to the row's end pose --

    def _replan_row(self, row, idx):
        goal_pose = row[-1]
        frame_id = self.get_parameter("target_structure.frame_id").value
        tcp_link = self.get_parameter("tcp_link").value

        # Goal expressed as Position + Orientation constraints (small
        # tolerance spheres/angles around the row's end pose) rather than a
        # pre-solved joint-space goal: OMPL samples/solves IK itself as part
        # of planning, which is more robust than a single external
        # /compute_ik call (KDL is a local numerical solver and can fail to
        # converge for a pose that is, in fact, reachable by some other
        # joint configuration -- this was tried first and consistently
        # failed with error_code=-31/NO_IK_SOLUTION even for poses the
        # Cartesian planner could get most of the way to).
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = frame_id
        position_constraint.link_name = tcp_link
        position_constraint.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        position_constraint.constraint_region.primitives = [sphere]
        position_constraint.constraint_region.primitive_poses = [goal_pose]

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = frame_id
        orientation_constraint.link_name = tcp_link
        orientation_constraint.orientation = goal_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation_constraint]

        goal = MoveGroup.Goal()
        goal.request.group_name = self.get_parameter("group_name").value
        goal.request.start_state.is_diff = True
        goal.request.num_planning_attempts = int(
            self.get_parameter("replanning_attempts").value
        )
        goal.request.allowed_planning_time = float(
            self.get_parameter("replanning_time_s").value
        )
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.goal_constraints = [constraints]

        # Plan only; we execute separately via /execute_trajectory, same as
        # the direct-Cartesian path, so both routes share one execution path.
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True

        self._move_group_client.wait_for_server()
        send_goal_future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"Row {idx + 1}: move_action goal rejected.")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Row {idx + 1}: OMPL could not find a collision-free joint-space "
                f"path either (error_code={result.error_code.val})."
            )
            return None

        return result.planned_trajectory

    # -- execution + spray_on -------------------------------------------------

    def _execute_with_spray(self, trajectory):
        if not self.get_parameter("execute").value:
            self.get_logger().info(
                "execute:=false (default): row planned but not sent to the "
                "controller."
            )
            return True

        self._set_spray_sync(True)

        self._execute_client.wait_for_server()
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_goal_future = self._execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("execute_trajectory goal was rejected.")
            self._set_spray_sync(False)
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        self._set_spray_sync(False)
        succeeded = result.status == GoalStatus.STATUS_SUCCEEDED
        if not succeeded:
            self.get_logger().error("Trajectory execution FAILED.")
        return succeeded

    def _set_spray_sync(self, on):
        if not self._spray_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "spray_controller_node not available; continuing without "
                "toggling spray_on."
            )
            return
        request = SetBool.Request()
        request.data = on
        future = self._spray_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def _stop_spray_sync(self):
        self._set_spray_sync(False)


def main(args=None):
    rclpy.init(args=args)
    node = ReplanningExecutorNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
