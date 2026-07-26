"""Generates the initial continuous coating trajectory (report Table VI:
"Plan the initial trajectory") as a raster/boustrophedon sweep over the
target structure's face, offset by d_standoff along the surface normal, and
submits it to MoveIt via the plain /compute_cartesian_path service -- no
moveit_py/pymoveit2 dependency, consistent with the rest of this package.

This implements report Table XVII Case 1 (no/avoidable obstacle -> full
coverage) and exercises Case 2 (partial obstruction): avoid_collisions=True
means MoveIt stops the Cartesian interpolation as soon as a waypoint (or the
motion approaching it) would collide with the temporary_obstacle inserted by
scene_setup_node, and reports how much of the path it could actually
complete (the `fraction` field). It does NOT implement Table VI's "Replan
the trajectory" step (recomputing an alternative route around a blocking
obstacle) -- that reactive replanning loop is future work (Progress Report 1
Sec. IX), out of scope for the "static obstacle" phase this node covers.

Assumes target_structure.local_normal is aligned with the target structure's
local X axis (true for the default config/scene_objects.yaml); the raster
sweep is generated across the object's local Y (width) and Z (height) axes.

    ros2 run irb2600_coating_cell trajectory_planner_node
"""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import SetBool

from irb2600_coating_cell.geometry_utils import (
    quaternion_with_z_axis,
    rotate_vector_by_quaternion,
)


class TrajectoryPlannerNode(Node):

    def __init__(self):
        super().__init__("trajectory_planner_node")

        self.declare_parameter("target_structure.frame_id", "world")
        self.declare_parameter("target_structure.position", [1.0, 0.0, 1.0])
        self.declare_parameter("target_structure.orientation_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("target_structure.size", [0.02, 1.0, 0.6])
        self.declare_parameter("target_structure.local_normal", [-1.0, 0.0, 0.0])

        # Table VIII: application distance 0.15 - 0.25 m.
        self.declare_parameter("d_standoff", 0.20)
        # Margin kept in from the panel edges so the nozzle footprint stays
        # on the structure.
        self.declare_parameter("edge_margin", 0.05)
        # Vertical spacing between raster passes.
        self.declare_parameter("row_pitch", 0.10)
        # Max Cartesian step used by compute_cartesian_path's interpolation.
        self.declare_parameter("max_step", 0.01)
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("tcp_link", "nozzle_tip")
        self.declare_parameter("execute", False)
        # Diagnostic switch: set to false to ask compute_cartesian_path to
        # ignore collisions entirely. If fraction jumps to ~1.0 with this
        # off, the low fraction was a (self-)collision; if it stays low
        # even with collision checking off, it's an IK/reachability
        # problem with the requested waypoint poses instead.
        self.declare_parameter("avoid_collisions", True)

        self._cartesian_path_client = self.create_client(
            GetCartesianPath, "compute_cartesian_path"
        )
        self._execute_client = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self._spray_client = self.create_client(
            SetBool, "/spray_controller_node/set_spray_on"
        )

        self.get_logger().info("Waiting for /compute_cartesian_path (move_group)...")
        self._cartesian_path_client.wait_for_service()

        self._run()

    # -- waypoint generation -------------------------------------------------

    def _generate_waypoints(self):
        p = self.get_parameter
        position = [float(v) for v in p("target_structure.position").value]
        rpy = [float(v) for v in p("target_structure.orientation_rpy").value]
        size = [float(v) for v in p("target_structure.size").value]
        local_normal = [float(v) for v in p("target_structure.local_normal").value]
        d_standoff = float(p("d_standoff").value)
        edge_margin = float(p("edge_margin").value)
        row_pitch = float(p("row_pitch").value)

        from irb2600_coating_cell.geometry_utils import quaternion_from_rpy

        structure_quat = quaternion_from_rpy(*rpy)
        normal_world = rotate_vector_by_quaternion(local_normal, structure_quat)

        width_eff = max(size[1] - 2.0 * edge_margin, 0.0)
        height_eff = max(size[2] - 2.0 * edge_margin, 0.0)
        n_rows = max(int(round(height_eff / row_pitch)) + 1, 1)

        waypoints = []
        for row in range(n_rows):
            local_z = -height_eff / 2.0 + row * (height_eff / max(n_rows - 1, 1))
            y_start, y_end = -width_eff / 2.0, width_eff / 2.0
            if row % 2 == 1:
                y_start, y_end = y_end, y_start

            for local_y in (y_start, y_end):
                # Point on the panel face, in the panel's local frame, offset
                # by half the thickness towards the working face
                # (local_normal), then pushed out by d_standoff.
                local_point = (
                    local_normal[0] * (size[0] / 2.0),
                    local_y,
                    local_z,
                )
                surface_point_world = rotate_vector_by_quaternion(local_point, structure_quat)
                x = position[0] + surface_point_world[0] + normal_world[0] * d_standoff
                y = position[1] + surface_point_world[1] + normal_world[1] * d_standoff
                z = position[2] + surface_point_world[2] + normal_world[2] * d_standoff

                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = x, y, z
                # Physically, the nozzle's approach vector must point INTO
                # the surface (like any real spray/weld/machining tool),
                # i.e. antiparallel to the outward surface normal n_s used
                # for the standoff offset above -- NOT parallel to it, even
                # though eq. 9 (theta_error = arccos(z_e . n_s) <= 10 deg)
                # reads as if z_e should equal n_s directly. Using +n_s here
                # (tool pointing away from the panel, back over its own
                # shoulder) was tested and made nearly the entire raster
                # kinematically unreachable (fraction ~0 with IK/collision
                # both failing regardless of the obstacle). Whichever
                # eventually computes eq. 9 as a reported metric should
                # measure the angle against -n_s (equivalently, against the
                # surface's inward normal) to match this convention.
                approach_direction = tuple(-c for c in normal_world)
                pose.orientation = quaternion_with_z_axis(approach_direction)
                waypoints.append(pose)

        return waypoints, normal_world

    # -- planning / execution -------------------------------------------------

    def _run(self):
        waypoints, normal_world = self._generate_waypoints()
        request = GetCartesianPath.Request()
        request.header.frame_id = self.get_parameter("target_structure.frame_id").value
        request.group_name = self.get_parameter("group_name").value
        request.link_name = self.get_parameter("tcp_link").value
        request.waypoints = waypoints
        request.max_step = float(self.get_parameter("max_step").value)
        request.jump_threshold = 0.0
        request.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)
        request.start_state.is_diff = True

        future = self._cartesian_path_client.call_async(request)
        future.add_done_callback(
            lambda f: self._on_cartesian_path_done(f, waypoints)
        )

    def _on_cartesian_path_done(self, future, waypoints):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"compute_cartesian_path call failed: {exc}")
            return

        fraction = response.fraction
        n_points = len(response.solution.joint_trajectory.points)
        length = self._approximate_length(waypoints)

        self.get_logger().info(
            "Cartesian path: fraction={:.3f}, trajectory_points={}, "
            "requested_waypoints={}, approx_length_L={:.3f} m".format(
                fraction, n_points, len(waypoints), length
            )
        )
        if fraction < 0.999:
            self.get_logger().warn(
                "Coverage incomplete (fraction < 1.0): the requested raster "
                "path is blocked before finishing, most likely by "
                "temporary_obstacle. This is Table VI 'Check collisions' "
                "surfacing an unsafe segment; automatic replanning around it "
                "(Table VI 'Replan the trajectory') is not implemented in "
                "this node (see module docstring)."
            )

        if fraction <= 0.0:
            self.get_logger().error("No valid Cartesian path found; nothing to execute.")
            return

        if self.get_parameter("execute").value:
            self._start_spray_then_execute(response.solution)
        else:
            self.get_logger().info(
                "execute:=false (default): trajectory computed but not sent "
                "to the controller. Re-run with --ros-args -p execute:=true "
                "to actually move the mock hardware / visualize execution."
            )

    @staticmethod
    def _approximate_length(waypoints):
        # eq. 5: L = sum ||p_e(q_{i+1}) - p_e(q_i)||, approximated here from
        # the *requested* Cartesian waypoints (upper bound on the executed
        # path length when fraction < 1).
        total = 0.0
        for a, b in zip(waypoints[:-1], waypoints[1:]):
            dx = b.position.x - a.position.x
            dy = b.position.y - a.position.y
            dz = b.position.z - a.position.z
            total += math.sqrt(dx * dx + dy * dy + dz * dz)
        return total

    def _start_spray_then_execute(self, robot_trajectory):
        if not self._spray_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "spray_controller_node not available; executing without "
                "toggling spray_on."
            )
            self._execute_trajectory(robot_trajectory)
            return

        request = SetBool.Request()
        request.data = True
        future = self._spray_client.call_async(request)
        future.add_done_callback(lambda f: self._execute_trajectory(robot_trajectory))

    def _execute_trajectory(self, robot_trajectory):
        self.get_logger().info("Waiting for /execute_trajectory action server...")
        self._execute_client.wait_for_server()

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = robot_trajectory
        send_goal_future = self._execute_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("execute_trajectory goal was rejected.")
            self._stop_spray()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_execute_result)

    def _on_execute_result(self, future):
        result = future.result()
        status = "SUCCEEDED" if result.status == GoalStatus.STATUS_SUCCEEDED else "FAILED"
        self.get_logger().info(f"Trajectory execution {status}.")
        self._stop_spray()

    def _stop_spray(self):
        if not self._spray_client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = False
        self._spray_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
