"""One-shot utility: sends the arm back to the "home" pose (all joints at 0,
matching config/irb2600.srdf's group_state "home") via a single MoveGroup
action call (plan + execute together), so you don't have to restart the
whole coating_cell_bringup.launch.py just to get a clean starting state
between manual test runs.

    ros2 run irb2600_coating_cell go_home_node
"""

import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient
from rclpy.node import Node

_ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class GoHomeNode(Node):

    def __init__(self):
        super().__init__("go_home_node")
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("planning_time_s", 3.0)

        self._client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for /move_action (move_group)...")
        self._client.wait_for_server()
        self._go_home()

    def _go_home(self):
        constraints = Constraints()
        for joint_name in _ARM_JOINTS:
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = 0.0
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal = MoveGroup.Goal()
        goal.request.group_name = self.get_parameter("group_name").value
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.request.allowed_planning_time = float(
            self.get_parameter("planning_time_s").value
        )
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.workspace_parameters.header.frame_id = "world"
        goal.request.workspace_parameters.min_corner.x = -3.0
        goal.request.workspace_parameters.min_corner.y = -3.0
        goal.request.workspace_parameters.min_corner.z = -3.0
        goal.request.workspace_parameters.max_corner.x = 3.0
        goal.request.workspace_parameters.max_corner.y = 3.0
        goal.request.workspace_parameters.max_corner.z = 3.0
        # plan_only=False: move_group plans AND executes in one action call,
        # no separate /execute_trajectory step needed for this simple case.
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True

        self.get_logger().info("Planning path to home (all joints = 0)...")
        send_goal_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("move_action goal rejected.")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_response = result_future.result()

        if result_response.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Home reached.")
        else:
            self.get_logger().error(
                f"Failed to reach home (status={result_response.status}, "
                f"error_code={result_response.result.error_code.val})."
            )


def main(args=None):
    rclpy.init(args=args)
    node = GoHomeNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
