"""Simulated RGB-D camera (report Sec. III-B / Table V / Table XVI): does not
process any image or point cloud. It republishes the same
target-structure/obstacle configuration that scene_setup_node inserted into
the planning scene (see config/scene_objects.yaml, the shared source of
truth for both nodes) as the pose/normal/clearance signals the report lists
as the perception subsystem's output.

Published topics:
    structure_pose   (geometry_msgs/PoseStamped)
    surface_normal   (geometry_msgs/Vector3Stamped)
    obstacle_pose    (geometry_msgs/PoseStamped)
    workspace_clear  (std_msgs/Bool)

workspace_clear is a coarse 2D proxy (distance from the obstacle to the
straight robot-base-to-panel line, in the XY plane), not a real collision
check -- that is MoveIt's job once an actual trajectory is planned (Task 5).
It only tells the control-logic subsystem "something is roughly in the way",
matching the report's explicit choice to keep perception a simplified
abstraction (Sec. III-B, "Identified Gap").
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Bool

from irb2600_coating_cell.geometry_utils import (
    point_segment_distance_2d,
    quaternion_from_rpy,
    rotate_vector_by_quaternion,
)


class PerceptionSimNode(Node):

    def __init__(self):
        super().__init__("perception_sim_node")

        self.declare_parameter("target_structure.frame_id", "world")
        self.declare_parameter("target_structure.position", [1.0, 0.0, 1.0])
        self.declare_parameter("target_structure.orientation_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("target_structure.local_normal", [-1.0, 0.0, 0.0])

        self.declare_parameter("obstacle.frame_id", "world")
        self.declare_parameter("obstacle.position", [0.5, 0.3, 1.0])
        self.declare_parameter("obstacle.orientation_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("obstacle.size", [0.15, 0.15, 0.8])

        self.declare_parameter("robot_base_xy", [0.0, 0.0])
        self.declare_parameter("d_safe", 0.05)
        self.declare_parameter("workspace_clear_margin", 0.15)
        self.declare_parameter("publish_rate_hz", 2.0)

        self._structure_pub = self.create_publisher(PoseStamped, "structure_pose", 10)
        self._normal_pub = self.create_publisher(Vector3Stamped, "surface_normal", 10)
        self._obstacle_pub = self.create_publisher(PoseStamped, "obstacle_pose", 10)
        self._clear_pub = self.create_publisher(Bool, "workspace_clear", 10)

        rate_hz = self.get_parameter("publish_rate_hz").value
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)

    def _on_timer(self):
        now = self.get_clock().now().to_msg()
        p = self.get_parameter

        structure_frame = p("target_structure.frame_id").value
        structure_pos = p("target_structure.position").value
        structure_rpy = [float(v) for v in p("target_structure.orientation_rpy").value]
        structure_quat = quaternion_from_rpy(*structure_rpy)

        structure_msg = PoseStamped()
        structure_msg.header.stamp = now
        structure_msg.header.frame_id = structure_frame
        (
            structure_msg.pose.position.x,
            structure_msg.pose.position.y,
            structure_msg.pose.position.z,
        ) = (float(v) for v in structure_pos)
        structure_msg.pose.orientation = structure_quat
        self._structure_pub.publish(structure_msg)

        local_normal = [float(v) for v in p("target_structure.local_normal").value]
        nx, ny, nz = rotate_vector_by_quaternion(local_normal, structure_quat)
        normal_msg = Vector3Stamped()
        normal_msg.header.stamp = now
        normal_msg.header.frame_id = structure_frame
        normal_msg.vector.x, normal_msg.vector.y, normal_msg.vector.z = nx, ny, nz
        self._normal_pub.publish(normal_msg)

        obstacle_frame = p("obstacle.frame_id").value
        obstacle_pos = [float(v) for v in p("obstacle.position").value]
        obstacle_rpy = [float(v) for v in p("obstacle.orientation_rpy").value]

        obstacle_msg = PoseStamped()
        obstacle_msg.header.stamp = now
        obstacle_msg.header.frame_id = obstacle_frame
        (
            obstacle_msg.pose.position.x,
            obstacle_msg.pose.position.y,
            obstacle_msg.pose.position.z,
        ) = obstacle_pos
        obstacle_msg.pose.orientation = quaternion_from_rpy(*obstacle_rpy)
        self._obstacle_pub.publish(obstacle_msg)

        clear_msg = Bool()
        clear_msg.data = self._compute_workspace_clear(obstacle_pos, structure_pos)
        self._clear_pub.publish(clear_msg)

    def _compute_workspace_clear(self, obstacle_pos, structure_pos):
        base_x, base_y = (float(v) for v in self.get_parameter("robot_base_xy").value)
        obstacle_size = [float(v) for v in self.get_parameter("obstacle.size").value]
        d_safe = float(self.get_parameter("d_safe").value)
        margin = float(self.get_parameter("workspace_clear_margin").value)

        obstacle_bounding_radius = 0.5 * max(obstacle_size[0], obstacle_size[1])
        distance = point_segment_distance_2d(
            obstacle_pos[0], obstacle_pos[1], base_x, base_y, structure_pos[0], structure_pos[1]
        )
        return distance > (obstacle_bounding_radius + d_safe + margin)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
