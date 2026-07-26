"""Inserts the target structure and the temporary obstacle (report Table VIII)
into the MoveIt planning scene as collision objects, via the standard
/apply_planning_scene service exposed by move_group. Using that service
instead of moveit_py/pymoveit2 keeps this node dependency-light and
independent of which Python MoveIt binding ends up being available.

Also publishes the same two objects as a plain visualization_msgs/MarkerArray
on ~/scene_markers. This project hit three separate, unrelated bugs in
moveit_rviz_plugin on ROS 2 Humble while testing this (a nonexistent Panel
class, a joint_limits type-mismatch exception in its robot-model loader,
and finally moveit_rviz_plugin/PlanningScene never actually subscribing to
/monitored_planning_scene despite move_group publishing it -- confirmed with
`ros2 topic info /monitored_planning_scene` showing 1 publisher, 0
subscribers). Rather than keep chasing bugs in that plugin, the markers use
only rviz_default_plugins/MarkerArray, which has no dependency on MoveIt's
RViz integration. Published with TRANSIENT_LOCAL durability so RViz gets
them immediately on connecting, regardless of launch ordering.

    ros2 run irb2600_coating_cell scene_setup_node

Re-apply after changing a pose/size parameter at runtime with:

    ros2 param set /scene_setup_node obstacle.position "[0.5, 0.1, 1.0]"
    ros2 service call /scene_setup_node/refresh_scene std_srvs/srv/Trigger
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from irb2600_coating_cell.geometry_utils import quaternion_from_rpy


class SceneSetupNode(Node):

    def __init__(self):
        super().__init__("scene_setup_node")

        self.declare_parameter("target_structure.frame_id", "world")
        self.declare_parameter("target_structure.position", [1.0, 0.0, 1.0])
        self.declare_parameter("target_structure.orientation_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("target_structure.size", [0.02, 1.0, 0.6])

        self.declare_parameter("obstacle.frame_id", "world")
        self.declare_parameter("obstacle.type", "box")
        self.declare_parameter("obstacle.position", [0.5, 0.3, 1.0])
        self.declare_parameter("obstacle.orientation_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("obstacle.size", [0.15, 0.15, 0.8])

        self._apply_scene_client = self.create_client(
            ApplyPlanningScene, "apply_planning_scene"
        )
        self.create_service(Trigger, "~/refresh_scene", self._on_refresh_scene)

        transient_local_qos = QoSProfile(depth=1)
        transient_local_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._marker_pub = self.create_publisher(
            MarkerArray, "~/scene_markers", transient_local_qos
        )
        # TRANSIENT_LOCAL only makes a publisher's last message "sticky" for
        # subscribers that *also* request TRANSIENT_LOCAL durability; RViz's
        # MarkerArray display does not by default, so a one-shot publish can
        # be missed by a RViz session that connects afterwards (observed:
        # markers visible on one bringup, gone on the next, depending on
        # process startup timing). Republishing periodically sidesteps that
        # regardless of any subscriber QoS/timing details.
        self.create_timer(2.0, self._publish_markers)

        self.get_logger().info(
            "Waiting for /apply_planning_scene (provided by move_group)..."
        )
        self._apply_scene_client.wait_for_service()
        self._apply_scene()

    def _make_box_object(self, object_id, frame_id, position, rpy, size):
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = object_id
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(size[0]), float(size[1]), float(size[2])]

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
        pose.orientation = quaternion_from_rpy(*[float(v) for v in rpy])

        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        return obj

    def _make_cylinder_object(self, object_id, frame_id, position, rpy, size):
        # size = [height, radius, _unused]
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = object_id
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [float(size[0]), float(size[1])]

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
        pose.orientation = quaternion_from_rpy(*[float(v) for v in rpy])

        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        return obj

    def _build_collision_objects(self):
        p = self.get_parameter
        target = self._make_box_object(
            "target_structure",
            p("target_structure.frame_id").value,
            p("target_structure.position").value,
            p("target_structure.orientation_rpy").value,
            p("target_structure.size").value,
        )

        obstacle_type = p("obstacle.type").value
        if obstacle_type == "cylinder":
            obstacle = self._make_cylinder_object(
                "temporary_obstacle",
                p("obstacle.frame_id").value,
                p("obstacle.position").value,
                p("obstacle.orientation_rpy").value,
                p("obstacle.size").value,
            )
        else:
            obstacle = self._make_box_object(
                "temporary_obstacle",
                p("obstacle.frame_id").value,
                p("obstacle.position").value,
                p("obstacle.orientation_rpy").value,
                p("obstacle.size").value,
            )

        return [target, obstacle]

    # Same color used for the collision object (PlanningScene.object_colors,
    # for if/when moveit_rviz_plugin/PlanningScene works) and for the
    # equivalent MarkerArray marker (the actually-reliable visualization).
    _OBJECT_COLORS = {
        "target_structure": ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.6),
        "temporary_obstacle": ColorRGBA(r=1.0, g=0.25, b=0.1, a=0.8),
    }

    def _make_marker(self, marker_id, object_id, frame_id, position, rpy, size, shape_type):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "coating_cell"
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
            float(v) for v in position
        )
        marker.pose.orientation = quaternion_from_rpy(*[float(v) for v in rpy])
        marker.color = self._OBJECT_COLORS[object_id]

        if shape_type == "cylinder":
            # size = [height, radius, _unused] (SolidPrimitive convention);
            # Marker.CYLINDER scale is [diameter_x, diameter_y, height].
            marker.type = Marker.CYLINDER
            marker.scale.x = 2.0 * float(size[1])
            marker.scale.y = 2.0 * float(size[1])
            marker.scale.z = float(size[0])
        else:
            marker.type = Marker.CUBE
            marker.scale.x, marker.scale.y, marker.scale.z = (float(v) for v in size)

        return marker

    def _build_markers(self):
        p = self.get_parameter
        target_marker = self._make_marker(
            0,
            "target_structure",
            p("target_structure.frame_id").value,
            p("target_structure.position").value,
            p("target_structure.orientation_rpy").value,
            p("target_structure.size").value,
            "box",
        )
        obstacle_marker = self._make_marker(
            1,
            "temporary_obstacle",
            p("obstacle.frame_id").value,
            p("obstacle.position").value,
            p("obstacle.orientation_rpy").value,
            p("obstacle.size").value,
            p("obstacle.type").value,
        )
        return MarkerArray(markers=[target_marker, obstacle_marker])

    def _apply_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = self._build_collision_objects()
        # Distinct colors so both objects read clearly at a glance in RViz:
        # blue/translucent for the target structure, warning red/orange for
        # the temporary obstacle. Collision-object primitives have no color
        # of their own in moveit_msgs/CollisionObject; PlanningScene.object_colors
        # is the mechanism moveit_rviz_plugin/PlanningScene reads (see
        # _build_markers/_OBJECT_COLORS above for the actually-reliable
        # rviz_default_plugins/MarkerArray equivalent).
        scene.object_colors = [
            ObjectColor(id=object_id, color=color)
            for object_id, color in self._OBJECT_COLORS.items()
        ]

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._apply_scene_client.call_async(request)
        future.add_done_callback(self._on_apply_scene_done)

        self._publish_markers()

    def _publish_markers(self):
        self._marker_pub.publish(self._build_markers())

    def _on_apply_scene_done(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - just logging, not recovering
            self.get_logger().error(f"apply_planning_scene call failed: {exc}")
            return
        if response.success:
            self.get_logger().info(
                "Planning scene updated: target_structure + temporary_obstacle."
            )
        else:
            self.get_logger().error("move_group rejected the planning scene update.")

    def _on_refresh_scene(self, request, response):
        self._apply_scene()
        response.success = True
        response.message = "Planning scene re-applied from current parameters."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SceneSetupNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
