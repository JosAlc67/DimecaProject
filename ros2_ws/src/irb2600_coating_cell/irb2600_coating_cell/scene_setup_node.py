"""Inserts the target structure and the temporary obstacle (report Table VIII)
into the MoveIt planning scene as collision objects, via the standard
/apply_planning_scene service exposed by move_group. Using that service
instead of moveit_py/pymoveit2 keeps this node dependency-light and
independent of which Python MoveIt binding ends up being available.

    ros2 run irb2600_coating_cell scene_setup_node

Re-apply after changing a pose/size parameter at runtime with:

    ros2 param set /scene_setup_node obstacle.position "[0.5, 0.1, 1.0]"
    ros2 service call /scene_setup_node/refresh_scene std_srvs/srv/Trigger
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

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

    def _apply_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = self._build_collision_objects()

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._apply_scene_client.call_async(request)
        future.add_done_callback(self._on_apply_scene_done)

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
