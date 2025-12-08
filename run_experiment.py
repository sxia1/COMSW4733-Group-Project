import trimesh
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    PlanningScene,
    CollisionObject,
)
from shape_msgs.msg import (
    Mesh,
    MeshTriangle,
    SolidPrimitive,
)
from builtin_interfaces.msg import Duration
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker, MarkerArray
from math import sqrt

X = -0.1
Y = 0.80
Z = 0.2
OX = -sqrt(0.5)
OY = 0.0
OZ = 0.0
OW = sqrt(0.5)

class AddTree(Node):
    def __init__(self):
        super().__init__("add_tree")

        self.scene_pub = self.create_publisher(
            PlanningScene,
            'planning_scene',
            10
        )

        # Load mesh using trimesh
        mesh_path = "./DATA/denoised_mesh.stl"
        mesh_model = trimesh.load(mesh_path)

        # Convert mesh into ROS message
        ros_mesh = Mesh()
        for face in mesh_model.faces:
            tri = MeshTriangle(vertex_indices=[int(face[0]), int(face[1]), int(face[2])])
            ros_mesh.triangles.append(tri)

        for vertex in mesh_model.vertices:
            ros_mesh.vertices.append(Point(
                x=float(vertex[0]),
                y=float(vertex[1]),
                z=float(vertex[2])
            ))

        # CollisionObject
        obj = CollisionObject()
        obj.id = "tree_mesh"
        obj.header.frame_id = "world"

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.2
        pose.pose.orientation.w = 1.0

        obj.meshes.append(ros_mesh)
        obj.mesh_poses.append(pose.pose)
        obj.operation = CollisionObject.ADD

        # Planning Scene update
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)

        self.get_logger().info("Publishing mesh object to planning_scene")
        self.scene_pub.publish(scene)

def make_sphere(x, y, z, w, scale=0.25):
    sphere = Marker()
    sphere.header.frame_id = "base_link"
    sphere.type = Marker.SPHERE
    sphere.pose.position.x = x
    sphere.pose.position.y = y
    sphere.pose.position.z = z
    sphere.pose.orientation.w = w
    sphere.scale.x = sphere.scale.y = sphere.scale.z = scale
    sphere.color.r = 1.0
    sphere.color.a = 0.6
    return sphere

class MoveItPoseGoal(Node):
    def __init__(self):
        super().__init__("moveit_pose_goal")
        # Create action client for MoveIt 2
        self._client = ActionClient(self, MoveGroup, "move_action")

    def send_goal(self):
        self._client.wait_for_server()

        req = MotionPlanRequest()
        req.planner_id = "BiTRRTkConfigDefault"
        req.group_name = "ur5e_arm"
        req.allowed_planning_time = 5.0

        pose_goal = Constraints()
        pose_goal.name = "end_effector_goal"

        pos = PositionConstraint()
        pos.header.frame_id="base_link"
        pos.link_name = "cutter"
        pos.weight = 1.0

        # Target center point
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = X
        target_pose.pose.position.y = Y
        target_pose.pose.position.z = Z
        target_pose.pose.orientation.x = OX
        target_pose.pose.orientation.y = OY
        target_pose.pose.orientation.z = OZ
        target_pose.pose.orientation.w = OW

        pos.target_point_offset.x = 0.0
        pos.target_point_offset.y = 0.0
        pos.target_point_offset.z = 0.0

        # Bounding region (small sphere around goal)
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [0.05]  # 5mm radius
        pos.constraint_region.primitives.append(region)
        pos.constraint_region.primitive_poses.append(target_pose.pose)

        pose_goal.position_constraints.append(pos)

        # Orientation constraint
        ori = OrientationConstraint()
        ori.link_name = "cutter"
        ori.header.frame_id = "base_link"
        ori.orientation = target_pose.pose.orientation
        ori.absolute_x_axis_tolerance = 0.1
        ori.absolute_y_axis_tolerance = 0.1
        ori.absolute_z_axis_tolerance = 0.1
        ori.weight = 1.0

        pose_goal.orientation_constraints.append(ori)

        req.goal_constraints.append(pose_goal)

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        #goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.plan_only = False

        future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if not result.accepted:
            self.get_logger().error("Goal rejected")
            return

        result_future = result.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("Movement completed!")


def main():
    rclpy.init()
    tree = AddTree()
    rclpy.spin_once(tree, timeout_sec=2.0)

    node = MoveItPoseGoal()
    node.send_goal()
    marker_pub = node.create_publisher(MarkerArray, "/constraint_markers", 10)
    markers = MarkerArray()
    sphere = make_sphere(X, Y, Z, OW)
    markers.markers.append(sphere)

    # Cone/arrow for orientation
    arrow = Marker()
    arrow.header.frame_id = "base_link"
    arrow.type = Marker.ARROW
    arrow.pose.position.x = X
    arrow.pose.position.y = Y
    arrow.pose.position.z = Z
    arrow.pose.orientation.x = OX
    arrow.pose.orientation.y = OY
    arrow.pose.orientation.z = OZ
    arrow.pose.orientation.w = OW
    arrow.scale.x = 0.10
    arrow.scale.y = 0.02
    arrow.scale.z = 0.03
    arrow.color.g = 1.0
    arrow.color.a = 0.8
    markers.markers.append(arrow)
    marker_pub.publish(markers)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
