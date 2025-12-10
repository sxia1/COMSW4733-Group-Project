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

OX = sqrt(0.5)
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
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.15
        pose.pose.position.z = 0.6
        pose.pose.orientation.x = OX
        pose.pose.orientation.y = OY
        pose.pose.orientation.z = OZ
        pose.pose.orientation.w = OW

        obj.meshes.append(ros_mesh)
        obj.mesh_poses.append(pose.pose)
        obj.operation = CollisionObject.ADD

        # Planning Scene update
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)

        self.get_logger().info("Publishing mesh object to planning_scene")
        self.scene_pub.publish(scene)

def main():
    rclpy.init()
    tree = AddTree()
    rclpy.spin_once(tree, timeout_sec=2.0)
    tree.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
