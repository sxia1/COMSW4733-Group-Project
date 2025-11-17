#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class UR5eMover(Node):
    def __init__(self):
        super().__init__('ur5e_mover')
        # Publisher to send joint positions to UR5e in Isaac Sim
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.get_logger().info("UR5e mover node started.")

    def move_to_joint_positions(self, joint_positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "mount_base"
        msg.name = [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
        ]
        msg.position = joint_positions
        msg.velocity = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        msg.effort = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        self.joint_pub.publish(msg)
        self.get_logger().info(f"Publishing JointState message: {msg}")

def main():
    rclpy.init()
    node = UR5eMover()

    while rclpy.ok():
        # Example joint positions [rad] for UR5e (6 joints)
        #joint_positions = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        #joint_positions =  [1.57, 1.57, 1.57, 1.57, 1.57, 1.57]
        joint_positions =  [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]
        #joint_positions = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
        node.move_to_joint_positions(joint_positions)

        # Keep node alive for a few seconds so Isaac Sim can receive command
        rclpy.spin_once(node, timeout_sec=5.0)
        time.sleep(1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

