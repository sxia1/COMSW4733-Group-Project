#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

#https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html#instantiating-moveit-py-and-planning-component
def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
):
    """A helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

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

def create_pose(w,x,y,z):
    # set pose goal with PoseStamped message
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "ur5e_link0"
    pose_goal.pose.orientation.w = w
    pose_goal.pose.position.x = x
    pose_goal.pose.position.y = y
    pose_goal.pose.position.z = z
    return pose_goal

def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    ur5e = MoveItPy(node_name="moveit_py")
    ur5e_arm = ur5e.get_planning_component("ur5e_arm")
    logger.info("MoveItPy instance created")
    
    # instantiate a RobotState instance using the current robot model
    robot_model = ur5e.get_robot_model()
    robot_state = RobotState(robot_model)

    # set plan start state to current state
    ur5e_arm.set_start_state_to_current_state()

    ur5e_arm.set_goal_state(
        pose_stamped_msg=pose_goal,
        pose_link="ur5e_link8"
    )
    ur5e_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="ur5e_link8")
    plan_and_execute(ur5e, ur5e_arm, logger)
    """
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
    """

if __name__ == '__main__':
    main()

