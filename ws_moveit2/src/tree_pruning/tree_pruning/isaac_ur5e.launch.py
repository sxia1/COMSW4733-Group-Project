import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Command-line arguments
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    moveit_config = (
        #MoveItConfigsBuilder("moveit_resources_ur5e")
        MoveItConfigsBuilder("ur5e_cutter")
        .robot_description(
            file_path="config/ur5e.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path="config/ur5e.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Static TF
    world2robot_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "ur5e_base_link"],
    )
    #cutter2camera_tf_node = Node(
    #    package="tf2_ros",
    #    executable="static_transform_publisher",
    #    name="static_transform_publisher",
    #    output="log",
    #    arguments=[
    #        "0.04",
    #        "0.0",
    #        "0.04",
    #        "0.0",
    #        "0.0",
    #        "0.0",
    #        "ur5e_cutter",
    #        "sim_camera",
    #    ],
    #)

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        #get_package_share_directory("moveit_resources_ur5e_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    ur5e_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur5e_arm_controller", "-c", "/controller_manager"],
    )

    ur5e_cutter_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur5e_cutter_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription(
        [
            ros2_control_hardware_type,
            world2robot_tf_node,
            #cutter2camera_tf_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            ur5e_arm_controller_spawner,
            ur5e_cutter_controller_spawner,
        ]
    )
