import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder

def generate_launch_description():

    moveit_ros_benchmarks_config = (
        ParameterBuilder("ur5e_cutter_moveit_config")
        .yaml(
            parameter_namespace="benchmark_config",
            file_path="config/benchmarks.yaml",
        )
        .to_dict()
    )

    moveit_configs = (
        MoveItConfigsBuilder("ur5e_cutter", package_name="ur5e_cutter_moveit_config")
        .robot_description(
            file_path="config/ur5e.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": "isaac"
            },
        )
        .planning_pipelines(pipelines=["ompl"]) #, "stomp", "pilz_industrial_motion_planner"])
        .moveit_cpp(
            os.path.join(
                get_package_share_directory("ur5e_cutter_moveit_config"),
                "config",
                "benchmarking_moveit_cpp.yaml",
            )
        )
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection", # needed to use sqlite instead of mongodb
        "warehouse_host": "/root/ws_moveit/src/ur5e_cutter_moveit_config/config/ur_benchmarks.sqlite",
        "warehouse_port": 33829
    }

    ompl_planning_pipeline_config = {
        # currently a workaround for the Error: Multiple planning plugins available. You should specify the '~planning_plugin' parameter. Using 'chomp_interface/CHOMPPlanner' for now.
        "planning_pipeline": "ompl",
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "planning_interface": "ompl_interface/OMPLPlanning",
    }

    # moveit_ros_benchmark demo executable
    moveit_ros_benchmarks_node = Node(
        name="moveit_run_benchmark",
        package="moveit_ros_benchmarks",
        # prefix='xterm -e gdb --ex=run --args',
        executable="moveit_run_benchmark",
        output="screen",
        parameters=[
            moveit_ros_benchmarks_config,
            moveit_configs.to_dict(),
            warehouse_ros_config,
            ompl_planning_pipeline_config,
        ],
    )

    # Warehouse mongodb server
    # mongodb_server_node = Node(
    #     package="warehouse_ros_sqlite",
    #     executable="mongo_wrapper_ros.py",
    #     parameters=[
    #         {"warehouse_port": 33829},
    #         {"warehouse_host": "localhost"},
    #         {"warehouse_plugin": "warehouse_ros_sqlit::DatabaseConnection"},
    #     ],
    #     output="screen",
    # )

    return LaunchDescription([moveit_ros_benchmarks_node]) #, mongodb_server_node])

