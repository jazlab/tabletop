"""
A launch file for running the motion planning python api tutorial
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def declare_arguments():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz", default_value="true", description="Launch RViz?"
            ),
            DeclareLaunchArgument(
                "ur_type",
                description="Typo/series of used UR robot.",
                choices=[
                    "ur3",
                    "ur3e",
                    "ur5",
                    "ur5e",
                    "ur10",
                    "ur10e",
                    "ur16e",
                    "ur20",
                    "ur30",
                ],
            ),
            DeclareLaunchArgument(
                "warehouse_sqlite_path",
                default_value=os.path.expanduser(
                    "~/.ros/warehouse_ros.sqlite"
                ),
                description="Path where the warehouse database should be stored",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Using or not time from simulation",
            ),
            DeclareLaunchArgument(
                "publish_robot_description_semantic",
                default_value="true",
                description="MoveGroup publishes robot description semantic",
            ),
            DeclareLaunchArgument(
                "rosbag_args",
                default_value="-a",
                description="Using or not time from simulation",
            ),
        ]
    )


def generate_launch_description():
    args = declare_arguments()
    rosbag_args = LaunchConfiguration("rosbag_args")
    launch_rviz = LaunchConfiguration("launch_rviz")
    ur_type = LaunchConfiguration("ur_type")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_description_semantic = LaunchConfiguration(
        "publish_robot_description_semantic"
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur", package_name="tabletop_moveit_config"
        )
        .robot_description_semantic(
            file_path="srdf/ur.srdf.xacro", mappings={"name": ur_type}
        )
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    moveit_py = Node(
        name="moveit_py",
        package="tabletop_server",
        executable="tabletop_server",
        output="both",
        parameters=[moveit_config.to_dict()],
    )
    ur_robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ur_robot_driver"),
                    "launch",
                    "ur_control.launch.py",
                )
            ]
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "robot_ip": "192.168.13.10",
            "reverse_ip": "192.168.13.11",
            "use_mock_hardware": "false",
            "controller_spawner_timeout": "120",
        }.items(),
    )
    teensy_controller = Node(
        name="teensy_controller",
        package="tabletop_server",
        executable="teensy_controller",
    )
    teensy_sensor = Node(
        name="teensy_sensor",
        package="tabletop_server",
        executable="teensy_sensor",
    )
    bag = ExecuteProcess(
        cmd=["ros2", "bag", "record", rosbag_args],
        output="screen",
    )

    return LaunchDescription(
        [
            args,
            moveit_py,
            ur_robot_driver,
            teensy_controller,
            teensy_sensor,
            bag,
        ]
    )
