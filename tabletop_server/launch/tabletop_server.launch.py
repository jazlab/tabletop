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


def declare_arguments():
    return LaunchDescription(
        [
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
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("tabletop_moveit_config"),
                    "launch",
                    "tabletop_moveit.launch.py",
                )
            ]
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "launch_rviz": "false",
        }.items(),
    )

    moveit_interface_server = Node(
        name="tabletop_moveit_interface",
        package="tabletop_moveit_interface",
        executable="server",
        output="screen",
    )

    tabletop_server = Node(
        name="tabletop_server",
        package="tabletop_server",
        executable="tabletop_server",
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
            ur_robot_driver,
            moveit,
            moveit_interface_server,
            tabletop_server,
            teensy_controller,
            teensy_sensor,
            bag,
        ]
    )
