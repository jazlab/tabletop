import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
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
        package="tabletop_moveit_interface",
        # executable="moveit_interface",
        executable="moveit_srv_node",
        # name="moveit_srv_node",
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
    bag = ExecuteProcess(cmd=["ros2", "bag", "record", "-a"], output="screen")

    return LaunchDescription(
        [
            ur_robot_driver,
            moveit,
            moveit_interface_server,
            tabletop_server,
            teensy_controller,
            teensy_sensor,
            bag,
        ]
    )
