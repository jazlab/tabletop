import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    command = Node(
        package="tabletop",
        executable="command",
        name="command",
        namespace="command",
    )
    teensy = Node(
        package="tabletop",
        executable="teensy",
        name="teensy",
        namespace="teensy",
    )
    bag = ExecuteProcess(cmd=["ros2", "bag", "record", "-a"], output="screen")
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
        },
    )
    return LaunchDescription(
        [
            command,
            teensy,
            bag,
            ur_robot_driver,
        ]
    )
