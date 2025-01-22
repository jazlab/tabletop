import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
            "robot_ip": "192.168.12.10",
            "reverse_ip": "192.168.12.11",
            "use_mock_hardware": "false",
            "controller_spawner_timeout": "120",
            "description_launchfile": os.path.join(
                get_package_share_directory("tabletop_description"),
                "launch",
                "rsp.launch.py",
            ),
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("tabletop_moveit_config"),
                    "launch",
                    "moveit.launch.py",
                )
            ]
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "launch_rviz": "true",
        }.items(),
    )

    return LaunchDescription([ur_robot_driver, moveit])
