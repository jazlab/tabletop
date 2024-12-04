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
                "rosbag_args",
                default_value="-a",
                description="Using or not time from simulation",
            ),
            DeclareLaunchArgument(
                "moveit_interface_service_name",
                default_value="tabletop_moveit_interface/goal_pose",
            ),
        ]
    )


def generate_launch_description():
    args = declare_arguments()
    rosbag_args = LaunchConfiguration("rosbag_args")
    moveit_interface_service_name = LaunchConfiguration(
        "moveit_interface_service_name"
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur", package_name="tabletop_moveit_config"
        )
        .robot_description_semantic(
            os.path.join("srdf", "ur.srdf.xacro"), {"name": "ur5e"}
        )
        .to_moveit_configs()
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
            "launch_rviz": "false",
        }.items(),
    )

    moveit_interface_server = Node(
        package="tabletop_moveit_interface",
        executable="server",
        output="screen",
        parameters=[
            {"service_name": moveit_interface_service_name},
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )
    tabletop_commander = Node(
        name="commander",
        package="tabletop_server",
        executable="commander",
        parameters=[
            {"moveit_interface_service_name": moveit_interface_service_name}
        ],
    )
    teensy_controller = Node(
        namespace="tabletop",
        name="teensy_controller",
        package="tabletop_server",
        executable="teensy_controller",
    )
    teensy_sensor = Node(
        namespace="tabletop",
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
            tabletop_commander,
            teensy_controller,
            teensy_sensor,
            bag,
        ]
    )
