"""
A launch file for running the motion planning python api tutorial
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except (
        OSError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def declare_arguments():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="Launch RViz?",
            ),
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur5e",
                description="Type/series of used UR robot.",
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
                default_value="--all",
                description="'ros2 bag' command line args",
            ),
            DeclareLaunchArgument(
                "rosbag_dir",
                default_value="/root/ws/src/tabletop/bags",
                description="Base directory to save rosbags",
            ),
            DeclareLaunchArgument(
                "timer_sec",
                default_value="1",
                description="Timer sec",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.13.10",
                description="IP address of the robot",
            ),
            DeclareLaunchArgument(
                "reverse_ip",
                default_value="192.168.13.11",
                description="Reverse IP address",
            ),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="Use mock hardware",
            ),
            DeclareLaunchArgument(
                "controller_spawner_timeout",
                default_value="120",
                description="Controller spawner timeout",
            ),
        ]
    )


def generate_launch_description():
    args = declare_arguments()
    launch_rviz = LaunchConfiguration("launch_rviz")
    ur_type = LaunchConfiguration("ur_type")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_description_semantic = LaunchConfiguration(
        "publish_robot_description_semantic"
    )
    controller_spawner_timeout = LaunchConfiguration(
        "controller_spawner_timeout"
    )
    robot_ip = LaunchConfiguration("robot_ip")
    reverse_ip = LaunchConfiguration("reverse_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    rosbag_args = LaunchConfiguration("rosbag_args")
    rosbag_dir = LaunchConfiguration("rosbag_dir")

    # Load configs
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur", package_name="tabletop_moveit_config"
        )
        .robot_description_semantic(
            file_path="srdf/ur.srdf.xacro", mappings={"name": ur_type}
        )
        .moveit_cpp(
            file_path="config/moveit_cpp.yaml",
        )
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    moveit_py_config = (
        moveit_config.to_dict()
        | warehouse_ros_config
        | {
            "use_sim_time": use_sim_time,
            "publish_robot_description_semantic": publish_robot_description_semantic,
        }
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("tabletop_moveit_config"),
        "config",
        "moveit.rviz",
    )

    commander_config = {
        "timer_sec": 1,
        "goals": [1],
    }

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
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "reverse_ip": reverse_ip,
            "use_mock_hardware": use_mock_hardware,
            "controller_spawner_timeout": controller_spawner_timeout,
            "launch_rviz": launch_rviz,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    commander = Node(
        package="tabletop_server",
        executable="commander_moveit_py",
        output="both",
        parameters=[moveit_py_config, commander_config],
        # prefix=["gdbserver :3000"],
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
        cwd=rosbag_dir,
        output="screen",
    )

    return LaunchDescription(
        [
            args,
            ur_robot_driver,
            commander,
            rviz,
            teensy_controller,
            teensy_sensor,
            bag,
        ]
    )


# def main():
#     ls = LaunchService()
#     ld = generate_launch_description()
#     ls.include_launch_description(ld)
#     return ls.run()


# if __name__ == "__main__":
#     main()
