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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except (
        OSError
    ) as e:  # parent of IOError, OSError *and* WindowsError where available
        print(f"Failed to load YAML file {absolute_file_path}: {e}")
        return None


def declare_arguments():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz_ur_driver",
                default_value="true",
                description="Launch RViz for UR Driver?",
            ),
            DeclareLaunchArgument(
                "launch_rviz_moveit",
                default_value="false",
                description="Launch RViz for MoveIt?",
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
                default_value="/root/ws/bags",
                description="Base directory to save rosbags",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.12.10",
                description="IP address of the robot",
            ),
            DeclareLaunchArgument(
                "reverse_ip",
                default_value="192.168.12.11",
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
            DeclareLaunchArgument(
                "kinematics_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("tabletop_server"),
                        "config",
                        "ursim_calibration.yaml",
                    ]
                ),
                description="Calibration file",
            ),
            DeclareLaunchArgument(
                "description_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("tabletop_description"),
                        "urdf",
                        "tabletop_control.urdf.xacro",
                    ]
                ),
                description="URDF/XACRO description file with the robot.",
            ),
        ]
    )


def generate_launch_description():
    args = declare_arguments()
    launch_rviz_ur_driver = LaunchConfiguration("launch_rviz_ur_driver")
    launch_rviz_moveit = LaunchConfiguration("launch_rviz_moveit")
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
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    description_file = LaunchConfiguration("description_file")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    rosbag_args = LaunchConfiguration("rosbag_args")
    rosbag_dir = LaunchConfiguration("rosbag_dir")

    # Load configs
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur", package_name="tabletop_moveit_config"
        )
        .robot_description_semantic(
            file_path="srdf/tabletop.srdf.xacro", mappings={"name": ur_type}
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

    moveit_rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("tabletop_moveit_config"),
            "config",
            "moveit.rviz",
        ]
    )

    ur_rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("ur_description"),
            "rviz",
            "view_robot.rviz",
        ]
    )

    commander_config = load_yaml("tabletop_server", "config/commander.yaml")

    # UR Robot Driver
    ur_robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur_robot_driver"),
                        "launch",
                        "ur_control.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "reverse_ip": reverse_ip,
            "use_mock_hardware": use_mock_hardware,
            "controller_spawner_timeout": controller_spawner_timeout,
            "launch_rviz": launch_rviz_ur_driver,
            "rviz_config_file": ur_rviz_config_file,
            "kinematics_params_file": kinematics_params_file,
            "description_file": description_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # MoveIt Rviz
    rviz = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz_moveit),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", moveit_rviz_config_file],
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

    # Commander
    commander = Node(
        package="tabletop_server",
        executable="commander",
        output="both",
        parameters=[moveit_py_config, commander_config],
        # prefix=["gdbserver :3000"],
    )

    # Teensy Controller
    teensy_controller = Node(
        namespace="tabletop",
        name="teensy_controller",
        package="tabletop_server",
        executable="teensy_controller",
    )

    # Teensy Sensor
    teensy_sensor = Node(
        namespace="tabletop",
        name="teensy_sensor",
        package="tabletop_server",
        executable="teensy_sensor",
    )

    # Bag
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
