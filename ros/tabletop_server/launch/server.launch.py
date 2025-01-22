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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path) as file:
        return yaml.safe_load(file)


def declare_arguments():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz?",
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("tabletop_description"),
                        "rviz",
                        "view_robot.rviz",
                    ]
                ),
                description="RViz config file",
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
                "description_launchfile",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("tabletop_description"),
                        "launch",
                        "rsp.launch.py",
                    ]
                ),
                description="URDF/XACRO description file with the robot.",
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
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    ur_type = LaunchConfiguration("ur_type")
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
    description_launchfile = LaunchConfiguration("description_launchfile")
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

    moveit_py_config = moveit_config.to_dict() | {
        "use_sim_time": use_sim_time,
        "publish_robot_description_semantic": publish_robot_description_semantic,
    }

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
            "launch_rviz": launch_rviz,
            "rviz_config_file": rviz_config_file,
            "kinematics_params_file": kinematics_params_file,
            "description_launchfile": description_launchfile,
            "description_file": description_file,
            "use_sim_time": use_sim_time,
        }.items(),
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
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Teensy Sensor
    teensy_sensor = Node(
        namespace="tabletop",
        name="teensy_sensor",
        package="tabletop_server",
        executable="teensy_sensor",
        parameters=[{"use_sim_time": use_sim_time}],
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
