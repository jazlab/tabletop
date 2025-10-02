import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, Node, SetROSLogDir
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def make_camera_node(name, camera_type, serial_number, **params):
    parameter_file = PathJoinSubstitution(
        [
            FindPackageShare("spinnaker_camera_driver"),
            "config",
            camera_type + ".yaml",
        ]
    )

    node = ComposableNode(
        package="spinnaker_camera_driver",
        plugin="spinnaker_camera_driver::CameraDriver",
        name=name,
        parameters=[
            {"parameter_file": parameter_file, "serial_number": serial_number},
            params,
        ],
        remappings=[
            ("~/control", "/exposure_control/control"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    return node


def declare_arguments():
    return [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Using or not time from simulation",
        ),
        DeclareLaunchArgument(
            "flir_log_level",
            default_value="INFO",
            description="Flir log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        # Outputs
        DeclareLaunchArgument(
            "flir_output",
            default_value="both",
            description="Flir output",
            choices=["log", "both", "screen", "own_log"],
        ),
        DeclareLaunchArgument(
            "launch_rviz_flir",
            default_value="true",
            choices=["true", "false"],
            description="Launch RViz for Flir camera feed?",
        ),
        DeclareLaunchArgument(
            "rviz_flir_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_server"),
                    "rviz",
                    "server.rviz",
                ]
            ),
            description="RViz config file",
        ),
    ]


def generate_launch_description():
    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # Flir multi-camera setup
    flir_config_file = os.path.join(
        get_package_share_directory("tabletop_server"), "config", "flir.yaml"
    )
    with open(flir_config_file, "r") as f:
        flir_config = yaml.safe_load(f)

    flir_nodes = []
    for config in flir_config["cameras"]:
        config = flir_config["common"] | config
        flir_nodes.append(make_camera_node(**config))

    flir_camera_container = ComposableNodeContainer(
        name="flir_camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=flir_nodes,
        output=LaunchConfiguration("flir_output"),
        on_exit=[Shutdown()],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="own_log",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        # arguments=[
        #     "-d",
        #     LaunchConfiguration("rviz_config_file_server"),
        #     "-l",
        # ],  # -l for ogre log
        cwd=LaunchLogDir(),
        condition=IfCondition(LaunchConfiguration("launch_rviz_flir")),
        on_exit=[Shutdown()],
    )
    launch_actions = [
        *declare_arguments(),
        set_ros_log_dir,
        flir_camera_container,
        rviz,
    ]

    return LaunchDescription(launch_actions)


# def main():
#     launch_logging_config.level = "DEBUG"
#     ls = LaunchService()
#     ld = generate_launch_description()
#     ls.include_launch_description(ld)
#     return ls.run()


# if __name__ == "__main__":
#     main()
