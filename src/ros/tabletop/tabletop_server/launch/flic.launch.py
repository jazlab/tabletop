from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    Shutdown,
)
from launch.substitutions import (
    LaunchConfiguration,
    LaunchLogDir,
)
from launch_ros.actions import Node, SetROSLogDir


def declare_arguments():
    return [
        DeclareLaunchArgument(
            "simulate",
            default_value="false",
            choices=["true", "false"],
            description="Simulate Flic",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Flic log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Using or not time from simulation",
        ),
    ]


def generate_launch_description():
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    flic = Node(
        name="flic",
        package="tabletop_server",
        executable="flic",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "simulate": LaunchConfiguration("simulate"),
            },
        ],
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        on_exit=[Shutdown()],
    )

    launch_actions = [*declare_arguments(), set_ros_log_dir, flic]

    return LaunchDescription(launch_actions)
