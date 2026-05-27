from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    Shutdown,
)
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    LaunchLogDir,
)
from launch_ros.actions import Node, SetROSLogDir


def declare_arguments():
    return [
        DeclareLaunchArgument(
            "use_scapy",
            default_value="false",
            choices=["true", "false"],
            description="Whether or not to use the scapy backend",
        ),
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
            description="Use simulated time",
        ),
    ]


def generate_launch_description():
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())
    executable = IfElseSubstitution(
        LaunchConfiguration("use_scapy"), "flic_scapy", "flic"
    )

    flic = Node(
        package="tabletop_rig",
        executable=executable,
        output="both",
        parameters=[
            {
                "simulate": LaunchConfiguration("simulate"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        on_exit=[Shutdown()],
    )

    launch_actions = [*declare_arguments(), set_ros_log_dir, flic]

    return LaunchDescription(launch_actions)
