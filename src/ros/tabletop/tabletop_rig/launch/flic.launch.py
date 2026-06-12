"""Launch file for the Flic button response time measurement.

Launches the Flic button interface node for measuring response times to
button presses during TableTop experiments.

Nodes Launched:
    flic (tabletop_rig): Flic button response time measurement interface

Example:
    ros2 launch tabletop_rig flic.launch.py simulate:=false
"""

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
            description="Use simulated time",
        ),
    ]


def generate_launch_description():
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    flic = Node(
        package="tabletop_rig",
        executable="flic",
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

    launch_actions = [
        *declare_arguments(),
        set_ros_log_dir,
        flic,
    ]

    return LaunchDescription(launch_actions)
