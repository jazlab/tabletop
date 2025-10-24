from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    LaunchLogDir,
)
from launch_ros.actions import Node, SetROSLogDir


def declare_arguments():
    return [
        # Common
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Using or not time from simulation",
        ),
        # Teensy
        DeclareLaunchArgument(
            "micro_ros_transport",
            default_value="serial",
            description="Micro ROS Agent transport protocol",
        ),
        DeclareLaunchArgument(
            "micro_ros_device",
            default_value="/dev/ttyACM0",
            description="Micro ROS Agent serial device",
        ),
        DeclareLaunchArgument(
            "micro_ros_baudrate",
            default_value="115200",
            description="Micro ROS Agent serial baudrate",
        ),
        DeclareLaunchArgument(
            "micro_ros_verbosity",
            default_value="4",
            description="Micro ROS Agent verbose level",
        ),
        DeclareLaunchArgument(
            "simulate",
            default_value="false",
            choices=["true", "false"],
            description="Use mock TeensyBoard",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Teensy log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "output",
            default_value="both",
            description="Micro ROS output",
            choices=["log", "both", "screen", "own_log"],
        ),
    ]


def generate_launch_description():
    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # Micro ROS Agent
    micro_ros_agent = Node(
        package="micro_ros_agent",
        name="micro_ros_agent",
        executable="micro_ros_agent",
        output=LaunchConfiguration("output"),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        arguments=[
            LaunchConfiguration("micro_ros_transport"),
            "--dev",
            LaunchConfiguration("micro_ros_device"),
            "--baudrate",
            LaunchConfiguration("micro_ros_baudrate"),
            "--verbose",
            LaunchConfiguration("micro_ros_verbosity"),
        ],
        condition=UnlessCondition(LaunchConfiguration("simulate")),
        on_exit=[Shutdown()],
    )

    # Mock Teensy
    mock_teensy = Node(
        name="teensy",
        package="tabletop_server",
        executable="mock_teensy",
        output=LaunchConfiguration("output"),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        condition=IfCondition(LaunchConfiguration("simulate")),
        on_exit=[Shutdown()],
    )

    launch_actions = [
        *declare_arguments(),
        set_ros_log_dir,
        micro_ros_agent,
        mock_teensy,
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
