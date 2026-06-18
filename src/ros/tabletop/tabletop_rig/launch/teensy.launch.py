"""Launch file for Teensy microcontroller interface.

Launches either the micro-ROS agent bridge to a real Teensy board or
a mock Teensy simulator node for testing without hardware.

Nodes Launched:
    micro_ros_agent (micro_ros_agent): Bridge to real Teensy via serial
    mock_teensy (tabletop_rig): Mock Teensy simulation node

Example:
    # Real hardware
    ros2 launch tabletop_rig teensy.launch.py simulate:=false \
        micro_ros_device:=/dev/ttyACM0

    # Simulation
    ros2 launch tabletop_rig teensy.launch.py simulate:=true
"""

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
            description="Log level",
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

    micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent_teensy",
        output="both",
        arguments=[
            LaunchConfiguration("micro_ros_transport"),
            "--dev",
            LaunchConfiguration("micro_ros_device"),
            "--baudrate",
            LaunchConfiguration("micro_ros_baudrate"),
            "--verbose",
            LaunchConfiguration("micro_ros_verbosity"),
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        condition=UnlessCondition(LaunchConfiguration("simulate")),
        on_exit=[Shutdown()],
    )

    mock_teensy = Node(
        name="teensy",
        package="tabletop_rig",
        executable="mock_teensy",
        output="both",
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
