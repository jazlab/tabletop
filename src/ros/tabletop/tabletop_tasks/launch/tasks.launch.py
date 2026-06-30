"""ROS2 launch file for running TableTop experimental tasks.

This launch file provides the entry point for running behavioral experiments
with the TableTop system. It launches the commander and (optionally) rosbag
recording, then configures the commander to run the specified task.

Launch Arguments:
    task: Name of the task configuration file (without .yaml extension).
        Default: "foraging_ordered"
        Special value "null" runs the commander without any task.
    robot_name: Robot model name for SRDF loading.
        Default: "tabletop"
    robot_mode: Robot connection mode.
        Options: "mock" (default), "real"
    rosbag: Whether to record a rosbag.
        Options: "true", "false" (default)

Usage:
    # Run with default foraging task
    ros2 launch tabletop_tasks tasks.launch.py

    # Run with specific task and real robot
    ros2 launch tabletop_tasks tasks.launch.py task:=smooth_pursuit robot_mode:=real

    # Run commander only without task (for debugging)
    ros2 launch tabletop_tasks tasks.launch.py task:=null
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import SetROSLogDir
from launch_ros.substitutions import FindPackageShare


def declare_arguments():
    """Declare launch arguments for the tasks launch file.

    Returns:
        List of DeclareLaunchArgument actions defining the configurable
        parameters for this launch file.
    """
    return [
        DeclareLaunchArgument(
            "task",
            default_value="foraging_ordered",
            description="Task configuration file",
        ),
        # Common
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use simulated time",
        ),
        # Commander
        DeclareLaunchArgument(
            "commander_log_level",
            default_value="INFO",
            description="Commander log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "commander_output",
            default_value="both",
            description="Commander output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # Bag
        DeclareLaunchArgument(
            "rosbag",
            default_value="false",
            choices=["true", "false"],
            description="Record rosbag?",
        ),
        DeclareLaunchArgument(
            "rosbag_log_level",
            default_value="INFO",
            description="ROS bag log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "rosbag_output",
            default_value="both",
            description="ROS bag output",
            choices=["log", "both", "screen", "own_log"],
        ),
    ]


def generate_launch_description():
    """Generate the launch description for running tasks.

    Configures the task runner by computing the coroutine configuration
    from launch arguments, then directly includes commander.launch.py
    and (optionally) rosbag.launch.py.

    Returns:
        LaunchDescription containing all launch actions.
    """
    task = LaunchConfiguration("task")

    coro_config = PathJoinSubstitution(
        [FindPackageShare("tabletop_tasks"), "config", [task, ".yaml"]]
    )

    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    commander = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=LaunchConfiguration("commander_output"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("tabletop_rig"),
                            "launch",
                            "commander.launch.py",
                        ]
                    ),
                ),
                launch_arguments={
                    "commander_log_level": LaunchConfiguration(
                        "commander_log_level"
                    ),
                    "coro_module": "tabletop_tasks",
                    "coro_name": "run_tasks",
                    "coro_config": coro_config,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
    )

    rosbag = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=LaunchConfiguration("rosbag_output"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("tabletop_rig"),
                            "launch",
                            "rosbag.launch.py",
                        ]
                    ),
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "log_level": LaunchConfiguration("rosbag_log_level"),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
        condition=IfCondition(LaunchConfiguration("rosbag")),
    )

    return LaunchDescription(
        [set_ros_log_dir, *declare_arguments(), commander, rosbag]
    )
