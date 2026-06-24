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
        Options: "true" (default), "false"

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
    EqualsSubstitution,
    IfElseSubstitution,
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
            "robot_name",
            default_value="tabletop",
            description="Robot name for SRDF",
        ),
        DeclareLaunchArgument(
            "robot_mode",
            default_value="mock",
            choices=["mock", "real"],
            description="Whether to use the mock robot or real robot",
        ),
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
            default_value="true",
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

    When task is "null", the commander launches without running any task
    coroutine, useful for debugging or manual operation.

    Returns:
        LaunchDescription containing all launch actions.
    """
    task = LaunchConfiguration("task")

    coro_config = IfElseSubstitution(
        EqualsSubstitution(task, "null"),
        if_value="null",
        else_value=PathJoinSubstitution(
            [
                FindPackageShare("tabletop_tasks"),
                "config",
                [task, ".yaml"],
            ]
        ),
    )
    coro_module = IfElseSubstitution(
        EqualsSubstitution(task, "null"),
        if_value="null",
        else_value="tabletop_tasks",
    )
    coro_name = IfElseSubstitution(
        EqualsSubstitution(task, "null"),
        if_value="null",
        else_value="run_tasks",
    )

    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # Commander: included directly (not via rig.launch.py aggregator).
    # GroupAction with scoped=True and forwarding=True is required so that
    # commander.launch.py's RegisterEventHandler actions are visible to the
    # top-level launch context (a plain scoped=False GroupAction breaks them).
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
                    "robot_name": LaunchConfiguration("robot_name"),
                    "robot_mode": LaunchConfiguration("robot_mode"),
                    "commander_log_level": LaunchConfiguration(
                        "commander_log_level"
                    ),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "coro_module": coro_module,
                    "coro_name": coro_name,
                    "coro_config": coro_config,
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
