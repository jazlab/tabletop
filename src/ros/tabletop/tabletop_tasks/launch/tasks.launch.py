"""ROS2 launch file for running TableTop experimental tasks.

This launch file provides the entry point for running behavioral experiments
with the TableTop system. It launches the complete rig infrastructure and
configures it to run the specified task.

Launch Arguments:
    task: Name of the task configuration file (without .yaml extension).
        Default: "foraging_ordered"
        Special value "null" runs the rig without any task.
    robot_name: Robot model name for SRDF loading.
        Default: "ur5e"
    robot_mode: Robot connection mode.
        Options: "mock" (default), "ursim", "real"

Usage:
    # Run with default foraging task
    ros2 launch tabletop_tasks tasks.launch.py

    # Run with specific task and real robot
    ros2 launch tabletop_tasks tasks.launch.py task:=smooth_pursuit robot_mode:=real

    # Run rig only without task (for debugging)
    ros2 launch tabletop_tasks tasks.launch.py task:=null
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
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
            default_value="foraging_random",
            description="Task configuration file",
        )
    ]


def generate_launch_description():
    """Generate the launch description for running tasks.

    Configures the task runner by computing the coroutine configuration
    from launch arguments and including the tabletop_rig launch file.

    When task is "null", the rig launches without running any task
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

    # TODO: Rig is currently unscoped because it included commander.launch.py,
    # which has event handlers and the context does not seem to persist for
    # launch entities started after the initial entities
    rig = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("tabletop_rig"),
                            "launch",
                            "rig.launch.py",
                        ]
                    ),
                ),
                launch_arguments={
                    "coro_module": coro_module,
                    "coro_name": coro_name,
                    "coro_config": coro_config,
                }.items(),
            ),
        ],
        scoped=False,
        forwarding=True,
    )

    return LaunchDescription([set_ros_log_dir, *declare_arguments(), rig])
