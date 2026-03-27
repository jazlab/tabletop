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
            default_value="foraging_ordered",
            description="Task configuration file",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="ur5e",
            description="Robot name for SRDF",
        ),
        DeclareLaunchArgument(
            "robot_mode",
            default_value="mock",
            choices=["mock", "ursim", "real"],
            description="Whether to use the mock robot, URSim, or real robot",
        ),
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

    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # Rig
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
                    "robot_name": LaunchConfiguration("robot_name"),
                    "robot_mode": LaunchConfiguration("robot_mode"),
                    "coro_module": coro_module,
                    "coro_name": coro_name,
                    "coro_config": coro_config,
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
    )

    # # Commander
    # commander = GroupAction(
    #     [
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 [
    #                     PathJoinSubstitution(
    #                         [
    #                             FindPackageShare("tabletop_rig"),
    #                             "launch",
    #                             "commander.launch.py",
    #                         ]
    #                     )
    #                 ]
    #             ),
    #             launch_arguments={
    #                 "robot_name": LaunchConfiguration("robot_name"),
    #                 "robot_mode": LaunchConfiguration("robot_mode"),
    #                 "coro_module": coro_module,
    #                 "coro_name": coro_name,
    #                 "coro_config": coro_config,
    #             }.items(),
    #         ),
    #     ],
    #     scoped=True,
    #     forwarding=True,
    # )

    launch_actions = [set_ros_log_dir, rig]

    return LaunchDescription(declare_arguments() + launch_actions)
