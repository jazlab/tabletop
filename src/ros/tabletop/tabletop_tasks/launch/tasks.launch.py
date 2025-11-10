from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
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
    return [
        DeclareLaunchArgument(
            "launch_rig",
            default_value="true",
            description="Launch rig?",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "task_config",
            default_value="foraging_ordered.yaml",
            description="Task configuration file",
        ),
    ]


def generate_launch_description():
    launch_rig = LaunchConfiguration("launch_rig")
    task_config = LaunchConfiguration("task_config")

    coroutine_config = IfElseSubstitution(
        EqualsSubstitution(task_config, "null"),
        if_value="null",
        else_value=PathJoinSubstitution(
            [
                FindPackageShare("tabletop_tasks"),
                "config",
                task_config,
            ]
        ),
    )
    coroutine_module = IfElseSubstitution(
        EqualsSubstitution(task_config, "null"),
        if_value="null",
        else_value="tabletop_tasks",
    )
    coroutine_name = IfElseSubstitution(
        EqualsSubstitution(task_config, "null"),
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
            ),
        ],
        condition=IfCondition(launch_rig),
        scoped=True,
        forwarding=True,
    )

    # Commander
    commander = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_rig"),
                                "launch",
                                "commander.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "coroutine_module": coroutine_module,
                    "coroutine_name": coroutine_name,
                    "coroutine_config": coroutine_config,
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
    )

    launch_actions = [set_ros_log_dir, rig, commander]

    return LaunchDescription(declare_arguments() + launch_actions)
