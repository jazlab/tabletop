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
        DeclareLaunchArgument(
            "launch_rig",
            default_value="true",
            description="Launch rig?",
            choices=["true", "false"],
        ),
    ]


def generate_launch_description():
    launch_rig = LaunchConfiguration("launch_rig")
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
        condition=IfCondition(launch_rig),
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
