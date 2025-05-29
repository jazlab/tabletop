from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
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
    return [
        DeclareLaunchArgument(
            "launch_server",
            default_value="true",
            description="Launch server?",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "task_config",
            default_value="foraging_ordered.yaml",
            description="Task configuration file",
        ),
    ]


def generate_launch_description():
    launch_server = LaunchConfiguration("launch_server")
    task_config = PathJoinSubstitution(
        [
            FindPackageShare("tabletop_tasks"),
            "config",
            LaunchConfiguration("task_config"),
        ]
    )

    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # Commander
    commander = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_server"),
                                "launch",
                                "commander.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "coroutine_module": "tabletop_tasks",
                    "coroutine_name": "run_tasks",
                    "coroutine_config": task_config,
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
    )

    # Server
    server = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("tabletop_server"),
                            "launch",
                            "server.launch.py",
                        ]
                    ),
                ),
            ),
        ],
        condition=IfCondition(launch_server),
        scoped=True,
        forwarding=True,
    )

    launch_actions = [
        set_ros_log_dir,
        commander,
        server,
    ]

    return LaunchDescription(declare_arguments() + launch_actions)
