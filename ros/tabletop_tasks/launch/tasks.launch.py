from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def declare_arguments():
    return [
        DeclareLaunchArgument(
            "task_executable",
            default_value="run_tasks",
            description="Name of the executable in tabletop_tasks package to run",
        ),
        DeclareLaunchArgument(
            "task_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_tasks"),
                    "config",
                    "tasks.yaml",
                ]
            ),
            description="Path to the task configuration file",
        ),
    ]


def generate_launch_description():
    task_executable = LaunchConfiguration("task_executable")
    task_config = LaunchConfiguration("task_config")

    server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_server"),
                    "launch",
                    "server.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "script_package": "tabletop_tasks",
            "script_executable": task_executable,
            "script_config": task_config,
        }.items(),
    )
    return LaunchDescription(declare_arguments() + [server_launch])
