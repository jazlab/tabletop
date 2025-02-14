from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_service import LaunchService
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Common
    test = LaunchConfiguration("test")

    test2_ld = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_server"),
                                "launch",
                                "test2.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={"test": "false"}.items(),
            )
        ],
        scoped=True,
        forwarding=True,
    )

    process = ExecuteProcess(
        cmd=["echo", "test1:"],
        output="both",
        condition=IfCondition(test),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("test", default_value="true"),
            test2_ld,
            process,
        ]
    )


def main():
    ls = LaunchService()
    ld = generate_launch_description()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    main()
