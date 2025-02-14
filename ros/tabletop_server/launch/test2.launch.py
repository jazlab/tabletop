from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_service import LaunchService
from launch.substitutions import (
    LaunchConfiguration,
)


def generate_launch_description():
    # Common
    test = LaunchConfiguration("test")

    process = ExecuteProcess(
        cmd=["echo", "test2:"],
        output="both",
        condition=IfCondition(test),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("test", default_value="true"),
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
