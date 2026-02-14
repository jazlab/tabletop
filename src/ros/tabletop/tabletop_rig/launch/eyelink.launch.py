from launch import (
    LaunchDescription,
)
from launch.actions import (
    DeclareLaunchArgument,
    Shutdown,
)
from launch.substitutions import (
    LaunchConfiguration,
    LaunchLogDir,
)
from launch_ros.actions import Node, SetROSLogDir


def declare_arguments():
    return [
        DeclareLaunchArgument(
            "simulate",
            default_value="false",
            choices=["true", "false"],
            description="Simulate Flic",
        ),
        # DeclareLaunchArgument(
        #     "initial_bag_dir",
        #     default_value="null",
        #     description="Initial bag directory for Eyelink",
        # ),
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Flic log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use simulated time",
        ),
    ]


def generate_launch_description():
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    eyelink = Node(
        package="tabletop_rig",
        executable="eyelink",
        output="both",
        parameters=[
            {
                "simulate": LaunchConfiguration("simulate"),
                # "session_bag_dir": ParameterValue(
                #     LaunchConfiguration("initial_bag_dir"), value_type=str
                # ),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        ros_arguments=[
            "--log-level",
            ["eyelink:=", LaunchConfiguration("log_level")],
        ],
        on_exit=[Shutdown()],
    )

    launch_actions = [*declare_arguments(), set_ros_log_dir, eyelink]

    return LaunchDescription(launch_actions)
