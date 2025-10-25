import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    Shutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitution import Substitution
from launch.substitutions import (
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetROSLogDir
from launch_ros.substitutions import FindPackageShare


def declare_arguments():
    return [
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Optitrack log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "output",
            default_value="both",
            description="Optitrack output",
            choices=["log", "both", "screen", "own_log"],
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Using or not time from simulation",
        ),
    ]


def print_substitutions(context, substitutions: dict[str, Substitution]):
    for name, substitution in substitutions.items():
        print(f"{name}: {substitution.perform(context)}")


def generate_launch_description():
    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # Optitrack driver
    optitrack = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("mocap4r2_optitrack_driver"),
                                "launch",
                                "optitrack2.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "output": LaunchConfiguration("output"),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
    )

    # Static transform publisher for optitrack frame
    tf_publisher = Node(
        name="optitrack_static_transform_publisher",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.4925",
            "--y",
            "0.6025",
            "--z",
            "0.31",
            "--yaw",
            str(math.pi / 2),
            "--pitch",
            "0",
            "--roll",
            str(math.pi / 2),
            "--frame-id",
            "world",
            "--child-frame-id",
            "optitrack",
        ],
        output=LaunchConfiguration("optitrack_output"),
        on_exit=[Shutdown()],
    )

    # Rviz visualizer for optitrack markers
    mocap4r2_marker_viz = Node(
        package="mocap4r2_marker_viz",
        executable="mocap4r2_marker_viz",
        output=LaunchConfiguration("optitrack_output"),
        emulate_tty=True,
        parameters=[
            {
                "mocap4r2_system": "optitrack",
                "marker_topics": ["markers", "predicted_markers"],
                "rb_topics": ["rigid_bodies"],
            }
        ],
        on_exit=[Shutdown()],
    )

    launch_actions = [
        *declare_arguments(),
        set_ros_log_dir,
        optitrack,
        tf_publisher,
        mocap4r2_marker_viz,
    ]

    return LaunchDescription(launch_actions)
