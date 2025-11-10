import math

import launch
import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    Shutdown,
)
from launch.substitutions import (
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import LifecycleNode, Node, SetROSLogDir
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare


def declare_arguments():
    return [
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_rig"),
                    "config",
                    "optitrack.yaml",
                ]
            ),
            description="Commander config file",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Optitrack log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Using or not time from simulation",
        ),
    ]


def generate_launch_description():
    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # Optitrack driver
    driver = LifecycleNode(
        name="mocap4r2_optitrack_driver_node",
        namespace="",
        package="mocap4r2_optitrack_driver",
        output="both",
        executable="mocap4r2_optitrack_driver_main",
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    # Make the driver node take the 'configure' transition
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(
                driver
            ),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the driver node take the 'activate' transition
    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(
                driver
            ),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    # Static transform publisher for optitrack frame
    tf_publisher = Node(
        name="optitrack_static_transform_publisher",
        package="tf2_ros",
        executable="static_transform_publisher",
        output="both",
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
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        on_exit=[Shutdown()],
    )

    # Rviz visualizer for optitrack markers
    marker_viz = Node(
        name="mocap4r2_maker_viz",
        package="mocap4r2_marker_viz",
        executable="mocap4r2_marker_viz",
        output="both",
        emulate_tty=True,
        parameters=[
            {
                "mocap4r2_system": "optitrack",
                "marker_topics": ["markers", "predicted_markers"],
                "rb_topics": ["rigid_bodies"],
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        on_exit=[Shutdown()],
    )

    launch_actions = [
        *declare_arguments(),
        set_ros_log_dir,
        driver,
        configure_event,
        activate_event,
        tf_publisher,
        marker_viz,
    ]

    return LaunchDescription(launch_actions)
