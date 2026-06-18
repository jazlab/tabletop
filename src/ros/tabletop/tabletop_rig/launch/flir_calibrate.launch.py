"""Launch file for FLIR camera calibration.

Launches the camera_calibration cameracalibrator node with FLIR cameras
from flir.launch.py for interactive checkerboard-based camera calibration.

Nodes Launched:
    flir.launch.py (included): FLIR camera driver
    cameracalibrator (camera_calibration): Interactive calibration tool

Config Files Loaded:
    - flir.yaml: Camera configuration (via flir.launch.py)

Example:
    ros2 launch tabletop_rig flir_calibrate.launch.py \
        camera:=left_front_top_cam size:=9x11 square:=0.015
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
            "camera",
            description="Camera to calibrate (e.g. left_front_top_cam)",
        ),
        DeclareLaunchArgument(
            "size",
            default_value="9x11",
            description="Calibration grid dimensions (e.g. 9x11)",
        ),
        DeclareLaunchArgument(
            "square",
            default_value="0.015",
            description="Calibration square size (e.g. 0.015)",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use simulated time",
        ),
    ]


def check_camera_fn(context):
    camera = LaunchConfiguration("camera").perform(context)
    if camera == "all":
        raise ValueError("Camera all is not allowed")


def generate_launch_description():
    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    check_camera = OpaqueFunction(function=check_camera_fn)

    # Flir (use group action to isolate the launch file)
    flir = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_rig"),
                                "launch",
                                "flir.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "camera": LaunchConfiguration("camera"),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
    )

    calibration = Node(
        package="camera_calibration",
        executable="cameracalibrator",
        arguments=[
            "--size",
            LaunchConfiguration("size"),
            "--square",
            LaunchConfiguration("square"),
            ["image:=/", LaunchConfiguration("camera"), "/image_raw"],
            ["camera:=/", LaunchConfiguration("camera")],
            # "--no-service-check",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        on_exit=[Shutdown()],
    )

    launch_actions = [
        *declare_arguments(),
        set_ros_log_dir,
        check_camera,
        flir,
        calibration,
    ]

    return LaunchDescription(launch_actions)
