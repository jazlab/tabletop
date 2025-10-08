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
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Using or not time from simulation",
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
            description="Flir log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "output",
            default_value="both",
            description="Flir output",
            choices=["log", "both", "screen", "own_log"],
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
                                FindPackageShare("tabletop_server"),
                                "launch",
                                "flir.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "camera": LaunchConfiguration("camera"),
                    "output": LaunchConfiguration("output"),
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
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        output=LaunchConfiguration("output"),
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


# def main():
#     launch_logging_config.level = "DEBUG"
#     ls = LaunchService()
#     ld = generate_launch_description()
#     ls.include_launch_description(ld)
#     return ls.run()


# if __name__ == "__main__":
#     main()
