import os
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchDescriptionEntity,
)
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import (
    LaunchConfiguration,
    LaunchLogDir,
)
from launch_ros.actions import SetROSLogDir


def declare_arguments():
    return [
        DeclareLaunchArgument(
            "image_transport",
            default_value="compressed",
            choices=["raw", "compressed", "ffmpeg"],
            description="Image transport to record for image topics",
        ),
        DeclareLaunchArgument(
            "rosbag_sigterm_timeout",
            default_value="60",
            description="Sigterm timeout for rosbag recorder (set to high so we don't lose data)",
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


def launch_setup(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    set_ros_log_dir = SetROSLogDir(LaunchLogDir().perform(context))

    # Create a new bag directory for the session and symlink to it
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    dirname = f"session_{timestamp}"
    session_bag_dir = os.path.join(os.environ["ROS_BAG_DIR"], dirname)
    symlink_path = os.path.join(os.environ["ROS_BAG_DIR"], "latest")
    bag_dir = os.path.join(session_bag_dir, "bag")

    os.makedirs(session_bag_dir, exist_ok=True)
    try:
        os.remove(symlink_path)
    except FileNotFoundError:
        pass
    os.symlink(dirname, symlink_path)

    # Get CLI args from config file
    config_file = os.path.join(
        get_package_share_directory("tabletop_rig"), "config", "rosbag.yaml"
    )
    with open(config_file, "r") as f:
        config: dict = yaml.safe_load(f)

    args = []
    if LaunchConfiguration("use_sim_time").perform(context) == "true":
        args.append("--use-sim-time")

    log_level = LaunchConfiguration("log_level").perform(context).lower()
    args.extend(["--log-level", log_level])

    max_bag_size_gb = config.get("max_bag_size_gb", None)
    if max_bag_size_gb is not None:
        max_bag_size = int(max_bag_size_gb * 10e9)
        args.extend(["--max-bag-size", str(max_bag_size)])

    max_cache_size_mb = config.get("max_cache_size_mb", None)
    if max_cache_size_mb is not None:
        max_cache_size = int(max_cache_size_mb * 10e6)
        args.extend(["--max-cache-size", str(max_cache_size)])

    topics = config.get("topics", [])
    if topics == "all":
        args.append("--all-topics")
    elif len(topics) > 0:
        transport = LaunchConfiguration("image_transport").perform(context)
        if transport != "raw":
            for i, topic in enumerate(topics):
                if "image_raw" in topic:
                    topics[i] = f"{topic}/{transport}"
        args.extend(["--topics", *topics])

    services = config.get("services", [])
    if services == "all":
        args.append("--all-services")
    elif len(services) > 0:
        args.extend(["--services", *services])

    sigterm_timeout = LaunchConfiguration("rosbag_sigterm_timeout").perform(
        context
    )

    bag_recorder = ExecuteProcess(
        name="rosbag_recorder",
        cmd=["ros2", "bag", "record", "-o", bag_dir, *args],
        output="both",
        sigterm_timeout=sigterm_timeout,
        on_exit=[Shutdown()],
    )
    # bag_converter = ExecuteProcess(
    #     name="bag_converter",
    #     cmd=[
    #         "ros2",
    #         "run",
    #         "tabletop_rig",
    #         "rosbag_to_csv",
    #         "--session-dir",
    #         session_bag_dir,
    #     ],
    #     shell=True,
    #     output="both",
    # )
    # bag_converter_handler = RegisterEventHandler(
    #     OnShutdown(on_shutdown=[bag_converter], handle_once=True)
    # )
    return [
        set_ros_log_dir,
        bag_recorder,
        # bag_converter_handler,
    ]


def generate_launch_description():
    return LaunchDescription(
        [*declare_arguments(), OpaqueFunction(function=launch_setup)]
    )
