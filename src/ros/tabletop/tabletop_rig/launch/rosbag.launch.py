"""Launch file for ROS 2 bag recording.

Launches the ros2 bag record command with configuration from rosbag.yaml.
Creates dated session directories and maintains a "latest" symlink. Supports
image transport plugin selection and topic/service filtering.

Executables Launched:
    ros2 bag record: ROS 2 data recording with configurable topics/services
    diagnostic_rosbag_recorder: clean recorder with scoped per-topic loss logs

Config Files Loaded:
    - rosbag.yaml: Recording configuration (topics, services, size limits)

Example:
    ros2 launch tabletop_rig rosbag.launch.py \
        image_transport:=compressed use_sim_time:=false
"""

import os
from datetime import datetime

import yaml
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
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
            default_value="90",
            description="Sigterm timeout for rosbag recorder (set to high so we don't lose data)",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "per_topic_loss_debug",
            default_value="true",
            choices=["true", "false"],
            description="Use scoped recorder DEBUG logs for per-topic DDS losses",
        ),
        DeclareLaunchArgument(
            "flir_preflight",
            default_value="true",
            choices=["true", "false"],
            description="Gate recorded tasks on FLIR image/rate readiness",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use simulated time",
        ),
    ]


def launch_setup(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    # Set up session directory with timestamp and latest symlink,
    # then build rosbag recorder command with topics/services from config
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
    share_dir = get_package_share_directory("tabletop_rig")
    config_file = os.path.join(share_dir, "config", "rosbag.yaml")
    qos_file = os.path.join(share_dir, "config", "rosbag_qos.yaml")
    fastdds_file = os.path.join(share_dir, "config", "fastdds_flir.xml")
    with open(config_file, "r") as f:
        config: dict = yaml.safe_load(f)

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    diagnostic_args = [
        "--output",
        bag_dir,
        "--storage",
        "mcap",
        "--qos-profile-overrides-path",
        qos_file,
    ]
    if use_sim_time == "true":
        diagnostic_args.append("--use-sim-time")

    log_level = LaunchConfiguration("log_level").perform(context).lower()
    diagnostic_args.extend(["--log-level", log_level])

    max_bag_size_gb = config.get("max_bag_size_gb", None)
    if max_bag_size_gb is not None:
        max_bag_size = int(max_bag_size_gb * 1_000_000_000)
        diagnostic_args.extend(["--max-bag-size", str(max_bag_size)])

    max_cache_size_mb = config.get("max_cache_size_mb", None)
    if max_cache_size_mb is not None:
        max_cache_size = int(max_cache_size_mb * 1_000_000)
        diagnostic_args.extend(["--max-cache-size", str(max_cache_size)])

    topics = config.get("topics", [])
    if topics == "all":
        diagnostic_args.append("--all-topics")
    elif len(topics) > 0:
        topics = list(topics)
        transport = LaunchConfiguration("image_transport").perform(context)
        if transport != "raw":
            for i, topic in enumerate(topics):
                if "image_raw" in topic:
                    topics[i] = f"{topic}/{transport}"
        diagnostic_args.extend(["--topics", *topics])

    services = config.get("services", [])
    if services == "all":
        diagnostic_args.append("--all-services")
    elif len(services) > 0:
        diagnostic_args.extend(["--services", *services])

    sigterm_timeout = LaunchConfiguration("rosbag_sigterm_timeout").perform(
        context
    )
    diagnostic_executable = os.path.join(
        get_package_prefix("tabletop_rig"),
        "lib",
        "tabletop_rig",
        "diagnostic_rosbag_recorder",
    )

    if LaunchConfiguration("per_topic_loss_debug").perform(context) == "true":
        diagnostic_args.append("--per-topic-loss-debug")
    if LaunchConfiguration("flir_preflight").perform(context) == "true":
        diagnostic_args.append("--flir-preflight")
    recorder_cmd = [diagnostic_executable, *diagnostic_args]
    recorder_cpu_set = os.environ.get(
        "TABLETOP_RECORDER_CPU_SET", ""
    ).strip()
    if recorder_cpu_set:
        print(f"Rosbag recorder CPU affinity: {recorder_cpu_set}")
        recorder_cmd = [
            "taskset",
            "--cpu-list",
            recorder_cpu_set,
            *recorder_cmd,
        ]

    bag_recorder = ExecuteProcess(
        name="rosbag_recorder",
        cmd=recorder_cmd,
        additional_env={
            "FASTDDS_DEFAULT_PROFILES_FILE": fastdds_file,
            # Jazzy's packaged Fast DDS 2.14 library uses the legacy name.
            "FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_file,
        },
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
