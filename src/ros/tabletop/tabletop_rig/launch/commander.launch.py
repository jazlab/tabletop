"""Launch file for the Commander node.

Launches the main commander orchestration node which coordinates all robot
interfaces (MoveIt, Teensy, Flic, Eyelink, Dashboard, Sound) for TableTop
experiments.

Nodes Launched:
    wait_for_robot_description (ur_robot_driver): Blocks until robot
        description is available
    commander (tabletop_rig): Main orchestration and experiment control

Config Files Loaded:
    - dual_tabletop.srdf.xacro: Robot semantic description (SRDF)
    - commander.yaml: Commander node parameters and overrides
    - moveit_cpp.yaml: MoveIt C++ interface configuration

Example:
    ros2 launch tabletop_rig commander.launch.py robot_mode:=mock
"""

import os

import yaml
from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchDescriptionEntity,
)
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    EqualsSubstitution,
    IfElseSubstitution,
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetROSLogDir
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

COMMANDER_OVERRIDES_TMP_PATH = "/tmp/commander_overrides.yaml"


def declare_arguments():
    return [
        DeclareLaunchArgument(
            "robot_name",
            default_value="tabletop",
            description="Robot name for MoveIt SRDF",
        ),
        DeclareLaunchArgument(
            "robot_mode",
            default_value="mock",
            choices=["mock", "ursim", "real"],
            description="Whether to use the mock robot, URSim, or real robot",
        ),
        DeclareLaunchArgument(
            "semantic_description_file",
            default_value=PathJoinSubstitution(
                ["srdf", "dual_tabletop.srdf.xacro"]
            ),
            description="SRDF/XACRO semantic robot description file.",
        ),
        DeclareLaunchArgument(
            "commander_param_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_rig"),
                    "config",
                    "commander.yaml",
                ]
            ),
            description="Commander parameter file",
        ),
        DeclareLaunchArgument(
            "commander_param_overrides_tmp_path",
            default_value=PathJoinSubstitution(
                ["/tmp", "commander_overrides.yaml"]
            ),
            description="Commander parameter overrides temporary path",
        ),
        DeclareLaunchArgument(
            "coro_module",
            default_value="null",
            description="Coroutine module",
        ),
        DeclareLaunchArgument(
            "coro_name",
            default_value="null",
            description="Coroutine name",
        ),
        DeclareLaunchArgument(
            "coro_config",
            default_value="null",
            description="Coroutine config",
        ),
        DeclareLaunchArgument(
            "new_cache",
            default_value="null",
            description="Whether to clear the trajectory cache",
            choices=["true", "false", "null"],
        ),
        DeclareLaunchArgument(
            "use_cache",
            default_value="null",
            description="Whether to use the trajectory cache",
            choices=["true", "false", "null"],
        ),
        DeclareLaunchArgument(
            "left_initial_object",
            default_value="null",
            description="The name or index of the initial attached object for the left robot",
        ),
        DeclareLaunchArgument(
            "right_initial_object",
            default_value="null",
            description="The name or index of the initial attached object for the right robot",
        ),
        DeclareLaunchArgument(
            "use_sound",
            default_value="null",
            description="Whether to enable sound from the commander",
            choices=["true", "false", "null"],
        ),
        DeclareLaunchArgument(
            "debug_commander",
            default_value="false",
            description="Whether to debug the commander",
            choices=["true", "false"],
        ),
        # ROS Warehouse
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.join(
                os.environ["TABLETOP_DIR"], "cache", "warehouse_ros.sqlite"
            ),
            description="Path where the warehouse database should be stored",
        ),
        # Commander sigterm timeout
        DeclareLaunchArgument(
            "commander_sigterm_timeout",
            default_value="10",
            description="Sigterm timeout for commander (set high so we have a chance to cleanup)",
        ),
        # Log levels
        DeclareLaunchArgument(
            "commander_log_level",
            default_value="INFO",
            description="Commander log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "moveit_log_level",
            default_value="WARN",
            description="MoveIt log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "rcl_log_level",
            default_value="WARN",
            description="ROS log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        # Sim time
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use simulated time",
        ),
    ]


def save_commander_overrides(context: LaunchContext, path: str):
    commander_overrides = {}

    simulate_commander = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration("robot_mode"), "mock"),
        "true",
        "false",
    )

    # Simulate
    simulate = simulate_commander.perform(context) == "true"
    commander_overrides["simulate"] = simulate

    # Clear cache
    new_cache = LaunchConfiguration("new_cache").perform(context)
    if new_cache != "null":
        commander_overrides[
            "common_manipulation_interface.trajectory_cache.kwargs.new_cache"
        ] = new_cache == "true"

    # Use cache
    use_cache = LaunchConfiguration("use_cache").perform(context)
    if use_cache != "null":
        commander_overrides[
            "common_manipulation_interface.trajectory_cache.use_cached_trajectories"
        ] = use_cache == "true"

    # Use sound
    use_sound = LaunchConfiguration("use_sound").perform(context)
    if use_sound != "null":
        commander_overrides["sound_interface.enable"] = use_sound == "true"

    # Initial attached object
    for prefix in ["left_", "right_"]:
        launch_config_name = f"{prefix}initial_object"
        param_prefix = f"{prefix}manipulation_interface"

        initial_object = LaunchConfiguration(launch_config_name).perform(
            context
        )
        if initial_object != "null":
            idx = initial_object.split(",")
            if len(idx) == 1:
                commander_overrides[
                    f"{param_prefix}.initial_attached_object_id"
                ] = initial_object
            elif len(idx) == 2:
                commander_overrides[
                    f"{param_prefix}.initial_attached_object_idx"
                ] = [
                    int(idx[0]),
                    int(idx[1]),
                ]
            else:
                raise ValueError(
                    f"Invalid initial object index: {initial_object}"
                )

    # Save the scoped overrides
    commander_overrides_scoped = {
        "/commander": {"ros__parameters": commander_overrides}
    }

    with open(path, "w") as f:
        yaml.dump(commander_overrides_scoped, f, sort_keys=False)

    return path


def launch_setup(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)

    param_file = LaunchConfiguration("commander_param_file").perform(context)

    overrides_file = LaunchConfiguration(
        "commander_param_overrides_tmp_path"
    ).perform(context)

    save_commander_overrides(context, overrides_file)

    robot_name = LaunchConfiguration("robot_name").perform(context)
    semantic_description_file = LaunchConfiguration(
        "semantic_description_file"
    ).perform(context)

    # MoveIt Config
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name=robot_name,
            package_name="tabletop_moveit_config",
        )
        .robot_description_semantic(
            file_path=semantic_description_file,
            mappings={"name": robot_name},
        )
        .planning_scene_monitor(
            # publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .moveit_cpp(
            file_path="config/moveit_cpp.yaml",
        )
        .to_moveit_configs()
    )

    # ROS Warehouse Config
    # warehouse_ros_config = {
    #     "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
    #     "warehouse_host": LaunchConfiguration("warehouse_sqlite_path").perform(
    #         context
    #     ),
    # }

    # ROS args
    ros_args: list[str] = []

    logger_level_map = {
        "default": "moveit_log_level",
        "commander": "commander_log_level",
        "trajectory_cache": "commander_log_level",
        "tabletop_task": "commander_log_level",
        "trial_generator": "commander_log_level",
        "rcl": "rcl_log_level",
        "rcl_action": "rcl_log_level",
        "rclcpp": "rcl_log_level",
        "rclcpp_action": "rcl_log_level",
        "pluginlib.ClassLoader": "rcl_log_level",
        "rmw_fastrtps_cpp": "rcl_log_level",
        # "trac_ik_kinematics_plugin": rcl_log_level,
    }
    for logger_name, config_name in logger_level_map.items():
        level = LaunchConfiguration(config_name).perform(context)
        if logger_name == "default":
            arg_value = level
        else:
            arg_value = f"{logger_name}:={level}"
        ros_args.extend(["--log-level", arg_value])

    # CLI Args
    cli_args: list[str] = []

    commander_arg_map = {
        "--coro-module": "coro_module",
        "--coro-name": "coro_name",
        "--coro-config": "coro_config",
        "--debug": "debug_commander",
    }
    for arg_name, config_name in commander_arg_map.items():
        arg_value = LaunchConfiguration(config_name).perform(context)
        if arg_name == "--debug":
            if arg_value == "true":
                cli_args.append(arg_name)
        else:
            if arg_value != "null":
                cli_args.extend([arg_name, arg_value])

    sigterm_timeout = LaunchConfiguration("commander_sigterm_timeout").perform(
        context
    )

    # Commander Node
    commander = Node(
        package="tabletop_rig",
        executable="commander",
        output="both",
        sigterm_timeout=sigterm_timeout,
        parameters=[
            moveit_config.to_dict(),
            # warehouse_ros_config,
            ParameterFile(param_file, allow_substs=True),
            ParameterFile(overrides_file, allow_substs=True),
            {
                "publish_robot_description_semantic": True,
                "use_sim_time": use_sim_time == "true",
            },
        ],
        ros_arguments=[*ros_args],
        arguments=[*cli_args],
        on_exit=[Shutdown()],
    )

    # Wait for robot_description topic to be published
    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="both",
    )
    robot_description_ready_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_robot_description,
            on_exit=[commander],
        )
    )

    return [
        set_ros_log_dir,
        *declare_arguments(),
        wait_robot_description,
        robot_description_ready_handler,
    ]


def generate_launch_description():
    return LaunchDescription(
        [*declare_arguments(), OpaqueFunction(function=launch_setup)]
    )
