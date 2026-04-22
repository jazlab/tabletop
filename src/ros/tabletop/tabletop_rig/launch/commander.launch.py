import os

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnExecutionComplete, OnProcessExit
from launch.substitutions import (
    EqualsSubstitution,
    IfElseSubstitution,
    LaunchConfiguration,
    LaunchLogDir,
    NotEqualsSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetROSLogDir
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


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
            "commander_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_rig"),
                    "config",
                    "commander.yaml",
                ]
            ),
            description="Commander config file",
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


def save_commander_overrides_fn(context, path: str):
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

    if LaunchConfiguration("commander_log_level").perform(context) == "DEBUG":
        print(commander_overrides_scoped)

    with open(path, "w") as f:
        yaml.dump(commander_overrides_scoped, f, sort_keys=False)


def generate_launch_description():
    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # Wait for robot_description topic to be published
    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="both",
    )

    commander_overrides_path = "/tmp/commander_overrides.yaml"

    save_commander_overrides = OpaqueFunction(
        function=save_commander_overrides_fn, args=[commander_overrides_path]
    )

    # MoveIt Config
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="tabletop", package_name="tabletop_moveit_config"
        )
        .robot_description_semantic(
            file_path="srdf/dual_tabletop.srdf.xacro",
            mappings={"name": LaunchConfiguration("robot_name")},
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
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": LaunchConfiguration("warehouse_sqlite_path"),
    }

    # Log levels
    logger_levels = [
        LaunchConfiguration("moveit_log_level"),
        ["commander:=", LaunchConfiguration("commander_log_level")],
        ["trajectory_cache:=", LaunchConfiguration("commander_log_level")],
        ["tabletop_task:=", LaunchConfiguration("commander_log_level")],
        ["trial_generator:=", LaunchConfiguration("commander_log_level")],
        ["rcl:=", LaunchConfiguration("rcl_log_level")],
        ["rcl_action:=", LaunchConfiguration("rcl_log_level")],
        ["rclcpp:=", LaunchConfiguration("rcl_log_level")],
        ["rclcpp_action:=", LaunchConfiguration("rcl_log_level")],
        ["pluginlib.ClassLoader:=", LaunchConfiguration("rcl_log_level")],
        ["rmw_fastrtps_cpp:=", LaunchConfiguration("rcl_log_level")],
        # ["trac_ik_kinematics_plugin:=", rcl_log_level],
    ]
    logger_levels_args = []
    for logger in logger_levels:
        logger_levels_args.extend(["--log-level", logger])

    # Commander Node
    commander = Node(
        package="tabletop_rig",
        executable="commander",
        output="both",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            ParameterFile(
                LaunchConfiguration("commander_config"), allow_substs=True
            ),
            ParameterFile(commander_overrides_path, allow_substs=True),
            {
                "publish_robot_description_semantic": True,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        ros_arguments=[*logger_levels_args],
        arguments=[
            IfElseSubstitution(
                NotEqualsSubstitution(
                    LaunchConfiguration("coro_module"), "null"
                ),
                if_value="--coro-module",
                else_value="",
            ),
            LaunchConfiguration("coro_module"),
            IfElseSubstitution(
                NotEqualsSubstitution(
                    LaunchConfiguration("coro_name"), "null"
                ),
                if_value="--coro-name",
                else_value="",
            ),
            LaunchConfiguration("coro_name"),
            IfElseSubstitution(
                NotEqualsSubstitution(
                    LaunchConfiguration("coro_config"), "null"
                ),
                if_value="--coro-config",
                else_value="",
            ),
            LaunchConfiguration("coro_config"),
            IfElseSubstitution(
                EqualsSubstitution(
                    LaunchConfiguration("debug_commander"), "true"
                ),
                if_value="--debug",
                else_value="",
            ),
        ],
        on_exit=[Shutdown()],
    )

    robot_description_ready_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_robot_description,
            on_exit=[save_commander_overrides],
        )
    )
    save_commander_overrides_handler = RegisterEventHandler(
        OnExecutionComplete(
            target_action=save_commander_overrides,
            on_completion=[commander],
        )
    )

    return LaunchDescription(
        [
            set_ros_log_dir,
            *declare_arguments(),
            # save_commander_overrides,
            wait_robot_description,
            robot_description_ready_handler,
            save_commander_overrides_handler,
        ]
    )
