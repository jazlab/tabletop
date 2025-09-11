import os

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
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


def save_yaml(file_path, data, sort_keys=False):
    with open(file_path, "w") as file:
        yaml.dump(data, file, sort_keys=sort_keys)


def declare_arguments():
    return [
        # Common
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Using or not time from simulation",
        ),
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
        ),
        DeclareLaunchArgument(
            "robot_mode",
            default_value="mock",
            choices=["mock", "ursim", "real"],
            description="Whether to use the mock robot, URSim, or real robot",
        ),
        # Commander
        DeclareLaunchArgument(
            "commander_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_server"),
                    "config",
                    "commander.yaml",
                ]
            ),
            description="Commander config file",
        ),
        DeclareLaunchArgument(
            "coroutine_module",
            default_value="null",
            description="Coroutine module",
        ),
        DeclareLaunchArgument(
            "coroutine_name",
            default_value="null",
            description="Coroutine name",
        ),
        DeclareLaunchArgument(
            "coroutine_config",
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
            "initial_object",
            default_value="null",
            description="The name or index of the initial attached object",
        ),
        DeclareLaunchArgument(
            "use_sound",
            default_value="null",
            description="Whether to enable sound from the commander",
            choices=["true", "false", "null"],
        ),
        DeclareLaunchArgument(
            "optimize_python",
            default_value="false",
            description="Whether to optimize the Python code for the "
            "commander with the PYTHONOPTIMIZE environment variable",
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
        DeclareLaunchArgument(
            "commander_output",
            default_value="both",
            description="Commander output",
            choices=["log", "both", "screen", "own_log"],
        ),
        DeclareLaunchArgument(
            "debug_commander",
            default_value="false",
            description="Whether to debug the commander",
            choices=["true", "false"],
        ),
    ]


def generate_launch_description():
    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # Conditional substitutions
    simulate_commander = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration("robot_mode"), "mock"),
        "true",
        "false",
    )

    commander_overrides_path = "/tmp/commander_overrides.yaml"

    def save_commander_overrides(context):
        commander_overrides = {}

        # Simulate
        simulate = simulate_commander.perform(context) == "true"
        commander_overrides["simulate"] = simulate

        # Velocity and acceleration scaling for simulation
        if simulate:
            commander_overrides["execution.velocity_scaling_factor"] = 0.5
            commander_overrides["execution.acceleration_scaling_factor"] = 0.5

        # Clear cache
        new_cache_value = LaunchConfiguration("new_cache").perform(context)
        if new_cache_value != "null":
            commander_overrides["trajectory_cache.kwargs.new_cache"] = (
                new_cache_value == "true"
            )

        # Use cache
        use_cache_value = LaunchConfiguration("use_cache").perform(context)
        if use_cache_value != "null":
            commander_overrides["trajectory_cache.use_cached_trajectories"] = (
                use_cache_value == "true"
            )

        # Use sound
        use_sound_value = LaunchConfiguration("use_sound").perform(context)
        if use_sound_value != "null":
            commander_overrides["sound.enable"] = use_sound_value == "true"

        # Initial attached object
        initial_object_value = LaunchConfiguration("initial_object").perform(
            context
        )
        if initial_object_value != "null":
            idx = initial_object_value.split(",")
            if len(idx) == 1:
                commander_overrides["initial_attached_object"] = (
                    initial_object_value
                )
            elif len(idx) == 2:
                commander_overrides["initial_attached_object_idx"] = [
                    int(idx[0]),
                    int(idx[1]),
                ]
            else:
                raise ValueError(
                    f"Invalid initial object index: {initial_object_value}"
                )

        # Save the scoped overrides
        commander_overrides_scoped = {
            "/commander": {"ros__parameters": commander_overrides}
        }

        if (
            LaunchConfiguration("commander_log_level").perform(context)
            == "DEBUG"
        ):
            print(commander_overrides_scoped)
        save_yaml(commander_overrides_path, commander_overrides_scoped)

    commander_overrides_action = OpaqueFunction(
        function=save_commander_overrides,
    )

    # MoveIt Config
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur", package_name="tabletop_moveit_config"
        )
        .robot_description_semantic(
            file_path="srdf/tabletop.srdf.xacro",
            mappings={"name": LaunchConfiguration("ur_type")},
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

    # Python Optimize
    optimize_python_action = SetEnvironmentVariable(
        name="PYTHONOPTIMIZE",
        value="1",
        condition=IfCondition(LaunchConfiguration("optimize_python")),
    )

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
        package="tabletop_server",
        executable="commander",
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
                    LaunchConfiguration("coroutine_module"), "null"
                ),
                if_value="--coroutine-module",
                else_value="",
            ),
            LaunchConfiguration("coroutine_module"),
            IfElseSubstitution(
                NotEqualsSubstitution(
                    LaunchConfiguration("coroutine_name"), "null"
                ),
                if_value="--coroutine-name",
                else_value="",
            ),
            LaunchConfiguration("coroutine_name"),
            IfElseSubstitution(
                NotEqualsSubstitution(
                    LaunchConfiguration("coroutine_config"), "null"
                ),
                if_value="--coroutine-config",
                else_value="",
            ),
            LaunchConfiguration("coroutine_config"),
            IfElseSubstitution(
                EqualsSubstitution(
                    LaunchConfiguration("debug_commander"), "true"
                ),
                if_value="--debug",
                else_value="",
            ),
        ],
        output=LaunchConfiguration("commander_output"),
        on_exit=[Shutdown()],
    )

    launch_actions = [
        set_ros_log_dir,
        commander_overrides_action,
        optimize_python_action,
        commander,
    ]

    return LaunchDescription(declare_arguments() + launch_actions)
