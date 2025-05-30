import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
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
from tabletop_utils.common import print_substitutions


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
        # ROS Warehouse
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value="/root/ws/src/tabletop/ros/warehouse_ros.sqlite",
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
            "moveit_py_log_level",
            default_value="WARN",
            description="MoveItPy log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "moveit_log_level",
            default_value="FATAL",
            description="MoveIt log level",
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
    # Common
    use_sim_time = LaunchConfiguration("use_sim_time")
    ur_type = LaunchConfiguration("ur_type")

    # Commander
    robot_mode = LaunchConfiguration("robot_mode")
    commander_config = LaunchConfiguration("commander_config")
    coroutine_module = LaunchConfiguration("coroutine_module")
    coroutine_name = LaunchConfiguration("coroutine_name")
    coroutine_config = LaunchConfiguration("coroutine_config")

    # ROS Warehouse
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")

    # Logging
    commander_log_level = LaunchConfiguration("commander_log_level")
    moveit_py_log_level = LaunchConfiguration("moveit_py_log_level")
    moveit_log_level = LaunchConfiguration("moveit_log_level")

    # Commander output
    commander_output = LaunchConfiguration("commander_output")

    # Debugger
    debug_commander = LaunchConfiguration("debug_commander")
    ################################################

    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # Conditional substitutions
    simulate_commander = IfElseSubstitution(
        EqualsSubstitution(robot_mode, "real"), "false", "true"
    )

    # Print substitutions
    print_substitutions_action = OpaqueFunction(
        function=lambda context: print_substitutions(
            context,
            {
                "simulate_commander": simulate_commander,
            },
        ),
        condition=IfCondition(
            EqualsSubstitution(commander_log_level, "DEBUG")
        ),
    )

    commander_overrides_path = "/tmp/commander_overrides.yaml"

    def save_commander_overrides(context):
        simulate = simulate_commander.perform(context).lower() == "true"
        commander_overrides = {
            "simulate": simulate,
        }
        commander_overrides_scoped = {
            "/commander": {"ros__parameters": commander_overrides}
        }
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
            file_path="srdf/tabletop.srdf.xacro", mappings={"name": ur_type}
        )
        .moveit_cpp(
            file_path="config/moveit_cpp.yaml",
        )
        .to_moveit_configs()
    )

    # ROS Warehouse Config
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # Commander Node
    commander = Node(
        package="tabletop_server",
        executable="commander",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            ParameterFile(commander_config, allow_substs=True),
            ParameterFile(commander_overrides_path, allow_substs=True),
            {
                "publish_robot_description_semantic": True,
                "use_sim_time": use_sim_time,
            },
        ],
        ros_arguments=[
            "--log-level",
            moveit_log_level,
            "--log-level",
            ["moveit_py:=", moveit_py_log_level],
            "--log-level",
            ["commander:=", commander_log_level],
        ],
        arguments=[
            IfElseSubstitution(
                NotEqualsSubstitution(coroutine_module, "null"),
                if_value="--coroutine-module",
                else_value="",
            ),
            coroutine_module,
            IfElseSubstitution(
                NotEqualsSubstitution(coroutine_name, "null"),
                if_value="--coroutine-name",
                else_value="",
            ),
            coroutine_name,
            IfElseSubstitution(
                NotEqualsSubstitution(coroutine_config, "null"),
                if_value="--coroutine-config",
                else_value="",
            ),
            coroutine_config,
            IfElseSubstitution(
                EqualsSubstitution(debug_commander, "true"),
                if_value="--debug",
                else_value="",
            ),
        ],
        output=commander_output,
        on_exit=[Shutdown()],
    )

    launch_actions = [
        set_ros_log_dir,
        print_substitutions_action,
        commander_overrides_action,
        commander,
    ]

    return LaunchDescription(declare_arguments() + launch_actions)
