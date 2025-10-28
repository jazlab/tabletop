import logging
import os
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch.substitution import Substitution
from launch.substitutions import (
    AndSubstitution,
    EqualsSubstitution,
    IfElseSubstitution,
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetROSLogDir
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def declare_arguments():
    return [
        # Common
        DeclareLaunchArgument(
            "robot_mode",
            default_value="mock",
            choices=["mock", "ursim", "real"],
            description="Whether to use the mock robot, URSim, or real robot",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Using or not time from simulation",
        ),
        # UR Driver
        DeclareLaunchArgument(
            "ur_launch",
            default_value="true",
            choices=["true", "false"],
            description="Launch UR Driver?",
        ),
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of UR robot.",
        ),
        DeclareLaunchArgument(
            "ur_controller_spawner_timeout",
            default_value="120",
            description="Controller spawner timeout",
        ),
        DeclareLaunchArgument(
            "ur_initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Initially loaded robot controller.",
        ),
        DeclareLaunchArgument(
            "ur_output",
            default_value="own_log",
            description="UR output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # Mock Dashboard
        DeclareLaunchArgument(
            "mock_dashboard_output",
            default_value="own_log",
            description="Mock Dashboard output",
            choices=["log", "both", "screen", "own_log"],
        ),
        DeclareLaunchArgument(
            "mock_dashboard_log_level",
            default_value="INFO",
            description="Mock Dashboard log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        # Teensy
        DeclareLaunchArgument(
            "teensy_launch",
            default_value="false",
            choices=["true", "false"],
            description="Launch Teensy?",
        ),
        DeclareLaunchArgument(
            "teensy_simulate",
            default_value="false",
            choices=["true", "false"],
            description="Use simulated teensy node",
        ),
        DeclareLaunchArgument(
            "teensy_log_level",
            default_value="INFO",
            description="Teensy log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "teensy_output",
            default_value="both",
            description="Teensy output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # Flic
        DeclareLaunchArgument(
            "flic_launch",
            default_value="false",
            choices=["true", "false"],
            description="Launch Flic?",
        ),
        DeclareLaunchArgument(
            "flic_simulate",
            default_value="false",
            choices=["true", "false"],
            description="Simulate flic button presses",
        ),
        DeclareLaunchArgument(
            "flic_log_level",
            default_value="INFO",
            description="Flic log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "flic_output",
            default_value="both",
            description="Flic output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # Flir
        DeclareLaunchArgument(
            "flir_launch",
            default_value="false",
            choices=["true", "false"],
            description="Launch Flir?",
        ),
        DeclareLaunchArgument(
            "flir_log_level",
            default_value="INFO",
            description="Flir log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "flir_output",
            default_value="both",
            description="Flir output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # Optitrack
        DeclareLaunchArgument(
            "optitrack_launch",
            default_value="false",
            choices=["true", "false"],
            description="Launch Optitrack?",
        ),
        DeclareLaunchArgument(
            "optitrack_log_level",
            default_value="INFO",
            description="Optitrack log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "optitrack_output",
            default_value="both",
            description="Optitrack output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # Eyelink
        DeclareLaunchArgument(
            "eyelink_launch",
            default_value="true",
            choices=["true", "false"],
            description="Launch Eyelink?",
        ),
        DeclareLaunchArgument(
            "eyelink_simulate",
            default_value="false",
            choices=["true", "false"],
            description="Force simulation of eyelink, even if Eyelink SDK is available",
        ),
        DeclareLaunchArgument(
            "eyelink_log_level",
            default_value="INFO",
            description="Eyelink log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "eyelink_output",
            default_value="both",
            description="Eyelink output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # RViz
        DeclareLaunchArgument(
            "rviz_launch",
            default_value="true",
            choices=["true", "false"],
            description="Launch RViz?",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_server"),
                    "rviz",
                    "server.rviz",
                ]
            ),
            description="RViz config file",
        ),
        DeclareLaunchArgument(
            "rviz_log_level",
            default_value="INFO",
            description="RViz Ogre log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "rviz_output",
            default_value="own_log",
            description="RViz output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # Bag
        DeclareLaunchArgument(
            "rosbag",
            default_value="true",
            choices=["true", "false"],
            description="Record rosbag?",
        ),
        DeclareLaunchArgument(
            "rosbag_output",
            default_value="both",
            description="Bag output",
            choices=["log", "both", "screen", "own_log"],
        ),
        # ROS Warehouse
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.join(
                os.environ["TABLETOP_DIR"], "cache", "warehouse_ros.sqlite"
            ),
            description="Path where the warehouse database should be stored",
        ),
    ]


def print_substitutions(context, substitutions: dict[str, Substitution]):
    for name, substitution in substitutions.items():
        print(f"{name}: {substitution.perform(context)}")


def generate_launch_description():
    # Set ROS Log Directory and use_sim_time parameter for all nodes
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # Conditional substitutions
    robot_ip = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration("robot_mode"), "real"),
        os.environ["ROBOT_IP"],
        os.environ["SIM_ROBOT_IP"],
    )
    reverse_ip = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration("robot_mode"), "real"),
        os.environ["REVERSE_IP"],
        os.environ["SIM_REVERSE_IP"],
    )
    use_mock_robot = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration("robot_mode"), "mock"),
        "true",
        "false",
    )
    kinematics_params_file = PathJoinSubstitution(
        [
            FindPackageShare("tabletop_description"),
            "config",
            IfElseSubstitution(
                EqualsSubstitution(LaunchConfiguration("robot_mode"), "real"),
                "ur5e_calibration.yaml",
                "ursim_calibration.yaml",
            ),
        ]
    )

    # Print substitutions
    print_substitutions_action = OpaqueFunction(
        function=print_substitutions,
        args=[
            {
                "robot_ip": robot_ip,
                "reverse_ip": reverse_ip,
                "use_mock_robot": use_mock_robot,
                "kinematics_params_file": kinematics_params_file,
            }
        ],
        condition=IfCondition(str(launch_config.level == logging.DEBUG)),
    )

    # Create a new bag directory for the session and symlink to it
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    dirname = f"session_{timestamp}"
    session_bag_dir = os.path.join(os.environ["ROS_BAG_DIR"], dirname)
    symlink_path = os.path.join(os.environ["ROS_BAG_DIR"], "latest")

    def _create_session_bag_dir(_: LaunchContext):
        os.makedirs(session_bag_dir, exist_ok=True)
        try:
            os.remove(symlink_path)
        except FileNotFoundError:
            pass
        os.symlink(dirname, symlink_path)

    create_session_bag_dir = OpaqueFunction(
        function=_create_session_bag_dir,
        condition=IfCondition(LaunchConfiguration("rosbag")),
    )

    # UR Robot Driver (use group action to isolate the launch file)
    ur_robot_driver = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=LaunchConfiguration("ur_output"),
            ),
            # SetUseSimTime(LaunchConfiguration("use_sim_time")),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ur_robot_driver"),
                                "launch",
                                "ur_control.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "ur_type": LaunchConfiguration("ur_type"),
                    "robot_ip": robot_ip,
                    "reverse_ip": reverse_ip,
                    "use_mock_hardware": use_mock_robot,
                    "controller_spawner_timeout": LaunchConfiguration(
                        "ur_controller_spawner_timeout"
                    ),
                    "initial_joint_controller": LaunchConfiguration(
                        "ur_initial_joint_controller"
                    ),
                    "launch_rviz": "false",
                    "kinematics_params_file": kinematics_params_file,
                    "description_launchfile": PathJoinSubstitution(
                        [
                            FindPackageShare("tabletop_description"),
                            "launch",
                            "rsp.launch.py",
                        ]
                    ),
                    "description_file": PathJoinSubstitution(
                        [
                            FindPackageShare("tabletop_description"),
                            "urdf",
                            "tabletop.urdf.xacro",
                        ]
                    ),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
        condition=IfCondition(LaunchConfiguration("ur_launch")),
    )

    # Mock Dashboard
    mock_dashboard = Node(
        package="tabletop_server",
        executable="mock_dashboard",
        output=LaunchConfiguration("mock_dashboard_output"),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("mock_dashboard_log_level"),
        ],
        condition=IfCondition(
            AndSubstitution(use_mock_robot, LaunchConfiguration("ur_launch"))
        ),
        on_exit=[Shutdown()],
    )

    teensy = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=LaunchConfiguration("teensy_output"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_server"),
                                "launch",
                                "teensy.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "simulate": LaunchConfiguration("teensy_simulate"),
                    "log_level": LaunchConfiguration("teensy_log_level"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
        condition=IfCondition(LaunchConfiguration("teensy_launch")),
    )

    flic = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=LaunchConfiguration("flic_output"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_server"),
                                "launch",
                                "flic.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "simulate": LaunchConfiguration("flic_simulate"),
                    "log_level": LaunchConfiguration("flic_log_level"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
        condition=IfCondition(LaunchConfiguration("flic_launch")),
    )

    optitrack = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=LaunchConfiguration("optitrack_output"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_server"),
                                "launch",
                                "optitrack.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "simulate": LaunchConfiguration("optitrack_simulate"),
                    "log_level": LaunchConfiguration("optitrack_log_level"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
        condition=IfCondition(LaunchConfiguration("optitrack_launch")),
    )

    # Flir (use group action to isolate the launch file)
    flir = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=LaunchConfiguration("flir_output"),
            ),
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
                    "camera": "all",
                    "log_level": LaunchConfiguration("flir_log_level"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
        condition=IfCondition(LaunchConfiguration("flir_launch")),
    )

    # Eyelink
    eyelink = Node(
        name="eyelink",
        package="tabletop_server",
        executable="eyelink",
        output=LaunchConfiguration("eyelink_output"),
        parameters=[
            {
                "simulate": LaunchConfiguration("eyelink_simulate"),
                "session_bag_dir": ParameterValue(
                    IfElseSubstitution(
                        LaunchConfiguration("rosbag"),
                        session_bag_dir,
                        "null",
                    ),
                    value_type=str,
                ),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("eyelink_log_level"),
        ],
        condition=IfCondition(LaunchConfiguration("eyelink_launch")),
        on_exit=[Shutdown()],
    )

    # RViz MoveIt Config
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur5e", package_name="tabletop_moveit_config"
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
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": LaunchConfiguration("warehouse_sqlite_path"),
    }

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output=LaunchConfiguration("rviz_output"),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=[
            "-d",
            LaunchConfiguration("rviz_config_file"),
            "-l",
        ],  # -l for ogre log
        cwd=LaunchLogDir(),
        ros_arguments=["--log-level", LaunchConfiguration("rviz_log_level")],
        condition=IfCondition(LaunchConfiguration("rviz_launch")),
        on_exit=[Shutdown()],
    )

    # Bag Recorder and Converter
    interfaces_config_file = os.path.join(
        get_package_share_directory("tabletop_server"),
        "config",
        "rosbag_interfaces.yaml",
    )
    with open(interfaces_config_file, "r") as f:
        interfaces_config = yaml.safe_load(f)
    args = []
    if interfaces_config["all"]:
        args.append("--all")
    else:
        if "topics" in interfaces_config:
            args.extend(["--topics", *interfaces_config["topics"]])
        if "services" in interfaces_config:
            args.extend(["--services", *interfaces_config["services"]])
    server_bag_dir = os.path.join(session_bag_dir, "server")

    bag_recorder = ExecuteProcess(
        name="rosbag_recorder",
        cmd=["ros2", "bag", "record", "-o", server_bag_dir, *args],
        output=LaunchConfiguration("rosbag_output"),
        condition=IfCondition(LaunchConfiguration("rosbag")),
        on_exit=[Shutdown()],
    )
    bag_converter = ExecuteProcess(
        name="bag_converter",
        cmd=[
            "ros2",
            "run",
            "tabletop_server",
            "rosbag_to_csv",
            "-d",
            session_bag_dir,
        ],
        shell=True,
        output=LaunchConfiguration("rosbag_output"),
    )
    bag_converter_handler = RegisterEventHandler(
        OnShutdown(on_shutdown=[bag_converter], handle_once=True),
        condition=IfCondition(LaunchConfiguration("rosbag")),
    )

    launch_actions = [
        *declare_arguments(),
        set_ros_log_dir,
        print_substitutions_action,
        create_session_bag_dir,
        ur_robot_driver,
        mock_dashboard,
        flic,
        teensy,
        optitrack,
        eyelink,
        flir,
        rviz,
        bag_recorder,
        bag_converter_handler,
    ]

    return LaunchDescription(launch_actions)


def main():
    launch_config.log_dir = os.path.join(os.environ["ROS_LOG_DIR"], "server")
    launch_config.level = logging.DEBUG
    ls = LaunchService()
    ld = generate_launch_description()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    main()
