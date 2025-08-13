import os
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    IfElseSubstitution,
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetROSLogDir
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from tabletop_utils.common import print_substitutions


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
            "robot_mode",
            default_value="mock",
            choices=["mock", "ursim", "real"],
            description="Whether to use the mock robot, URSim, or real robot",
        ),
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
        ),
        # UR Robot Driver
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="120",
            description="Controller spawner timeout",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Initially loaded robot controller.",
        ),
        # Teensy
        DeclareLaunchArgument(
            "micro_ros_transport",
            default_value="serial",
            description="Micro ROS Agent transport protocol",
        ),
        DeclareLaunchArgument(
            "micro_ros_device",
            default_value="/dev/ttyACM0",
            description="Micro ROS Agent serial device",
        ),
        DeclareLaunchArgument(
            "micro_ros_baudrate",
            default_value="115200",
            description="Micro ROS Agent serial baudrate",
        ),
        DeclareLaunchArgument(
            "micro_ros_verbosity",
            default_value="4",
            description="Micro ROS Agent verbose level",
        ),
        DeclareLaunchArgument(
            "use_mock_teensy",
            default_value="false",
            choices=["true", "false"],
            description="Use mock TeensyBoard",
        ),
        # Flic
        DeclareLaunchArgument(
            "simulate_flic",
            default_value="true",
            choices=["true", "false"],
            description="Simulate Flic",
        ),
        # RViz
        DeclareLaunchArgument(
            "launch_rviz_server",
            default_value="true",
            choices=["true", "false"],
            description="Launch RViz?",
        ),
        DeclareLaunchArgument(
            "rviz_config_file_server",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_server"),
                    "rviz",
                    "server.rviz",
                ]
            ),
            description="RViz config file",
        ),
        # Bag
        DeclareLaunchArgument(
            "rosbag_record",
            default_value="true",
            choices=["true", "false"],
            description="Record rosbag?",
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
            "default_log_level",
            default_value="INFO",
            description="Default log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "teensy_log_level",
            default_value="INFO",
            description="Teensy log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "flic_log_level",
            default_value="INFO",
            description="Flic log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "eyelink_log_level",
            default_value="INFO",
            description="Eyelink log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "rviz_log_level",
            default_value="INFO",
            description="RViz Ogre log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        # Outputs
        DeclareLaunchArgument(
            "mock_dashboard_output",
            default_value="own_log",
            description="Mock Dashboard output",
            choices=["log", "both", "screen", "own_log"],
        ),
        DeclareLaunchArgument(
            "ur_output",
            default_value="own_log",
            description="UR output",
            choices=["log", "both", "screen", "own_log"],
        ),
        DeclareLaunchArgument(
            "commander_output",
            default_value="both",
            description="Commander output",
            choices=["log", "both", "screen", "own_log"],
        ),
        DeclareLaunchArgument(
            "micro_ros_output",
            default_value="own_log",
            description="Micro ROS output",
            choices=["log", "both", "screen", "own_log"],
        ),
        DeclareLaunchArgument(
            "mock_teensy_output",
            default_value="both",
            description="Mock Teensy output",
            choices=["log", "both", "screen", "own_log"],
        ),
        DeclareLaunchArgument(
            "flic_output",
            default_value="both",
            description="Flic output",
            choices=["log", "both", "screen", "own_log"],
        ),
        DeclareLaunchArgument(
            "eyelink_output",
            default_value="both",
            description="Eyelink output",
            choices=["log", "both", "screen", "own_log"],
        ),
        DeclareLaunchArgument(
            "rviz_output",
            default_value="own_log",
            description="RViz output",
            choices=["log", "both", "screen", "own_log"],
        ),
        DeclareLaunchArgument(
            "bag_output",
            default_value="both",
            description="Bag output",
            choices=["log", "both", "screen", "own_log"],
        ),
    ]


def generate_launch_description():
    # Common
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_mode = LaunchConfiguration("robot_mode")
    ur_type = LaunchConfiguration("ur_type")

    # UR Robot Driver
    controller_spawner_timeout = LaunchConfiguration(
        "controller_spawner_timeout"
    )
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")

    # Teensy
    micro_ros_transport = LaunchConfiguration("micro_ros_transport")
    micro_ros_device = LaunchConfiguration("micro_ros_device")
    micro_ros_baudrate = LaunchConfiguration("micro_ros_baudrate")
    micro_ros_verbosity = LaunchConfiguration("micro_ros_verbosity")
    use_mock_teensy = LaunchConfiguration("use_mock_teensy")

    # Flic
    simulate_flic = LaunchConfiguration("simulate_flic")

    # RViz
    launch_rviz_server = LaunchConfiguration("launch_rviz_server")
    rviz_config_file_server = LaunchConfiguration("rviz_config_file_server")

    # Bag
    rosbag_record = LaunchConfiguration("rosbag_record")

    # ROS Warehouse
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")

    # Logging
    default_log_level = LaunchConfiguration("default_log_level")
    teensy_log_level = LaunchConfiguration("teensy_log_level")
    flic_log_level = LaunchConfiguration("flic_log_level")
    eyelink_log_level = LaunchConfiguration("eyelink_log_level")
    rviz_log_level = LaunchConfiguration("rviz_log_level")

    # Outputs
    mock_dashboard_output = LaunchConfiguration("mock_dashboard_output")
    ur_output = LaunchConfiguration("ur_output")
    micro_ros_output = LaunchConfiguration("micro_ros_output")
    mock_teensy_output = LaunchConfiguration("mock_teensy_output")
    flic_output = LaunchConfiguration("flic_output")
    eyelink_output = LaunchConfiguration("eyelink_output")
    rviz_output = LaunchConfiguration("rviz_output")
    bag_output = LaunchConfiguration("bag_output")
    ###########################################################################

    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # Conditional substitutions
    robot_ip = IfElseSubstitution(
        EqualsSubstitution(robot_mode, "real"),
        os.environ["ROBOT_IP"],
        os.environ["SIM_ROBOT_IP"],
    )
    reverse_ip = IfElseSubstitution(
        EqualsSubstitution(robot_mode, "real"),
        os.environ["REVERSE_IP"],
        os.environ["SIM_REVERSE_IP"],
    )
    use_mock_robot = IfElseSubstitution(
        EqualsSubstitution(robot_mode, "mock"), "true", "false"
    )
    kinematics_params_file = PathJoinSubstitution(
        [
            FindPackageShare("tabletop_description"),
            "config",
            IfElseSubstitution(
                EqualsSubstitution(robot_mode, "real"),
                "ur5e_calibration.yaml",
                "ursim_calibration.yaml",
            ),
        ]
    )

    # Create a new bag directory for the session and symlink to it
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    dirname = f"session_{timestamp}"
    session_bag_dir = os.path.join(os.environ["ROS_BAG_DIR"], dirname)
    os.makedirs(session_bag_dir, exist_ok=True)
    try:
        os.remove(os.path.join(os.environ["ROS_BAG_DIR"], "latest"))
    except FileNotFoundError:
        pass
    os.symlink(dirname, os.path.join(os.environ["ROS_BAG_DIR"], "latest"))

    # Print substitutions
    print_substitutions_action = OpaqueFunction(
        function=lambda context: print_substitutions(
            context,
            {
                "robot_ip": robot_ip,
                "reverse_ip": reverse_ip,
                "use_mock_robot": use_mock_robot,
                "kinematics_params_file": kinematics_params_file,
            },
        ),
        condition=IfCondition(EqualsSubstitution(default_log_level, "DEBUG")),
    )

    # UR Robot Driver (use group action to isolate the launch file)
    ur_robot_driver = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=ur_output,
            ),
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
                    "ur_type": ur_type,
                    "robot_ip": robot_ip,
                    "reverse_ip": reverse_ip,
                    "use_mock_hardware": use_mock_robot,
                    "controller_spawner_timeout": controller_spawner_timeout,
                    "initial_joint_controller": initial_joint_controller,
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
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
    )

    # Mock Dashboard
    mock_dashboard = Node(
        package="tabletop_server",
        executable="mock_dashboard",
        output=mock_dashboard_output,
        parameters=[{"use_sim_time": use_sim_time}],
        ros_arguments=["--log-level", default_log_level],
        condition=IfCondition(use_mock_robot),
        on_exit=[Shutdown()],
    )

    # Micro ROS Agent
    micro_ros_agent = Node(
        package="micro_ros_agent",
        name="micro_ros_agent",
        executable="micro_ros_agent",
        output=micro_ros_output,
        parameters=[{"use_sim_time": use_sim_time}],
        ros_arguments=["--log-level", teensy_log_level],
        arguments=[
            micro_ros_transport,
            "--dev",
            micro_ros_device,
            "--baudrate",
            micro_ros_baudrate,
            "--verbose",
            micro_ros_verbosity,
        ],
        condition=UnlessCondition(use_mock_teensy),
        on_exit=[Shutdown()],
    )

    # Mock Teensy
    mock_teensy = Node(
        name="teensy",
        package="tabletop_server",
        executable="mock_teensy",
        output=mock_teensy_output,
        parameters=[{"use_sim_time": use_sim_time}],
        ros_arguments=["--log-level", teensy_log_level],
        condition=IfCondition(use_mock_teensy),
        on_exit=[Shutdown()],
    )

    # Flic
    flic = Node(
        name="flic",
        package="tabletop_server",
        executable="flic",
        output=flic_output,
        parameters=[
            {"use_sim_time": use_sim_time, "simulate": simulate_flic},
        ],
        ros_arguments=["--log-level", flic_log_level],
        on_exit=[Shutdown()],
    )

    # Eyelink
    eyelink = Node(
        name="eyelink",
        package="tabletop_server",
        executable="eyelink",
        output=eyelink_output,
        parameters=[
            {"use_sim_time": use_sim_time, "session_bag_dir": session_bag_dir},
        ],
        ros_arguments=["--log-level", eyelink_log_level],
    )

    # RViz MoveIt Config
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur5e", package_name="tabletop_moveit_config"
        )
        .robot_description_semantic(
            file_path="srdf/tabletop.srdf.xacro", mappings={"name": ur_type}
        )
        .moveit_cpp(
            file_path="config/moveit_cpp.yaml",
        )
        .to_moveit_configs()
    )
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # RViz
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz_server),
        executable="rviz2",
        output=rviz_output,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
        arguments=["-d", rviz_config_file_server, "-l"],  # -l for ogre log
        ros_arguments=["--log-level", rviz_log_level],
    )

    # Bag
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
    bag = ExecuteProcess(
        name="rosbag",
        cmd=["ros2", "bag", "record", "-o", server_bag_dir, *args],
        output=bag_output,
        condition=IfCondition(rosbag_record),
    )

    launch_actions = [
        set_ros_log_dir,
        print_substitutions_action,
        ur_robot_driver,
        mock_dashboard,
        micro_ros_agent,
        mock_teensy,
        flic,
        eyelink,
        rviz_node,
        bag,
    ]

    return LaunchDescription(declare_arguments() + launch_actions)


# def main():
#     launch_logging_config.level = "DEBUG"
#     ls = LaunchService()
#     ld = generate_launch_description()
#     ls.include_launch_description(ld)
#     return ls.run()


# if __name__ == "__main__":
#     main()
