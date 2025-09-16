import math
import os
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
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
from launch.substitution import Substitution
from launch.substitutions import (
    EqualsSubstitution,
    IfElseSubstitution,
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, Node, SetROSLogDir
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

# TODO: Don't shutdown on exit, try to restart the node instead


def print_substitutions(context, substitutions: dict[str, Substitution]):
    for name, substitution in substitutions.items():
        print(f"{name}: {substitution.perform(context)}")


FLIR_COMMON_PARAMS = {
    "debug": False,
    "compute_brightness": True,
    "dump_node_map": False,
    "adjust_timestamp": True,
    "gain_auto": "Off",
    "gain": 0,
    "exposure_auto": "Off",
    "exposure_time": 9000,
    "line2_selector": "Line2",
    "line2_v33enable": False,
    "line3_selector": "Line3",
    "line3_linemode": "Input",
    "trigger_selector": "FrameStart",
    "trigger_mode": "On",
    "trigger_source": "Line3",
    "trigger_delay": 9,
    "trigger_overlap": "ReadOut",
    "chunk_mode_active": True,
    "chunk_selector_frame_id": "FrameID",
    "chunk_enable_frame_id": True,
    "chunk_selector_exposure_time": "ExposureTime",
    "chunk_enable_exposure_time": True,
    "chunk_selector_gain": "Gain",
    "chunk_enable_gain": True,
    "chunk_selector_timestamp": "Timestamp",
    "chunk_enable_timestamp": True,
}


def make_camera_node(name, camera_type, serial_number):
    parameter_file = PathJoinSubstitution(
        [
            FindPackageShare("spinnaker_camera_driver"),
            "config",
            camera_type + ".yaml",
        ]
    )

    node = ComposableNode(
        package="spinnaker_camera_driver",
        plugin="spinnaker_camera_driver::CameraDriver",
        name=name,
        parameters=[
            FLIR_COMMON_PARAMS,
            {"parameter_file": parameter_file, "serial_number": serial_number},
        ],
        remappings=[
            ("~/control", "/exposure_control/control"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    return node


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
        condition=IfCondition(
            EqualsSubstitution(
                LaunchConfiguration("default_log_level"), "DEBUG"
            )
        ),
    )

    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

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
        condition=IfCondition(LaunchConfiguration("rosbag_record")),
    )

    # UR Robot Driver (use group action to isolate the launch file)
    ur_robot_driver = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=LaunchConfiguration("ur_output"),
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
                    "ur_type": LaunchConfiguration("ur_type"),
                    "robot_ip": robot_ip,
                    "reverse_ip": reverse_ip,
                    "use_mock_hardware": use_mock_robot,
                    "controller_spawner_timeout": LaunchConfiguration(
                        "controller_spawner_timeout"
                    ),
                    "initial_joint_controller": LaunchConfiguration(
                        "initial_joint_controller"
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
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
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
        output=LaunchConfiguration("mock_dashboard_output"),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("default_log_level"),
        ],
        condition=IfCondition(use_mock_robot),
        on_exit=[Shutdown()],
    )

    # Micro ROS Agent
    micro_ros_agent = Node(
        package="micro_ros_agent",
        name="micro_ros_agent",
        executable="micro_ros_agent",
        output=LaunchConfiguration("micro_ros_output"),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ros_arguments=["--log-level", LaunchConfiguration("teensy_log_level")],
        arguments=[
            LaunchConfiguration("micro_ros_transport"),
            "--dev",
            LaunchConfiguration("micro_ros_device"),
            "--baudrate",
            LaunchConfiguration("micro_ros_baudrate"),
            "--verbose",
            LaunchConfiguration("micro_ros_verbosity"),
        ],
        condition=UnlessCondition(LaunchConfiguration("use_mock_teensy")),
        on_exit=[Shutdown()],
    )

    # Mock Teensy
    mock_teensy = Node(
        name="teensy",
        package="tabletop_server",
        executable="mock_teensy",
        output=LaunchConfiguration("mock_teensy_output"),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ros_arguments=["--log-level", LaunchConfiguration("teensy_log_level")],
        condition=IfCondition(LaunchConfiguration("use_mock_teensy")),
        on_exit=[Shutdown()],
    )

    # Flic
    flic = Node(
        name="flic",
        package="tabletop_server",
        executable="flic",
        output=LaunchConfiguration("flic_output"),
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "simulate": LaunchConfiguration("simulate_flic"),
            },
        ],
        ros_arguments=["--log-level", LaunchConfiguration("flic_log_level")],
        on_exit=[Shutdown()],
    )

    # Eyelink
    eyelink = Node(
        name="eyelink",
        package="tabletop_server",
        executable="eyelink",
        output=LaunchConfiguration("eyelink_output"),
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "session_bag_dir": ParameterValue(
                    IfElseSubstitution(
                        LaunchConfiguration("rosbag_record"),
                        session_bag_dir,
                        "null",
                    ),
                    value_type=str,
                ),
            },
        ],
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("eyelink_log_level"),
        ],
        on_exit=[Shutdown()],
    )

    # Static transform publisher for optitrack
    optitrack_transform_publisher = Node(
        name="optitrack_static_transform_publisher",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.4925",
            "--y",
            "0.6025",
            "--z",
            "0.31",
            "--yaw",
            str(math.pi / 2),
            "--pitch",
            "0",
            "--roll",
            str(math.pi / 2),
            "--frame-id",
            "world",
            "--child-frame-id",
            "optitrack",
        ],
        output="both",
        on_exit=[Shutdown()],
    )

    # Rviz visualizer for optitrack markers
    mocap4r2_marker_viz = Node(
        package="mocap4r2_marker_viz",
        executable="mocap4r2_marker_viz",
        output="both",
        emulate_tty=True,
        parameters=[
            {
                "mocap4r2_system": "optitrack",
                "marker_topics": ["markers", "predicted_markers"],
                "rb_topics": ["rigid_bodies"],
            }
        ],
        on_exit=[Shutdown()],
    )

    # Flir multi-camera setup
    flir_config_file = os.path.join(
        get_package_share_directory("tabletop_server"), "config", "flir.yaml"
    )
    with open(flir_config_file, "r") as f:
        flir_config = yaml.safe_load(f)

    flir_nodes = []
    for config in flir_config["cameras"]:
        config = flir_config["common"] | config
        print(config)
        flir_nodes.append(make_camera_node(**config))
    flir_camera_container = ComposableNodeContainer(
        name="flir_camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=flir_nodes,
        output="both",
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
    rviz_node = Node(
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
            LaunchConfiguration("rviz_config_file_server"),
            "-l",
        ],  # -l for ogre log
        cwd=LaunchLogDir(),
        ros_arguments=["--log-level", LaunchConfiguration("rviz_log_level")],
        condition=IfCondition(LaunchConfiguration("launch_rviz_server")),
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
        output=LaunchConfiguration("bag_output"),
        on_exit=[Shutdown()],
    )
    bag_recorder = ExecuteProcess(
        name="rosbag_recorder",
        cmd=["ros2", "bag", "record", "-o", server_bag_dir, *args],
        output=LaunchConfiguration("bag_output"),
        condition=IfCondition(LaunchConfiguration("rosbag_record")),
        on_exit=[bag_converter],
    )

    launch_actions = [
        *declare_arguments(),
        print_substitutions_action,
        set_ros_log_dir,
        create_session_bag_dir,
        ur_robot_driver,
        mock_dashboard,
        micro_ros_agent,
        mock_teensy,
        flic,
        eyelink,
        optitrack_transform_publisher,
        mocap4r2_marker_viz,
        flir_camera_container,
        rviz_node,
        bag_recorder,
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
