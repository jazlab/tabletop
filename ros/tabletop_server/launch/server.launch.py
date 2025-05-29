from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
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
        # UR Robot Driver
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="120",
            description="Controller spawner timeout",
        ),
        DeclareLaunchArgument(
            "description_launchfile",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_description"),
                    "launch",
                    "rsp.launch.py",
                ]
            ),
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_description"),
                    "urdf",
                    "tabletop_control.urdf.xacro",
                ]
            ),
            description="URDF/XACRO description file with the robot.",
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
            default_value="true",
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
            default_value="false",
            choices=["true", "false"],
            description="Record rosbag?",
        ),
        DeclareLaunchArgument(
            "rosbag_args",
            default_value="--all",
            description="'ros2 bag' command line args",
        ),
        DeclareLaunchArgument(
            "rosbag_dir",
            default_value="/root/ws/src/tabletop/ros/bags",
            description="Base directory to save rosbags",
        ),
        # ROS Warehouse
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value="/root/ws/src/tabletop/ros/warehouse_ros.sqlite",
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
            default_value="own_log",
            description="Mock Teensy output",
            choices=["log", "both", "screen", "own_log"],
        ),
        DeclareLaunchArgument(
            "flic_output",
            default_value="own_log",
            description="Flic output",
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
            default_value="own_log",
            description="Bag output",
            choices=["log", "both", "screen", "own_log"],
        ),
    ]


def generate_launch_description():
    # Common
    use_sim_time = LaunchConfiguration("use_sim_time")
    ur_type = LaunchConfiguration("ur_type")
    robot_mode = LaunchConfiguration("robot_mode")

    # UR Robot Driver
    controller_spawner_timeout = LaunchConfiguration(
        "controller_spawner_timeout"
    )
    description_launchfile = LaunchConfiguration("description_launchfile")
    description_file = LaunchConfiguration("description_file")

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
    rosbag_args = LaunchConfiguration("rosbag_args")
    rosbag_dir = LaunchConfiguration("rosbag_dir")

    # ROS Warehouse
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")

    # Logging
    default_log_level = LaunchConfiguration("default_log_level")
    teensy_log_level = LaunchConfiguration("teensy_log_level")

    # Outputs
    mock_dashboard_output = LaunchConfiguration("mock_dashboard_output")
    ur_output = LaunchConfiguration("ur_output")
    micro_ros_output = LaunchConfiguration("micro_ros_output")
    mock_teensy_output = LaunchConfiguration("mock_teensy_output")
    flic_output = LaunchConfiguration("flic_output")
    rviz_output = LaunchConfiguration("rviz_output")
    bag_output = LaunchConfiguration("bag_output")
    ###########################################################################

    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # Conditional substitutions
    robot_ip = IfElseSubstitution(
        EqualsSubstitution(robot_mode, "real"),
        "192.168.13.20",
        "192.168.12.20",
    )
    reverse_ip = IfElseSubstitution(
        EqualsSubstitution(robot_mode, "real"),
        "192.168.13.10",
        "192.168.12.10",
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
                    "launch_rviz": "false",
                    "kinematics_params_file": kinematics_params_file,
                    "description_launchfile": description_launchfile,
                    "description_file": description_file,
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
    )

    # Mock Teensy
    mock_teensy = Node(
        name="teensy",
        package="tabletop_server",
        executable="mock_teensy",
        output=mock_teensy_output,
        parameters=[{"use_sim_time": use_sim_time}],
        ros_arguments=["--log-level", ["teensy:=", teensy_log_level]],
        condition=IfCondition(use_mock_teensy),
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
        ros_arguments=["--log-level", ["flic:=", default_log_level]],
    )

    # RViz MoveIt Config
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
        arguments=["-d", rviz_config_file_server],  # -l for ogre log
        ros_arguments=["--log-level", default_log_level],
    )

    # Bag
    bag = ExecuteProcess(
        cmd=["ros2", "bag", "record", rosbag_args],
        cwd=rosbag_dir,
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
