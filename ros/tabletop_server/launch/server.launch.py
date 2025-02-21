from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_service import LaunchService
from launch.logging import launch_config as launch_logging_config
from launch.substitutions import (
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetROSLogDir
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def declare_arguments():
    return [
        # Common
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
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
        # UR Robot Driver
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.12.10",
            description="IP address of the robot",
        ),
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="192.168.12.11",
            description="Reverse IP address",
        ),
        DeclareLaunchArgument(
            "use_mock_robot",
            default_value="false",
            description="Use mock robot",
        ),
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="120",
            description="Controller spawner timeout",
        ),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_description"),
                    "config",
                    "ursim_calibration.yaml",
                ]
            ),
            description="Calibration file",
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
            "publish_robot_description_semantic",
            default_value="true",
            description="MoveGroup publishes robot description semantic",
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
            description="Use mock TeensyBoard",
        ),
        # RViz
        DeclareLaunchArgument(
            "launch_rviz_server",
            default_value="true",
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
        # Logging
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "commander_log_level",
            default_value="INFO",
            description="Commander log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        # VSCode Debugging
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Debug mode",
        ),
    ]


def generate_launch_description():
    # Common
    use_sim_time = LaunchConfiguration("use_sim_time")
    ur_type = LaunchConfiguration("ur_type")

    # UR Robot Driver
    controller_spawner_timeout = LaunchConfiguration(
        "controller_spawner_timeout"
    )
    robot_ip = LaunchConfiguration("robot_ip")
    reverse_ip = LaunchConfiguration("reverse_ip")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    description_launchfile = LaunchConfiguration("description_launchfile")
    description_file = LaunchConfiguration("description_file")
    use_mock_robot = LaunchConfiguration("use_mock_robot")

    # Commander
    commander_config = LaunchConfiguration("commander_config")
    publish_robot_description_semantic = LaunchConfiguration(
        "publish_robot_description_semantic"
    )

    # Teensy
    micro_ros_transport = LaunchConfiguration("micro_ros_transport")
    micro_ros_device = LaunchConfiguration("micro_ros_device")
    micro_ros_baudrate = LaunchConfiguration("micro_ros_baudrate")
    micro_ros_verbosity = LaunchConfiguration("micro_ros_verbosity")
    use_mock_teensy = LaunchConfiguration("use_mock_teensy")

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
    log_level = LaunchConfiguration("log_level")
    commander_log_level = LaunchConfiguration("commander_log_level")
    ################################################
    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # UR Robot Driver
    # Use group action to isolate the launch file
    ur_robot_driver = GroupAction(
        [
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
            )
        ],
        scoped=True,
        forwarding=True,
    )

    # Commander
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
    # TODO: Add node-specific params/remappings a la
    # https://docs.ros.org/en/jazzy/How-To-Guides/Node-arguments.html#logger-configuration
    commander = Node(
        package="tabletop_server",
        executable="commander",
        parameters=[
            moveit_config.to_dict(),
            ParameterFile(commander_config, allow_substs=True),
            {
                "publish_robot_description_semantic": publish_robot_description_semantic,
                "use_sim_time": use_sim_time,
            },
        ],
        ros_arguments=["--log-level", ["commander:=", commander_log_level]],
        output="both",
    )

    # Micro ROS Agent
    micro_ros_agent = Node(
        package="micro_ros_agent",
        name="micro_ros_agent",
        executable="micro_ros_agent",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}],
        ros_arguments=["--log-level", log_level],
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
        output="both",
        parameters=[{"use_sim_time": use_sim_time}],
        ros_arguments=["--log-level", log_level],
        condition=IfCondition(use_mock_teensy),
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz_server),
        executable="rviz2",
        output="both",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
        arguments=["-d", rviz_config_file_server],
        ros_arguments=["--log-level", log_level],
    )

    # Bag
    bag = ExecuteProcess(
        cmd=["ros2", "bag", "record", rosbag_args],
        cwd=rosbag_dir,
        output="both",
        condition=IfCondition(rosbag_record),
    )

    return LaunchDescription(
        declare_arguments()
        + [
            set_ros_log_dir,
            ur_robot_driver,
            commander,
            micro_ros_agent,
            mock_teensy,
            rviz_node,
            bag,
        ]
    )


def main():
    launch_logging_config.level = "DEBUG"
    ls = LaunchService()
    ld = generate_launch_description()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    main()
