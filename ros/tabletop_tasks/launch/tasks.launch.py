"""
A launch file for running the motion planning python api tutorial
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from tabletop_server.utils import save_yaml, string_to_bool


def declare_arguments():
    return [
        # Common
        DeclareLaunchArgument(
            "tmp_dir",
            default_value="/tmp",
            description="Temporary directory for saving configs as yaml files",
        ),
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
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_description"),
                    "rviz",
                    "view_robot.rviz",
                ]
            ),
            description="RViz config file",
        ),
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
            "use_mock_hardware",
            default_value="false",
            description="Use mock hardware",
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
                    FindPackageShare("tabletop_server"),
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
        # Commander overrides
        DeclareLaunchArgument(
            "planning_group_name",
            default_value="none",
            description="MoveIt group name",
        ),
        DeclareLaunchArgument(
            "planning_pose_link",
            default_value="none",
            description="Pose link",
        ),
        DeclareLaunchArgument(
            "planning_pipeline",
            default_value="none",
            description="Planning pipeline",
        ),
        DeclareLaunchArgument(
            "dashboard_program",
            default_value="none",
            description="UR program name",
        ),
        DeclareLaunchArgument(
            "dashboard_installation",
            default_value="none",
            description="UR installation name",
        ),
        DeclareLaunchArgument(
            "waypoints_path",
            default_value="none",
            description="List of waypoint names in order of execution",
        ),
        # MoveIt
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="true",
            description="MoveGroup publishes robot description semantic",
        ),
        # Bag
        DeclareLaunchArgument(
            "rosbag_args",
            default_value="--all",
            description="'ros2 bag' command line args",
        ),
        DeclareLaunchArgument(
            "rosbag_dir",
            default_value="/root/ws/bags",
            description="Base directory to save rosbags",
        ),
        # Debug
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Debug mode",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
    ]


def launch_setup(context):
    # Common
    tmp_dir = LaunchConfiguration("tmp_dir")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ur_type = LaunchConfiguration("ur_type")

    # UR Robot Driver
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    controller_spawner_timeout = LaunchConfiguration(
        "controller_spawner_timeout"
    )
    robot_ip = LaunchConfiguration("robot_ip")
    reverse_ip = LaunchConfiguration("reverse_ip")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    description_launchfile = LaunchConfiguration("description_launchfile")
    description_file = LaunchConfiguration("description_file")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Commander
    planning_group_name = LaunchConfiguration("planning_group_name")
    planning_pose_link = LaunchConfiguration("planning_pose_link")
    planning_pipeline = LaunchConfiguration("planning_pipeline")
    dashboard_program = LaunchConfiguration("dashboard_program")
    dashboard_installation = LaunchConfiguration("dashboard_installation")
    waypoints_path = LaunchConfiguration("waypoints_path")

    # MoveIt
    publish_robot_description_semantic = LaunchConfiguration(
        "publish_robot_description_semantic"
    )

    # Bag
    rosbag_args = LaunchConfiguration("rosbag_args")
    rosbag_dir = LaunchConfiguration("rosbag_dir")

    # Debug
    debug = LaunchConfiguration("debug")
    log_level = LaunchConfiguration("log_level")

    # Load configs
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
        .to_dict()
    )

    task_executor_yaml = PathJoinSubstitution(
        [
            FindPackageShare("tabletop_tasks"),
            "config",
            "task_executor.yaml",
        ]
    )

    task_executor_overrides = {
        name: value
        for name, value in {
            "dashboard.installation": dashboard_installation.perform(context),
            "dashboard.program": dashboard_program.perform(context),
            "planning.group_name": planning_group_name.perform(context),
            "planning.pose_link": planning_pose_link.perform(context),
            "planning.pipeline": planning_pipeline.perform(context).split(","),
            "waypoints.path": waypoints_path.perform(context).split(","),
        }.items()
        if value not in ["none", ["none"]]
    }
    task_executor_overrides = {
        "/task_executor": {"ros__parameters": task_executor_overrides}
    }

    task_executor_overrides_yaml = (
        f"{tmp_dir.perform(context)}/task_executor_overrides.yaml"
    )
    save_yaml(task_executor_overrides_yaml, task_executor_overrides)

    # UR Robot Driver
    ur_robot_driver = IncludeLaunchDescription(
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
            "use_mock_hardware": use_mock_hardware,
            "controller_spawner_timeout": controller_spawner_timeout,
            "launch_rviz": launch_rviz,
            "rviz_config_file": rviz_config_file,
            "kinematics_params_file": kinematics_params_file,
            "description_launchfile": description_launchfile,
            "description_file": description_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Task Executor
    task_executor = Node(
        package="tabletop_tasks",
        executable="task_executor",
        output="both",
        parameters=[
            moveit_config,
            task_executor_yaml,
            task_executor_overrides_yaml,
            {
                "publish_robot_description_semantic": publish_robot_description_semantic,
                "use_sim_time": use_sim_time,
            },
        ],
        ros_arguments=[
            "--log-level",
            log_level,
        ],
        prefix=["gdbserver :3000"]
        if string_to_bool(debug.perform(context))
        else [],
    )

    # Teensy Controller
    sensor = Node(
        name="sensor",
        package="tabletop_server",
        executable="sensor",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Teensy Sensor
    teensy = Node(
        name="teensy",
        package="tabletop_server",
        executable="mock_teensy",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Bag
    bag = ExecuteProcess(
        cmd=["ros2", "bag", "record", rosbag_args],
        cwd=rosbag_dir,
        output="screen",
    )

    return [ur_robot_driver, task_executor, sensor, teensy, bag]


def generate_launch_description():
    # launch.logging.launch_config.level = logging.DEBUG
    return LaunchDescription(
        declare_arguments() + [OpaqueFunction(function=launch_setup)]
    )


# def main():
#     ls = LaunchService()
#     ld = generate_launch_description()
#     ls.include_launch_description(ld)
#     return ls.run()


# if __name__ == "__main__":
#     main()
