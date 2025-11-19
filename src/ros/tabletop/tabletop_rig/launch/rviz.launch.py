import os

from launch import (
    LaunchDescription,
)
from launch.actions import (
    DeclareLaunchArgument,
    Shutdown,
)
from launch.substitutions import (
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetROSLogDir
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def declare_arguments():
    return [
        DeclareLaunchArgument(
            "robot_name",
            default_value="ur5e",
            description="Robot name for MoveIt SRDF",
        ),
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.join(
                os.environ["TABLETOP_CACHE_DIR"], "warehouse_ros.sqlite"
            ),
            description="Path where the warehouse database should be stored",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("tabletop_rig"), "rviz", "rig.rviz"]
            ),
            description="RViz config file",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Flic log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use simulated time",
        ),
    ]


def generate_launch_description():
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # RViz MoveIt Config
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur", package_name="tabletop_moveit_config"
        )
        .robot_description_semantic(
            file_path="srdf/tabletop.srdf.xacro",
            mappings={"name": LaunchConfiguration("robot_name")},
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
        output="both",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            # moveit_config.to_dict(), # TODO: Figure out which one to use
            warehouse_ros_config,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=[
            "-d",
            LaunchConfiguration("config_file"),
            "-l",  # -l for ogre log
        ],
        cwd=LaunchLogDir(),
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        on_exit=[Shutdown()],
    )

    launch_actions = [*declare_arguments(), set_ros_log_dir, rviz]

    return LaunchDescription(launch_actions)
