"""Launch file for RViz visualization with MoveIt planning.

Launches RViz with MoveIt plugin and warehouse database connection for
visualization and interactive motion planning of the dual-arm robot.

Nodes Launched:
    wait_for_robot_description (ur_robot_driver): Blocks until robot
        description is available
    rviz2: RViz visualization with MoveIt plugin

Config Files Loaded:
    - rig.rviz: RViz configuration with MoveIt displays
    - moveit_cpp.yaml: MoveIt C++ configuration (via MoveItConfigsBuilder)

Example:
    ros2 launch tabletop_rig rviz.launch.py robot_name:=tabletop
"""

from launch import (
    LaunchDescription,
)
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
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
            default_value="tabletop",
            description="Robot name for MoveIt SRDF",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("tabletop_rig"), "config", "rig.rviz"]
            ),
            description="RViz config file",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Log level",
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

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="tabletop", package_name="tabletop_moveit_config"
        )
        .robot_description_semantic(
            file_path="srdf/dual_tabletop.srdf.xacro",
            mappings={"name": LaunchConfiguration("robot_name")},
        )
        .moveit_cpp(
            file_path="config/moveit_cpp.yaml",
        )
        .to_moveit_configs()
    )

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="both",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="both",
        parameters=[
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
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

    robot_description_ready_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_robot_description,
            on_exit=[rviz],
        )
    )

    launch_actions = [
        *declare_arguments(),
        set_ros_log_dir,
        wait_robot_description,
        robot_description_ready_handler,
    ]

    return LaunchDescription(launch_actions)
