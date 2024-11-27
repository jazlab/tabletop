import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except (
        OSError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def declare_arguments():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz", default_value="true", description="Launch RViz?"
            ),
            DeclareLaunchArgument(
                "ur_type",
                description="Typo/series of used UR robot.",
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
                "warehouse_sqlite_path",
                default_value=os.path.expanduser(
                    "~/.ros/warehouse_ros.sqlite"
                ),
                description="Path where the warehouse database should be stored",
            ),
            DeclareLaunchArgument(
                "launch_servo",
                default_value="false",
                description="Launch Servo?",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Using or not time from simulation",
            ),
            DeclareLaunchArgument(
                "publish_robot_description_semantic",
                default_value="true",
                description="MoveGroup publishes robot description semantic",
            ),
        ]
    )


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    ur_type = LaunchConfiguration("ur_type")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_description_semantic = LaunchConfiguration(
        "publish_robot_description_semantic"
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(
            Path("srdf") / "ur.srdf.xacro", {"name": ur_type}
        )
        .to_moveit_configs()
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur", package_name="tabletop_moveit_config"
        )
        .robot_description_semantic(
            Path("srdf") / "ur.srdf.xacro", {"name": ur_type}
        )
        .moveit_cpp(
            file_path=get_package_share_directory("moveit2_tutorials")
            + "/config/motion_planning_python_api_tutorial.yaml"
        )
        .to_moveit_configs()
    )

    tabletop_server = Node(
        name="moveit_py",
        package="tabletop_server",
        executable="tabletop_server",
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    ld = LaunchDescription()
    ld.add_entity(declare_arguments())

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )
    ld.add_action(wait_robot_description)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": publish_robot_description_semantic,
            },
        ],
    )

    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
        output="screen",
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[move_group_node, rviz_node, servo_node],
            )
        ),
    )

    return ld

    rviz_config_file = os.path.join(
        get_package_share_directory("moveit2_tutorials"),
        "config",
        "motion_planning_python_api_tutorial.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="log",
    )

    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "panda_hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=[
                    "ros2 run controller_manager spawner {}".format(controller)
                ],
                shell=True,
                output="log",
            )
        ]
    ur_robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ur_robot_driver"),
                    "launch",
                    "ur_control.launch.py",
                )
            ]
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "robot_ip": "192.168.13.10",
            "reverse_ip": "192.168.13.11",
            "use_mock_hardware": "false",
            "controller_spawner_timeout": "120",
        }.items(),
    )
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur5e", package_name="tabletop_moveit_config"
        )
        .trajectory_execution(
            file_path="config/gripper_moveit_controllers.yaml"
        )
        .moveit_cpp(
            file_path=get_package_share_directory("moveit_py_example")
            + "/config/moveit_py.yaml"
        )
        .to_moveit_configs()
    )
    teensy_controller = Node(
        package="tabletop_server",
        executable="teensy_controller",
        name="teensy_controller",
    )
    teensy_sensor = Node(
        package="tabletop_server",
        executable="teensy_sensor",
        name="teensy_sensor",
    )
    bag = ExecuteProcess(cmd=["ros2", "bag", "record", "-a"], output="screen")

    return LaunchDescription(
        [
            ur_robot_driver,
            tabletop_server,
            teensy_controller,
            teensy_sensor,
            bag,
        ]
    )
