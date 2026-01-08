import logging
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch.substitution import Substitution
from launch.substitutions import (
    EnvironmentVariable,
    EqualsSubstitution,
    IfElseSubstitution,
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetROSLogDir
from launch_ros.substitutions import FindPackageShare


def declare_arguments():
    return [
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
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Mock Dashboard log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use simulated time",
        ),
    ]


def print_substitutions(context, substitutions: dict[str, Substitution]):
    for name, substitution in substitutions.items():
        print(f"{name}: {substitution.perform(context)}")


def generate_launch_description():
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
    use_mock_hardware = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration("robot_mode"), "mock"),
        "true",
        "false",
    )
    kinematics_params_file = PathJoinSubstitution(
        [
            FindPackageShare("tabletop_description"),
            "config",
            IfElseSubstitution(
                EqualsSubstitution(LaunchConfiguration("robot_mode"), "ursim"),
                "ursim_calibration.yaml",
                "ur5e_calibration.yaml",
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
                "use_mock_hardware": use_mock_hardware,
                "kinematics_params_file": kinematics_params_file,
            }
        ],
        condition=IfCondition(str(launch_config.level == logging.DEBUG)),
    )

    # UR Robot Driver (use group action to isolate the launch file)
    driver = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=EnvironmentVariable(
                    "OVERRIDE_LAUNCH_PROCESS_OUTPUT", default_value="both"
                ),
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
                    "use_mock_hardware": use_mock_hardware,
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
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=True,
    )

    # Mock Dashboard
    mock_dashboard = Node(
        package="tabletop_rig",
        executable="mock_dashboard",
        output="both",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        condition=IfCondition(use_mock_hardware),
        on_exit=[Shutdown()],
    )

    launch_actions = [
        *declare_arguments(),
        set_ros_log_dir,
        print_substitutions_action,
        driver,
        mock_dashboard,
    ]

    return LaunchDescription(launch_actions)
