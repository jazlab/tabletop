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
from launch_ros.actions import Node, PushROSNamespace, SetROSLogDir
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
            "left_reverse_port",
            default_value="50001",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller (left robot).",
        ),
        DeclareLaunchArgument(
            "left_script_sender_port",
            default_value="50002",
            description="The driver will offer an interface to query the external_control URScript on this port (left robot).",
        ),
        DeclareLaunchArgument(
            "left_trajectory_port",
            default_value="50003",
            description="Port that will be opened for trajectory control (left robot).",
        ),
        DeclareLaunchArgument(
            "left_script_command_port",
            default_value="50004",
            description="Port that will be opened to forward URScript commands to the robot (left robot).",
        ),
        DeclareLaunchArgument(
            "right_reverse_port",
            default_value="50005",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller (right robot).",
        ),
        DeclareLaunchArgument(
            "right_script_sender_port",
            default_value="50006",
            description="The driver will offer an interface to query the external_control URScript on this port (right robot).",
        ),
        DeclareLaunchArgument(
            "right_trajectory_port",
            default_value="50007",
            description="Port that will be opened for trajectory control (right robot).",
        ),
        DeclareLaunchArgument(
            "right_script_command_port",
            default_value="50008",
            description="Port that will be opened to forward URScript commands to the robot (right robot).",
        ),
        DeclareLaunchArgument(
            "left_base_origin_xyz",
            default_value="0.665 1.0625 0.3085",
            description="Space-separated 3D translation from world to left robot base frame",
        ),
        DeclareLaunchArgument(
            "left_base_origin_rpy",
            default_value="0.0 0.0 -1.5707",
            description="Space-separated 3D Euler rotation from world to base frame",
        ),
        DeclareLaunchArgument(
            "right_base_origin_xyz",
            default_value="1.2554 1.0625 0.3085",
            # default_value="1.2 1.25 0.3085",
            # default_value="1.33 1.07 0.3085", # This value works for all objects except (7,1)
            description="Space-separated 3D translation from world to right robot base frame",
        ),
        DeclareLaunchArgument(
            "right_base_origin_rpy",
            default_value="0.0 0.0 -1.5707",
            description="Space-separated 3D Euler rotation from world to base frame",
        ),
        DeclareLaunchArgument(
            "left_initial_positions_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_description"),
                    "config",
                    "left_initial_positions.yaml",
                ]
            ),
            description="Initial positions file for the robot (left robot).",
        ),
        DeclareLaunchArgument(
            "right_initial_positions_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_description"),
                    "config",
                    "right_initial_positions.yaml",
                ]
            ),
            description="Initial positions file for the robot (right robot).",
        ),
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",  # TODO: Was originally true, see if false causes problems
            description="Enables the safety limits controller if true.",
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
    use_mock_hardware = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration("robot_mode"), "mock"),
        "true",
        "false",
    )
    left_robot_ip = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration("robot_mode"), "real"),
        os.environ["LEFT_ROBOT_IP"],
        os.environ["LEFT_SIM_ROBOT_IP"],
    )
    right_robot_ip = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration("robot_mode"), "real"),
        os.environ["RIGHT_ROBOT_IP"],
        os.environ["RIGHT_SIM_ROBOT_IP"],
    )
    reverse_ip = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration("robot_mode"), "real"),
        os.environ["REVERSE_IP"],
        os.environ["SIM_REVERSE_IP"],
    )
    left_kinematics_params_file = PathJoinSubstitution(
        [
            FindPackageShare("tabletop_description"),
            "config",
            IfElseSubstitution(
                EqualsSubstitution(LaunchConfiguration("robot_mode"), "ursim"),
                "ursim_calibration.yaml",
                "left_ur5e_calibration.yaml",
            ),
        ]
    )
    right_kinematics_params_file = PathJoinSubstitution(
        [
            FindPackageShare("tabletop_description"),
            "config",
            IfElseSubstitution(
                EqualsSubstitution(LaunchConfiguration("robot_mode"), "ursim"),
                "ursim_calibration.yaml",
                "right_ur5e_calibration.yaml",
            ),
        ]
    )

    # Print substitutions
    print_substitutions_action = OpaqueFunction(
        function=print_substitutions,
        args=[
            {
                "left_robot_ip": left_robot_ip,
                "right_robot_ip": right_robot_ip,
                "reverse_ip": reverse_ip,
                "use_mock_hardware": use_mock_hardware,
                "left_kinematics_params_file": left_kinematics_params_file,
                "right_kinematics_params_file": right_kinematics_params_file,
            }
        ],
        condition=IfCondition(str(launch_config.level == logging.DEBUG)),
    )

    # UR Robot Driver (use group action to isolate the launch file)
    left_robot = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=EnvironmentVariable(
                    "OVERRIDE_LAUNCH_PROCESS_OUTPUT", default_value="both"
                ),
            ),
            PushROSNamespace("left"),
            # SetRemap("/controller_manager", "/left/controller_manager"),
            # SetUseSimTime(LaunchConfiguration("use_sim_time")),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_rig"),
                                "launch",
                                "ur_control.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "tf_prefix": "left_",
                    "ur_type": LaunchConfiguration("ur_type"),
                    "robot_ip": left_robot_ip,
                    "reverse_ip": reverse_ip,
                    "use_mock_hardware": use_mock_hardware,
                    "controller_spawner_timeout": LaunchConfiguration(
                        "controller_spawner_timeout"
                    ),
                    "initial_joint_controller": LaunchConfiguration(
                        "initial_joint_controller"
                    ),
                    "launch_rviz": "false",
                    "kinematics_params_file": left_kinematics_params_file,
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
                    "reverse_port": LaunchConfiguration("left_reverse_port"),
                    "script_sender_port": LaunchConfiguration(
                        "left_script_sender_port"
                    ),
                    "trajectory_port": LaunchConfiguration(
                        "left_trajectory_port"
                    ),
                    "script_command_port": LaunchConfiguration(
                        "left_script_command_port"
                    ),
                    "base_origin_xyz": LaunchConfiguration(
                        "left_base_origin_xyz"
                    ),
                    "base_origin_rpy": LaunchConfiguration(
                        "left_base_origin_rpy"
                    ),
                    "controllers_file": PathJoinSubstitution(
                        [
                            FindPackageShare("tabletop_rig"),
                            "config",
                            "controllers.yaml",
                        ]
                    ),
                    # "initial_positions_file": LaunchConfiguration(
                    #     "left_initial_positions_file"
                    # ),
                    "safety_limits": LaunchConfiguration("safety_limits"),
                }.items(),
            ),
            Node(
                package="tabletop_rig",
                executable="mock_dashboard_client",
                output="both",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}
                ],
                ros_arguments=[
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                condition=IfCondition(use_mock_hardware),
                on_exit=[Shutdown()],
            ),
        ],
        scoped=True,
        forwarding=True,
    )

    right_robot = GroupAction(
        [
            SetEnvironmentVariable(
                name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
                value=EnvironmentVariable(
                    "OVERRIDE_LAUNCH_PROCESS_OUTPUT", default_value="both"
                ),
            ),
            PushROSNamespace("right"),
            # SetRemap("/controller_manager", "/right/controller_manager"),
            # SetUseSimTime(LaunchConfiguration("use_sim_time")),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tabletop_rig"),
                                "launch",
                                "ur_control.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "tf_prefix": "right_",
                    "ur_type": LaunchConfiguration("ur_type"),
                    "robot_ip": right_robot_ip,
                    "reverse_ip": reverse_ip,
                    "use_mock_hardware": use_mock_hardware,
                    "controller_spawner_timeout": LaunchConfiguration(
                        "controller_spawner_timeout"
                    ),
                    "initial_joint_controller": LaunchConfiguration(
                        "initial_joint_controller"
                    ),
                    "launch_rviz": "false",
                    "kinematics_params_file": right_kinematics_params_file,
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
                    "reverse_port": LaunchConfiguration("right_reverse_port"),
                    "script_sender_port": LaunchConfiguration(
                        "right_script_sender_port"
                    ),
                    "trajectory_port": LaunchConfiguration(
                        "right_trajectory_port"
                    ),
                    "script_command_port": LaunchConfiguration(
                        "right_script_command_port"
                    ),
                    "base_origin_xyz": LaunchConfiguration(
                        "right_base_origin_xyz"
                    ),
                    "base_origin_rpy": LaunchConfiguration(
                        "right_base_origin_rpy"
                    ),
                    "controllers_file": PathJoinSubstitution(
                        [
                            FindPackageShare("tabletop_rig"),
                            "config",
                            "controllers.yaml",
                        ]
                    ),
                    # "initial_positions_file": LaunchConfiguration(
                    #     "left_initial_positions_file"
                    # ),
                    "safety_limits": LaunchConfiguration("safety_limits"),
                }.items(),
            ),
            Node(
                package="tabletop_rig",
                executable="mock_dashboard_client",
                output="both",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}
                ],
                ros_arguments=[
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                condition=IfCondition(use_mock_hardware),
                on_exit=[Shutdown()],
            ),
        ],
        scoped=True,
        forwarding=True,
    )

    launch_actions = [
        *declare_arguments(),
        set_ros_log_dir,
        print_substitutions_action,
        left_robot,
        right_robot,
    ]

    return LaunchDescription(launch_actions)
