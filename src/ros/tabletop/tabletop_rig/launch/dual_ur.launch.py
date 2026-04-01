# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Denis Stogl


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
)
from launch.substitution import Substitution
from launch.substitutions import (
    AndSubstitution,
    EnvironmentVariable,
    EqualsSubstitution,
    IfElseSubstitution,
    LaunchConfiguration,
    LaunchLogDir,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushROSNamespace, SetROSLogDir
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

UR_TYPE_CHOICES = [
    "ur3",
    "ur3e",
    "ur5",
    "ur5e",
    "ur7e",
    "ur10",
    "ur10e",
    "ur12e",
    "ur16e",
    "ur8long",
    "ur15",
    "ur18",
    "ur20",
    "ur30",
]


def declare_arguments():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_mode",
            default_value="mock",
            choices=["mock", "ursim", "real"],
            description="Whether to use the mock robot, URSim, or real robot",
        )
    ]

    # Conditional substitutions
    left_robot_ip = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration("robot_mode"), "real"),
        EnvironmentVariable("LEFT_ROBOT_IP"),
        EnvironmentVariable("LEFT_SIM_ROBOT_IP"),
    )
    right_robot_ip = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration("robot_mode"), "real"),
        EnvironmentVariable("RIGHT_ROBOT_IP"),
        EnvironmentVariable("RIGHT_SIM_ROBOT_IP"),
    )
    reverse_ip = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration("robot_mode"), "real"),
        EnvironmentVariable("REVERSE_IP"),
        EnvironmentVariable("SIM_REVERSE_IP"),
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

    # Shared arguments
    declared_arguments.extend(
        [
            DeclareLaunchArgument(
                "reverse_ip",
                default_value=reverse_ip,
                description="IP for robot-to-driver communication.",
            ),
            DeclareLaunchArgument(
                "safety_limits",
                default_value="true",
                description="Enables the safety limits controller if true.",
            ),
            DeclareLaunchArgument(
                "safety_pos_margin",
                default_value="0.15",
                description="The margin to lower and upper limits in the safety controller.",
            ),
            DeclareLaunchArgument(
                "safety_k_position",
                default_value="20",
                description="k-position factor in the safety controller.",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("tabletop_rig"),
                        "config",
                        "dual_controllers.yaml",
                    ]
                ),
                description="YAML file with the dual controllers configuration.",
            ),
            DeclareLaunchArgument(
                "description_launchfile",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("tabletop_description"),
                        "launch",
                        "dual_rsp.launch.py",
                    ]
                ),
                description="Launchfile providing the dual robot description.",
            ),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="Start robot with mock hardware mirroring command to its states.",
            ),
            DeclareLaunchArgument(
                "mock_sensor_commands",
                default_value="false",
                description="Enable mock command interfaces for sensors.",
            ),
            DeclareLaunchArgument(
                "headless_mode",
                default_value="false",
                description="Enable headless mode for robot control.",
            ),
            DeclareLaunchArgument(
                "controller_spawner_timeout",
                default_value="10",
                description="Timeout used when spawning controllers.",
            ),
            DeclareLaunchArgument(
                "initial_joint_controller",
                default_value="scaled_joint_trajectory_controller",
                choices=[
                    "scaled_joint_trajectory_controller",
                    "joint_trajectory_controller",
                    "forward_velocity_controller",
                    "forward_position_controller",
                    "freedrive_mode_controller",
                    "passthrough_trajectory_controller",
                ],
                description="Initially loaded robot controller (prefixed per arm).",
            ),
            DeclareLaunchArgument(
                "activate_joint_controller",
                default_value="true",
                description="Activate loaded joint controller.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz?",
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("ur_description"),
                        "rviz",
                        "view_robot.rviz",
                    ]
                ),
                description="RViz config file to use when launching rviz.",
            ),
            DeclareLaunchArgument(
                "launch_dashboard_client",
                default_value="true",
                description="Launch Dashboard Client?",
            ),
            DeclareLaunchArgument(
                name="update_rate_config_file",
                default_value=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ur_robot_driver"),
                            "config",
                        ]
                    ),
                    "/",
                    LaunchConfiguration("left_ur_type"),
                    "_update_rate.yaml",
                ],
                description="Update rate config (uses left_ur_type by default).",
            ),
        ]
    )

    # Per-arm UR type
    for side in ("left", "right"):
        declared_arguments.append(
            DeclareLaunchArgument(
                f"{side}_ur_type",
                default_value="ur5e",
                description=f"Type/series of the {side} UR robot.",
                choices=UR_TYPE_CHOICES,
            )
        )

    # Per-arm arguments
    for side in ("left", "right"):
        ur_type = LaunchConfiguration(f"{side}_ur_type")

        defaults = {
            "left": {
                "tf_prefix": "left_",
                "robot_ip": left_robot_ip,
                "kinematics_params_file": left_kinematics_params_file,
                "script_command_port": "50014",
                "reverse_port": "50011",
                "script_sender_port": "50012",
                "trajectory_port": "50013",
                "tool_tcp_port": "54322",
                "base_origin_xyz": "0.665 1.0625 0.3085",
                "base_origin_rpy": "0.0 0.0 -1.5707",
            },
            "right": {
                "tf_prefix": "right_",
                "robot_ip": right_robot_ip,
                "kinematics_params_file": right_kinematics_params_file,
                "script_command_port": "50004",
                "reverse_port": "50001",
                "script_sender_port": "50002",
                "trajectory_port": "50003",
                "tool_tcp_port": "54321",
                "base_origin_xyz": "1.2554 1.0625 0.3085",
                "base_origin_rpy": "0.0 0.0 -1.5707",
            },
        }[side]

        declared_arguments.extend(
            [
                DeclareLaunchArgument(
                    f"{side}_robot_ip",
                    default_value=defaults["robot_ip"],
                    description=f"IP address of the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_kinematics_params_file",
                    default_value=defaults["kinematics_params_file"],
                    description=f"Calibration config for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_joint_limit_params_file",
                    default_value=PathJoinSubstitution(
                        [
                            FindPackageShare("ur_description"),
                            "config",
                            ur_type,
                            "joint_limits.yaml",
                        ]
                    ),
                    description=f"Joint limits config for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_physical_params_file",
                    default_value=PathJoinSubstitution(
                        [
                            FindPackageShare("ur_description"),
                            "config",
                            ur_type,
                            "physical_parameters.yaml",
                        ]
                    ),
                    description=f"Physical parameters config for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_visual_params_file",
                    default_value=PathJoinSubstitution(
                        [
                            FindPackageShare("ur_description"),
                            "config",
                            ur_type,
                            "visual_parameters.yaml",
                        ]
                    ),
                    description=f"Visual parameters config for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_initial_positions_file",
                    default_value=PathJoinSubstitution(
                        [
                            FindPackageShare("tabletop_description"),
                            "config",
                            f"{side}_initial_positions.yaml",
                        ]
                    ),
                    description=f"Initial positions file for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_script_command_port",
                    default_value=defaults["script_command_port"],
                    description=f"URScript command port for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_reverse_port",
                    default_value=defaults["reverse_port"],
                    description=f"Reverse port for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_script_sender_port",
                    default_value=defaults["script_sender_port"],
                    description=f"Script sender port for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_trajectory_port",
                    default_value=defaults["trajectory_port"],
                    description=f"Trajectory port for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_use_tool_communication",
                    default_value="false",
                    description=f"Enable tool communication for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_tool_parity",
                    default_value="0",
                    description=f"Tool serial parity for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_tool_baud_rate",
                    default_value="115200",
                    description=f"Tool serial baud rate for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_tool_stop_bits",
                    default_value="1",
                    description=f"Tool serial stop bits for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_tool_rx_idle_chars",
                    default_value="1.5",
                    description=f"Tool RX idle chars for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_tool_tx_idle_chars",
                    default_value="3.5",
                    description=f"Tool TX idle chars for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_tool_device_name",
                    default_value=f"/tmp/ttyUR_{side}",
                    description=f"Tool device name for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_tool_tcp_port",
                    default_value=defaults["tool_tcp_port"],
                    description=f"Tool TCP port for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_tool_voltage",
                    default_value="0",
                    description=f"Tool voltage for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_base_origin_xyz",
                    default_value="0.0 0.0 0.0",
                    description=f"3D translation from world to {side} base frame.",
                ),
                DeclareLaunchArgument(
                    f"{side}_base_origin_rpy",
                    default_value="0.0 0.0 0.0",
                    description=f"3D Euler rotation from world to {side} base frame.",
                ),
            ]
        )

    return declared_arguments


def launch_setup(context):
    # Shared arguments
    controllers_file = LaunchConfiguration("controllers_file")
    description_launchfile = LaunchConfiguration("description_launchfile")
    use_mock_hardware = EqualsSubstitution(
        LaunchConfiguration("robot_mode"), "mock"
    )
    controller_spawner_timeout = LaunchConfiguration(
        "controller_spawner_timeout"
    )
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration(
        "activate_joint_controller"
    )
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")

    # Shared controller manager node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            LaunchConfiguration("update_rate_config_file"),
            ParameterFile(controllers_file, allow_substs=True),
        ],
        output="screen",
        on_exit=[Shutdown()],
    )

    # RSP using dual_rsp.launch.py — passes all per-arm parameters through
    rsp_launch_args = {
        "safety_limits": LaunchConfiguration("safety_limits"),
        "safety_pos_margin": LaunchConfiguration("safety_pos_margin"),
        "safety_k_position": LaunchConfiguration("safety_k_position"),
        "use_mock_hardware": use_mock_hardware,
        "mock_sensor_commands": LaunchConfiguration("mock_sensor_commands"),
        "headless_mode": headless_mode,
    }
    for side in ("left", "right"):
        for key in [
            "ur_type",
            "robot_ip",
            "reverse_ip",
            "kinematics_params_file",
            "joint_limit_params_file",
            "physical_params_file",
            "visual_params_file",
            "initial_positions_file",
            "script_command_port",
            "reverse_port",
            "script_sender_port",
            "trajectory_port",
            "use_tool_communication",
            "tool_parity",
            "tool_baud_rate",
            "tool_stop_bits",
            "tool_rx_idle_chars",
            "tool_tx_idle_chars",
            "tool_device_name",
            "tool_tcp_port",
            "tool_voltage",
            "base_origin_xyz",
            "base_origin_rpy",
        ]:
            rsp_launch_args[f"{side}_{key}"] = LaunchConfiguration(
                f"{side}_{key}"
            )

    rsp = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(description_launchfile),
        launch_arguments=rsp_launch_args.items(),
    )

    # RViz (shared, single instance)
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Per-arm controller spawning
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    base_controllers_active = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "tcp_pose_broadcaster",
        "ur_configuration_controller",
    ]
    base_controllers_inactive = [
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        "forward_effort_controller",
        "force_mode_controller",
        "passthrough_trajectory_controller",
        "freedrive_mode_controller",
        "tool_contact_controller",
    ]

    controller_spawners = []
    for side in ("left", "right"):
        active = [f"{side}_{c}" for c in base_controllers_active]
        inactive = [f"{side}_{c}" for c in base_controllers_inactive]

        if activate_joint_controller.perform(context) == "true":
            prefixed = f"{side}_{initial_joint_controller.perform(context)}"
            active.append(prefixed)
            inactive.remove(prefixed)

        if use_mock_hardware.perform(context) == "true":
            active.remove(f"{side}_tcp_pose_broadcaster")

        controller_spawners.append(controller_spawner(active))
        controller_spawners.append(controller_spawner(inactive, active=False))

    # Per-arm nodes grouped into namespaces
    per_arm_groups = []
    for side in ("left", "right"):
        robot_ip = LaunchConfiguration(f"{side}_robot_ip")
        use_tool_communication = LaunchConfiguration(
            f"{side}_use_tool_communication"
        )
        tool_device_name = LaunchConfiguration(f"{side}_tool_device_name")
        tool_tcp_port = LaunchConfiguration(f"{side}_tool_tcp_port")

        dashboard_client_node = IncludeLaunchDescription(
            condition=IfCondition(
                AndSubstitution(
                    launch_dashboard_client,
                    NotSubstitution(use_mock_hardware),
                )
            ),
            launch_description_source=AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur_robot_driver"),
                        "launch",
                        "ur_dashboard_client.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "robot_ip": robot_ip,
            }.items(),
        )

        mock_dashboard_client_node = Node(
            package="tabletop_rig",
            executable="mock_dashboard_client",
            output="both",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ros_arguments=[
                "--log-level",
                LaunchConfiguration("log_level"),
            ],
            condition=IfCondition(
                AndSubstitution(launch_dashboard_client, use_mock_hardware)
            ),
            on_exit=[Shutdown()],
        )

        robot_state_helper_node = Node(
            package="ur_robot_driver",
            executable="robot_state_helper",
            name="ur_robot_state_helper",
            output="screen",
            condition=UnlessCondition(use_mock_hardware),
            parameters=[
                {"headless_mode": headless_mode},
                {"robot_ip": robot_ip},
            ],
            on_exit=[Shutdown()],
        )

        tool_communication_node = Node(
            package="ur_robot_driver",
            condition=IfCondition(use_tool_communication),
            executable="tool_communication.py",
            name="ur_tool_comm",
            output="screen",
            parameters=[
                {
                    "robot_ip": robot_ip,
                    "tcp_port": tool_tcp_port,
                    "device_name": tool_device_name,
                }
            ],
            on_exit=[Shutdown()],
        )

        urscript_interface = Node(
            package="ur_robot_driver",
            executable="urscript_interface",
            parameters=[{"robot_ip": robot_ip}],
            output="screen",
            condition=UnlessCondition(use_mock_hardware),
            on_exit=[Shutdown()],
        )

        controller_stopper_node = Node(
            package="ur_robot_driver",
            executable="controller_stopper_node",
            name="controller_stopper",
            output="screen",
            emulate_tty=True,
            condition=UnlessCondition(use_mock_hardware),
            parameters=[
                {"headless_mode": headless_mode},
                {"joint_controller_active": activate_joint_controller},
                {
                    "consistent_controllers": [
                        f"{side}_io_and_status_controller",
                        f"{side}_force_torque_sensor_broadcaster",
                        f"{side}_joint_state_broadcaster",
                        f"{side}_speed_scaling_state_broadcaster",
                        f"{side}_tcp_pose_broadcaster",
                        f"{side}_ur_configuration_controller",
                    ]
                },
            ],
            on_exit=[Shutdown()],
        )

        trajectory_until_node = Node(
            package="ur_robot_driver",
            executable="trajectory_until_node",
            name="trajectory_until_node",
            output="screen",
            parameters=[
                {
                    "motion_controller": [
                        side,
                        "_",
                        initial_joint_controller,
                    ],
                },
            ],
            on_exit=[Shutdown()],
        )

        per_arm_groups.append(
            GroupAction(
                [
                    PushROSNamespace(side),
                    dashboard_client_node,
                    mock_dashboard_client_node,
                    robot_state_helper_node,
                    tool_communication_node,
                    urscript_interface,
                    controller_stopper_node,
                    trajectory_until_node,
                ],
                scoped=True,
                forwarding=True,
            )
        )

    nodes_to_start = (
        [
            control_node,
            rsp,
            rviz_node,
        ]
        + per_arm_groups
        + controller_spawners
    )

    return nodes_to_start


def print_substitutions(context, substitutions: dict[str, Substitution]):
    for name, substitution in substitutions.items():
        print(f"{name}: {substitution.perform(context)}")


def generate_launch_description():
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    return LaunchDescription(
        [
            *declare_arguments(),
            set_ros_log_dir,
            OpaqueFunction(function=launch_setup),
        ]
    )
