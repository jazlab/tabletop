# Copyright (c) 2024 FZI Forschungszentrum Informatik
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

"""ROS 2 launch file for dual-arm robot state publisher.

This launch file generates the URDF robot description from the
dual_tabletop.urdf.xacro file (containing two UR arms) and starts the
robot_state_publisher node. Each arm has its own ur_type, IP, calibration,
and port configuration.

Launch Arguments:
    left_ur_type / right_ur_type: UR robot series per arm
    left_robot_ip / right_robot_ip: IP address per arm
    safety_limits: Enable safety limits controller (default: true)
    use_mock_hardware: Use mock hardware for simulation (default: false)

Nodes Launched:
    robot_state_publisher: Publishes robot transforms based on URDF

Example:
    ros2 launch tabletop_description dual_rsp.launch.py \
        left_ur_type:=ur5e left_robot_ip:=192.168.12.20 \
        right_ur_type:=ur5e right_robot_ip:=192.168.12.21
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    "ur15",
    "ur20",
    "ur30",
]


def declare_arguments():
    # Shared arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_description"),
                    "urdf",
                    "dual_tabletop.urdf.xacro",
                ]
            ),
            description="URDF/XACRO description file with the dual robot.",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="tabletop",
            description="Name of URDF robot.",
        ),
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="0.0.0.0",
            description="IP for robot-to-driver communication.",
        ),
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            choices=["true", "false"],
            description="Enables the safety limits controller.",
        ),
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="Margin to lower and upper limits in the safety controller.",
        ),
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            choices=["true", "false"],
            description="Start robot with mock hardware.",
        ),
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            choices=["true", "false"],
            description="Enable mock command interfaces for sensors.",
        ),
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            choices=["true", "false"],
            description="Enable headless mode for robot control.",
        ),
        DeclareLaunchArgument(
            "save_urdf",
            default_value="true",
            choices=["true", "false"],
            description="Save parsed URDF to urdf_save_file.",
        ),
        DeclareLaunchArgument(
            "urdf_save_file",
            default_value=PathJoinSubstitution(
                [
                    EnvironmentVariable("TABLETOP_CACHE_DIR"),
                    "dual_tabletop.urdf",
                ]
            ),
            description="Path to save parsed URDF.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
            description="Node log levels",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use simulated time",
        ),
    ]

    side_defaults = {
        "left": {
            "tf_prefix": "left_",
            "script_command_port": "50014",
            "reverse_port": "50011",
            "script_sender_port": "50012",
            "trajectory_port": "50013",
            "tool_tcp_port": "54322",
            # "base_origin_xyz": "0.7 1.0625 0.3085",
            "base_origin_xyz": "0.6025 1.0625 0.3085",
            # "base_origin_xyz": "0.5566 0.7215 0.3085",
            "base_origin_rpy": "0.0 0.0 1.5707",
        },
        "right": {
            "tf_prefix": "right_",
            "script_command_port": "50004",
            "reverse_port": "50001",
            "script_sender_port": "50002",
            "trajectory_port": "50003",
            "tool_tcp_port": "54321",
            # "base_origin_xyz": "1.2554 1.0625 0.3085",
            "base_origin_xyz": "1.3025 1.0625 0.3085",
            # "base_origin_xyz": "1.3484 0.7215 0.3085",
            "base_origin_rpy": "0.0 0.0 -1.5707",
        },
    }

    # Per-arm arguments
    for side in ["left", "right"]:
        declared_arguments.append(
            DeclareLaunchArgument(
                f"{side}_ur_type",
                default_value="ur5e",
                description=f"Type/series of the {side} UR robot.",
                choices=UR_TYPE_CHOICES,
            )
        )

        defaults = side_defaults[side]
        declared_arguments.extend(
            [
                DeclareLaunchArgument(
                    f"{side}_ur_type",
                    default_value="ur5e",
                    description=f"Type/series of the {side} UR robot.",
                    choices=UR_TYPE_CHOICES,
                ),
                DeclareLaunchArgument(
                    f"{side}_tf_prefix",
                    default_value=defaults["tf_prefix"],
                    description=f"tf_prefix of the joint names for the {side} robot",
                ),
                DeclareLaunchArgument(
                    f"{side}_robot_ip",
                    default_value="0.0.0.0",
                    description=f"IP address of the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_kinematics_params_file",
                    default_value=PathJoinSubstitution(
                        [
                            FindPackageShare("tabletop_description"),
                            "config",
                            f"{side}_ur5e_calibration.yaml",
                        ]
                    ),
                    description=f"Calibration config for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_joint_limit_params_file",
                    default_value=PathJoinSubstitution(
                        [
                            FindPackageShare("ur_description"),
                            "config",
                            LaunchConfiguration(f"{side}_ur_type"),
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
                            LaunchConfiguration(f"{side}_ur_type"),
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
                            LaunchConfiguration(f"{side}_ur_type"),
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
                    f"{side}_tool_tcp_port",
                    default_value=defaults["tool_tcp_port"],
                    description=f"Tool TCP port for the {side} robot.",
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
                    f"{side}_tool_voltage",
                    default_value="0",
                    description=f"Tool voltage for the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_base_origin_xyz",
                    default_value=defaults["base_origin_xyz"],
                    description=f"3D translation from world to {side} base frame.",
                ),
                DeclareLaunchArgument(
                    f"{side}_base_origin_rpy",
                    default_value=defaults["base_origin_rpy"],
                    description=f"3D Euler rotation from world to {side} base frame.",
                ),
            ]
        )

    return declared_arguments


def generate_launch_description():
    # Shared parameters
    script_filename = PathJoinSubstitution(
        [
            FindPackageShare("ur_client_library"),
            "resources",
            "external_control.urscript",
        ]
    )
    input_recipe_filename = PathJoinSubstitution(
        [
            FindPackageShare("ur_robot_driver"),
            "resources",
            "rtde_input_recipe.txt",
        ]
    )
    output_recipe_filename = PathJoinSubstitution(
        [
            FindPackageShare("ur_robot_driver"),
            "resources",
            "rtde_output_recipe.txt",
        ]
    )

    xacro_args = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        LaunchConfiguration("description_file"),
        " ",
        "name:=",
        LaunchConfiguration("robot_name"),
        " ",
        "reverse_ip:=",
        LaunchConfiguration("reverse_ip"),
        " ",
        "safety_limits:=",
        LaunchConfiguration("safety_limits"),
        " ",
        "safety_pos_margin:=",
        LaunchConfiguration("safety_pos_margin"),
        " ",
        "safety_k_position:=",
        LaunchConfiguration("safety_k_position"),
        " ",
        "use_mock_hardware:=",
        LaunchConfiguration("use_mock_hardware"),
        " ",
        "mock_sensor_commands:=",
        LaunchConfiguration("mock_sensor_commands"),
        " ",
        "headless_mode:=",
        LaunchConfiguration("headless_mode"),
        " ",
    ]

    # Per-arm parameters
    for side in ("left", "right"):
        xacro_args.extend(
            [
                f"{side}_ur_type:=",
                LaunchConfiguration(f"{side}_ur_type"),
                " ",
                f"{side}_robot_ip:=",
                LaunchConfiguration(f"{side}_robot_ip"),
                " ",
                f"{side}_tf_prefix:=",
                LaunchConfiguration(f"{side}_tf_prefix"),
                " ",
                f"{side}_joint_limit_params:=",
                LaunchConfiguration(f"{side}_joint_limit_params_file"),
                " ",
                f"{side}_kinematics_params:=",
                LaunchConfiguration(f"{side}_kinematics_params_file"),
                " ",
                f"{side}_physical_params:=",
                LaunchConfiguration(f"{side}_physical_params_file"),
                " ",
                f"{side}_visual_params:=",
                LaunchConfiguration(f"{side}_visual_params_file"),
                " ",
                f"{side}_initial_positions_file:=",
                LaunchConfiguration(f"{side}_initial_positions_file"),
                " ",
                f"{side}_script_filename:=",
                script_filename,
                " ",
                f"{side}_input_recipe_filename:=",
                input_recipe_filename,
                " ",
                f"{side}_output_recipe_filename:=",
                output_recipe_filename,
                " ",
                f"{side}_script_command_port:=",
                LaunchConfiguration(f"{side}_script_command_port"),
                " ",
                f"{side}_reverse_port:=",
                LaunchConfiguration(f"{side}_reverse_port"),
                " ",
                f"{side}_script_sender_port:=",
                LaunchConfiguration(f"{side}_script_sender_port"),
                " ",
                f"{side}_trajectory_port:=",
                LaunchConfiguration(f"{side}_trajectory_port"),
                " ",
                f"{side}_use_tool_communication:=",
                LaunchConfiguration(f"{side}_use_tool_communication"),
                " ",
                f"{side}_tool_parity:=",
                LaunchConfiguration(f"{side}_tool_parity"),
                " ",
                f"{side}_tool_baud_rate:=",
                LaunchConfiguration(f"{side}_tool_baud_rate"),
                " ",
                f"{side}_tool_stop_bits:=",
                LaunchConfiguration(f"{side}_tool_stop_bits"),
                " ",
                f"{side}_tool_rx_idle_chars:=",
                LaunchConfiguration(f"{side}_tool_rx_idle_chars"),
                " ",
                f"{side}_tool_tx_idle_chars:=",
                LaunchConfiguration(f"{side}_tool_tx_idle_chars"),
                " ",
                f"{side}_tool_device_name:=",
                LaunchConfiguration(f"{side}_tool_device_name"),
                " ",
                f"{side}_tool_tcp_port:=",
                LaunchConfiguration(f"{side}_tool_tcp_port"),
                " ",
                f"{side}_tool_voltage:=",
                LaunchConfiguration(f"{side}_tool_voltage"),
                " ",
                f"{side}_base_origin_xyz:=",
                "'",
                LaunchConfiguration(f"{side}_base_origin_xyz"),
                "'",
                " ",
                f"{side}_base_origin_rpy:=",
                "'",
                LaunchConfiguration(f"{side}_base_origin_rpy"),
                "'",
                " ",
            ]
        )

    robot_description_content = Command(xacro_args)

    def urdf_save(context):
        urdf_str = robot_description_content.perform(context)
        urdf_save_file = LaunchConfiguration("urdf_save_file").perform(context)
        os.makedirs(os.path.dirname(urdf_save_file), exist_ok=True)
        if os.path.exists(urdf_save_file):
            os.remove(urdf_save_file)
        with open(urdf_save_file, "x") as f:
            f.write(urdf_str)

    urdf_save_action = OpaqueFunction(
        function=urdf_save,
        condition=IfCondition(LaunchConfiguration("save_urdf")),
    )
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content, value_type=str
        )
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        on_exit=[Shutdown()],
    )

    return LaunchDescription([*declare_arguments(), urdf_save_action, rsp])
