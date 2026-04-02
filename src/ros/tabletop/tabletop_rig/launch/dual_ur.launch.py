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

SHARED_CONTROLLERS_ACTIVE = ["joint_state_broadcaster"]
SHARED_CONTROLLERS_INACTIVE = []
PER_ARM_CONTROLLERS_ACTIVE = [
    "io_and_status_controller",
    "speed_scaling_state_broadcaster",
    "force_torque_sensor_broadcaster",
    "tcp_pose_broadcaster",
    "ur_configuration_controller",
]
PER_ARM_CONTROLLERS_INACTIVE = [
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


def declare_arguments():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_mode",
            default_value="mock",
            choices=["mock", "ursim", "real"],
            description="Whether to use the mock robot, URSim, or real robot",
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
            "ur_type",
            default_value="ur5e",
            description="Type/series of both UR robots (this launch file only works with two of the same robot).",
            choices=UR_TYPE_CHOICES,
        ),
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control.",
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
                LaunchConfiguration("ur_type"),
                "_update_rate.yaml",
            ],
            description="Update rate config.",
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="false", description="Launch RViz?"
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
            description="RViz config file (absolute path) to use when launching rviz.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Node log levels",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use simulated time",
        ),
    ]

    # Conditional substitutions
    reverse_ip = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration("robot_mode"), "real"),
        EnvironmentVariable("REVERSE_IP"),
        EnvironmentVariable("SIM_REVERSE_IP"),
    )
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
    left_kinematics_params_file = PathJoinSubstitution(
        [
            FindPackageShare("tabletop_description"),
            "config",
            IfElseSubstitution(
                EqualsSubstitution(LaunchConfiguration("robot_mode"), "ursim"),
                "left_ursim_calibration.yaml",
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
                "right_ursim_calibration.yaml",
                "right_ur5e_calibration.yaml",
            ),
        ]
    )

    # Shared arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_ip",
            default_value=reverse_ip,
            description="IP for robot-to-driver communication.",
        )
    )

    side_defaults = {
        "left": {
            "tf_prefix": "left_",
            "robot_ip": left_robot_ip,
            "kinematics_params_file": left_kinematics_params_file,
        },
        "right": {
            "tf_prefix": "right_",
            "robot_ip": right_robot_ip,
            "kinematics_params_file": right_kinematics_params_file,
        },
    }

    # Per-arm arguments
    for side in ("left", "right"):
        defaults = side_defaults[side]
        declared_arguments.extend(
            [
                DeclareLaunchArgument(
                    f"{side}_tf_prefix",
                    default_value=defaults["tf_prefix"],
                    description=f"tf_prefix of the joint names for the {side} robot",
                ),
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
                    f"{side}_use_tool_communication",
                    default_value="false",
                    description=f"Enable tool communication for the {side} robot.",
                ),
            ]
        )

    return declared_arguments


def controller_spawner(controllers, active):
    inactive_flag = ["--inactive"] if not active else []
    return Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=[
            "--controller-manager",
            "controller_manager",
            "--controller-manager-timeout",
            LaunchConfiguration("controller_spawner_timeout"),
        ]
        + inactive_flag
        + controllers,
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
    )


def setup_controller_spawners(context):
    use_mock_hardware = EqualsSubstitution(
        LaunchConfiguration("robot_mode"), "mock"
    )

    active_controllers = []
    inactive_controllers = []
    active_controllers.extend(SHARED_CONTROLLERS_ACTIVE)
    inactive_controllers.extend(SHARED_CONTROLLERS_INACTIVE)

    for side in ("left", "right"):
        active_controllers.extend(
            [f"{side}_{x}" for x in PER_ARM_CONTROLLERS_ACTIVE]
        )
        inactive_controllers.extend(
            [f"{side}_{x}" for x in PER_ARM_CONTROLLERS_INACTIVE]
        )

        if (
            LaunchConfiguration("activate_joint_controller").perform(context)
            == "true"
        ):
            prefixed = f"{side}_{LaunchConfiguration('initial_joint_controller').perform(context)}"
            active_controllers.append(prefixed)
            inactive_controllers.remove(prefixed)

        if use_mock_hardware.perform(context) == "true":
            active_controllers.remove(f"{side}_tcp_pose_broadcaster")

    return [
        controller_spawner(active_controllers, active=True),
        controller_spawner(inactive_controllers, active=False),
    ]


def generate_launch_description():
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    use_mock_hardware = EqualsSubstitution(
        LaunchConfiguration("robot_mode"), "mock"
    )

    # Shared controller manager node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            LaunchConfiguration("update_rate_config_file"),
            ParameterFile(
                LaunchConfiguration("controllers_file"), allow_substs=True
            ),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        on_exit=[Shutdown()],
    )

    # Dual robot state publisher
    rsp = GroupAction(
        [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    LaunchConfiguration("description_launchfile")
                ),
                launch_arguments={
                    "use_mock_hardware": use_mock_hardware,
                    "headless_mode": LaunchConfiguration("headless_mode"),
                    "reverse_ip": LaunchConfiguration("reverse_ip"),
                    "left_ur_type": LaunchConfiguration("ur_type"),
                    "right_ur_type": LaunchConfiguration("ur_type"),
                    "left_tf_prefix": LaunchConfiguration("left_tf_prefix"),
                    "right_tf_prefix": LaunchConfiguration("right_tf_prefix"),
                    "left_robot_ip": LaunchConfiguration("left_robot_ip"),
                    "right_robot_ip": LaunchConfiguration("right_robot_ip"),
                    "left_kinematics_params_file": LaunchConfiguration(
                        "left_kinematics_params_file"
                    ),
                    "right_kinematics_params_file": LaunchConfiguration(
                        "right_kinematics_params_file"
                    ),
                }.items(),
            )
        ],
        scoped=True,
        forwarding=True,
    )

    # Controller spawners
    controller_spawners = OpaqueFunction(function=setup_controller_spawners)

    # Rviz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        on_exit=[Shutdown()],
    )

    # Per-arm namespaced nodes
    per_arm_groups = []
    for side in ("left", "right"):
        dashboard_client = IncludeLaunchDescription(
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
                "robot_ip": LaunchConfiguration(f"{side}_robot_ip"),
            }.items(),
            condition=IfCondition(
                AndSubstitution(
                    LaunchConfiguration("launch_dashboard_client"),
                    NotSubstitution(use_mock_hardware),
                )
            ),
        )

        mock_dashboard_client = Node(
            package="tabletop_rig",
            executable="mock_dashboard_client",
            output="both",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ros_arguments=[
                "--log-level",
                LaunchConfiguration("log_level"),
            ],
            condition=IfCondition(
                AndSubstitution(
                    LaunchConfiguration("launch_dashboard_client"),
                    use_mock_hardware,
                )
            ),
            on_exit=[Shutdown()],
        )

        robot_state_helper = Node(
            package="ur_robot_driver",
            executable="robot_state_helper",
            name="ur_robot_state_helper",
            output="both",
            parameters=[
                {"headless_mode": LaunchConfiguration("headless_mode")},
                {"robot_ip": LaunchConfiguration(f"{side}_robot_ip")},
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            ros_arguments=[
                "--log-level",
                LaunchConfiguration("log_level"),
            ],
            condition=UnlessCondition(use_mock_hardware),
            on_exit=[Shutdown()],
        )

        tool_communication = Node(
            package="ur_robot_driver",
            executable="tool_communication.py",
            name="ur_tool_comm",
            output="both",
            parameters=[
                {
                    "robot_ip": LaunchConfiguration(f"{side}_robot_ip"),
                    "tcp_port": LaunchConfiguration(f"{side}_tool_tcp_port"),
                    "device_name": LaunchConfiguration(
                        f"{side}_tool_device_name"
                    ),
                },
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            ros_arguments=[
                "--log-level",
                LaunchConfiguration("log_level"),
            ],
            condition=IfCondition(
                LaunchConfiguration(f"{side}_use_tool_communication")
            ),
            on_exit=[Shutdown()],
        )

        urscript_interface = Node(
            package="ur_robot_driver",
            executable="urscript_interface",
            output="both",
            parameters=[
                {"robot_ip": LaunchConfiguration(f"{side}_robot_ip")},
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            ros_arguments=[
                "--log-level",
                LaunchConfiguration("log_level"),
            ],
            condition=UnlessCondition(use_mock_hardware),
            on_exit=[Shutdown()],
        )

        other_side = "right" if side == "left" else "left"

        consistent_controllers = (
            SHARED_CONTROLLERS_ACTIVE
            + [f"{side}_{x}" for x in PER_ARM_CONTROLLERS_ACTIVE]
            + [
                f"{other_side}_{x}"
                for x in (
                    PER_ARM_CONTROLLERS_ACTIVE + PER_ARM_CONTROLLERS_INACTIVE
                )
            ],
        )

        controller_stopper = Node(
            package="ur_robot_driver",
            executable="controller_stopper_node",
            name="controller_stopper",
            output="both",
            emulate_tty=True,
            parameters=[
                {"headless_mode": LaunchConfiguration("headless_mode")},
                {
                    "joint_controller_active": LaunchConfiguration(
                        "activate_joint_controller"
                    )
                },
                {"consistent_controllers": consistent_controllers},
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            ros_arguments=[
                "--log-level",
                LaunchConfiguration("log_level"),
            ],
            condition=UnlessCondition(use_mock_hardware),
            on_exit=[Shutdown()],
        )

        trajectory_until = Node(
            package="ur_robot_driver",
            executable="trajectory_until_node",
            name="trajectory_until_node",
            output="both",
            parameters=[
                {
                    "motion_controller": [
                        side,
                        "_",
                        LaunchConfiguration("initial_joint_controller"),
                    ],
                },
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            ros_arguments=[
                "--log-level",
                LaunchConfiguration("log_level"),
            ],
            on_exit=[Shutdown()],
        )

        per_arm_groups.append(
            GroupAction(
                [
                    PushROSNamespace(side),
                    dashboard_client,
                    mock_dashboard_client,
                    robot_state_helper,
                    tool_communication,
                    urscript_interface,
                    controller_stopper,
                    trajectory_until,
                ],
                scoped=True,
                forwarding=True,
            )
        )

    return LaunchDescription(
        [
            set_ros_log_dir,
            *declare_arguments(),
            controller_manager,
            rsp,
            controller_spawners,
            *per_arm_groups,
            rviz,
        ]
    )
