"""Launch file for dual UR robot kinematics calibration.

Launches per-arm calibration correction processes for both left and right
UR robots. Runs the ur_calibration package's calibration correction to
refine the robot kinematics parameters based on measured data.

Included Launch Files:
    - ur_calibration.launch.py (ur_calibration): Per-arm calibration
        (PushROSNamespace scoped to left/right)

Example:
    ros2 launch tabletop_description dual_ur_calibrate.launch.py \
        robot_mode:=real
"""

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
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import PushROSNamespace, SetROSLogDir
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
            default_value="real",
            choices=["real"],
            description="Robot to calibrate (real hardware only)",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Node log levels",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
    ]

    # Calibration is only meaningful against real hardware.
    left_robot_ip = EnvironmentVariable("LEFT_ROBOT_IP")
    right_robot_ip = EnvironmentVariable("RIGHT_ROBOT_IP")
    left_kinematics_params_file = PathJoinSubstitution(
        [
            FindPackageShare("tabletop_description"),
            "config",
            "left_ur5e_calibration.yaml",
        ]
    )
    right_kinematics_params_file = PathJoinSubstitution(
        [
            FindPackageShare("tabletop_description"),
            "config",
            "right_ur5e_calibration.yaml",
        ]
    )

    side_defaults = {
        "left": {
            "robot_ip": left_robot_ip,
            "kinematics_params_file": left_kinematics_params_file,
        },
        "right": {
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
                    f"{side}_robot_ip",
                    default_value=defaults["robot_ip"],
                    description=f"IP address of the {side} robot.",
                ),
                DeclareLaunchArgument(
                    f"{side}_kinematics_params_file",
                    default_value=defaults["kinematics_params_file"],
                    description=f"File to save kinematics calibration data for the {side} robot.",
                ),
            ]
        )

    return declared_arguments


def generate_launch_description():
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    per_arm_groups = []
    for side in ("left", "right"):
        ld = IncludeLaunchDescription(
            launch_description_source=AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur_calibration"),
                        "launch",
                        "calibration_correction.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "robot_ip": LaunchConfiguration(f"{side}_robot_ip"),
                "target_filename": LaunchConfiguration(
                    f"{side}_kinematics_params_file"
                ),
            }.items(),
        )

        per_arm_groups.append(
            GroupAction(
                [PushROSNamespace(side), ld],
                scoped=True,
                forwarding=True,
            )
        )

    return LaunchDescription(
        [set_ros_log_dir, *declare_arguments(), *per_arm_groups]
    )
