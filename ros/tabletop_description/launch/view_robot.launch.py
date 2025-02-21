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

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def declare_arguments():
    return [
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
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.12.11",
            description="The IP address of the robot",
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
            description="URDF/XACRO description file (absolute path) with the robot.",
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
            "joint_state_publisher_gui",
            default_value="false",
            description="Whether to launch the joint state publisher GUI.",
        ),
        DeclareLaunchArgument(
            "initial_joint_state",
            default_value="[-1.5707, 1.5707, 0, 0, 0, 0]",
            description="The initial joint state to set.",
        ),
    ]


def generate_launch_description():
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    description_launchfile = LaunchConfiguration("description_launchfile")

    joint_state_publisher_params = {
        "zeros": {
            "shoulder_pan_joint": 0,
            "shoulder_lift_joint": -math.pi / 2,
            "elbow_joint": math.pi / 2,
            "wrist_1_joint": 0,
            "wrist_2_joint": 0,
            "wrist_3_joint": 0,
        },
    }

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="log",
        parameters=[joint_state_publisher_params],
        condition=UnlessCondition(
            LaunchConfiguration("joint_state_publisher_gui")
        ),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[joint_state_publisher_params],
        output="log",
        condition=IfCondition(
            LaunchConfiguration("joint_state_publisher_gui")
        ),
    )

    robot_state_publisher_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(description_launchfile),
        launch_arguments={
            "robot_ip": robot_ip,
            "ur_type": ur_type,
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declare_arguments() + nodes_to_start)
