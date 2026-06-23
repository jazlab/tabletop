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
"""ROS 2 launch file for visualizing the robot model in RViz.

This launch file provides a standalone visualization setup for viewing and
interacting with the robot URDF model. It launches joint state publishers
(with optional GUI) and RViz for 3D visualization.

Useful for verifying URDF changes, testing joint limits, and debugging
robot description issues without connecting to hardware.

Launch Arguments:
    ur_type: UR robot series (default: ur5e)
    robot_ip: Robot IP for URDF generation (default: 192.168.12.20)
    description_launchfile: Path to robot state publisher launch file
    rviz_config_file: Path to RViz configuration file
    joint_state_publisher_gui: Launch interactive joint slider GUI (default: false)
    initial_joint_state: Initial joint positions as list

Nodes Launched:
    joint_state_publisher: Publishes static joint states (or GUI version)
    robot_state_publisher: Publishes robot transforms (via included launch)
    rviz2: 3D visualization (exits on close)

Example:
    # View robot with joint slider GUI
    ros2 launch tabletop_description dual_view_robot.launch.py joint_state_publisher_gui:=true

Author: Denis Stogl
"""

import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

INITIAL_JOINT_STATE_PARAMS = {
    "zeros": {
        "left_shoulder_pan_joint": 0,
        "left_shoulder_lift_joint": -math.pi / 2,
        "left_elbow_joint": math.pi / 2,
        "left_wrist_1_joint": 0,
        "left_wrist_2_joint": 0,
        "left_wrist_3_joint": 0,
        "right_shoulder_pan_joint": 0,
        "right_shoulder_lift_joint": -math.pi / 2,
        "right_elbow_joint": math.pi / 2,
        "right_wrist_1_joint": 0,
        "right_wrist_2_joint": 0,
        "right_wrist_3_joint": 0,
    },
}


def declare_arguments():
    """Declare launch arguments for robot visualization.

    Returns:
        List of DeclareLaunchArgument actions for configuring the
        robot type, description files, RViz config, and joint publisher options.
    """
    return [
        DeclareLaunchArgument(
            "description_launchfile",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_description"),
                    "launch",
                    "dual_rsp.launch.py",
                ]
            ),
            description="URDF/XACRO description file (absolute path) with the robot.",
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_description"),
                    "config",
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
    ]


def generate_launch_description():
    """Generate the launch description for robot visualization.

    Creates and configures nodes for robot visualization:
    - Joint state publisher (static or GUI-based)
    - Robot state publisher (via included launch file)
    - RViz2 with configured view

    The joint state publisher initializes joints to a neutral pose with
    shoulder_lift and elbow at 90 degrees.

    Returns:
        LaunchDescription containing all visualization nodes.
    """
    # Initialize Arguments
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    description_launchfile = LaunchConfiguration("description_launchfile")

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="log",
        parameters=[INITIAL_JOINT_STATE_PARAMS],
        condition=UnlessCondition(
            LaunchConfiguration("joint_state_publisher_gui")
        ),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[INITIAL_JOINT_STATE_PARAMS],
        output="log",
        condition=IfCondition(
            LaunchConfiguration("joint_state_publisher_gui")
        ),
    )

    robot_state_publisher_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(description_launchfile)
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        on_exit=[Shutdown(reason="rviz2_shutdown")],
    )

    nodes_to_start = [
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declare_arguments() + nodes_to_start)
