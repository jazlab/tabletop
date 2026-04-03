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
"""ROS 2 launch file for MoveIt 2 motion planning.

This launch file starts the MoveIt move_group node with the TableTop robot
configuration. It waits for the robot description to be available before
launching to ensure proper initialization.

The launch file configures:
- MoveIt semantic robot description (SRDF) from xacro
- Warehouse database for storing robot states (SQLite backend)
- Optional RViz visualization with MoveIt plugin

Launch Arguments:
    launch_rviz: Start RViz with MoveIt configuration (default: true)
    rviz_config_file: Path to RViz config file
    warehouse_sqlite_path: Path to SQLite database for warehouse
    use_sim_time: Use simulated time (default: false)
    publish_robot_description_semantic: Publish SRDF to topic (default: true)

Nodes Launched:
    wait_for_robot_description: Blocks until robot_description is available
    move_group: MoveIt planning and execution node
    rviz2_moveit: RViz with MoveIt configuration (optional)

Example:
    ros2 launch tabletop_moveit_config moveit.launch.py

Author: Felix Exner
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def declare_arguments():
    """Declare launch arguments for MoveIt configuration.

    Returns:
        LaunchDescription containing argument declarations for RViz,
        robot type, warehouse database, and simulation time settings.
    """
    return [
        DeclareLaunchArgument(
            "robot_name",
            default_value="tabletop",
            description="Robot name for MoveIt SRDF",
        ),
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="true",
            description="MoveGroup publishes robot description semantic",
        ),
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value="/root/ws/.ros/warehouse_ros.sqlite",
            description="Path where the warehouse database should be stored",
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tabletop_moveit_config"),
                    "config",
                    "moveit.rviz",
                ]
            ),
            description="Path to RViz config file",
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
            description="Use simulated time",
        ),
    ]


def generate_launch_description():
    """Generate the launch description for MoveIt motion planning.

    Builds MoveIt configuration using MoveItConfigsBuilder, sets up
    warehouse database connection, and creates nodes for motion planning.
    Uses event handlers to ensure move_group and RViz start only after
    robot description is available.

    Returns:
        LaunchDescription with wait_for_robot_description node and
        event-triggered move_group and RViz nodes.
    """
    # MoveIt Config
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="tabletop", package_name="tabletop_moveit_config"
        )
        .robot_description_semantic(
            file_path="srdf/dual_tabletop.srdf.xacro",
            mappings={"name": LaunchConfiguration("robot_name")},
        )
        .planning_scene_monitor(
            # publish_robot_description=True,
            # publish_planning_scene=False,
            # publish_geometry_updates=False,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "pilz_industrial_motion_planner"],
        )
        # .moveit_cpp(
        #     file_path="config/moveit_cpp.yaml",
        # )
        .to_moveit_configs()
    )

    for pipeline in moveit_config.planning_pipelines.keys():
        if pipeline in ("planning_pipelines", "default_planning_pipeline"):
            continue
        moveit_config.planning_pipelines[pipeline]["response_adapters"] = [
            "default_planning_response_adapters/AddTimeOptimalParameterization",
            "default_planning_response_adapters/ValidateSolution",
            "default_planning_response_adapters/DisplayMotionPath",
        ]

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": LaunchConfiguration("warehouse_sqlite_path"),
    }

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "publish_robot_description_semantic": LaunchConfiguration(
                    "publish_robot_description_semantic"
                ),
            },
        ],
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        on_exit=[Shutdown()],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        parameters=[
            # moveit_config.robot_description,
            # moveit_config.robot_description_semantic,
            # moveit_config.robot_description_kinematics,
            # moveit_config.planning_pipelines,
            # moveit_config.joint_limits,
            moveit_config.to_dict(),  # TODO: Figure out which one to use
            warehouse_ros_config,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=[
            "-d",
            LaunchConfiguration("rviz_config_file"),
            "-l",  # -l for ogre log
        ],
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        cwd=LaunchLogDir(),
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        on_exit=[Shutdown()],
    )

    robot_description_ready_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_robot_description,
            on_exit=[move_group, rviz],
        )
    )

    return LaunchDescription(
        [
            *declare_arguments(),
            wait_robot_description,
            robot_description_ready_handler,
        ]
    )
