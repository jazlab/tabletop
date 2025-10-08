import os
from collections.abc import Sequence
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, Node, SetROSLogDir
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def flatten_dict(d: Any, prefix: str = "", sep: str = ".") -> dict:
    if not isinstance(d, dict):
        return {prefix: d}
    result = {}
    if prefix:
        prefix += sep
    else:
        prefix = ""
    for k, v in d.items():
        result.update(flatten_dict(v, f"{prefix}{k}", sep))
    return result


def make_tf_publisher(
    name: str,
    position: Sequence[float] = (0.0, 0.0, 0.0),
    rpy: Sequence[float] = (0.0, 0.0, 0.0),
):
    node = Node(
        name=f"{name}_static_transform_publisher",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            str(position[0]),
            "--y",
            str(position[1]),
            "--z",
            str(position[2]),
            "--roll",
            str(rpy[0]),
            "--pitch",
            str(rpy[1]),
            "--yaw",
            str(rpy[2]),
            "--frame-id",
            "world",
            "--child-frame-id",
            name,
        ],
        output=LaunchConfiguration("flir_output"),
        on_exit=[Shutdown()],
    )
    return node


def make_camera_node(
    name: str,
    camera_type: str,
    **params,
) -> ComposableNode:
    parameter_file = PathJoinSubstitution(
        [
            FindPackageShare("spinnaker_camera_driver"),
            "config",
            camera_type + ".yaml",
        ]
    )

    params[f"{name}.image_raw"] = params.pop("image_transport_plugins")
    params = flatten_dict(params)

    # for plugin, plugin_params in transport_plugins.items():
    #     enable_plugins.append(f"image_transport/{plugin}")
    #     for k, v in plugin_params.items():
    #         params[f"{name}.image_raw.{plugin}.{k}"] = v

    # if enable_plugins:
    #     params[f"{name}.image_raw.enable_pub_plugins"] = enable_plugins

    # node = Node(
    #     package="spinnaker_camera_driver",
    #     executable="camera_driver_node",
    #     name=name,
    #     parameters=[
    #         params,
    #         {"parameter_file": parameter_file},
    #     ],
    #     log_level=LaunchConfiguration("flir_log_level"),
    #     output=LaunchConfiguration("flir_output"),
    #     remappings=[
    #         ("~/control", "/exposure_control/control"),
    #     ],
    #     on_exit=[Shutdown()],
    # )
    node = ComposableNode(
        package="spinnaker_camera_driver",
        plugin="spinnaker_camera_driver::CameraDriver",
        name=name,
        parameters=[
            params,
            {"parameter_file": parameter_file},
        ],
        remappings=[
            ("~/control", "/exposure_control/control"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    return node


def declare_arguments():
    return [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Using or not time from simulation",
        ),
        DeclareLaunchArgument(
            "calibrate",
            default_value="false",
            choices=["true", "false"],
            description="Whether to calibrate the camera",
        ),
        DeclareLaunchArgument(
            "calibrate_camera",
            default_value="null",
            description="Camera to calibrate (e.g. left_front_top_cam)",
        ),
        DeclareLaunchArgument(
            "calibration_size",
            default_value="9x11",
            description="Calibration grid dimensions (e.g. 9x11)",
        ),
        DeclareLaunchArgument(
            "calibration_square",
            default_value="0.015",
            description="Calibration square size (e.g. 0.015)",
        ),
        DeclareLaunchArgument(
            "flir_log_level",
            default_value="INFO",
            description="Flir log level",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        ),
        DeclareLaunchArgument(
            "flir_output",
            default_value="both",
            description="Flir output",
            choices=["log", "both", "screen", "own_log"],
        ),
    ]


def generate_launch_description():
    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # Flir multi-camera setup
    flir_config_file = os.path.join(
        get_package_share_directory("tabletop_server"), "config", "flir.yaml"
    )
    with open(flir_config_file, "r") as f:
        flir_config: dict = yaml.safe_load(f)

    flir_nodes: list[ComposableNode] = []
    # flir_nodes: list[Node] = []
    tf_nodes: list[Node] = []
    for config in flir_config["cameras"]:
        config: dict = flir_config["common"] | config
        if "pose" in config:
            pose: dict = config.pop("pose")
            tf_nodes.append(make_tf_publisher(config["name"], **pose))
        flir_nodes.append(make_camera_node(**config))

    flir_container = ComposableNodeContainer(
        name="flir_camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=flir_nodes,
        output=LaunchConfiguration("flir_output"),
        on_exit=[Shutdown()],
    )

    flir_calibration = Node(
        package="camera_calibration",
        executable="cameracalibrator",
        arguments=[
            "--size",
            LaunchConfiguration("calibration_size"),
            "--square",
            LaunchConfiguration("calibration_square"),
            "--no-service-check",
        ],
        condition=IfCondition(LaunchConfiguration("flir_calibrate")),
        output=LaunchConfiguration("flir_output"),
        on_exit=[Shutdown()],
    )

    launch_actions = [
        *declare_arguments(),
        set_ros_log_dir,
        *tf_nodes,
        flir_container,
        flir_calibration,
    ]

    return LaunchDescription(launch_actions)


# def main():
#     launch_logging_config.level = "DEBUG"
#     ls = LaunchService()
#     ld = generate_launch_description()
#     ls.include_launch_description(ld)
#     return ls.run()


# if __name__ == "__main__":
#     main()
