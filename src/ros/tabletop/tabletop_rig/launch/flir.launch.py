import os
from collections.abc import Sequence
from copy import copy
from typing import Any, Optional

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import Condition, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    LaunchLogDir,
    OrSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, Node, SetROSLogDir
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def declare_arguments():
    return [
        DeclareLaunchArgument(
            "camera",
            default_value="all",
            description="Camera to calibrate (e.g. left_front_top_cam), or 'all'",
        ),
        DeclareLaunchArgument(
            "camera_type",
            default_value="blackfly_s",
            description="Camera type (e.g. blackfly_s)",
        ),
        DeclareLaunchArgument(
            "factory_reset",
            default_value="false",
            description="Factory reset the cameras (resets the camera to factory settings)",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "device_reset",
            default_value="false",
            description="Device reset the cameras (powercycles the camera)",
            choices=["true", "false"],
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
            choices=["true", "false"],
            description="Use simulated time",
        ),
    ]


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
    condition: Optional[Condition] = None,
):
    node = Node(
        name=f"{name}_static_transform_publisher",
        package="tf2_ros",
        executable="static_transform_publisher",
        output="both",
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
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        on_exit=[Shutdown()],
        condition=condition,
    )
    return node


def make_camera_node(
    name: str,
    condition: Optional[Condition] = None,
    **params,
) -> ComposableNode:
    # Dynamically modify and flatten the parameters
    params = copy(params)
    params[f"{name}.image_raw"] = params.pop("image_transport_plugins")
    params = flatten_dict(params)

    # Load the camera parameter file
    parameter_file = PathJoinSubstitution(
        [
            FindPackageShare("spinnaker_camera_driver"),
            "config",
            [LaunchConfiguration("camera_type"), ".yaml"],
        ]
    )
    # node = Node(
    #     package="spinnaker_camera_driver",
    #     executable="camera_driver_node",
    #     name=name,
    #     parameters=[
    #         params,
    #         {"parameter_file": parameter_file},
    #     ],
    #     log_level=LaunchConfiguration("flir_log_level"),
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
            {"factory_reset": LaunchConfiguration("factory_reset")},
            {"device_reset": LaunchConfiguration("device_reset")},
            {"parameter_file": parameter_file},
        ],
        remappings=[
            ("~/control", "/exposure_control/control"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
        condition=condition,
    )
    return node


def check_camera_fn(context, cameras: list[str]):
    camera = LaunchConfiguration("camera").perform(context)
    if camera != "all" and camera not in cameras:
        raise ValueError(f"Camera {camera} not found in flir.yaml")


def generate_launch_description():
    # Set ROS Log Directory
    set_ros_log_dir = SetROSLogDir(LaunchLogDir())

    # Load flir config
    flir_config_file = os.path.join(
        get_package_share_directory("tabletop_rig"), "config", "flir.yaml"
    )
    with open(flir_config_file, "r") as f:
        flir_config: dict = yaml.safe_load(f)

    cameras = [config["name"] for config in flir_config["cameras"]]
    check_camera = OpaqueFunction(function=check_camera_fn, args=[cameras])

    flir_nodes: list[ComposableNode] = []
    tf_nodes: list[Node] = []
    for config in flir_config["cameras"]:
        config: dict = flir_config["common"] | config
        name = config.pop("name")
        condition = IfCondition(
            OrSubstitution(
                EqualsSubstitution(LaunchConfiguration("camera"), "all"),
                EqualsSubstitution(LaunchConfiguration("camera"), name),
            )
        )
        if "pose" in config:
            pose: dict = config.pop("pose")
            tf_nodes.append(
                make_tf_publisher(name, condition=condition, **pose)
            )

        flir_nodes.append(
            make_camera_node(name, condition=condition, **config)
        )

    flir_container = ComposableNodeContainer(
        name="flir_camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="both",
        composable_node_descriptions=flir_nodes,
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        on_exit=[Shutdown()],
    )

    launch_actions = [
        *declare_arguments(),
        set_ros_log_dir,
        check_camera,
        *tf_nodes,
        flir_container,
    ]

    return LaunchDescription(launch_actions)
