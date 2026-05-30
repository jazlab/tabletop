import os
from collections.abc import Sequence
from copy import deepcopy
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
            description=(
                "Camera to run (e.g. left_front_top_cam), or 'all'. "
                "Because the synchronized driver instantiates every camera "
                "inside a single node, selecting one will simply omit the "
                "others from the driver's camera list."
            ),
        ),
        DeclareLaunchArgument(
            "camera_type",
            default_value="blackfly_s",
            description="Camera type (e.g. blackfly_s)",
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
    return Node(
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


def build_camera_params(
    name: str,
    serial: str,
    cam_cfg: dict,
    parameter_file,
) -> dict:
    """Flatten one camera's parameters under the `{name}.` prefix.

    The synchronized driver expects every per-camera setting to be passed as
    `{name}.{key}` (and nested keys are dotted further, e.g.
    `{name}.image_raw.compressed.jpeg_quality`).
    """
    cam_cfg = deepcopy(cam_cfg)
    # Image-transport plugin options are addressed by the publisher topic
    # (image_raw) rather than as plain camera params.
    if "image_transport_plugins" in cam_cfg:
        cam_cfg["image_raw"] = cam_cfg.pop("image_transport_plugins")
    cam_cfg["serial_number"] = serial
    cam_cfg["frame_id"] = name
    cam_cfg["parameter_file"] = parameter_file
    cam_cfg["exposure_controller_name"] = f"{name}.exposure_controller"
    return flatten_dict(cam_cfg, prefix=name)


def launch_setup(context, *args, **kwargs):
    # Load the synchronized FLIR config.
    cfg_path = os.path.join(
        get_package_share_directory("tabletop_rig"),
        "config",
        "flir_synchronized.yaml",
    )
    with open(cfg_path, "r") as f:
        flir_config: dict = yaml.safe_load(f)

    all_cameras = [c["name"] for c in flir_config["cameras"]]
    camera_arg = LaunchConfiguration("camera").perform(context)
    if camera_arg != "all" and camera_arg not in all_cameras:
        raise ValueError(
            f"Camera {camera_arg!r} not found in flir_synchronized.yaml "
            f"(known: {all_cameras})"
        )
    selected = [
        c
        for c in flir_config["cameras"]
        if camera_arg == "all" or c["name"] == camera_arg
    ]

    # Path to the per-camera-type tuning yaml shipped with
    # spinnaker_camera_driver (e.g. blackfly_s.yaml).
    parameter_file = PathJoinSubstitution(
        [
            FindPackageShare("spinnaker_camera_driver"),
            "config",
            [LaunchConfiguration("camera_type"), ".yaml"],
        ]
    )

    cam_names = [c["name"] for c in selected]
    exp_ctrl_names = [f"{n}.exposure_controller" for n in cam_names]

    # Top-level driver parameters: which sub-objects to instantiate, plus any
    # driver-wide options from the yaml's `driver:` block.
    driver_params: dict = {
        "cameras": cam_names,
        "exposure_controllers": exp_ctrl_names,
    }
    driver_params.update(flatten_dict(flir_config.get("driver", {})))

    # Replicate the exposure_controller template once per camera.
    exp_template = flir_config.get("exposure_controller", {})
    for ctrl_name in exp_ctrl_names:
        driver_params.update(flatten_dict(exp_template, prefix=ctrl_name))

    # Per-camera params (common merged with per-camera override).
    common = flir_config.get("common", {})
    tf_nodes: list[Node] = []
    for entry in selected:
        entry = deepcopy(entry)
        name = entry.pop("name")
        serial = entry.pop("serial_number")
        pose = entry.pop("pose", None)
        cam_cfg = deepcopy(common) | entry
        driver_params.update(
            build_camera_params(name, serial, cam_cfg, parameter_file)
        )

        if pose is not None:
            condition = IfCondition(
                OrSubstitution(
                    EqualsSubstitution(LaunchConfiguration("camera"), "all"),
                    EqualsSubstitution(LaunchConfiguration("camera"), name),
                )
            )
            tf_nodes.append(
                make_tf_publisher(name, condition=condition, **pose)
            )

    container = ComposableNodeContainer(
        name="flir_camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="both",
        composable_node_descriptions=[
            ComposableNode(
                package="spinnaker_synchronized_camera_driver",
                plugin=(
                    "spinnaker_synchronized_camera_driver::"
                    "SynchronizedCameraDriver"
                ),
                name="cam_sync",
                parameters=[driver_params],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        on_exit=[Shutdown()],
    )

    return [*tf_nodes, container]


def generate_launch_description():
    return LaunchDescription(
        [
            *declare_arguments(),
            SetROSLogDir(LaunchLogDir()),
            OpaqueFunction(function=launch_setup),
        ]
    )
