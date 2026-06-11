import os
from collections.abc import Sequence
from copy import deepcopy
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import (
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, Node, SetROSLogDir
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

NODE_NAME = "cam_sync"


def declare_arguments():
    return [
        DeclareLaunchArgument(
            "camera_param_dir",
            default_value=PathJoinSubstitution(
                [FindPackageShare("tabletop_rig"), "config"]
            ),
            description="Directory to look for the camera parameter files",
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
    )


def launch_setup(context, *args, **kwargs):
    # Get launch configurations
    camera_param_dir = LaunchConfiguration("camera_param_dir").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    use_sim_time = bool(LaunchConfiguration("use_sim_time").perform(context))

    # Load the synchronized FLIR config.
    cfg_path = os.path.join(
        get_package_share_directory("tabletop_rig"),
        "config",
        "flir_synchronized.yaml",
    )
    with open(cfg_path, "r") as f:
        config: dict = yaml.safe_load(f)

    # Required keys in config file
    cameras = config["cameras"]
    camera_types = config["camera_types"]
    camera_params = config["camera_params"]
    enable_exposure_controllers = config["enable_exposure_controllers"]

    # Optional keys in config file
    camera_params_common = config.get("camera_params_common", {})
    image_transport_plugins = config.get("image_transport_plugins", None)
    exp_ctrl_params_common = config.get(
        "exposure_controller_params_common", {}
    )
    exp_ctrl_params = config.get("exposure_controller_params", {})
    exp_ctrl_camera_param_overrides = config.get(
        "exposure_controller_camera_param_overrides", {}
    )
    camera_poses = config.get("camera_poses", {})

    # Top-level driver parameters: which sub-objects to instantiate, plus any
    # driver-wide options from the yaml's `driver:` block.
    driver_params: dict = {"cameras": cameras, "use_sim_time": use_sim_time}

    # Per-camera params (common merged with per-camera override).
    tf_nodes: list[Node] = []
    for name in cameras:
        params = camera_params[name]
        assert "serial_number" in params
        params = flatten_dict(camera_params_common) | flatten_dict(params)
        params = deepcopy(params)

        params["frame_id"] = name
        params["parameter_file"] = os.path.join(
            camera_param_dir,
            f"{camera_types[name]}.yaml",
        )

        if enable_exposure_controllers:
            exp_name = f"{name}.exposure_controller"
            driver_params.setdefault("exposure_controllers", []).append(
                exp_name
            )

            params["exposure_controller_name"] = exp_name
            params.update(flatten_dict(exp_ctrl_camera_param_overrides))

            exp_params = exp_ctrl_params.get(exp_name, {})
            exp_params = flatten_dict(exp_ctrl_params_common) | flatten_dict(
                exp_params
            )
            assert "type" in exp_params
            driver_params.update(flatten_dict(exp_params, prefix=exp_name))

        driver_params.update(flatten_dict(params, prefix=name))

        # Need to set image transport plugins to '<full.topic.name>.<param>'
        if image_transport_plugins is not None:
            driver_params.update(
                flatten_dict(
                    image_transport_plugins,
                    prefix=f"{NODE_NAME}.{name}.image_raw",
                )
            )

        if name in camera_poses:
            pose_kwargs = camera_poses[name]
            tf_nodes.append(make_tf_publisher(name, **pose_kwargs))

    # Exposure controller params
    # if enable_exposure_controllers:
    #     exp_controllers = [f"{n}.exposure_controller" for n in cameras]
    #     driver_params["exposure_controllers"] = exp_controllers
    #     for name in exp_controllers:
    #         params = exp_ctrl_params.get(name, {})
    #         params = flatten_dict(exp_ctrl_params_common) | flatten_dict(
    #             params
    #         )
    #         assert "type" in params
    #         driver_params.update(flatten_dict(params, prefix=name))

    for k, v in driver_params.items():
        print(f"{k}: {v}")

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
                name=NODE_NAME,
                parameters=[driver_params],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        ros_arguments=["--log-level", log_level],
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
