import inspect
import os
from collections.abc import Callable, Coroutine
from typing import Any, Optional, Protocol

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from rclpy.client import Client
from rclpy.node import Node
from tf_transformations import (
    quaternion_from_euler as quaternion_from_euler_tf,
)
from tf_transformations import (
    quaternion_from_matrix,
    translation_from_matrix,
)


class SrvType(Protocol):
    Request: Any
    Response: Any


class SrvTypeRequest(Protocol):
    pass


class SrvTypeResponse(Protocol):
    success: bool


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path) as file:
        return yaml.safe_load(file)


def save_yaml(file_path, data):
    with open(file_path, "w") as file:
        yaml.dump(data, file, default_flow_style=True, sort_keys=False)


def string_to_bool(value: str) -> bool:
    if value == "true":
        return True
    elif value == "false":
        return False
    else:
        raise ValueError(
            f"Boolean launch argument {value} must be 'true' or 'false'"
        )


def quaternion_from_euler(
    roll: float, pitch: float, yaw: float, axes: str = "sxyz"
) -> Quaternion:
    """
    Convert roll, pitch, yaw angles (in radians) to a geometry_msgs/Quaternion message.
    """
    quaternion = quaternion_from_euler_tf(roll, pitch, yaw, axes)
    return Quaternion(
        x=quaternion[0],
        y=quaternion[1],
        z=quaternion[2],
        w=quaternion[3],
    )


def pose_from_matrix(matrix: np.ndarray) -> Pose:
    translation = translation_from_matrix(matrix)
    quaternion = quaternion_from_matrix(matrix)
    pose = Pose()
    pose.position.x = translation[0]
    pose.position.y = translation[1]
    pose.position.z = translation[2]
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose


def pose_stamped_from_params(node: Node, prefix: str):
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = node.get_parameter(
        f"{prefix}.header.frame_id"
    ).value
    pose = Pose()
    pose.position.x = node.get_parameter(f"{prefix}.pose.position.x").value
    pose.position.y = node.get_parameter(f"{prefix}.pose.position.y").value
    pose.position.z = node.get_parameter(f"{prefix}.pose.position.z").value
    pose.orientation = quaternion_from_euler(
        node.get_parameter(f"{prefix}.pose.orientation.roll").value,  # type: ignore
        node.get_parameter(f"{prefix}.pose.orientation.pitch").value,  # type: ignore
        node.get_parameter(f"{prefix}.pose.orientation.yaw").value,  # type: ignore
    )
    pose_stamped.pose = pose
    return pose_stamped


def validate_service_response(
    response: Optional[SrvTypeResponse],
    service_client: Client,
) -> SrvTypeResponse:
    if response is None:
        error_msg = f"{service_client.service_name} service call timed out!"
        raise TimeoutError(error_msg)
    elif not response.success:
        error_msg = f"{service_client.service_name} service call failed!"
        raise ServiceCallError(error_msg)
    else:
        return response


def create_coroutine_wrapper(
    fn: Callable[..., Any],
) -> Callable[..., Coroutine[Any, Any, Any]]:
    """
    Wrap a function in a coroutine.
    """
    if inspect.iscoroutinefunction(fn):
        raise ValueError("Function is already a coroutine")
    else:

        async def wrapper(*args, **kwargs):
            nonlocal fn
            return fn(*args, **kwargs)

        return wrapper


class MaxAttemptsReachedError(Exception):
    pass


class ServiceCallError(Exception):
    pass
