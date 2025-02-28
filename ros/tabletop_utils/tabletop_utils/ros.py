import os
from collections.abc import Iterable, Mapping
from typing import Any, Optional, Protocol

import numpy as np
import trimesh
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    MoveItErrorCodes,
    ObjectColor,
)
from rclpy.client import Client
from shape_msgs.msg import Mesh, MeshTriangle
from std_msgs.msg import ColorRGBA, Header
from tf_transformations import (
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_from_matrix,
    translation_matrix,
)

from tabletop_utils.common import is_iterable

# Constants
"""
MoveIt error code map from error code to string, for logging.
"""
moveit_error_code_map = {
    MoveItErrorCodes.SUCCESS: "SUCCESS",
    MoveItErrorCodes.UNDEFINED: "UNDEFINED",
    MoveItErrorCodes.FAILURE: "FAILURE",
    MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
    MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
    MoveItErrorCodes.PREEMPTED: "PREEMPTED",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    MoveItErrorCodes.START_STATE_INVALID: "START_STATE_INVALID",
    MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "GOAL_CONSTRAINTS_VIOLATED",
    MoveItErrorCodes.GOAL_STATE_INVALID: "GOAL_STATE_INVALID",
    MoveItErrorCodes.UNRECOGNIZED_GOAL_TYPE: "UNRECOGNIZED_GOAL_TYPE",
    MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
    MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
    MoveItErrorCodes.INVALID_LINK_NAME: "INVALID_LINK_NAME",
    MoveItErrorCodes.INVALID_OBJECT_NAME: "INVALID_OBJECT_NAME",
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
    MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: "COLLISION_CHECKING_UNAVAILABLE",
    MoveItErrorCodes.ROBOT_STATE_STALE: "ROBOT_STATE_STALE",
    MoveItErrorCodes.SENSOR_INFO_STALE: "SENSOR_INFO_STALE",
    MoveItErrorCodes.COMMUNICATION_FAILURE: "COMMUNICATION_FAILURE",
    MoveItErrorCodes.CRASH: "CRASH",
    MoveItErrorCodes.ABORT: "ABORT",
    MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
}

"""
RGBA color map from color name to RGBA tuple.
"""
rgba_map = {
    "red": (1.0, 0.0, 0.0, 1.0),
    "green": (0.0, 1.0, 0.0, 1.0),
    "blue": (0.0, 0.0, 1.0, 1.0),
    "yellow": (1.0, 1.0, 0.0, 1.0),
    "cyan": (0.0, 1.0, 1.0, 1.0),
    "magenta": (1.0, 0.0, 1.0, 1.0),
    "white": (1.0, 1.0, 1.0, 1.0),
    "black": (0.0, 0.0, 0.0, 1.0),
    "gray": (0.5, 0.5, 0.5, 1.0),
    "orange": (1.0, 0.5, 0.0, 1.0),
    "purple": (0.5, 0.0, 0.5, 1.0),
    "pink": (1.0, 0.75, 0.8, 1.0),
    "brown": (0.6, 0.4, 0.2, 1.0),
    "teal": (0.0, 0.5, 0.5, 1.0),
    "olive": (0.5, 0.5, 0.0, 1.0),
    "navy": (0.0, 0.0, 0.5, 1.0),
    "maroon": (0.5, 0.0, 0.0, 1.0),
    "lime": (0.75, 1.0, 0.0, 1.0),
    "coral": (1.0, 0.5, 0.31, 1.0),
}

"""
Collision object operation map from operation name to collision object operation.
"""
collision_object_operation_map = {
    "add": CollisionObject.ADD,
    "remove": CollisionObject.REMOVE,
    "append": CollisionObject.APPEND,
    "move": CollisionObject.MOVE,
}


class SrvTypeRequest(Protocol):
    pass


class SrvTypeResponse(Protocol):
    success: bool


# Protocol definitions
class SrvType(Protocol):
    Request: Any
    Response: Any


# Exception definitions
class MaxAttemptsReachedError(Exception):
    pass


class ServiceCallError(Exception):
    pass


# ROS2 utility functions
def load_yaml_from_package(
    package_name: str, file_path: str
) -> dict[str, Any]:
    """
    Load a YAML file from a package.
    """
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path) as file:
        return yaml.safe_load(file)


def validate_service_response(
    response: Optional[SrvTypeResponse],
    service_client: Client,
) -> None:
    """
    Validate the response from a service call.

    Args:
        response: The response from a service call.
        service_client: The client that made the service call.

    Returns:
        The response from the service call.

    Raises:
        TimeoutError: If the service call timed out.
        ServiceCallError: If the service call failed.
    """
    if response is None:
        error_msg = f"{service_client.service_name} service call timed out!"
        raise TimeoutError(error_msg)
    elif hasattr(response, "success") and not response.success:  # type: ignore
        error_msg = f"{service_client.service_name} service call failed!"
        raise ServiceCallError(error_msg)


def msg_to_dict(msg: Any):
    if isinstance(msg, Mapping):
        return {k: msg_to_dict(v) for k, v in msg.items()}
    elif is_iterable(msg):
        return [msg_to_dict(item) for item in msg]
    elif hasattr(msg, "get_fields_and_field_types"):
        nested_dict = {}
        for field in msg.get_fields_and_field_types().keys():
            nested_dict[field] = msg_to_dict(getattr(msg, field))
        return nested_dict
    else:
        return msg


def quaternion_msg_from_euler(
    roll: float, pitch: float, yaw: float, *, axes: str = "sxyz"
) -> Quaternion:
    """
    Convert roll, pitch, yaw angles (in radians) to a geometry_msgs/Quaternion message.
    """
    quaternion = quaternion_from_euler(roll, pitch, yaw, axes)
    return Quaternion(
        x=quaternion[0],
        y=quaternion[1],
        z=quaternion[2],
        w=quaternion[3],
    )


def pose_msg(
    position: Optional[Iterable[float] | Mapping[str, float]] = None,
    orientation: Optional[Iterable[float] | Mapping[str, float]] = None,
    rpy: Optional[Iterable[float] | Mapping[str, float]] = None,
) -> Pose:
    """
    Convert a dictionary of parameters to a geometry_msgs/Pose message.
    """
    pose = Pose()
    if position is None and orientation is None and rpy is None:
        raise ValueError(
            "No position or orientation parameters found in input dictionary"
        )

    # Position extraction
    if position is not None:
        if isinstance(position, Mapping):
            pose.position = Point(**position)  # type: ignore
        elif is_iterable(position):
            x, y, z = position
            pose.position = Point(x=x, y=y, z=z)
        else:
            raise ValueError(
                f"Invalid position type: expected Mapping or Iterable, got {type(position)}"
            )

    # Orientation extraction
    if rpy is not None:
        if isinstance(rpy, Mapping):
            pose.orientation = quaternion_msg_from_euler(**rpy)  # type: ignore
        elif is_iterable(rpy):
            pose.orientation = quaternion_msg_from_euler(*rpy)
        else:
            raise ValueError(
                f"Invalid rpy type: expected Mapping or Iterable, got {type(rpy)}"
            )

    elif orientation is not None:
        if isinstance(orientation, Mapping):
            pose.orientation = Quaternion(**orientation)  # type: ignore
        elif is_iterable(orientation):
            x, y, z, w = orientation
            pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
        else:
            raise ValueError(
                f"Invalid orientation type: expected Mapping or Iterable, got {type(orientation)}"
            )

    return pose


def pose_stamped_msg(
    header: Mapping[str, Any],
    pose: Mapping[str, Any],
) -> PoseStamped:
    """
    Convert a dictionary of parameters to a geometry_msgs/PoseStamped message.
    """
    pose_stamped = PoseStamped()
    pose_stamped.header = Header(**header)  # type: ignore
    pose_stamped.pose = pose_msg(**pose)  # type: ignore
    return pose_stamped


def pose_msg_from_matrix(matrix: np.ndarray) -> Pose:
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


def matrix_from_pose_msg(pose: Pose) -> np.ndarray:
    translation = translation_matrix(
        [pose.position.x, pose.position.y, pose.position.z]
    )
    rotation = quaternion_matrix(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )
    return translation @ rotation


def mesh_collision_object_msg(
    geometry: trimesh.Trimesh | trimesh.Scene,
    object_id: str,
    operation: str,
    header_frame_id: str,
    pose: Pose,
    subframe_names: list[str] = [],
    subframe_poses: list[Pose] = [],
) -> CollisionObject:
    """
    Create a collision object from a trimesh geometry.

    Args:
        geometry (trimesh.Trimesh | trimesh.Scene): The trimesh geometry to create the collision object from.
        object_id (str): The ID of the object.
        operation (str): The operation to perform on the collision object.
        header_frame_id (str): The ID of the reference frame.
        pose (Pose): The pose of the collision object (defined relative to `header_frame_id`).
        subframe_names (list[str]): The names of the subframes.
        subframe_poses (list[Pose]): The poses of the subframes (defined relative to `pose`).
    """
    if hasattr(geometry, "to_mesh"):
        mesh = geometry.to_mesh()  # type: ignore
    else:
        mesh = geometry

    collision_object = CollisionObject()
    collision_object.header.frame_id = header_frame_id
    collision_object.id = object_id

    msg = Mesh()
    msg.triangles = list(
        map(lambda t: MeshTriangle(vertex_indices=t), mesh.faces)  # type: ignore
    )
    msg.vertices = list(
        map(
            lambda v: Point(x=v[0], y=v[1], z=v[2]),
            mesh.vertices,  # type: ignore
        )
    )

    collision_object.meshes.append(msg)  # type: ignore
    collision_object.mesh_poses.append(pose)  # type: ignore

    for subframe_name, subframe_pose in zip(subframe_names, subframe_poses):
        collision_object.subframe_names.append(subframe_name)  # type: ignore
        collision_object.subframe_poses.append(subframe_pose)  # type: ignore

    collision_object.operation = collision_object_operation_map[operation]
    return collision_object


def attached_collision_object_msg(
    object_id: str,
    operation: str,
    link_name: Optional[str] = None,
    touch_links: Optional[list[str]] = None,
) -> AttachedCollisionObject:
    attached_collision_object = AttachedCollisionObject()
    attached_collision_object.object.id = object_id
    attached_collision_object.object.operation = (
        collision_object_operation_map[operation]
    )

    if link_name is not None:
        attached_collision_object.link_name = link_name
    elif operation == "add":
        raise ValueError("link_name must be provided for add operation")

    if touch_links is not None:
        attached_collision_object.touch_links = touch_links

    return attached_collision_object


def object_color_msg(
    object_id: str, color: str | Iterable[float] | Mapping[str, float]
) -> ObjectColor:
    object_color = ObjectColor()
    object_color.id = object_id
    if isinstance(color, str):
        try:
            rgba = rgba_map[color]
        except KeyError:
            raise ValueError(f"Invalid color: {color}")
    elif isinstance(color, Mapping):
        rgba = list(color.values())
    elif is_iterable(color):
        rgba = list(color)
        if len(rgba) != 4:
            raise ValueError(
                f"Color must be a list of 4 floats, got {len(rgba)}"
            )
    else:
        raise ValueError(f"Invalid color type: {type(color)}")

    object_color.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    return object_color
