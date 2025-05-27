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
from shape_msgs.msg import Mesh, MeshTriangle, Plane, SolidPrimitive
from std_msgs.msg import ColorRGBA, Header
from tf_transformations import (
    euler_from_quaternion,
    inverse_matrix,
    quaternion_about_axis,
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
    "ADD": CollisionObject.ADD,
    "REMOVE": CollisionObject.REMOVE,
    "APPEND": CollisionObject.APPEND,
    "MOVE": CollisionObject.MOVE,
}

"""
Solid primitive type map from type name to solid primitive type.
"""
solid_primitive_type_map = {
    "BOX": SolidPrimitive.BOX,
    "SPHERE": SolidPrimitive.SPHERE,
    "CYLINDER": SolidPrimitive.CYLINDER,
    "CONE": SolidPrimitive.CONE,
    "PRISM": SolidPrimitive.PRISM,
}


# Protocol definitions


class SrvTypeRequest(Protocol):
    pass


class SrvTypeResponse(Protocol):
    success: bool


class SrvType(Protocol):
    Request: Any
    Response: Any


# Exception definitions


class MaxAttemptsReachedError(Exception):
    pass


class ServiceCallError(Exception):
    pass


class ServiceCallUnsuccessfulError(Exception):
    pass


# Generic ROS2 utilities


def load_yaml_from_package(package_name: str, file_path: str) -> Any:
    """Load a YAML file from a ROS package share directory.

    Args:
        package_name: The name of the ROS package.
        file_path: The path to the YAML file within the package.

    Returns:
        The loaded YAML data.
    """
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path) as file:
        return yaml.safe_load(file)


def validate_service_response(
    response: Optional[SrvTypeResponse],
    service_client: Client,
) -> None:
    """Validate the response from a service call.

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
        error_msg = (
            f"{service_client.service_name} service call returned "
            f"unsuccessfully with response: {msg_to_dict(response)}"
        )
        raise ServiceCallUnsuccessfulError(error_msg)


def msg_to_dict(msg: Any) -> dict[str, Any] | list[Any] | Any:
    """Convert a ROS message to a dictionary."""
    if isinstance(msg, Mapping):
        return {k: msg_to_dict(v) for k, v in msg.items()}
    elif is_iterable(msg):
        return [msg_to_dict(item) for item in msg]
    elif hasattr(msg, "get_fields_and_field_types"):
        return {
            field: msg_to_dict(getattr(msg, field))
            for field in msg.get_fields_and_field_types().keys()
        }
    else:
        return msg


# Geometric ROS2 message utilities


def array_from_point_msg(point: Point) -> np.ndarray:
    """Convert a geometry_msgs/Point message to a numpy array."""
    return np.array([point.x, point.y, point.z])


def array_from_quaternion_msg(quaternion: Quaternion) -> np.ndarray:
    """Convert a geometry_msgs/Quaternion message to a numpy array."""
    return np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])


def arrays_from_pose_msg(pose: Pose) -> tuple[np.ndarray, np.ndarray]:
    """Convert a geometry_msgs/Pose message to position and orientation arrays."""
    position = array_from_point_msg(pose.position)
    orientation = array_from_quaternion_msg(pose.orientation)
    return position, orientation


def quaternion_msg_from_euler(
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    *,
    axes: str = "sxyz",
) -> Quaternion:
    """Convert roll, pitch, yaw angles (in radians) to a geometry_msgs/Quaternion message.

    Args:
        roll: The roll angle in radians.
        pitch: The pitch angle in radians.
        yaw: The yaw angle in radians.
        axes: The axes scheme to use for the rotation.

    Returns:
        The quaternion message.
    """
    quaternion = quaternion_from_euler(roll, pitch, yaw, axes)
    return Quaternion(
        x=quaternion[0],
        y=quaternion[1],
        z=quaternion[2],
        w=quaternion[3],
    )


def quaternion_msg_from_axis_angle(
    axis: Iterable[float], angle: float
) -> Quaternion:
    """Convert an axis and angle to a geometry_msgs/Quaternion message."""
    quaternion = quaternion_about_axis(angle, axis)
    return Quaternion(
        x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
    )


def euler_from_quaternion_msg(
    quaternion: Quaternion, axes: str = "sxyz"
) -> tuple[float, float, float]:
    """Convert a geometry_msgs/Quaternion message to roll, pitch, yaw angles (in radians)."""
    return euler_from_quaternion(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w], axes=axes
    )


def pose_msg(
    position: Optional[Point | Iterable[float] | Mapping[str, float]] = None,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    orientation: Optional[
        Quaternion | Iterable[float] | Mapping[str, float]
    ] = None,
    rpy: Optional[Iterable[float] | Mapping[str, float]] = None,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
) -> Pose:
    """Convert a dictionary of parameters to a geometry_msgs/Pose message."""
    pose = Pose()

    # Position extraction
    if position is not None:
        if any((x, y, z)):
            raise ValueError("position and x, y, z cannot both be provided")

        if isinstance(position, Point):
            pose.position = position
        elif isinstance(position, Mapping):
            pose.position = Point(**position)  # type: ignore
        elif is_iterable(position):
            x, y, z = position
            pose.position = Point(x=x, y=y, z=z)
        else:
            raise ValueError(
                f"Invalid position type: expected Mapping or Iterable, got {type(position)}"
            )
    elif any((x, y, z)):
        pose.position = Point(x=x, y=y, z=z)

    # Orientation extraction
    if rpy is not None:
        if any((roll, pitch, yaw)):
            raise ValueError(
                "rpy and roll, pitch, yaw cannot both be provided"
            )
        if orientation is not None:
            raise ValueError("orientation and rpy cannot both be provided")
        if isinstance(rpy, Mapping):
            pose.orientation = quaternion_msg_from_euler(**rpy)  # type: ignore
        else:
            pose.orientation = quaternion_msg_from_euler(*rpy)
    elif any((roll, pitch, yaw)):
        if orientation is not None:
            raise ValueError(
                "orientation and roll, pitch, yaw cannot both be provided"
            )
        pose.orientation = quaternion_msg_from_euler(roll, pitch, yaw)
    elif orientation is not None:
        if isinstance(orientation, Quaternion):
            pose.orientation = orientation
        elif isinstance(orientation, Mapping):
            pose.orientation = Quaternion(**orientation)  # type: ignore
        else:
            x, y, z, w = orientation
            pose.orientation = Quaternion(x=x, y=y, z=z, w=w)

    return pose


def pose_stamped_msg(
    *,
    header: Optional[Header | Mapping[str, Any]] = None,
    frame_id: Optional[str] = None,
    timestamp: Optional[float] = None,
    pose: Optional[Pose | Mapping[str, Any]] = None,
    position: Optional[Point | Iterable[float] | Mapping[str, float]] = None,
    rpy: Optional[Iterable[float] | Mapping[str, float]] = None,
    orientation: Optional[
        Quaternion | Iterable[float] | Mapping[str, float]
    ] = None,
) -> PoseStamped:
    """Create a PoseStamped message from:
    - a header or frame_id, but not both,
    - a pose or at least one of position, rpy, or orientation, but not both.

    Args:
        header: The header of the pose.
        frame_id: The frame id of the pose.
        timestamp: The timestamp of the pose.
        position: The position of the pose.
        rpy: The roll, pitch, and yaw of the pose.
        orientation: The orientation of the pose.

    Returns:
        The PoseStamped message.
    """

    pose_stamped = PoseStamped()
    if header is not None:
        if frame_id is not None or timestamp is not None:
            raise ValueError(
                "Either header or (at least one of frame_id and timestamp) "
                "must be provided, but not both"
            )
        if isinstance(header, Header):
            pose_stamped.header = header
        else:
            pose_stamped.header = Header(**header)
    else:
        if frame_id is not None:
            pose_stamped.header.frame_id = frame_id
        if timestamp is not None:
            pose_stamped.header.stamp = timestamp

    if pose is not None:
        if position is not None or rpy is not None or orientation is not None:
            raise ValueError(
                "Either pose or position/rpy/orientation must be provided, "
                "but not both"
            )
        if isinstance(pose, Pose):
            pose_stamped.pose = pose
        else:
            pose_stamped.pose = pose_msg(**pose)
    elif position is not None or rpy is not None or orientation is not None:
        pose_stamped.pose = pose_msg(
            position=position, rpy=rpy, orientation=orientation
        )

    return pose_stamped


def pose_msg_from_matrix(matrix: np.ndarray) -> Pose:
    """Convert a 4x4 transformation matrix to a geometry_msgs/Pose message."""
    return pose_msg(
        position=translation_from_matrix(matrix),
        orientation=quaternion_from_matrix(matrix),
    )


def all_close_points(p1: Point, p2: Point, **all_close_kwargs: Any) -> bool:
    """Check if two points are close to each other."""
    p1_array = array_from_point_msg(p1)
    p2_array = array_from_point_msg(p2)
    return np.allclose(p1_array, p2_array, **all_close_kwargs)


def all_close_quaternions(
    q1: Quaternion, q2: Quaternion, **all_close_kwargs: Any
) -> bool:
    """Check if two quaternions are close to each other."""
    q1_array = array_from_quaternion_msg(q1)
    q2_array = array_from_quaternion_msg(q2)
    return np.allclose(q1_array, q2_array, **all_close_kwargs)


def all_close_poses(pose1: Pose, pose2: Pose, **all_close_kwargs: Any) -> bool:
    """Check if two poses are close to each other."""
    return all_close_points(
        pose1.position, pose2.position, **all_close_kwargs
    ) and all_close_quaternions(
        pose1.orientation, pose2.orientation, **all_close_kwargs
    )


def matrix_from_point_msg(point: Point) -> np.ndarray:
    """Convert a geometry_msgs/Point message to a 4x4 transformation matrix."""
    return translation_matrix([point.x, point.y, point.z])


def matrix_from_quaternion_msg(quaternion: Quaternion) -> np.ndarray:
    """Convert a geometry_msgs/Quaternion message to a 4x4 transformation matrix."""
    return quaternion_matrix(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    )


def matrix_from_pose_msg(pose: Pose | Mapping[str, Any]) -> np.ndarray:
    """Convert a geometry_msgs/Pose message to a 4x4 transformation matrix."""
    if not isinstance(pose, Pose):
        pose = pose_msg(**pose)
    translation = matrix_from_point_msg(pose.position)
    rotation = matrix_from_quaternion_msg(pose.orientation)
    return translation @ rotation


def change_reference_frame_pose_stamped(
    old_pose_stamped: PoseStamped,
    old_frame_transform: np.ndarray,
    new_frame_transform: np.ndarray,
    new_frame_id: str,
) -> PoseStamped:
    """Transforms a pose from one frame to another.

    Args:
        old_pose_stamped (PoseStamped): The pose to transform.
        old_frame_transform (np.ndarray): The transform from the old frame to the world frame.
        new_frame_transform (np.ndarray): The transform from the new frame to the world frame.
        new_frame_id (str): The ID of the new frame.
    """

    old_pose_matrix = matrix_from_pose_msg(old_pose_stamped.pose)

    # Compute the new pose in the transformed frame
    reference_frame_transform = (
        inverse_matrix(new_frame_transform) @ old_frame_transform
    )
    new_pose_matrix = reference_frame_transform @ old_pose_matrix

    # Convert back to Pose message
    new_pose = pose_msg_from_matrix(new_pose_matrix)

    # Create new PoseStamped message with updated frame_id
    new_pose_stamped = pose_stamped_msg(
        header=old_pose_stamped.header,
        pose=new_pose,
    )
    new_pose_stamped.header.frame_id = new_frame_id

    return new_pose_stamped


def plane_collision_object_msg(
    object_id: str,
    coef: list[float],
    header_frame_id: str,
    operation: str = "ADD",
) -> CollisionObject:
    """Create a collision object from a plane."""
    collision_object = CollisionObject()
    collision_object.header.frame_id = header_frame_id
    collision_object.id = object_id

    collision_object.planes.append(Plane(coef=coef))  # type: ignore

    collision_object.operation = collision_object_operation_map[operation]

    return collision_object


def primitive_collision_object_msg(
    object_id: str,
    type: str,
    dimensions: list[float],
    pose_stamped: PoseStamped,
    subframe_names: Optional[list[str]] = None,
    subframe_poses: Optional[list[Pose]] = None,
    operation: str = "ADD",
) -> CollisionObject:
    """Create a collision object from a primitive."""
    collision_object = CollisionObject()
    collision_object.header.frame_id = pose_stamped.header.frame_id
    collision_object.id = object_id

    collision_object.primitives.append(  # type: ignore
        SolidPrimitive(
            type=solid_primitive_type_map[type], dimensions=dimensions
        )
    )
    collision_object.primitive_poses.append(pose_stamped.pose)  # type: ignore

    if subframe_names is not None and subframe_poses is not None:
        for subframe_name, subframe_pose in zip(
            subframe_names, subframe_poses
        ):
            collision_object.subframe_names.append(subframe_name)  # type: ignore
            collision_object.subframe_poses.append(subframe_pose)  # type: ignore

    collision_object.operation = collision_object_operation_map[operation]
    return collision_object


def mesh_collision_object_msg(
    object_id: str,
    geometry: trimesh.Trimesh | trimesh.Scene,
    pose_stamped: PoseStamped,
    subframe_names: Optional[list[str]] = None,
    subframe_poses: Optional[list[Pose]] = None,
    operation: str = "ADD",
) -> CollisionObject:
    """Create a collision object from a trimesh geometry.

    Args:
        geometry (trimesh.Trimesh | trimesh.Scene): The trimesh geometry to create the collision object from.
        object_id (str): The ID of the object.
        operation (str): The operation to perform on the collision object.
        pose_stamped (PoseStamped): The pose of the collision object.
        subframe_names (list[str]): The names of the subframes.
        subframe_poses (list[Pose]): The poses of the subframes (defined relative to `pose_stamped`).
    """
    if isinstance(geometry, trimesh.Scene):
        mesh = geometry.to_mesh()
    else:
        mesh = geometry

    collision_object = CollisionObject()
    collision_object.header.frame_id = pose_stamped.header.frame_id
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
    collision_object.mesh_poses.append(pose_stamped.pose)  # type: ignore

    if subframe_names is not None and subframe_poses is not None:
        for subframe_name, subframe_pose in zip(
            subframe_names, subframe_poses
        ):
            collision_object.subframe_names.append(subframe_name)  # type: ignore
            collision_object.subframe_poses.append(subframe_pose)  # type: ignore

    collision_object.operation = collision_object_operation_map[operation]
    return collision_object


def attached_collision_object_msg(
    object_id: str,
    operation: str,
    link_name: str = "",
    touch_links: Optional[list[str]] = None,
) -> AttachedCollisionObject:
    """Create an AttachedCollisionObject message."""
    attached_collision_object = AttachedCollisionObject()
    attached_collision_object.object.id = object_id

    attached_collision_object.link_name = link_name
    if not link_name and operation == "ADD":
        raise ValueError("link_name must be provided for ADD operation")

    if touch_links is not None:
        attached_collision_object.touch_links = touch_links

    attached_collision_object.object.operation = (
        collision_object_operation_map[operation]
    )
    return attached_collision_object


def object_color_msg(
    object_id: str, color: str | Iterable[float] | Mapping[str, float]
) -> ObjectColor:
    """Create an ObjectColor message."""
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
