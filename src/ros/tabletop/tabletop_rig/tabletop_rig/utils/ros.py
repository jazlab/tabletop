"""ROS2 utility functions for geometry, collision objects, and robot state.

This module provides a comprehensive set of utilities for working with ROS2
messages, including:

- Time conversion utilities
- Geometric message creation and manipulation (Point, Pose, Quaternion)
- Homogeneous transformation matrix operations
- Collision object creation from primitives and meshes
- Robot state and trajectory utilities
- Approximate equality comparisons for geometric types

The module uses the transformations library for matrix operations and
trimesh for mesh processing.
"""

import os
from collections.abc import Iterable, Mapping
from copy import deepcopy
from typing import Any, Literal, Optional, Protocol

import numpy as np
import trimesh
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit.core.robot_state import (  # type: ignore[reportMissingModuleSource]
    RobotState,
)
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    ObjectColor,
)
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
from rclpy.time import Time
from shape_msgs.msg import Mesh as MeshMsg
from shape_msgs.msg import MeshTriangle, Plane, SolidPrimitive
from std_msgs.msg import ColorRGBA, Header
from transformations import (
    euler_from_quaternion,
    inverse_matrix,
    quaternion_about_axis,
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_from_matrix,
    translation_matrix,
)

from tabletop_py.utils.common import is_iterable

# Constants


COLOR_MAP: dict[str, tuple[float, float, float, float]] = {
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
"""dict[str, tuple]: Maps color names to RGBA tuples (values 0.0-1.0)."""

COLLISION_OBJECT_OPERATION_MAP: dict[str, bytes] = {
    "ADD": CollisionObject.ADD,
    "REMOVE": CollisionObject.REMOVE,
    "APPEND": CollisionObject.APPEND,
    "MOVE": CollisionObject.MOVE,
}
"""dict[str, int]: Maps operation names to CollisionObject operation constants."""

SOLID_PRIMITIVE_TYPE_MAP: dict[str, int] = {
    "BOX": SolidPrimitive.BOX,
    "SPHERE": SolidPrimitive.SPHERE,
    "CYLINDER": SolidPrimitive.CYLINDER,
    "CONE": SolidPrimitive.CONE,
    "PRISM": SolidPrimitive.PRISM,
}
"""dict[str, int]: Maps primitive type names to SolidPrimitive type constants."""


# Protocol definitions


class SrvTypeRequest(Protocol):
    """Protocol defining the interface for ROS2 service request types."""


class SrvTypeResponse(Protocol):
    """Protocol defining the interface for ROS2 service response types.

    Attributes:
        success: Whether the service call completed successfully.
    """

    success: bool


class SrvType(Protocol):
    """Protocol defining the interface for ROS2 service types.

    Attributes:
        Request: The request message class for this service.
        Response: The response message class for this service.
    """

    Request: Any
    Response: Any


class ActionClientResultType(Protocol):
    """Protocol defining the interface for ROS2 action client result types.

    Attributes:
        status: Integer defining goal status
        result: Goal response
    """

    status: int
    result: Any


# Generic ROS2 utilities


def load_yaml_from_package(package_name: str, file_path: str) -> Any:
    """Load a YAML file from a ROS2 package's share directory.

    Args:
        package_name: The name of the ROS2 package containing the file.
        file_path: The relative path to the YAML file within the package's
            share directory.

    Returns:
        The parsed YAML data as Python objects (dict, list, etc.).

    Raises:
        PackageNotFoundError: If the package cannot be found.
        FileNotFoundError: If the YAML file doesn't exist.
    """
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path) as file:
        return yaml.safe_load(file)


# ROS2 time utilities


def seconds_from_ros_time(timestamp: Time | TimeMsg) -> float:
    """Convert a ROS2 time representation to floating-point seconds.

    Args:
        timestamp: Either an rclpy.time.Time object or a
            builtin_interfaces/Time message.

    Returns:
        The time as seconds since epoch as a float.

    Raises:
        ValueError: If timestamp is neither Time nor TimeMsg type.
    """
    if isinstance(timestamp, Time):
        return timestamp.nanoseconds / 1e9
    elif isinstance(timestamp, TimeMsg):
        return float(timestamp.sec) + float(timestamp.nanosec) / 1e9
    else:
        raise ValueError(f"Invalid timestamp type: {type(timestamp).__name__}")


def time_msg_from_seconds(seconds: float) -> TimeMsg:
    """Convert floating-point seconds to a ROS2 Time message.

    Args:
        seconds: Time in seconds (can include fractional component).

    Returns:
        A builtin_interfaces/Time message with sec and nanosec fields.
    """
    return TimeMsg(
        sec=int(seconds), nanosec=int((seconds - int(seconds)) * 1e9)
    )


# ROS2 geometric message utilities


def array_from_point_msg(point: Point) -> np.ndarray:
    """Convert a geometry_msgs/Point message to a numpy array.

    Args:
        point: The Point message to convert.

    Returns:
        A numpy array of shape (3,) containing [x, y, z].
    """
    return np.array([point.x, point.y, point.z])


def array_from_quaternion_msg(quaternion: Quaternion) -> np.ndarray:
    """Convert a geometry_msgs/Quaternion message to a normalized numpy array.

    Args:
        quaternion: The Quaternion message to convert.

    Returns:
        A normalized numpy array of shape (4,) in [w, x, y, z] order.
    """
    q = np.array([quaternion.w, quaternion.x, quaternion.y, quaternion.z])
    q = q / np.linalg.norm(q)
    return q


def arrays_from_pose_msg(
    pose: Pose, *, euler: bool = False, axes: str = "sxyz"
) -> tuple[np.ndarray, np.ndarray]:
    """Convert a geometry_msgs/Pose message to position and orientation arrays.

    Args:
        pose: The Pose message to convert.
        euler: If True, return orientation as Euler angles instead of quaternion.
        axes: Euler angle convention when euler=True (default: "sxyz").

    Returns:
        A tuple of (position, orientation) where:
        - position is a numpy array of shape (3,) containing [x, y, z]
        - orientation is either a quaternion array (4,) in [w, x, y, z] order
          or Euler angles array (3,) in [roll, pitch, yaw] order
    """
    position = array_from_point_msg(pose.position)
    if euler:
        orientation = euler_array_from_quaternion_msg(pose.orientation, axes)
    else:
        orientation = array_from_quaternion_msg(pose.orientation)
    return position, orientation


def quaternion_msg(w: float, x: float, y: float, z: float) -> Quaternion:
    """Create a normalized geometry_msgs/Quaternion message.

    Args:
        w: The scalar (real) component.
        x: The x component of the vector part.
        y: The y component of the vector part.
        z: The z component of the vector part.

    Returns:
        A normalized Quaternion message.
    """
    q = np.array([w, x, y, z])
    q = q / np.linalg.norm(q)
    w, x, y, z = (float(p) for p in q)
    return Quaternion(w=w, x=x, y=y, z=z)


def quaternion_msg_from_euler(
    roll: float, pitch: float, yaw: float, *, axes: str = "sxyz"
) -> Quaternion:
    """Convert Euler angles to a geometry_msgs/Quaternion message.

    Args:
        roll: Rotation about the x-axis in radians.
        pitch: Rotation about the y-axis in radians.
        yaw: Rotation about the z-axis in radians.
        axes: Euler angle convention specifying axis sequence and frame type.
            First character is 's' for static or 'r' for rotating frame.
            Following characters specify axis sequence (default: "sxyz").

    Returns:
        A normalized Quaternion message representing the rotation.
    """
    return quaternion_msg(*quaternion_from_euler(roll, pitch, yaw, axes))


def quaternion_msg_from_axis_angle(
    axis: Iterable[float], angle: float
) -> Quaternion:
    """Convert axis-angle representation to a geometry_msgs/Quaternion message.

    Args:
        axis: A 3-element iterable specifying the rotation axis (will be normalized).
        angle: The rotation angle in radians.

    Returns:
        A normalized Quaternion message representing the rotation.
    """
    return quaternion_msg(*quaternion_about_axis(angle, axis))


def normalize_quaternion_msg(quaternion: Quaternion) -> Quaternion:
    """Normalize a geometry_msgs/Quaternion message.

    Args:
        quaternion: The Quaternion message to normalize.

    Returns:
        A new Quaternion message with unit magnitude.
    """
    return quaternion_msg(
        quaternion.w, quaternion.x, quaternion.y, quaternion.z
    )


def euler_array_from_quaternion_msg(
    quaternion: Quaternion, axes: str = "sxyz"
) -> np.ndarray:
    """Convert a geometry_msgs/Quaternion message to Euler angles.

    Args:
        quaternion: The Quaternion message to convert.
        axes: Euler angle convention (default: "sxyz").

    Returns:
        A numpy array of shape (3,) containing [roll, pitch, yaw] in radians,
        normalized to the range [-pi, pi].
    """
    rpy = np.array(
        euler_from_quaternion(
            [quaternion.w, quaternion.x, quaternion.y, quaternion.z], axes=axes
        )
    )
    # Convert to [-pi, pi] range
    return (rpy + np.pi) % (2 * np.pi) - np.pi


def pose_msg(
    *,
    position: Optional[Point | Iterable[float] | Mapping[str, float]] = None,
    orientation: Optional[
        Quaternion | Iterable[float] | Mapping[str, float]
    ] = None,
    rpy: Optional[Iterable[float] | Mapping[str, float]] = None,
) -> Pose:
    """Create a geometry_msgs/Pose message from flexible input types.

    This factory function accepts multiple input formats for both position
    and orientation, providing convenient pose construction.

    Args:
        position: The position, which can be:
            - A Point message
            - An iterable of [x, y, z]
            - A mapping with 'x', 'y', 'z' keys
        orientation: The orientation as quaternion, which can be:
            - A Quaternion message
            - An iterable of [w, x, y, z]
            - A mapping with 'w', 'x', 'y', 'z' keys
        rpy: The orientation as Euler angles, which can be:
            - An iterable of [roll, pitch, yaw] in radians
            - A mapping with 'roll', 'pitch', 'yaw' keys

    Returns:
        A Pose message with the specified position and orientation.

    Raises:
        ValueError: If both orientation and rpy are provided, or if
            inputs have invalid types.
    """
    pose = Pose()

    # Position extraction
    if position is not None:
        position = deepcopy(position)
        if isinstance(position, Point):
            pose.position = position
        elif isinstance(position, Mapping):
            pose.position = Point(**position)  # type: ignore
        elif is_iterable(position):
            x, y, z = (float(p) for p in position)
            pose.position = Point(x=x, y=y, z=z)
        else:
            raise ValueError(
                f"Invalid position type: expected Mapping or Iterable, got {type(position)}"
            )

    # Orientation extraction
    if rpy is not None:
        if orientation is not None:
            raise ValueError("orientation and rpy cannot both be provided")
        rpy = deepcopy(rpy)
        if isinstance(rpy, Mapping):
            pose.orientation = quaternion_msg_from_euler(**rpy)  # type: ignore
        elif is_iterable(rpy):
            pose.orientation = quaternion_msg_from_euler(*rpy)
        else:
            raise ValueError(
                f"Invalid rpy type: expected Mapping or Iterable, got {type(rpy)}"
            )
    elif orientation is not None:
        orientation = deepcopy(orientation)
        if isinstance(orientation, Quaternion):
            pose.orientation = normalize_quaternion_msg(orientation)
        elif isinstance(orientation, Mapping):
            pose.orientation = quaternion_msg(**orientation)  # type: ignore
        elif is_iterable(orientation):
            pose.orientation = quaternion_msg(*orientation)
        else:
            raise ValueError(
                f"Invalid orientation type: expected Quaternion, Mapping, or Iterable, got {type(orientation)}"
            )

    return pose


def pose_stamped_msg(
    *,
    header: Optional[Header | Mapping[str, Any]] = None,
    frame_id: Optional[str] = None,
    timestamp: Optional[Time | Mapping[str, Any]] = None,
    pose: Optional[Pose | Mapping[str, Any]] = None,
    position: Optional[Point | Iterable[float] | Mapping[str, float]] = None,
    rpy: Optional[Iterable[float] | Mapping[str, float]] = None,
    orientation: Optional[
        Quaternion | Iterable[float] | Mapping[str, float]
    ] = None,
) -> PoseStamped:
    """Create a geometry_msgs/PoseStamped message from flexible input types.

    This factory function provides two mutually exclusive ways to specify
    the header and pose components:
    - Header: Provide either `header` OR (`frame_id` and/or `timestamp`)
    - Pose: Provide either `pose` OR (`position` and/or `rpy`/`orientation`)

    Args:
        header: Complete Header message or mapping with header fields.
        frame_id: The coordinate frame ID for the pose.
        timestamp: The timestamp for the pose.
        pose: Complete Pose message or mapping with pose fields.
        position: Position as Point, iterable [x,y,z], or mapping.
        rpy: Orientation as Euler angles [roll, pitch, yaw] in radians.
        orientation: Orientation as Quaternion, iterable [w,x,y,z], or mapping.

    Returns:
        A PoseStamped message with the specified header and pose.

    Raises:
        ValueError: If conflicting arguments are provided (e.g., both
            header and frame_id, or both pose and position).
    """

    pose_stamped = PoseStamped()
    if header is not None:
        if frame_id is not None or timestamp is not None:
            raise ValueError(
                "Either header or (at least one of frame_id and timestamp) must be provided, but not both"
            )

        header = deepcopy(header)
        if isinstance(header, Header):
            pose_stamped.header = header
        else:
            pose_stamped.header = Header(**header)
    else:
        if frame_id is not None:
            pose_stamped.header.frame_id = frame_id
        if timestamp is not None:
            timestamp = deepcopy(timestamp)
            if isinstance(timestamp, Time):
                pose_stamped.header.stamp = timestamp
            else:
                pose_stamped.header.stamp = Time(**timestamp)

    if pose is not None:
        if position is not None or rpy is not None or orientation is not None:
            raise ValueError(
                "Either pose or position/rpy/orientation must be provided, but not both"
            )

        pose = deepcopy(pose)
        if isinstance(pose, Pose):
            pose_stamped.pose = pose
        else:
            pose_stamped.pose = pose_msg(**pose)
    elif position is not None or rpy is not None or orientation is not None:
        pose_stamped.pose = pose_msg(
            position=position, rpy=rpy, orientation=orientation
        )

    return pose_stamped


# Comparison utilities


def all_close_iterables(
    a1: Iterable[float] | np.ndarray,
    a2: Iterable[float] | np.ndarray,
    tolerance: float | Iterable[float] | np.ndarray,
) -> bool:
    """Check if corresponding elements of two iterables are within tolerance.

    Args:
        a1: First array or iterable of numeric values.
        a2: Second array or iterable of numeric values.
        tolerance: Maximum allowed difference. Can be a scalar (applied to all
            elements) or an array of per-element tolerances.

    Returns:
        True if all element-wise differences are strictly less than tolerance.
    """
    if not isinstance(a1, np.ndarray):
        a1 = np.array(a1)
    if not isinstance(a2, np.ndarray):
        a2 = np.array(a2)
    if not isinstance(tolerance, np.ndarray):
        tolerance = np.array(tolerance)
    diff = np.abs(a1 - a2)
    return bool(np.all(diff < tolerance))


def all_close_dicts(
    d1: dict[str, float],
    d2: dict[str, float],
    tolerance: float | dict[str, float],
) -> bool:
    """Check if corresponding values in two dicts are within tolerance.

    Args:
        d1: First dictionary with string keys and numeric values.
        d2: Second dictionary with the same keys as d1.
        tolerance: Maximum allowed difference. Can be a scalar or a dict
            mapping keys to per-key tolerances.

    Returns:
        True if all value differences are within their respective tolerances.
    """
    if isinstance(tolerance, Mapping):
        for k, v in d1.items():
            if abs(v - d2[k]) > tolerance[k]:
                return False
    else:
        for k, v in d1.items():
            if abs(v - d2[k]) > tolerance:
                return False
    return True


def all_close_points(
    p1: Point, p2: Point, tolerance: float | Iterable[float]
) -> bool:
    """Check if two Point messages are within tolerance.

    Args:
        p1: First Point message.
        p2: Second Point message.
        tolerance: Maximum allowed difference for x, y, z components.
            Can be scalar or per-axis [x_tol, y_tol, z_tol].

    Returns:
        True if all position components are within tolerance.
    """
    p1_array = array_from_point_msg(p1)
    p2_array = array_from_point_msg(p2)
    return all_close_iterables(p1_array, p2_array, tolerance)


def all_close_quaternions(
    q1: Quaternion, q2: Quaternion, tolerance: float | Iterable[float]
) -> bool:
    """Check if two Quaternion messages represent similar orientations.

    This function accounts for quaternion double-cover, where q and -q
    represent the same orientation.

    Args:
        q1: First Quaternion message.
        q2: Second Quaternion message.
        tolerance: Maximum allowed difference for quaternion components.
            Can be scalar or per-component [w_tol, x_tol, y_tol, z_tol].

    Returns:
        True if quaternions are within tolerance (considering double-cover).
    """
    q1_array = array_from_quaternion_msg(q1)
    q2_array = array_from_quaternion_msg(q2)
    return all_close_iterables(
        q1_array, q2_array, tolerance
    ) or all_close_iterables(q1_array, -q2_array, tolerance)


def all_close_poses(
    pose1: Pose,
    pose2: Pose,
    position_tolerance: float | Iterable[float] | np.ndarray,
    orientation_tolerance: float | Iterable[float] | np.ndarray,
) -> bool:
    """Check if two Pose messages are within tolerance.

    Args:
        pose1: First Pose message.
        pose2: Second Pose message.
        position_tolerance: Maximum allowed position difference.
        orientation_tolerance: Maximum allowed quaternion component difference.

    Returns:
        True if both position and orientation are within their tolerances.
    """
    all_close_positions = all_close_points(
        pose1.position, pose2.position, position_tolerance
    )
    all_close_orientations = all_close_quaternions(
        pose1.orientation, pose2.orientation, orientation_tolerance
    )

    return all_close_positions and all_close_orientations


def all_close_poses_stamped(
    pose_stamped1: PoseStamped,
    pose_stamped2: PoseStamped,
    position_tolerance: float | Iterable[float] | np.ndarray,
    orientation_tolerance: float | Iterable[float] | np.ndarray,
) -> bool:
    """Check if two PoseStamped messages are within tolerance.

    Args:
        pose_stamped1: First PoseStamped message.
        pose_stamped2: Second PoseStamped message.
        position_tolerance: Maximum allowed position difference.
        orientation_tolerance: Maximum allowed quaternion component difference.

    Returns:
        True if poses are within tolerance.

    Raises:
        ValueError: If the frame_ids don't match (poses not comparable).
    """
    if pose_stamped1.header.frame_id != pose_stamped2.header.frame_id:
        raise ValueError("PoseStamped messages must have the same frame_id")
    return all_close_poses(
        pose_stamped1.pose,
        pose_stamped2.pose,
        position_tolerance,
        orientation_tolerance,
    )


def all_close_robot_states(
    state1: RobotState,
    state2: RobotState,
    position_tolerance: float | dict[str, float],
    velocity_tolerance: Optional[float | dict[str, float]] = None,
    acceleration_tolerance: Optional[float | dict[str, float]] = None,
) -> bool:
    """Check if two RobotState objects have similar joint values.

    Args:
        state1: First RobotState.
        state2: Second RobotState.
        position_tolerance: Maximum allowed joint position difference.
            Can be scalar or per-joint dict.
        velocity_tolerance: Optional maximum allowed joint velocity difference.
            If None, velocities are not compared.
        acceleration_tolerance: Optional maximum allowed joint acceleration
            difference. If None, accelerations are not compared.

    Returns:
        True if all compared joint values are within their tolerances.
    """
    if not all_close_dicts(
        state1.joint_positions, state2.joint_positions, position_tolerance
    ):
        return False

    if velocity_tolerance is not None and not all_close_dicts(
        state1.joint_velocities,
        state2.joint_velocities,
        velocity_tolerance,
    ):
        return False

    if acceleration_tolerance is not None and not all_close_dicts(
        state1.joint_accelerations,
        state2.joint_accelerations,
        acceleration_tolerance,
    ):
        return False

    return True


# Homogeneous transformation utilities


def pose_msg_from_matrix(matrix: np.ndarray) -> Pose:
    """Convert a 4x4 homogeneous transformation matrix to a Pose message.

    Args:
        matrix: A 4x4 numpy array representing a homogeneous transformation
            with rotation in the upper-left 3x3 and translation in the
            right column.

    Returns:
        A Pose message with position and orientation extracted from the matrix.
    """
    return pose_msg(
        position=translation_from_matrix(matrix),
        orientation=quaternion_from_matrix(matrix),
    )


def matrix_from_point_msg(point: Point) -> np.ndarray:
    """Convert a Point message to a 4x4 translation matrix.

    Args:
        point: The Point message containing x, y, z translation.

    Returns:
        A 4x4 numpy array representing pure translation (identity rotation).
    """
    return translation_matrix([point.x, point.y, point.z])


def matrix_from_quaternion_msg(quaternion: Quaternion) -> np.ndarray:
    """Convert a Quaternion message to a 4x4 rotation matrix.

    Args:
        quaternion: The Quaternion message to convert.

    Returns:
        A 4x4 numpy array representing pure rotation (zero translation).
    """
    return quaternion_matrix(
        [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
    )


def matrix_from_pose_msg(pose: Pose | Mapping[str, Any]) -> np.ndarray:
    """Convert a Pose message to a 4x4 homogeneous transformation matrix.

    Args:
        pose: A Pose message or mapping with position/orientation fields.

    Returns:
        A 4x4 numpy array combining rotation and translation.
    """
    if not isinstance(pose, Pose):
        pose = pose_msg(**pose)
    translation = matrix_from_point_msg(pose.position)
    rotation = matrix_from_quaternion_msg(pose.orientation)
    return translation @ rotation


def change_reference_frame_pose(
    old_pose: Pose,
    old_frame_transform: np.ndarray | Pose,
    new_frame_transform: np.ndarray | Pose,
) -> Pose:
    """Transform a pose from one reference frame to another.

    Given a pose expressed in an old frame, and transforms from both the old
    and new frames to a common world frame, computes the pose in the new frame.

    Args:
        old_pose: The pose to transform, expressed in the old frame.
        old_frame_transform: Transform from old frame to world frame.
        new_frame_transform: Transform from new frame to world frame.

    Returns:
        The pose expressed in the new reference frame.
    """
    old_pose_matrix = matrix_from_pose_msg(old_pose)
    if isinstance(old_frame_transform, Pose):
        old_frame_transform = matrix_from_pose_msg(old_frame_transform)
    if isinstance(new_frame_transform, Pose):
        new_frame_transform = matrix_from_pose_msg(new_frame_transform)

    # Compute the new pose in the transformed frame
    reference_frame_transform = (
        inverse_matrix(new_frame_transform) @ old_frame_transform
    )
    new_pose_matrix = reference_frame_transform @ old_pose_matrix

    # Convert back to Pose message
    return pose_msg_from_matrix(new_pose_matrix)


def change_reference_frame_pose_stamped(
    old_pose_stamped: PoseStamped,
    old_frame_transform: np.ndarray | Pose,
    new_frame_transform: np.ndarray | Pose,
    new_frame_id: str,
) -> PoseStamped:
    """Transform a PoseStamped from one reference frame to another.

    Args:
        old_pose_stamped: The stamped pose to transform.
        old_frame_transform: Transform from the old frame to world frame
            (as 4x4 matrix or Pose).
        new_frame_transform: Transform from the new frame to world frame
            (as 4x4 matrix or Pose).
        new_frame_id: The frame_id string for the new reference frame.

    Returns:
        A PoseStamped with the transformed pose and new frame_id.
    """
    new_pose = change_reference_frame_pose(
        old_pose_stamped.pose, old_frame_transform, new_frame_transform
    )
    return pose_stamped_msg(frame_id=new_frame_id, pose=new_pose)


# Collision object utilities


def add_collision_object_msg(
    object_id: str,
    pose_stamped: PoseStamped,
    subframe_names: Optional[list[str]] = None,
    subframe_poses: Optional[list[Pose]] = None,
) -> CollisionObject:
    """Create a base CollisionObject message with ADD operation.

    This creates a collision object with basic fields set but no geometry.
    Use the specialized functions (add_primitive_collision_object_msg, etc.)
    to create objects with actual collision geometry.

    Args:
        object_id: Unique identifier for the collision object.
        pose_stamped: The pose and frame of the object.
        subframe_names: Optional list of named subframe identifiers.
        subframe_poses: Optional list of poses for subframes (relative to object pose).
            Must be same length as subframe_names if provided.

    Returns:
        A CollisionObject message with ADD operation and no geometry.
    """
    collision_object = CollisionObject()
    collision_object.header.frame_id = pose_stamped.header.frame_id
    collision_object.id = object_id
    collision_object.pose = pose_stamped.pose
    collision_object.operation = CollisionObject.ADD

    if subframe_names is not None and subframe_poses is not None:
        for subframe_name, subframe_pose in zip(
            subframe_names, subframe_poses
        ):
            collision_object.subframe_names.append(subframe_name)  # type: ignore
            collision_object.subframe_poses.append(subframe_pose)  # type: ignore

    return collision_object


def add_plane_collision_object_msg(
    object_id: str,
    pose_stamped: PoseStamped,
    coef: list[float],
) -> CollisionObject:
    """Create a collision object with a plane geometry.

    Args:
        object_id: Unique identifier for the collision object.
        pose_stamped: The pose and frame of the plane.
        coef: Plane equation coefficients [a, b, c, d] where ax + by + cz + d = 0.

    Returns:
        A CollisionObject message with plane geometry.
    """
    collision_object = add_collision_object_msg(object_id, pose_stamped)
    collision_object.planes.append(Plane(coef=coef))  # type: ignore
    return collision_object


def add_primitive_collision_object_msg(
    object_id: str,
    pose_stamped: PoseStamped,
    *,
    type: str,
    dimensions: list[float],
    subframe_names: Optional[list[str]] = None,
    subframe_poses: Optional[list[Pose]] = None,
) -> CollisionObject:
    """Create a collision object with a primitive geometry.

    Args:
        object_id: Unique identifier for the collision object.
        pose_stamped: The pose and frame of the object.
        type: Primitive type name ("BOX", "SPHERE", "CYLINDER", "CONE", "PRISM").
        dimensions: Dimensions for the primitive. Format depends on type:
            - BOX: [x, y, z] extents
            - SPHERE: [radius]
            - CYLINDER: [height, radius]
            - CONE: [height, radius]
        subframe_names: Optional list of named subframe identifiers.
        subframe_poses: Optional list of poses for subframes.

    Returns:
        A CollisionObject message with the specified primitive geometry.
    """
    collision_object = add_collision_object_msg(
        object_id, pose_stamped, subframe_names, subframe_poses
    )
    collision_object.primitives.append(  # type: ignore
        SolidPrimitive(
            type=SOLID_PRIMITIVE_TYPE_MAP[type], dimensions=dimensions
        )
    )

    return collision_object


TrimeshPrimitive = (
    trimesh.primitives.Box
    | trimesh.primitives.Sphere
    | trimesh.primitives.Cylinder
)
"""Type alias for trimesh primitive geometry types."""


def add_primitive_collision_object_msg_from_mesh(
    object_id: str,
    pose_stamped: PoseStamped,
    *,
    mesh: trimesh.Trimesh | trimesh.Scene,
    primitive_type: Literal[
        "bounding_primitive",
        "bounding_box",
        "bounding_box_oriented",
        "bounding_sphere",
        "bounding_cylinder",
    ] = "bounding_primitive",
    subframe_names: Optional[list[str]] = None,
    subframe_poses: Optional[list[Pose]] = None,
) -> CollisionObject:
    """Create a collision object with primitive geometry derived from a mesh.

    Computes a bounding primitive from the mesh geometry for use as a
    simplified collision shape. Useful for faster collision checking
    with acceptable approximation.

    Args:
        object_id: Unique identifier for the collision object.
        pose_stamped: The pose and frame of the object.
        mesh: The trimesh geometry to derive the primitive from.
        primitive_type: Type of bounding primitive to compute:
            - "bounding_primitive": Auto-select best fit primitive
            - "bounding_box": Axis-aligned bounding box
            - "bounding_box_oriented": Oriented bounding box
            - "bounding_sphere": Bounding sphere
            - "bounding_cylinder": Bounding cylinder
        subframe_names: Optional list of named subframe identifiers.
        subframe_poses: Optional list of poses for subframes.

    Returns:
        A CollisionObject with primitive geometry approximating the mesh.

    Raises:
        ValueError: If the computed primitive type is not supported.
    """
    collision_object = add_collision_object_msg(
        object_id, pose_stamped, subframe_names, subframe_poses
    )

    geometries: Iterable[TrimeshPrimitive]
    if isinstance(mesh, trimesh.Scene):
        geometries = mesh.geometry.values()
    else:
        geometries = [mesh.bounding_primitive]

    for geometry in geometries:
        primitive = geometry.__getattribute__(primitive_type)
        params = primitive.to_dict()

        if isinstance(primitive, trimesh.primitives.Box):
            primitive_msg = SolidPrimitive(
                type=SolidPrimitive.BOX, dimensions=params["extents"]
            )
        elif isinstance(primitive, trimesh.primitives.Sphere):
            primitive_msg = SolidPrimitive(
                type=SolidPrimitive.SPHERE, dimensions=[params["radius"]]
            )
        elif isinstance(primitive, trimesh.primitives.Cylinder):
            primitive_msg = SolidPrimitive(
                type=SolidPrimitive.CYLINDER,
                dimensions=[params["height"], params["radius"]],
            )
        else:
            raise ValueError(f"Invalid primitive type: {type(primitive)}")

        collision_object.primitives.append(primitive_msg)  # type: ignore
        pose = pose_msg_from_matrix(np.array(params["transform"]))
        collision_object.primitive_poses.append(pose)  # type: ignore

    return collision_object


def add_mesh_collision_object_msg(
    object_id: str,
    pose_stamped: PoseStamped,
    *,
    mesh: trimesh.Trimesh | trimesh.Scene | MeshMsg,
    subframe_names: Optional[list[str]] = None,
    subframe_poses: Optional[list[Pose]] = None,
) -> CollisionObject:
    """Create a collision object with mesh geometry.

    Uses the full mesh triangles for precise collision detection.
    More accurate than primitive approximations but slower to check.

    Args:
        object_id: Unique identifier for the collision object.
        pose_stamped: The pose and frame of the object.
        mesh: The mesh geometry, either as a trimesh object or pre-built
            ROS Mesh message.
        subframe_names: Optional list of named subframe identifiers.
        subframe_poses: Optional list of poses for subframes
            (defined relative to the object pose).

    Returns:
        A CollisionObject message with full mesh geometry.
    """
    collision_object = add_collision_object_msg(
        object_id, pose_stamped, subframe_names, subframe_poses
    )

    if not isinstance(mesh, MeshMsg):
        if isinstance(mesh, trimesh.Scene):
            geometry = mesh.to_mesh()
        else:
            geometry = mesh
        mesh = MeshMsg()
        mesh.triangles = list(
            map(lambda t: MeshTriangle(vertex_indices=t), geometry.faces)
        )
        mesh.vertices = list(
            map(lambda v: Point(x=v[0], y=v[1], z=v[2]), geometry.vertices)
        )

    collision_object.meshes.append(mesh)  # type: ignore

    return collision_object


def attached_collision_object_msg(
    object_id: str,
    operation: str,
    link_name: str = "",
    touch_links: Optional[list[str]] = None,
) -> AttachedCollisionObject:
    """Create an AttachedCollisionObject message.

    Attached collision objects move with a robot link and can ignore
    collisions with specified touch links (e.g., gripper fingers).

    Args:
        object_id: The ID of the collision object to attach/detach.
        operation: The operation to perform ("ADD", "REMOVE", "APPEND", "MOVE").
        link_name: The robot link to attach to. Required for ADD operation.
        touch_links: Optional list of links that should ignore collisions
            with this object (e.g., for grasped objects).

    Returns:
        An AttachedCollisionObject message.

    Raises:
        ValueError: If link_name is not provided for ADD operation.
    """
    attached_collision_object = AttachedCollisionObject()
    attached_collision_object.object.id = object_id

    if not link_name and operation == "ADD":
        raise ValueError("link_name must be provided for ADD operation")
    attached_collision_object.link_name = link_name

    if touch_links is not None:
        attached_collision_object.touch_links = touch_links

    attached_collision_object.object.operation = (
        COLLISION_OBJECT_OPERATION_MAP[operation]
    )
    return attached_collision_object


def object_color_msg(
    object_id: str, color: str | Iterable[float] | Mapping[str, float]
) -> ObjectColor:
    """Create an ObjectColor message for visualization.

    Args:
        object_id: The ID of the object to color.
        color: The color specification, which can be:
            - A color name string from COLOR_MAP (e.g., "red", "blue")
            - An iterable of [r, g, b, a] values (0.0-1.0)
            - A mapping with 'r', 'g', 'b', 'a' keys

    Returns:
        An ObjectColor message for use with planning scene visualization.

    Raises:
        ValueError: If the color name is not found or format is invalid.
    """
    object_color = ObjectColor()
    object_color.id = object_id
    if isinstance(color, str):
        try:
            rgba = COLOR_MAP[color]
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


# Robot trajectory utilities


def robot_trajectory_from_msg(
    trajectory_msg: RobotTrajectoryMsg,
    state: RobotState,
    joint_model_group_name: str,
) -> RobotTrajectory:
    """Convert a RobotTrajectory message to a MoveIt RobotTrajectory object.

    Args:
        trajectory_msg: The ROS message containing trajectory waypoints.
        state: A reference RobotState used to initialize the trajectory.
            Must not be dirty (have uncommitted changes).
        joint_model_group_name: The name of the joint model group
            (e.g., "manipulator", "arm") this trajectory applies to.

    Returns:
        A MoveIt RobotTrajectory object.

    Raises:
        ValueError: If the provided state is dirty.
    """
    if state.dirty:
        raise ValueError("Robot state is dirty")
    trajectory = RobotTrajectory(state.robot_model)
    trajectory.set_robot_trajectory_msg(state, trajectory_msg)
    trajectory.joint_model_group_name = joint_model_group_name
    return trajectory


def robot_trajectory_copy(
    trajectory: RobotTrajectory,
) -> RobotTrajectory:
    """Create a deep copy of a RobotTrajectory object.

    Args:
        trajectory: The trajectory to copy.

    Returns:
        A new RobotTrajectory with the same waypoints and joint model group.
    """
    state = trajectory[0]
    new_trajectory = RobotTrajectory(state.robot_model)
    new_trajectory.set_robot_trajectory_msg(
        state, trajectory.get_robot_trajectory_msg()
    )
    new_trajectory.joint_model_group_name = trajectory.joint_model_group_name
    return new_trajectory
