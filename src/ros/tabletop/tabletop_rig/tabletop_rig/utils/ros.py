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
from copy import copy, deepcopy
from typing import Any, Literal, Optional, Protocol

import numpy as np
import trimesh
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from moveit.core.robot_state import (  # type: ignore[reportMissingModuleSource]
    RobotState,
)
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from moveit_msgs.msg import (
    AttachedCollisionObject,
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    ObjectColor,
    OrientationConstraint,
    PositionConstraint,
    VisibilityConstraint,
)
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
from rclpy.time import Duration, Time
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


def seconds_from_ros_time(
    timestamp: Time | TimeMsg | Duration | DurationMsg,
) -> float:
    """Convert a ROS2 time representation to floating-point seconds.

    Args:
        timestamp: Either an rclpy.time.Time object or a
            builtin_interfaces/Time message.

    Returns:
        The time as seconds since epoch as a float.

    Raises:
        ValueError: If timestamp is neither Time nor TimeMsg type.
    """
    if isinstance(timestamp, (Time, Duration)):
        return timestamp.nanoseconds / 1e9
    elif isinstance(timestamp, (TimeMsg, DurationMsg)):
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


# Motion planning constraint utilities


def _vector3_msg(
    v: Vector3 | Iterable[float] | Mapping[str, float],
) -> Vector3:
    """Convert flexible input to a geometry_msgs/Vector3 message."""
    v = deepcopy(v)
    if isinstance(v, Vector3):
        return v
    if isinstance(v, Mapping):
        return Vector3(**v)  # type: ignore[reportCallIssue]
    if is_iterable(v):
        x, y, z = (float(p) for p in v)
        return Vector3(x=x, y=y, z=z)
    raise ValueError(
        f"Invalid Vector3 input type: expected Vector3, Mapping, or Iterable, "
        f"got {type(v)}"
    )


def joint_constraint_msg(
    *,
    joint_name: str,
    position: float,
    tolerance_above: float = 0.0,
    tolerance_below: float = 0.0,
    weight: float = 1.0,
) -> JointConstraint:
    """Create a moveit_msgs/JointConstraint message.

    Args:
        joint_name: The joint this constraint applies to.
        position: The target position (radians or meters depending on joint type).
        tolerance_above: Allowed positive deviation from `position`.
        tolerance_below: Allowed negative deviation from `position`.
        weight: Relative weight of this constraint (planner-specific).

    Returns:
        A JointConstraint message.
    """
    return JointConstraint(
        joint_name=joint_name,
        position=float(position),
        tolerance_above=float(tolerance_above),
        tolerance_below=float(tolerance_below),
        weight=float(weight),
    )


def bounding_volume_msg(
    *,
    primitives: Optional[Iterable[SolidPrimitive | Mapping[str, Any]]] = None,
    primitive_poses: Optional[Iterable[Pose | Mapping[str, Any]]] = None,
    meshes: Optional[Iterable[MeshMsg]] = None,
    mesh_poses: Optional[Iterable[Pose | Mapping[str, Any]]] = None,
) -> BoundingVolume:
    """Create a moveit_msgs/BoundingVolume message.

    Args:
        primitives: List of SolidPrimitives, or mappings with `type` (str
            mapping into SOLID_PRIMITIVE_TYPE_MAP) and `dimensions` keys.
        primitive_poses: Poses for each primitive (same length as `primitives`).
            Accepts Pose messages or mappings consumed by `pose_msg()`.
        meshes: List of pre-built Mesh messages.
        mesh_poses: Poses for each mesh. Accepts Pose or mapping.

    Returns:
        A BoundingVolume message.

    Raises:
        ValueError: If primitives and primitive_poses (or meshes and
            mesh_poses) have mismatched lengths.
    """
    bv = BoundingVolume()

    primitive_msgs: list[SolidPrimitive] = []
    if primitives is not None:
        for p in primitives:
            p = deepcopy(p)
            if isinstance(p, SolidPrimitive):
                primitive_msgs.append(p)
            else:
                primitive_msgs.append(
                    SolidPrimitive(
                        type=SOLID_PRIMITIVE_TYPE_MAP[p["type"]],
                        dimensions=list(p["dimensions"]),
                    )
                )

    primitive_pose_msgs: list[Pose] = []
    if primitive_poses is not None:
        for p in primitive_poses:
            if isinstance(p, Pose):
                primitive_pose_msgs.append(deepcopy(p))
            else:
                primitive_pose_msgs.append(pose_msg(**p))

    if len(primitive_msgs) != len(primitive_pose_msgs):
        raise ValueError(
            f"primitives ({len(primitive_msgs)}) and primitive_poses "
            f"({len(primitive_pose_msgs)}) must have the same length"
        )

    mesh_msgs: list[MeshMsg] = list(meshes) if meshes is not None else []
    mesh_pose_msgs: list[Pose] = []
    if mesh_poses is not None:
        for p in mesh_poses:
            if isinstance(p, Pose):
                mesh_pose_msgs.append(deepcopy(p))
            else:
                mesh_pose_msgs.append(pose_msg(**p))

    if len(mesh_msgs) != len(mesh_pose_msgs):
        raise ValueError(
            f"meshes ({len(mesh_msgs)}) and mesh_poses "
            f"({len(mesh_pose_msgs)}) must have the same length"
        )

    bv.primitives = primitive_msgs
    bv.primitive_poses = primitive_pose_msgs
    bv.meshes = mesh_msgs
    bv.mesh_poses = mesh_pose_msgs
    return bv


def _header_msg(
    *,
    header: Optional[Header | Mapping[str, Any]] = None,
    frame_id: Optional[str] = None,
    timestamp: Optional[Time | Mapping[str, Any]] = None,
) -> Header:
    """Build a std_msgs/Header from either `header` or `frame_id`/`timestamp`."""
    if header is not None:
        if frame_id is not None or timestamp is not None:
            raise ValueError(
                "Either header or (frame_id and/or timestamp) must be "
                "provided, but not both"
            )
        header = deepcopy(header)
        if isinstance(header, Header):
            return header
        return Header(**header)
    out = Header()
    if frame_id is not None:
        out.frame_id = frame_id
    if timestamp is not None:
        timestamp = deepcopy(timestamp)
        if isinstance(timestamp, Time):
            out.stamp = timestamp
        else:
            out.stamp = Time(**timestamp)
    return out


def position_constraint_msg(
    *,
    link_name: str,
    header: Optional[Header | Mapping[str, Any]] = None,
    frame_id: Optional[str] = None,
    timestamp: Optional[Time | Mapping[str, Any]] = None,
    target_point_offset: Optional[
        Vector3 | Iterable[float] | Mapping[str, float]
    ] = None,
    constraint_region: Optional[BoundingVolume | Mapping[str, Any]] = None,
    weight: float = 1.0,
) -> PositionConstraint:
    """Create a moveit_msgs/PositionConstraint message.

    Args:
        link_name: The link whose position is constrained.
        header: Complete Header or mapping with `frame_id`/`stamp` keys.
            Mutually exclusive with `frame_id`/`timestamp`.
        frame_id: Frame the position constraint is expressed in.
        timestamp: Optional Time stamp.
        target_point_offset: Offset of the constrained point within
            `link_name`'s frame. Accepts Vector3, iterable, or mapping.
        constraint_region: Allowed region. Accepts a BoundingVolume or
            a mapping consumed by `bounding_volume_msg()`.
        weight: Relative weight of this constraint.

    Returns:
        A PositionConstraint message.
    """
    pc = PositionConstraint()
    pc.header = _header_msg(
        header=header, frame_id=frame_id, timestamp=timestamp
    )
    pc.link_name = link_name
    if target_point_offset is not None:
        pc.target_point_offset = _vector3_msg(target_point_offset)
    if constraint_region is not None:
        constraint_region = deepcopy(constraint_region)
        if isinstance(constraint_region, BoundingVolume):
            pc.constraint_region = constraint_region
        else:
            pc.constraint_region = bounding_volume_msg(**constraint_region)
    pc.weight = float(weight)
    return pc


def orientation_constraint_msg(
    *,
    link_name: str,
    header: Optional[Header | Mapping[str, Any]] = None,
    frame_id: Optional[str] = None,
    timestamp: Optional[Time | Mapping[str, Any]] = None,
    orientation: Optional[
        Quaternion | Iterable[float] | Mapping[str, float]
    ] = None,
    rpy: Optional[Iterable[float] | Mapping[str, float]] = None,
    absolute_x_axis_tolerance: float = 0.0,
    absolute_y_axis_tolerance: float = 0.0,
    absolute_z_axis_tolerance: float = 0.0,
    parameterization: int = OrientationConstraint.XYZ_EULER_ANGLES,
    weight: float = 1.0,
) -> OrientationConstraint:
    """Create a moveit_msgs/OrientationConstraint message.

    Args:
        link_name: The link whose orientation is constrained.
        header: Complete Header or mapping.
        frame_id: Frame the orientation is expressed in (mutually
            exclusive with header).
        timestamp: Optional Time stamp.
        orientation: Target orientation as Quaternion, iterable [w,x,y,z],
            or mapping. Mutually exclusive with `rpy`.
        rpy: Target orientation as Euler angles [roll, pitch, yaw] in radians.
        absolute_x_axis_tolerance: Allowed deviation about the x axis.
        absolute_y_axis_tolerance: Allowed deviation about the y axis.
        absolute_z_axis_tolerance: Allowed deviation about the z axis.
        parameterization: How to interpret the per-axis tolerances
            (XYZ_EULER_ANGLES or ROTATION_VECTOR).
        weight: Relative weight of this constraint.

    Returns:
        An OrientationConstraint message.
    """
    oc = OrientationConstraint()
    oc.header = _header_msg(
        header=header, frame_id=frame_id, timestamp=timestamp
    )
    oc.link_name = link_name

    if orientation is not None and rpy is not None:
        raise ValueError("orientation and rpy cannot both be provided")
    if rpy is not None:
        rpy = deepcopy(rpy)
        if isinstance(rpy, Mapping):
            oc.orientation = quaternion_msg_from_euler(**rpy)  # type: ignore
        else:
            oc.orientation = quaternion_msg_from_euler(*rpy)
    elif orientation is not None:
        orientation = deepcopy(orientation)
        if isinstance(orientation, Quaternion):
            oc.orientation = normalize_quaternion_msg(orientation)
        elif isinstance(orientation, Mapping):
            oc.orientation = quaternion_msg(**orientation)  # type: ignore
        elif is_iterable(orientation):
            oc.orientation = quaternion_msg(*orientation)
        else:
            raise ValueError(f"Invalid orientation type: {type(orientation)}")

    oc.absolute_x_axis_tolerance = float(absolute_x_axis_tolerance)
    oc.absolute_y_axis_tolerance = float(absolute_y_axis_tolerance)
    oc.absolute_z_axis_tolerance = float(absolute_z_axis_tolerance)
    oc.parameterization = int(parameterization)
    oc.weight = float(weight)
    return oc


def visibility_constraint_msg(
    *,
    target_radius: float,
    target_pose: PoseStamped | Mapping[str, Any],
    sensor_pose: PoseStamped | Mapping[str, Any],
    cone_sides: int = 4,
    max_view_angle: float = 0.0,
    max_range_angle: float = 0.0,
    sensor_view_direction: int = VisibilityConstraint.SENSOR_Z,
    weight: float = 1.0,
) -> VisibilityConstraint:
    """Create a moveit_msgs/VisibilityConstraint message.

    Args:
        target_radius: Radius of the target visibility disk (m).
        target_pose: Pose of the target. Accepts PoseStamped or a mapping
            consumed by `pose_stamped_msg()`.
        sensor_pose: Pose of the sensor. Accepts PoseStamped or mapping.
        cone_sides: Number of sides used to approximate the visibility cone.
        max_view_angle: Max allowed view angle (rad). 0 disables the check.
        max_range_angle: Max allowed range angle (rad). 0 disables the check.
        sensor_view_direction: Which sensor axis the cone opens along
            (SENSOR_Z, SENSOR_Y, SENSOR_X).
        weight: Relative weight of this constraint.

    Returns:
        A VisibilityConstraint message.
    """
    vc = VisibilityConstraint()
    vc.target_radius = float(target_radius)
    target_pose = deepcopy(target_pose)
    if isinstance(target_pose, PoseStamped):
        vc.target_pose = target_pose
    else:
        vc.target_pose = pose_stamped_msg(**target_pose)
    sensor_pose = deepcopy(sensor_pose)
    if isinstance(sensor_pose, PoseStamped):
        vc.sensor_pose = sensor_pose
    else:
        vc.sensor_pose = pose_stamped_msg(**sensor_pose)
    vc.cone_sides = int(cone_sides)
    vc.max_view_angle = float(max_view_angle)
    vc.max_range_angle = float(max_range_angle)
    vc.sensor_view_direction = int(sensor_view_direction)
    vc.weight = float(weight)
    return vc


def goal_constraints_from_pose_stamped(
    *,
    link_name: str,
    pose_stamped: PoseStamped | Mapping[str, Any],
    tolerance_pos: float | Iterable[float] = 1e-3,
    tolerance_angle: float | Iterable[float] = 1e-2,
    weight: float = 1.0,
) -> Constraints:
    """Build goal constraints for a link from a Cartesian pose goal.

    This is a Python reimplementation of MoveIt's
    ``kinematic_constraints::constructGoalConstraints(link_name, pose, ...)``.
    The returned message contains exactly one `PositionConstraint` and one
    `OrientationConstraint`, both referencing ``link_name`` and expressed in
    the pose's header frame.

    The shape of ``tolerance_pos`` selects the position constraint region,
    matching the two C++ overloads:
    - A scalar is the radius of a SPHERE centered on the goal position.
    - A 3-element iterable is the [x, y, z] extents of a BOX centered on the
      goal position.

    The orientation constraint always uses ROTATION_VECTOR parameterization
    (as the C++ does), so a scalar ``tolerance_angle`` is an isotropic ball
    applied to all three axes, while a 3-element iterable sets per-axis bounds.

    Args:
        link_name: The link constrained by both sub-constraints.
        pose_stamped: The Cartesian goal. Accepts a PoseStamped or a mapping
            consumed by `pose_stamped_msg()`. Its header frame is propagated
            to both constraints.
        tolerance_pos: Sphere radius (scalar) or box xyz extents (3-iterable)
            for the position constraint region.
        tolerance_angle: Isotropic (scalar) or per-axis (3-iterable) absolute
            orientation tolerance.
        weight: Relative weight applied to both sub-constraints.

    Returns:
        A Constraints message with one position and one orientation constraint.

    Raises:
        ValueError: If a vector tolerance does not have exactly three elements.
    """
    if not isinstance(pose_stamped, PoseStamped):
        pose_stamped = pose_stamped_msg(**pose_stamped)

    header = pose_stamped.header
    goal_position = pose_stamped.pose.position
    goal_orientation = pose_stamped.pose.orientation

    # A scalar position tolerance is a sphere radius; a 3-vector is box dims.
    if is_iterable(tolerance_pos):
        dimensions = [float(t) for t in tolerance_pos]  # type: ignore
        if len(dimensions) != 3:
            raise ValueError(
                "tolerance_pos must be a scalar (sphere radius) or a "
                f"3-element iterable (box xyz extents), got {len(dimensions)}"
            )
        primitive: dict[str, Any] = {"type": "BOX", "dimensions": dimensions}
    else:
        primitive = {"type": "SPHERE", "dimensions": [float(tolerance_pos)]}  # type: ignore

    # The constraint-region primitive is centered on the goal position. Its
    # orientation is irrelevant for a sphere and held at identity for a box.
    region_pose = pose_msg(
        position=goal_position, orientation=(1.0, 0.0, 0.0, 0.0)
    )
    position_constraint = position_constraint_msg(
        link_name=link_name,
        header=header,
        constraint_region=bounding_volume_msg(
            primitives=[primitive],
            primitive_poses=[region_pose],
        ),
        weight=weight,
    )

    # A scalar angular tolerance is applied isotropically to all three axes.
    if is_iterable(tolerance_angle):
        angles = [float(t) for t in tolerance_angle]  # type: ignore
        if len(angles) != 3:
            raise ValueError(
                "tolerance_angle must be a scalar or a 3-element iterable "
                f"(xyz), got {len(angles)}"
            )
    else:
        angles = [float(tolerance_angle)] * 3  # type: ignore

    orientation_constraint = orientation_constraint_msg(
        link_name=link_name,
        header=header,
        orientation=goal_orientation,
        absolute_x_axis_tolerance=angles[0],
        absolute_y_axis_tolerance=angles[1],
        absolute_z_axis_tolerance=angles[2],
        parameterization=OrientationConstraint.ROTATION_VECTOR,
        weight=weight,
    )

    return Constraints(
        position_constraints=[position_constraint],
        orientation_constraints=[orientation_constraint],
    )


def goal_constraints_from_robot_state(
    *,
    robot_state: RobotState,
    group_name: str,
    tolerance: float = float(np.finfo(float).eps),
    tolerance_above: Optional[float] = None,
    tolerance_below: Optional[float] = None,
    weight: float = 1.0,
) -> Constraints:
    """Build joint-space goal constraints from a RobotState.

    This is a Python reimplementation of MoveIt's
    ``kinematic_constraints::constructGoalConstraints(state, jmg, ...)``. It
    emits one `JointConstraint` per active joint of ``group_name``, each
    pinned to that joint's position in ``robot_state``.

    Args:
        robot_state: The state from which to read the goal joint positions.
        group_name: The joint model group whose active joints are constrained.
        tolerance: Symmetric tolerance applied above and below each joint
            position. Defaults to machine epsilon (an effectively exact goal),
            matching the C++ default. Ignored for a side when the
            corresponding ``tolerance_above``/``tolerance_below`` is given.
        tolerance_above: Optional override for the above tolerance on every
            joint.
        tolerance_below: Optional override for the below tolerance on every
            joint.
        weight: Relative weight applied to every joint constraint.

    Returns:
        A Constraints message containing one joint constraint per active joint.
    """
    above = tolerance if tolerance_above is None else tolerance_above
    below = tolerance if tolerance_below is None else tolerance_below
    positions = get_joint_group_positions(robot_state, group_name)
    joint_constraints = [
        joint_constraint_msg(
            joint_name=joint_name,
            position=position,
            tolerance_above=above,
            tolerance_below=below,
            weight=weight,
        )
        for joint_name, position in positions.items()
    ]
    return Constraints(joint_constraints=joint_constraints)


def constraints_msg(
    *,
    name: str = "",
    robot_state_goal: Optional[Mapping[str, Any]] = None,
    pose_goal: Optional[Mapping[str, Any]] = None,
    joint_constraints: Optional[
        Iterable[JointConstraint | Mapping[str, Any]]
    ] = None,
    position_constraints: Optional[
        Iterable[PositionConstraint | Mapping[str, Any]]
    ] = None,
    orientation_constraints: Optional[
        Iterable[OrientationConstraint | Mapping[str, Any]]
    ] = None,
    visibility_constraints: Optional[
        Iterable[VisibilityConstraint | Mapping[str, Any]]
    ] = None,
) -> Constraints:
    """Create a moveit_msgs/Constraints message.

    A `Constraints` is an AND of every sub-constraint it contains; the
    planner-facing goal is a *list* of these, interpreted as an OR of
    alternatives.

    Each sub-constraint list accepts either pre-built messages or
    plain mappings (which are routed through the matching `*_constraint_msg()`
    factory). The latter form lets YAML files express constraints declaratively
    without needing their own YAML tags.

    Two convenience goal forms are also accepted, mirroring MoveIt's
    ``constructGoalConstraints`` helpers:
    - ``pose_goal`` derives a position + orientation constraint for a link
      from a Cartesian pose (see `goal_constraints_from_pose_stamped`). The
      resulting sub-constraints are appended to any explicitly provided
      position/orientation constraints.
    - ``robot_state_goal`` derives a full set of joint constraints from a
      RobotState (see `goal_constraints_from_robot_state`). Because this is a
      complete joint-space goal, it is mutually exclusive with every other
      argument that contributes constraints.

    Args:
        name: Optional identifier for this Constraints set.
        joint_constraints: Sub-constraints on joint positions.
        position_constraints: Sub-constraints on link positions.
        orientation_constraints: Sub-constraints on link orientations.
        visibility_constraints: Sub-constraints for sensor visibility.
        pose_goal: Mapping of keyword arguments forwarded to
            `goal_constraints_from_pose_stamped()` (e.g. ``link_name``,
            ``pose_stamped``, ``tolerance_pos``, ``tolerance_angle``).
            Mutually exclusive with ``robot_state_goal``.
        robot_state_goal: Mapping of keyword arguments forwarded to
            `goal_constraints_from_robot_state()` (e.g. ``robot_state``,
            ``group_name``, ``tolerance``). Mutually exclusive with every
            other constraint-bearing argument.

    Returns:
        A Constraints message.

    Raises:
        ValueError: If both ``pose_goal`` and ``robot_state_goal`` are given,
            or if ``robot_state_goal`` is combined with any other constraint.
    """
    if pose_goal is not None and robot_state_goal is not None:
        raise ValueError(
            "pose_goal and robot_state_goal cannot both be provided"
        )

    if robot_state_goal is not None and any(
        c is not None
        for c in (
            pose_goal,
            joint_constraints,
            position_constraints,
            orientation_constraints,
            visibility_constraints,
        )
    ):
        raise ValueError(
            "robot_state_goal is a complete joint-space goal and cannot be "
            "combined with any other constraints"
        )

    jcs: list[JointConstraint] = []
    pcs: list[PositionConstraint] = []
    ocs: list[OrientationConstraint] = []
    vcs: list[VisibilityConstraint] = []

    if robot_state_goal is not None:
        goal_constraints = goal_constraints_from_robot_state(
            **robot_state_goal
        )
        jcs.extend(goal_constraints.joint_constraints)

    if pose_goal is not None:
        goal_constraints = goal_constraints_from_pose_stamped(**pose_goal)
        pcs.extend(goal_constraints.position_constraints)
        ocs.extend(goal_constraints.orientation_constraints)

    if joint_constraints is not None:
        for c in joint_constraints:
            if isinstance(c, JointConstraint):
                jcs.append(deepcopy(c))
            else:
                jcs.append(joint_constraint_msg(**c))

    if position_constraints is not None:
        for c in position_constraints:
            if isinstance(c, PositionConstraint):
                pcs.append(deepcopy(c))
            else:
                pcs.append(position_constraint_msg(**c))

    if orientation_constraints is not None:
        for c in orientation_constraints:
            if isinstance(c, OrientationConstraint):
                ocs.append(deepcopy(c))
            else:
                ocs.append(orientation_constraint_msg(**c))

    if visibility_constraints is not None:
        for c in visibility_constraints:
            if isinstance(c, VisibilityConstraint):
                vcs.append(deepcopy(c))
            else:
                vcs.append(visibility_constraint_msg(**c))

    constraints = Constraints(
        name=name,
        joint_constraints=jcs,
        position_constraints=pcs,
        orientation_constraints=ocs,
        visibility_constraints=vcs,
    )

    return constraints


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
    if not all_close_points(
        pose1.position, pose2.position, position_tolerance
    ):
        return False

    if not all_close_quaternions(
        pose1.orientation, pose2.orientation, orientation_tolerance
    ):
        return False

    return True


def all_close_poses_stamped(
    pose_stamped1: PoseStamped,
    pose_stamped2: PoseStamped,
    *,
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


def get_joint_group_positions(
    state: RobotState, group_name: str
) -> dict[str, float]:
    joint_names: list[str] = state.robot_model.get_joint_model_group(
        group_name
    ).active_joint_model_names
    positions = state.joint_positions
    return {x: positions[x] for x in joint_names}


def get_joint_group_velocities(
    state: RobotState, group_name: str
) -> dict[str, float]:
    joint_names: list[str] = state.robot_model.get_joint_model_group(
        group_name
    ).active_joint_model_names
    velocities = state.joint_velocities
    return {x: velocities[x] for x in joint_names}


def get_joint_group_accelerations(
    state: RobotState, group_name: str
) -> dict[str, float]:
    joint_names: list[str] = state.robot_model.get_joint_model_group(
        group_name
    ).active_joint_model_names
    accelerations = state.joint_accelerations
    return {x: accelerations[x] for x in joint_names}


def all_close_robot_states(
    state1: RobotState,
    state2: RobotState,
    *,
    group_name: str | Literal["all"],
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
    if group_name == "all":
        positions1 = copy(state1.joint_positions)
        positions2 = copy(state2.joint_positions)
    else:
        positions1 = get_joint_group_positions(state1, group_name)
        positions2 = get_joint_group_positions(state2, group_name)

    # diffs: dict[str, float] = {}
    # for joint in positions1.keys():
    #     diffs[joint] = (positions1[joint] - positions2[joint] + np.pi) % (
    #         2 * np.pi
    #     ) - np.pi
    #
    # if not all_close_dicts(
    #     diffs, {k: 0 for k in diffs.keys()}, position_tolerance
    # ):
    #     return False

    if not all_close_dicts(positions1, positions2, position_tolerance):
        return False

    if velocity_tolerance is not None:
        if group_name == "all":
            velocities1 = state1.joint_velocities
            velocities2 = state2.joint_velocities
        else:
            velocities1 = get_joint_group_velocities(state1, group_name)
            velocities2 = get_joint_group_velocities(state2, group_name)

        if not all_close_dicts(velocities1, velocities2, velocity_tolerance):
            return False

    if acceleration_tolerance is not None:
        if group_name == "all":
            accelerations1 = state1.joint_accelerations
            accelerations2 = state2.joint_accelerations
        else:
            accelerations1 = get_joint_group_accelerations(state1, group_name)
            accelerations2 = get_joint_group_accelerations(state2, group_name)

        if not all_close_dicts(
            accelerations1, accelerations2, acceleration_tolerance
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

    if subframe_names is not None or subframe_poses is not None:
        if subframe_names is None or subframe_poses is None:
            raise ValueError(
                "Both 'subframe_names' and 'subframe_poses' must be provided if one is provided"
            )
        if len(subframe_names) != len(subframe_poses):
            raise ValueError(
                "Number of 'subframe_names' and 'subframe_poses' must match"
            )
        collision_object.subframe_names = subframe_names
        collision_object.subframe_poses = subframe_poses

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
            (e.g., "left_manipulator", "right_manipulator") this trajectory
            applies to.

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
