import os
from collections.abc import Iterable, Mapping
from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
from typing import Any, Literal, Optional, Protocol

import numpy as np
import rclpy
import trimesh
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit.core.controller_manager import ExecutionStatus  # type: ignore
from moveit.core.planning_scene import PlanningScene  # type: ignore
from moveit.core.robot_state import RobotState  # type: ignore
from moveit.core.robot_trajectory import RobotTrajectory  # type: ignore
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    MoveItErrorCodes,
    ObjectColor,
)
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
from rclpy.client import Client
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.impl.rcutils_logger import RcutilsLogger
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

MOVEIT_ERROR_CODE_MAP = {
    v: k
    for k, v in type(
        MoveItErrorCodes
    )._Metaclass_MoveItErrorCodes__constants.items()  # type: ignore
}
"""MoveIt error code map from error code to string, for logging."""


COLOR_MAP = {
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
"""RGBA color map from color name to RGBA tuple."""

COLLISION_OBJECT_OPERATION_MAP = {
    "ADD": CollisionObject.ADD,
    "REMOVE": CollisionObject.REMOVE,
    "APPEND": CollisionObject.APPEND,
    "MOVE": CollisionObject.MOVE,
}
"""Collision object operation map from operation name to collision object operation."""

SOLID_PRIMITIVE_TYPE_MAP = {
    "BOX": SolidPrimitive.BOX,
    "SPHERE": SolidPrimitive.SPHERE,
    "CYLINDER": SolidPrimitive.CYLINDER,
    "CONE": SolidPrimitive.CONE,
    "PRISM": SolidPrimitive.PRISM,
}
"""Solid primitive type map from type name to solid primitive type."""

# Logging utilities


def ros_log(
    logger: RcutilsLogger,
    message: Any,
    severity: str | LoggingSeverity = "INFO",
    **kwargs,
):
    """Log a message with the given severity."""

    if not isinstance(severity, LoggingSeverity):
        severity = LoggingSeverity[severity]

    if rclpy.ok():  # type: ignore
        match severity:
            case LoggingSeverity.DEBUG:
                return logger.debug(message, **kwargs)
            case LoggingSeverity.INFO:
                return logger.info(message, **kwargs)
            case LoggingSeverity.WARN:
                return logger.warning(message, **kwargs)
            case LoggingSeverity.ERROR:
                return logger.error(message, **kwargs)
            case LoggingSeverity.FATAL:
                return logger.fatal(message, **kwargs)
            case _:
                raise ValueError(f"Invalid severity: {severity}")
    elif severity >= logger.get_effective_level():
        print(f"{severity.name}: {message}")
        return True
    else:
        return False


# ROS message utilities


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


# Protocol definitions


class SrvTypeRequest(Protocol):
    """Protocol for a ROS2 service request type."""


class SrvTypeResponse(Protocol):
    """Protocol for a ROS2 service response type."""

    success: bool


class SrvType(Protocol):
    """Protocol for a ROS2 service type."""

    Request: Any
    Response: Any


# Enums


class TrajectoryErrorCodes(Enum):
    """Trajectory error codes."""

    TOTG_FAILED = -1
    SMOOTHING_FAILED = -2
    INVALID_TRAJECTORY = -3


# Exception definitions


class ROSSleepError(Exception):
    """Error while sleeping in a ROS node."""


class ServiceCallTimeoutError(Exception):
    """Service call timed out."""


class ServiceCallUnsuccessfulError(Exception):
    """Service call returned with a failure status."""


class ActionCallUnsuccessfulError(Exception):
    """Action call failed."""


class CommanderRecoverableError(Exception):
    """Recoverable error that can be retried."""


class PlanningError(CommanderRecoverableError):
    """Planning error."""


class PlanOnceError(PlanningError):
    """Planning error."""

    def __init__(self, error_code: MoveItErrorCodes):
        self.error_code = error_code
        super().__init__(
            f"Plan once error: {MOVEIT_ERROR_CODE_MAP[error_code.val]}"
        )

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, PlanOnceError):
            return self.error_code.val == other.error_code.val
        return False


class MaxPlanningAttemptsReachedError(PlanningError):
    """Maximum number of planning attempts reached."""

    def __init__(self, errors: list[PlanOnceError]):
        self.errors = errors
        if all(e == errors[0] for e in errors):
            error_code_str = f"same error: {errors[0]}"
        else:
            error_code_strs = [str(e) for e in errors]
            error_code_str = f"different errors: {error_code_strs}"
        super().__init__(
            f"Max planning attempts ({len(errors)}) reached with {error_code_str}"
        )


class ExecutionError(CommanderRecoverableError):
    """Execution error."""


class TrajectoryError(ExecutionError):
    """Trajectory error."""

    def __init__(self, error_code: TrajectoryErrorCodes):
        self.error_code = error_code
        super().__init__(f"Trajectory error: {error_code}")

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, TrajectoryError):
            return self.error_code == other.error_code
        return False


class ExecutionRejectedError(ExecutionError):
    """Execution rejected (robot did not move)."""

    def __init__(self, execution_status: ExecutionStatus):
        self.execution_status = execution_status
        super().__init__(f"Execution rejected: {execution_status.status}")

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, ExecutionRejectedError):
            return (
                self.execution_status.status == other.execution_status.status
            )
        return False


class ExecutionInterruptedError(ExecutionError):
    """Execution interrupted (robot moved but not to the goal)."""

    def __init__(self, execution_status: ExecutionStatus):
        self.execution_status = execution_status
        super().__init__(f"Execution interrupted: {execution_status.status}")


class NotSafeToExecuteError(ExecutionError):
    """Not safe to execute."""

    def __init__(self, execution_status: Optional[ExecutionStatus] = None):
        self.execution_status = execution_status
        msg = "Not safe to execute"
        if execution_status is not None:
            msg += f": {execution_status.status}"
        super().__init__(msg)


class ObjectManipulationError(CommanderRecoverableError):
    """Error while manipulating object."""


# Type aliases


PlanningGoalT = RobotState | PoseStamped | str


# Request definitions


@dataclass(slots=True, kw_only=True)
class PlanRequest:
    """Request for a plan."""

    goal: RobotState | PoseStamped
    start_state: RobotState
    pose_link: str
    group_name: str
    planning_pipeline: str
    path_constraints: Constraints | None
    planning_scene: PlanningScene | None
    max_plan_attempts: int

    def __post_init__(self):
        """Type check the request."""
        for name in PlanRequest.__slots__:
            self._validate_attribute(name, getattr(self, name))

    def _validate_attribute(self, name: str, value: Any):
        """Check the types of the request."""
        if name not in PlanRequest.__slots__:
            raise AttributeError(f"Invalid attribute: {name}")

        match name:
            case "goal":
                if not isinstance(value, (RobotState, PoseStamped)):
                    raise ValueError(f"Invalid goal type: {type(value)}")
            case "start_state":
                if not isinstance(value, RobotState):
                    raise ValueError(
                        f"Invalid start state type: {type(value)}"
                    )
            case "pose_link":
                if not isinstance(value, str):
                    raise ValueError(f"Invalid pose link type: {type(value)}")
            case "group_name":
                if not isinstance(value, str):
                    raise ValueError(f"Invalid group name type: {type(value)}")
            case "planning_pipeline":
                if not isinstance(value, str):
                    raise ValueError(
                        f"Invalid planning pipeline type: {type(value)}"
                    )
            case "path_constraints":
                if value is not None and not isinstance(value, Constraints):
                    raise ValueError(
                        f"Invalid path constraints type: {type(value)}"
                    )
            case "planning_scene":
                if value is not None and not isinstance(value, PlanningScene):
                    raise ValueError(
                        f"Invalid planning scene type: {type(value)}"
                    )
            case "max_plan_attempts":
                if not isinstance(value, int):
                    raise ValueError(
                        f"Invalid max plan attempts type: {type(value)}"
                    )
            case _:
                raise ValueError(f"Invalid attribute: {name}")

    def __setattr__(self, name: str, value: Any) -> None:
        """Set an attribute."""
        self._validate_attribute(name, value)
        object.__setattr__(self, name, value)


@dataclass(slots=True)
class ExecuteRequest:
    """Request for an execute."""

    trajectory: RobotTrajectory
    validate_trajectory: bool
    apply_totg: bool
    apply_smoothing: bool
    velocity_scaling_factor: float
    acceleration_scaling_factor: float
    path_tolerance: float
    resample_dt: float
    min_angle_change: float
    mitigate_overshoot: bool
    overshoot_threshold: float

    def __post_init__(self):
        """Type check the request."""
        for name in ExecuteRequest.__slots__:
            self._validate_attribute(name, getattr(self, name))

    def _validate_attribute(self, name: str, value: Any):
        """Check the types of the request."""
        if name not in ExecuteRequest.__slots__:
            raise AttributeError(f"Invalid attribute: {name}")

        match name:
            case "trajectory":
                if not isinstance(value, RobotTrajectory):
                    raise ValueError(f"Invalid trajectory type: {type(value)}")
            case "validate_trajectory":
                if not isinstance(value, bool):
                    raise ValueError(
                        f"Invalid validate trajectory type: {type(value)}"
                    )
            case "apply_totg":
                if not isinstance(value, bool):
                    raise ValueError(f"Invalid apply totg type: {type(value)}")
            case "apply_smoothing":
                if not isinstance(value, bool):
                    raise ValueError(
                        f"Invalid apply smoothing type: {type(value)}"
                    )
            case "velocity_scaling_factor":
                if not isinstance(value, (float, int)):
                    raise ValueError(
                        f"Invalid velocity scaling factor type: {type(value)}"
                    )
            case "acceleration_scaling_factor":
                if not isinstance(value, (float, int)):
                    raise ValueError(
                        f"Invalid acceleration scaling factor type: {type(value)}"
                    )
            case "path_tolerance":
                if not isinstance(value, (float, int)):
                    raise ValueError(
                        f"Invalid path tolerance type: {type(value)}"
                    )
            case "resample_dt":
                if not isinstance(value, (float, int)):
                    raise ValueError(
                        f"Invalid resample dt type: {type(value)}"
                    )
            case "min_angle_change":
                if not isinstance(value, (float, int)):
                    raise ValueError(
                        f"Invalid min angle change type: {type(value)}"
                    )
            case "mitigate_overshoot":
                if not isinstance(value, bool):
                    raise ValueError(
                        f"Invalid mitigate overshoot type: {type(value)}"
                    )
            case "overshoot_threshold":
                if not isinstance(value, (float, int)):
                    raise ValueError(
                        f"Invalid overshoot threshold type: {type(value)}"
                    )
            case "max_execution_attempts":
                if not isinstance(value, int):
                    raise ValueError(
                        f"Invalid max execution attempts type: {type(value)}"
                    )
            case _:
                raise ValueError(f"Invalid attribute: {name}")

    def __setattr__(self, name: str, value: Any) -> None:
        """Set an attribute."""
        self._validate_attribute(name, value)
        object.__setattr__(self, name, value)


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
    response: SrvTypeResponse | None,
    service_client: Client,
) -> None:
    """Validate the response from a service call.

    Args:
        response: The response from a service call.
        service_client: The client that made the service call.

    Returns:
        The response from the service call.

    Raises:
        ServiceCallTimeoutError: If the service call timed out.
        ServiceCallUnsuccessfulError: If the service call returned with a failure status.
    """
    if response is None:
        error_msg = f"{service_client.service_name} service call timed out!"
        raise ServiceCallTimeoutError(error_msg)
    elif hasattr(response, "success") and not response.success:  # type: ignore
        error_msg = (
            f"{service_client.service_name} service call returned "
            f"unsuccessfully with response: {msg_to_dict(response)}"
        )
        raise ServiceCallUnsuccessfulError(error_msg)


# ROS2 time utilities


def seconds_from_ros_time(timestamp: Time | TimeMsg) -> float:
    """Convert a ROS2 Time message to seconds."""
    if isinstance(timestamp, Time):
        return timestamp.nanoseconds / 1e9
    elif isinstance(timestamp, TimeMsg):
        return float(timestamp.sec) + float(timestamp.nanosec) / 1e9
    else:
        raise ValueError(f"Invalid timestamp type: {type(timestamp).__name__}")


def time_msg_from_seconds(seconds: float) -> TimeMsg:
    """Convert seconds to a ROS2 Time message."""
    return TimeMsg(
        sec=int(seconds), nanosec=int((seconds - int(seconds)) * 1e9)
    )


# ROS2 geometric message utilities


def array_from_point_msg(point: Point) -> np.ndarray:
    """Convert a geometry_msgs/Point message to a numpy array."""
    return np.array([point.x, point.y, point.z])


def array_from_quaternion_msg(quaternion: Quaternion) -> np.ndarray:
    """Convert a geometry_msgs/Quaternion message to a normalized numpy array."""
    q = np.array([quaternion.w, quaternion.x, quaternion.y, quaternion.z])
    q = q / np.linalg.norm(q)
    return q


def arrays_from_pose_msg(
    pose: Pose, *, euler: bool = False, axes: str = "sxyz"
) -> tuple[np.ndarray, np.ndarray]:
    """Convert a geometry_msgs/Pose message to position and normalized quaternion arrays."""
    position = array_from_point_msg(pose.position)
    if euler:
        orientation = euler_array_from_quaternion_msg(pose.orientation, axes)
    else:
        orientation = array_from_quaternion_msg(pose.orientation)
    return position, orientation


def quaternion_msg(w: float, x: float, y: float, z: float) -> Quaternion:
    """Convert a quaternion to a geometry_msgs/Quaternion message."""
    q = np.array([w, x, y, z])
    q = q / np.linalg.norm(q)
    w, x, y, z = (float(p) for p in q)
    return Quaternion(w=w, x=x, y=y, z=z)


def quaternion_msg_from_euler(
    roll: float, pitch: float, yaw: float, *, axes: str = "sxyz"
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
    return quaternion_msg(*quaternion_from_euler(roll, pitch, yaw, axes))


def quaternion_msg_from_axis_angle(
    axis: Iterable[float], angle: float
) -> Quaternion:
    """Convert an axis and angle to a geometry_msgs/Quaternion message."""
    return quaternion_msg(*quaternion_about_axis(angle, axis))


def normalize_quaternion_msg(quaternion: Quaternion) -> Quaternion:
    """Convert a quaternion to a normalized geometry_msgs/Quaternion message."""
    return quaternion_msg(
        quaternion.w, quaternion.x, quaternion.y, quaternion.z
    )


def euler_array_from_quaternion_msg(
    quaternion: Quaternion, axes: str = "sxyz"
) -> np.ndarray:
    """Convert a geometry_msgs/Quaternion message to roll, pitch, yaw angles (in radians)."""
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
    """Convert a dictionary of parameters to a geometry_msgs/Pose message."""
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
                "Either pose or position/rpy/orientation must be provided, "
                "but not both"
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
    """Check if two arrays are close to each other."""
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
    """Check if two dictionaries are close to each other."""
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
    """Check if two points are close to each other."""
    p1_array = array_from_point_msg(p1)
    p2_array = array_from_point_msg(p2)
    return all_close_iterables(p1_array, p2_array, tolerance)


def all_close_quaternions(
    q1: Quaternion, q2: Quaternion, tolerance: float | Iterable[float]
) -> bool:
    """Check if two quaternions are close to each other."""
    q1_array = array_from_quaternion_msg(q1)
    q2_array = array_from_quaternion_msg(q2)
    return all_close_iterables(q1_array, q2_array, tolerance)


def all_close_poses(
    pose1: Pose,
    pose2: Pose,
    position_tolerance: float | Iterable[float] | np.ndarray,
    orientation_tolerance: float | Iterable[float] | np.ndarray,
    use_euler_tolerance: bool = False,
) -> bool:
    """Check if two poses are close to each other.

    Args:
        pose1: The first pose.
        pose2: The second pose.
        position_tolerance: The tolerance for the position.
        orientation_tolerance: The quaternion or euler angle tolerance for the orientation.
        use_euler_tolerance: Whether to use euler tolerance instead of quaternion tolerance.
    """
    all_close_positions = all_close_points(
        pose1.position, pose2.position, position_tolerance
    )

    if use_euler_tolerance:
        euler_angles1 = euler_array_from_quaternion_msg(pose1.orientation)
        euler_angles2 = euler_array_from_quaternion_msg(pose2.orientation)
        all_close_orientations = all_close_iterables(
            euler_angles1, euler_angles2, orientation_tolerance
        )
    else:
        all_close_orientations = all_close_quaternions(
            pose1.orientation, pose2.orientation, orientation_tolerance
        )

    return all_close_positions and all_close_orientations


def all_close_poses_stamped(
    pose_stamped1: PoseStamped,
    pose_stamped2: PoseStamped,
    position_tolerance: float | Iterable[float] | np.ndarray,
    orientation_tolerance: float | Iterable[float] | np.ndarray,
    use_euler_tolerance: bool = False,
) -> bool:
    """Check if two poses are close to each other.

    Args:
        pose_stamped1: The first pose.
        pose_stamped2: The second pose.
        *args: Arguments to pass to all_close_poses.
        **kwargs: Keyword arguments to pass to all_close_poses.

    Returns:
        True if the poses are close to each other, False otherwise.
    """
    if pose_stamped1.header.frame_id != pose_stamped2.header.frame_id:
        raise ValueError("PoseStamped messages must have the same frame_id")
    return all_close_poses(
        pose_stamped1.pose,
        pose_stamped2.pose,
        position_tolerance,
        orientation_tolerance,
        use_euler_tolerance,
    )


def all_close_robot_states(
    state1: RobotState,
    state2: RobotState,
    position_tolerance: float | dict[str, float],
    velocity_tolerance: Optional[float | dict[str, float]] = None,
    acceleration_tolerance: Optional[float | dict[str, float]] = None,
) -> bool:
    """Check if two robot states are close to each other."""
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
    """Convert a 4x4 transformation matrix to a geometry_msgs/Pose message."""
    return pose_msg(
        position=translation_from_matrix(matrix),
        orientation=quaternion_from_matrix(matrix),
    )


def matrix_from_point_msg(point: Point) -> np.ndarray:
    """Convert a geometry_msgs/Point message to a 4x4 transformation matrix."""
    return translation_matrix([point.x, point.y, point.z])


def matrix_from_quaternion_msg(quaternion: Quaternion) -> np.ndarray:
    """Convert a geometry_msgs/Quaternion message to a 4x4 transformation matrix."""
    return quaternion_matrix(
        [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
    )


def matrix_from_pose_msg(pose: Pose | Mapping[str, Any]) -> np.ndarray:
    """Convert a geometry_msgs/Pose message to a 4x4 transformation matrix."""
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
    """Change the reference frame of a pose."""
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
    """Transforms a pose from one frame to another.

    Args:
        old_pose_stamped (PoseStamped): The pose to transform.
        old_frame_transform (np.ndarray): The transform from the old frame to the world frame.
        new_frame_transform (np.ndarray): The transform from the new frame to the world frame.
        new_frame_id (str): The ID of the new frame.
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
    """Create a collision object message."""
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
    """Create a collision object from a plane."""
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
    """Create a collision object from a primitive.

    Args:
        object_id (str): The ID of the collision object.
        pose_stamped (PoseStamped): The pose of the collision object.
        type (str): The type of the primitive.
        dimensions (list[float]): The dimensions of the primitive.
    """
    collision_object = add_collision_object_msg(
        object_id, pose_stamped, subframe_names, subframe_poses
    )
    collision_object.primitives.append(  # type: ignore
        SolidPrimitive(
            type=SOLID_PRIMITIVE_TYPE_MAP[type], dimensions=dimensions
        )
    )
    # collision_object.primitive_poses.append(pose_stamped.pose)  # type: ignore

    return collision_object


TrimeshPrimitive = (
    trimesh.primitives.Box
    | trimesh.primitives.Sphere
    | trimesh.primitives.Cylinder
)


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
    """Create a collision object from a trimesh geometry."""
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
    """Create a collision object from a trimesh geometry.

    Args:
        object_id (str): The ID of the collision object.
        mesh (trimesh.Trimesh | trimesh.Scene | MeshMsg): The trimesh geometry
            or mesh message to create the collision object from.
        pose_stamped (PoseStamped): The pose of the collision object.
        subframe_names (list[str]): The names of the subframes.
        subframe_poses (list[Pose]): The poses of the subframes
            (defined relative to `pose_stamped`).
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
    """Create an AttachedCollisionObject message."""
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
    """Create an ObjectColor message."""
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


def robot_trajectory_from_msg(
    trajectory_msg: RobotTrajectoryMsg,
    state: RobotState,
    joint_model_group_name: str,
) -> RobotTrajectory:
    """Convert a RobotTrajectory message to a RobotTrajectory object."""
    if state.dirty:
        raise ValueError("Robot state is dirty")
    trajectory = RobotTrajectory(state.robot_model)
    trajectory.set_robot_trajectory_msg(state, trajectory_msg)
    trajectory.joint_model_group_name = joint_model_group_name
    return trajectory


def robot_trajectory_copy(
    trajectory: RobotTrajectory,
) -> RobotTrajectory:
    """Copy a RobotTrajectory object."""
    state = trajectory[0]
    new_trajectory = RobotTrajectory(state.robot_model)
    new_trajectory.set_robot_trajectory_msg(
        state, trajectory.get_robot_trajectory_msg()
    )
    new_trajectory.joint_model_group_name = trajectory.joint_model_group_name
    return new_trajectory
