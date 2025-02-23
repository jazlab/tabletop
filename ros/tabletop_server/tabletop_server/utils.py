import inspect
import logging
import os
from collections import OrderedDict
from collections.abc import Callable, Coroutine, Iterable, Mapping
from typing import Any, Optional, Protocol

import numpy as np
import pyfqmr
import pyglet
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
from std_msgs.msg import ColorRGBA
from tf_transformations import (
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_from_matrix,
    translation_matrix,
)

moveit_error_code_to_str = {
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

color_name_to_rgba = {
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


# Protocol definitions
class SrvType(Protocol):
    Request: Any
    Response: Any


class SrvTypeRequest(Protocol):
    pass


class SrvTypeResponse(Protocol):
    success: bool


# Exception definitions
class MaxAttemptsReachedError(Exception):
    pass


class ServiceCallError(Exception):
    pass


# ROS2 utility functions
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path) as file:
        return yaml.safe_load(file)


def save_yaml(file_path, data):
    with open(file_path, "w") as file:
        yaml.dump(data, file, default_flow_style=True, sort_keys=False)


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


def is_iterable(obj: Any) -> bool:
    if isinstance(obj, (str, Mapping)):
        return False
    try:
        iter(obj)
    except Exception:
        return False
    return True


def pose_msg_from_dict(params: dict[str, Any]) -> Pose:
    """
    Convert a dictionary of parameters to a geometry_msgs/Pose message.
    """
    pose = Pose()
    if "position" not in params and not (
        "orientation" in params or "rpy" in params
    ):
        raise ValueError(
            "No position or orientation parameters found in input dictionary"
        )

    # Position extraction
    if "position" in params:
        position = params["position"]
        if is_iterable(position):
            pose.position.x, pose.position.y, pose.position.z = position
        elif isinstance(position, dict):
            pose.position.x = position["x"]
            pose.position.y = position["y"]
            pose.position.z = position["z"]
        else:
            raise ValueError(
                f"Invalid position type: expected Sequence or dict, got {type(position)}"
            )

    # Orientation extraction
    if "rpy" in params:
        rpy = params["rpy"]
        if isinstance(rpy, Mapping):
            pose.orientation = quaternion_msg_from_euler(**rpy)
        elif is_iterable(rpy):
            pose.orientation = quaternion_msg_from_euler(*rpy)
        else:
            raise ValueError(
                f"Invalid rpy array length: expected 3, got {len(rpy)}"
            )
    elif "orientation" in params:
        orientation = params["orientation"]
        if is_iterable(orientation):
            x, y, z, w = orientation
            pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
        elif isinstance(orientation, dict):
            pose.orientation = Quaternion(
                x=orientation["x"],
                y=orientation["y"],
                z=orientation["z"],
                w=orientation["w"],
            )
        else:
            raise ValueError(
                f"Invalid orientation type: expected Sequence or dict, got {type(orientation)}"
            )

    return pose


def pose_stamped_msg_from_dict(params: dict[str, Any]) -> PoseStamped:
    """
    Convert a dictionary of parameters to a geometry_msgs/PoseStamped message.
    """
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = params["header"]["frame_id"]
    pose_stamped.pose = pose_msg_from_dict(params["pose"])
    return pose_stamped


def create_object_color_msg(
    object_id: str, color: str | Iterable[float] | Mapping[str, float]
) -> ObjectColor:
    object_color = ObjectColor()
    object_color.id = object_id
    if isinstance(color, str):
        try:
            rgba = color_name_to_rgba[color]
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


# Mesh utility functions
def load_geometry(
    path: str, scale: float = 1.0
) -> trimesh.Trimesh | trimesh.Scene:
    """
    Load a mesh from a file and scale it.
    """
    if os.path.splitext(path)[1] == ".stl":
        geometry = trimesh.load_mesh(path)
    elif os.path.splitext(path)[1] == ".dae":
        geometry = trimesh.load_scene(path)
    else:
        raise ValueError(
            f"Unsupported mesh file extension '{os.path.splitext(path)[1]}' for {path}"
        )
    return geometry.apply_scale(scale)


def _simplify_quadratic_decimation_mesh(
    mesh: trimesh.Trimesh,
    target_count: int = 100,
    aggressiveness: int = 7,
    preserve_border: bool = True,
):
    mesh_simplifier = pyfqmr.Simplify()  # type: ignore
    mesh_simplifier.setMesh(mesh.vertices, mesh.faces)
    logging.basicConfig(level=logging.DEBUG)
    logging.getLogger("pyfqmr")
    mesh_simplifier.simplify_mesh(
        target_count=target_count,
        aggressiveness=aggressiveness,
        preserve_border=preserve_border,
        verbose=True,
    )
    vertices, faces, _ = mesh_simplifier.getMesh()
    mesh_simplified = trimesh.Trimesh(vertices=vertices, faces=faces)
    return mesh_simplified


def simplify_quadratic_decimation(
    geometry: trimesh.Trimesh | trimesh.Scene,
    target_count: int = 100,
    aggressiveness: int = 7,
    preserve_border: bool = True,
):
    if hasattr(geometry, "geometry"):
        updates = {
            k: simplify_quadratic_decimation(
                v,
                target_count=target_count,
                aggressiveness=aggressiveness,
                preserve_border=preserve_border,
            )
            for k, v in geometry.geometry.items()  # type: ignore
        }
        geometry.geometry.update(updates)  # type: ignore
        return geometry
    else:
        return _simplify_quadratic_decimation_mesh(
            geometry,  # type: ignore
            target_count,
            aggressiveness,
            preserve_border,
        )


def simplify_bounding_primitive(
    geometry: trimesh.Trimesh | trimesh.Scene,
):
    if hasattr(geometry, "geometry"):
        updates = {
            k: simplify_bounding_primitive(v)
            for k, v in geometry.geometry.items()  # type: ignore
        }
        geometry.geometry.update(updates)  # type: ignore
        return geometry
    else:
        return geometry.bounding_primitive.to_mesh()  # type: ignore


def get_bounding_primitives_mesh_scene(scene: trimesh.Scene):
    return OrderedDict(
        (k, v.bounding_primitive) for k, v in scene.geometry.items()
    )


def simplify_convex_hull(
    geometry: trimesh.Trimesh | trimesh.Scene,
):
    if hasattr(geometry, "geometry"):
        updates = {
            k: simplify_convex_hull(v)
            for k, v in geometry.geometry.items()  # type: ignore
        }
        geometry.geometry.update(updates)  # type: ignore
        return geometry
    else:
        return geometry.convex_hull


def visualize_geometry(geometry: trimesh.Trimesh | trimesh.Scene):
    try:
        if hasattr(geometry, "lights"):
            geometry.lights = []  # type: ignore

        window = geometry.show(start_loop=False)
        pyglet.app.run()
    except KeyboardInterrupt:
        if window is not None:
            window.close()


def collision_object_from_geometry(
    geometry: trimesh.Trimesh | trimesh.Scene,
    object_id: str,
    pose: Optional[Pose] = None,
    reference_frame_id: str = "world",
    subframe_names: list[str] = [],
    subframe_poses: list[Optional[Pose]] = [],
    operation: str = "add",
) -> CollisionObject:
    """
    Create a collision object from a mesh. Returns the collision object and the
    transformation matrix used to reorient the mesh.
    """
    if hasattr(geometry, "to_mesh"):
        mesh = geometry.to_mesh()  # type: ignore
    else:
        mesh = geometry

    collision_object = CollisionObject()
    collision_object.header.frame_id = reference_frame_id
    collision_object.id = object_id

    if pose is None:
        pose = Pose()

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
        collision_object.subframe_poses.append(  # type: ignore
            subframe_pose if subframe_pose is not None else Pose()
        )
    match operation:
        case "add":
            collision_object.operation = CollisionObject.ADD
        case "remove":
            collision_object.operation = CollisionObject.REMOVE
        case "append":
            collision_object.operation = CollisionObject.APPEND
        case "move":
            collision_object.operation = CollisionObject.MOVE
        case _:
            raise ValueError(f"Invalid operation: {operation}")

    return collision_object


def attached_collision_object_from_geometry(
    geometry: trimesh.Trimesh | trimesh.Scene,
    object_id: str,
    link_name: str,
    touch_links: list[str] = [],
    pose: Optional[Pose] = None,
    base_frame_id: str = "world",
    subframe_names: list[str] = [],
    subframe_poses: list[Optional[Pose]] = [],
) -> AttachedCollisionObject:
    collision_object = collision_object_from_geometry(
        geometry,
        object_id,
        pose,
        base_frame_id,
        subframe_names,
        subframe_poses,
    )
    attached_collision_object = AttachedCollisionObject()
    attached_collision_object.object = collision_object
    attached_collision_object.link_name = link_name
    attached_collision_object.touch_links = touch_links
    return attached_collision_object
