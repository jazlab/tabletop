import inspect
import os
from collections.abc import Callable, Coroutine
from typing import Any, Optional, Protocol

import numpy as np
import trimesh
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import CollisionObject
from rclpy.client import Client
from rclpy.node import Node
from shape_msgs.msg import Mesh, MeshTriangle
from tf_transformations import (
    inverse_matrix,
    quaternion_from_matrix,
    translation_from_matrix,
)
from tf_transformations import (
    quaternion_from_euler as quaternion_from_euler_tf,
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


def load_mesh(
    path: str, scale: float = 1.0, max_faces: int = 1000
) -> trimesh.Trimesh:
    """
    Load a mesh from a file and return a trimesh.Trimesh object.
    If the mesh has more than max_faces, its bounding primitive is used instead
    """
    mesh = trimesh.load_mesh(path)
    mesh = mesh.apply_scale(scale)
    if len(mesh.faces) > max_faces:
        mesh = mesh.bounding_primitive.to_mesh()
    return mesh


def collision_object_from_mesh(
    mesh: trimesh.Trimesh, id: str, base_frame_id: str = "world"
) -> CollisionObject:
    """
    Create a collision object from a mesh. Returns the collision object and the
    transformation matrix used to reorient the mesh.
    """
    collision_object = CollisionObject()
    collision_object.header.frame_id = base_frame_id
    collision_object.id = id

    tf = mesh.apply_obb()
    tf_inv = inverse_matrix(tf)

    mesh_msg = Mesh()
    mesh_msg.triangles = list(
        map(
            lambda t: MeshTriangle(vertex_indices=t),
            mesh.faces,
        )
    )
    mesh_msg.vertices = list(
        map(
            lambda v: Point(x=v[0], y=v[1], z=v[2]),
            mesh.vertices,
        )
    )
    mesh_pose = pose_from_matrix(tf_inv)

    collision_object.meshes.append(mesh_msg)  # type: ignore
    collision_object.mesh_poses.append(mesh_pose)  # type: ignore
    collision_object.operation = CollisionObject.ADD

    return collision_object


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
