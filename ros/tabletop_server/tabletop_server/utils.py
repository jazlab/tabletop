import inspect
import logging
import os
from collections import OrderedDict
from collections.abc import Callable, Coroutine
from typing import Any, Optional, Protocol

import numpy as np
import pyfqmr
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
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_from_matrix,
    translation_matrix,
)


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
    roll: float, pitch: float, yaw: float, axes: str = "sxyz"
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
    quaternion = quaternion_matrix(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )
    return translation @ quaternion


def pose_msg_from_params(node: Node, prefix: str):
    pose = Pose()
    pose.position.x = node.get_parameter(f"{prefix}.position.x").value
    pose.position.y = node.get_parameter(f"{prefix}.position.y").value
    pose.position.z = node.get_parameter(f"{prefix}.position.z").value
    return pose


def pose_stamped_msg_from_params(node: Node, prefix: str):
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = node.get_parameter(
        f"{prefix}.header.frame_id"
    ).value
    pose_stamped.pose = pose_msg_from_params(node, f"{prefix}.pose")
    return pose_stamped


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
    if hasattr(geometry, "lights"):
        geometry.lights = []  # type: ignore
        geometry.show()
    else:
        geometry.show()


def collision_object_from_geometry(
    geometry: trimesh.Trimesh | trimesh.Scene,
    id: str,
    base_frame_id: str = "world",
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
    collision_object.header.frame_id = base_frame_id
    collision_object.id = id

    tf = mesh.apply_obb()
    tf_inv = inverse_matrix(tf)

    mesh_msg = Mesh()
    mesh_msg.triangles = list(
        map(lambda t: MeshTriangle(vertex_indices=t), mesh.faces)  # type: ignore
    )
    mesh_msg.vertices = list(
        map(
            lambda v: Point(x=v[0], y=v[1], z=v[2]),
            mesh.vertices,  # type: ignore
        )
    )
    mesh_pose = pose_msg_from_matrix(tf_inv)

    collision_object.meshes.append(mesh_msg)  # type: ignore
    collision_object.mesh_poses.append(mesh_pose)  # type: ignore
    collision_object.operation = CollisionObject.ADD

    return collision_object
