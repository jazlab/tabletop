"""MoveIt planning scene management interface.

This module provides a comprehensive interface for managing the MoveIt planning
scene, including collision objects, attached objects, and the allowed collision
matrix. It is the foundation of the MoveIt interface hierarchy.

Key Capabilities:
- Loading collision objects from YAML configuration and mesh files
- Adding/removing world and attached collision objects
- Managing the allowed collision matrix for selective collision checking
- Supporting both mesh and primitive collision geometries
- Named robot state and pose management
- Planning scene hashing for trajectory cache validation

The PlanningSceneInterface is designed to be subclassed by higher-level
interfaces that add motion planning and execution capabilities.

Inheritance Hierarchy:
    BaseInterface
    └── PlanningSceneInterface
        └── PlanAndExecuteInterface
            └── ObjectManipulationInterface
                └── MoveItInterface
"""

import hashlib
import json
import os
from collections.abc import Iterable, Mapping
from copy import deepcopy
from glob import glob
from typing import Any, ContextManager, Literal, NamedTuple, Optional

import numpy as np
import pandas as pd
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.collision_detection import (  # type: ignore[reportMissingModuleSource]
    AllowedCollisionMatrix,
)
from moveit.core.planning_scene import (  # type: ignore[reportMissingModuleSource]
    PlanningScene,
)
from moveit.core.robot_model import (  # type: ignore[reportMissingModuleSource]
    RobotModel,
)
from moveit.core.robot_state import (  # type: ignore[reportMissingModuleSource]
    RobotState,
)
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from moveit.planning import (
    MoveItPy,
    PlanningComponent,
    PlanningSceneMonitor,
)
from moveit_msgs.msg import AllowedCollisionMatrix as AllowedCollisionMatrixMsg
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    LinkPadding,
    ObjectColor,
)
from moveit_msgs.msg import PlanningScene as PlanningSceneMsg
from rclpy.impl.logging_severity import LoggingSeverity
from transformations import identity_matrix

from tabletop_py.utils.mesh import (
    load_geometry,
    simplify_convex_hull,
    simplify_quadratic_decimation,
    transform_geometry,
)
from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.nodes.base import BaseNode
from tabletop_rig.utils.logging import SeverityString
from tabletop_rig.utils.ros import (
    add_mesh_collision_object_msg,
    add_plane_collision_object_msg,
    add_primitive_collision_object_msg,
    add_primitive_collision_object_msg_from_mesh,
    arrays_from_pose_msg,
    attached_collision_object_msg,
    change_reference_frame_pose_stamped,
    matrix_from_pose_msg,
    object_color_msg,
    pose_msg,
    pose_msg_from_matrix,
    pose_stamped_msg,
)


class GridObject(NamedTuple):
    object_id: str
    grid_idx: tuple[int, int]
    pose_stamped: PoseStamped


class MoveItInterface(BaseInterface):
    """Interface for managing MoveIt planning scene components.

    Provides methods to:
    - Load and manage collision objects from configuration
    - Control object attachment to robot links
    - Configure allowed collision matrix entries
    - Query and set named robot states and poses
    - Generate scene hashes for cache invalidation

    This class manages the PlanningSceneMonitor from MoveItPy and provides
    convenient wrappers for common planning scene operations.

    Attributes:
        _moveit_py: The MoveItPy instance for planning scene access.
        _planning_scene_monitor: Monitor for planning scene updates.
        _robot_model: The robot model from URDF/SRDF.
        _planning_frame: The reference frame for planning.
        _pose_link: Default end-effector link for pose targets.
        _group_name: Default planning group name.
    """

    moveit_py: MoveItPy
    grid_objects_by_id: dict[str, GridObject]
    grid_objects_by_idx: dict[tuple[int, int], GridObject]

    def __init__(
        self,
        node: BaseNode,
        name: str,
        *,
        parameter_fallback_prefix: Optional[str] = None,
    ) -> None:
        """Initializes the MoveItSceneInterface

        Initializes MoveItPy and planning scene

        Args:
            node: Parent ROS node for accessing parameters and communicating with other nodes
            logger_name: Name given to child logger of parent node
        """
        super().__init__(
            node, name, parameter_fallback_prefix=parameter_fallback_prefix
        )

        self.moveit_py = MoveItPy(
            "moveit_py",
            provide_planning_service=True,
            install_signal_handlers=False,
        )

        self.grid_objects_by_id = {}
        self.grid_objects_by_idx = {}

        self._init_planning_scene()

        self._init_link_padding()

        self._init_collision_detector()

        # self.log(self.robot_model.get_model_info())

        self.log("MoveIt interface initialized")

    def _init_planning_scene(self):
        """Setup the planning scene

        Adds plane, primitive, and mesh collision objects from the planning
        scene configuration.
        """
        self.log("Initializing planning scene")

        self.remove_all_collision_objects()

        config: dict[str, Any] = self.param("planning_scene")

        cache_dir = os.path.expanduser(os.path.expandvars(config["cache_dir"]))
        if not os.path.isabs(cache_dir):
            raise ValueError(
                f"Planning scene cache directory must be absolute: {cache_dir}"
            )

        scene_path = os.path.join(cache_dir, "scene.txt")
        collision_matrix_path = os.path.join(cache_dir, "collision_matrix.csv")
        config_path = os.path.join(cache_dir, "config.yaml")
        scene_hash_path = os.path.join(cache_dir, "scene_hash.txt")
        grid_objects_path = os.path.join(cache_dir, "grid_objects.yaml")

        if config["use_saved_scene"]:
            if all(
                os.path.exists(path)
                for path in [
                    scene_path,
                    collision_matrix_path,
                    config_path,
                    scene_hash_path,
                    grid_objects_path,
                ]
            ):
                with open(config_path, "r") as f:
                    saved_config = yaml.safe_load(f)
                with open(scene_hash_path, "r") as f:
                    saved_scene_hash = f.read().strip()
                if (
                    saved_config == config
                    and saved_scene_hash
                    == self.scene_hash(include_robot=False)
                ):
                    self.load_planning_scene(scene_path)
                    self.load_collision_matrix(collision_matrix_path)
                    self.load_grid_objects(grid_objects_path)
                    return
                else:
                    self.log(
                        "Saved planning scene config or rig hash mismatch.",
                        severity="WARN",
                    )
            else:
                self.log(
                    "One or more saved planning scene files do not exist.",
                    severity="WARN",
                )

        self.log("Initializing planning scene from config")

        orig_config = deepcopy(config)

        # Add primitive collision objects
        if "primitives" in config:
            for object_id, kwargs in config["primitives"].items():
                self.add_primitive_collision_object(
                    object_id=object_id, **kwargs
                )

        # Add plane collision objects
        if "planes" in config:
            for object_id, kwargs in config["planes"].items():
                self.add_plane_collision_object(object_id=object_id, **kwargs)

        # Add dynamic object meshes
        self.add_grid_mesh_collision_objects(**config["object_meshes"])

        # Add rig mesh collision objects
        for object_id, kwargs in config["rig_meshes"].items():
            self.add_mesh_collision_object(object_id=object_id, **kwargs)

        # Save planning scene to file
        os.makedirs(cache_dir, exist_ok=True)
        self.save_planning_scene(scene_path)
        self.save_collision_matrix(collision_matrix_path)
        self.save_grid_objects(grid_objects_path)
        with open(scene_hash_path, "w") as f:
            f.write(self.scene_hash(include_robot=False))
        with open(config_path, "w") as f:
            yaml.dump(orig_config, f)

    def _init_link_padding(self):
        """Set the link padding for the planning scene."""
        config: dict[str, Any] = self.param("link_padding")
        with self.planning_scene_rw() as scene:
            msg = PlanningSceneMsg(
                is_diff=True,
                link_padding=[
                    LinkPadding(link_name=name, padding=padding)
                    for name, padding in config.items()
                ],
            )
            if not scene.set_planning_scene_diff_msg(msg):
                raise RuntimeError("Failed to set link padding")

    def _init_collision_detector(self):
        """Initialize the collision detector."""
        with self.planning_scene_rw() as scene:
            scene.allocate_collision_detector("bullet")

    ###########################################################################
    ########## MoveItPy Convenience Methods and Properties ####################
    ###########################################################################

    @property
    def planning_scene_monitor(self) -> PlanningSceneMonitor:
        """Get the planning scene monitor."""
        return self.moveit_py.get_planning_scene_monitor()

    def planning_scene_rw(
        self,
    ) -> ContextManager[PlanningScene]:
        """Get the planning scene in read-write mode."""
        return self.planning_scene_monitor.read_write()

    def planning_scene_ro(
        self,
    ) -> ContextManager[PlanningScene]:
        """Get the planning scene in read-only mode."""
        return self.planning_scene_monitor.read_only()

    def get_planning_scene_copy(self) -> PlanningScene:
        """Get a copy of the planning scene."""
        with self.planning_scene_ro() as scene:
            return deepcopy(scene)

    @property
    def planning_frame(self) -> str:
        """Get the planning frame from the planning scene."""
        with self.planning_scene_ro() as scene:
            planning_frame = scene.planning_frame
            assert planning_frame == "world"
            return planning_frame

    @property
    def collision_object_ids(self) -> list[str]:
        """Get the collision object ids from the planning scene."""
        with self.planning_scene_ro() as scene:
            collision_objects: list[CollisionObject] = (
                scene.planning_scene_message.world.collision_objects
            )
            return [x.id for x in collision_objects]

    @property
    def collision_objects(self) -> dict[str, CollisionObject]:
        """Get the collision objects from the planning scene."""
        with self.planning_scene_ro() as scene:
            collision_objects: list[CollisionObject] = (
                scene.planning_scene_message.world.collision_objects
            )
            return {x.id: deepcopy(x) for x in collision_objects}

    @property
    def attached_collision_object_ids(self) -> list[str]:
        """Get the attached collision object ids from the planning scene."""
        with self.planning_scene_ro() as scene:
            attached_collision_objects: list[AttachedCollisionObject] = (
                scene.planning_scene_message.robot_state.attached_collision_objects
            )
            return [x.object.id for x in attached_collision_objects]

    @property
    def attached_collision_objects(self) -> dict[str, AttachedCollisionObject]:
        """Get the attached collision objects from the planning scene."""
        with self.planning_scene_ro() as scene:
            attached_collision_objects: list[AttachedCollisionObject] = (
                scene.planning_scene_message.robot_state.attached_collision_objects
            )
            return {
                x.object.id: deepcopy(x) for x in attached_collision_objects
            }

    @property
    def collision_matrix_df(self) -> pd.DataFrame:
        """Get the collision matrix as a pandas DataFrame."""
        with self.planning_scene_ro() as scene:
            msg: AllowedCollisionMatrixMsg = (
                scene.planning_scene_message.allowed_collision_matrix
            )
            object_ids = list(msg.entry_names)
            matrix = np.array([row.enabled for row in msg.entry_values])

        matrix_df = pd.DataFrame(matrix, columns=object_ids, index=object_ids)  # type: ignore

        return matrix_df  # pyright: ignore[reportReturnType]

    @property
    def robot_model(self) -> RobotModel:
        """Get the robot model."""
        return self.moveit_py.get_robot_model()

    def get_current_state(self) -> RobotState:
        """Get the current state from the planning scene"""
        with self.planning_scene_ro() as scene:
            return deepcopy(scene.current_state)

    def get_joint_names(self, group_name: str) -> list[str]:
        return self.robot_model.get_joint_model_group(
            group_name
        ).active_joint_model_names

    def get_planning_component(self, group_name: str) -> PlanningComponent:
        """Get the planning component for a given planning group name.

        Args:
            group_name: The name of the planning group.

        Returns:
            The planning component for the specified group.
        """
        return self.moveit_py.get_planning_component(group_name)

    def get_named_target_states(self, group_name: str) -> list[str]:
        """Get the named target states from the planning component."""
        return self.get_planning_component(group_name).named_target_states

    def get_target_state(
        self, target_name: str, group_name: str
    ) -> RobotState:
        """Get the named target state from the planning component."""
        joint_positions: dict[str, float] = self.get_planning_component(
            group_name
        ).get_named_target_state_values(target_name)
        assert set(joint_positions.keys()) == set(
            self.get_joint_names(group_name)
        )
        robot_state = self.get_current_state()
        robot_state.joint_positions = joint_positions
        robot_state.update()
        return robot_state

    ###########################################################################
    ########## Poses and Frame Transformations ################################
    ###########################################################################

    def create_pose_stamped(
        self, *, frame_id: Optional[str] = None, **kwargs: Any
    ) -> PoseStamped:
        """Create a PoseStamped message from keyword arguments.

        Uses planning frame as default frame id if not specified.
        """
        if frame_id is None:
            frame_id = self.planning_frame
        return pose_stamped_msg(frame_id=frame_id, **kwargs)

    def get_frame_transform(self, frame_id: str) -> np.ndarray:
        """
        Get the frame transform for a given frame id from the planning scene.
        """
        with self.planning_scene_ro() as scene:
            if not scene.knows_frame_transform(frame_id):
                raise ValueError(f"Frame transform to {frame_id} is undefined")
            tf = scene.get_frame_transform(frame_id)
            assert (
                frame_id == self.planning_frame
                or not (tf == identity_matrix()).all()
            )
            return tf

    def change_reference_frame(
        self, pose_stamped: PoseStamped, new_frame_id: str
    ) -> PoseStamped:
        """Change the reference frame of a pose stamped message."""
        if pose_stamped.header.frame_id == new_frame_id:
            self.log(
                f"Pose stamped message already in frame {new_frame_id}",
                severity="WARN",
            )
            return pose_stamped

        old_frame_transform = self.get_frame_transform(
            pose_stamped.header.frame_id
        )
        new_frame_transform = self.get_frame_transform(new_frame_id)
        return change_reference_frame_pose_stamped(
            old_pose_stamped=pose_stamped,
            old_frame_transform=old_frame_transform,
            new_frame_transform=new_frame_transform,
            new_frame_id=new_frame_id,
        )

    def get_frame_pose_stamped(self, frame_id: str) -> PoseStamped:
        """Get the frame pose relative to the planning frame for a given frame id."""
        return pose_stamped_msg(
            pose=pose_msg_from_matrix(self.get_frame_transform(frame_id)),
            frame_id=self.planning_frame,
        )

    def get_link_pose_stamped(
        self, link: str, frame_id: Optional[str] = None
    ) -> PoseStamped:
        """Get the current end-effector pose."""
        with self.planning_scene_ro() as scene:
            eef_pose = scene.current_state.get_pose(link)

        pose_stamped = pose_stamped_msg(
            pose=eef_pose, frame_id=self.planning_frame
        )

        # If a frame id is provided, change the reference frame
        if frame_id is not None and frame_id != self.planning_frame:
            pose_stamped = self.change_reference_frame(
                pose_stamped=pose_stamped,
                new_frame_id=frame_id,
            )

        return pose_stamped

    ###########################################################################
    ########## Scene Saving and Loading #######################################
    ###########################################################################

    def scene_hash(self, include_robot: bool) -> str:
        """Get the hash of the rig, for consistency purposes.

        Returns:
            The hash of the rig.
        """
        config = self.param("planning_scene")

        hash_algorithm = hashlib.md5()

        # Rig mesh collision objects
        keys_to_hash = ["pose_stamped", "correction", "scale"]
        for object_id, kwargs in config["rig_meshes"].items():
            hash_algorithm.update(object_id.encode("utf-8"))
            with open(kwargs["path"], "rb") as f:
                while chunk := f.read(8192):
                    hash_algorithm.update(chunk)
            for key in keys_to_hash:
                if key in kwargs:
                    hash_algorithm.update(
                        json.dumps(kwargs[key], sort_keys=True).encode("utf-8")
                    )

        # Plane collision objects
        if "planes" in config:
            keys_to_hash = ["pose_stamped", "coef"]
            for object_id, kwargs in config["planes"].items():
                hash_algorithm.update(object_id.encode("utf-8"))
                for key in keys_to_hash:
                    if key in kwargs:
                        hash_algorithm.update(
                            json.dumps(kwargs[key], sort_keys=True).encode(
                                "utf-8"
                            )
                        )

        # Primitive collision objects
        if "primitives" in config:
            keys_to_hash = ["pose_stamped", "type", "dimensions"]
            for object_id, kwargs in config["primitives"].items():
                hash_algorithm.update(object_id.encode("utf-8"))
                for key in keys_to_hash:
                    if key in kwargs:
                        hash_algorithm.update(
                            json.dumps(kwargs[key], sort_keys=True).encode(
                                "utf-8"
                            )
                        )

        # Dynamic collision objects
        keys_to_hash = ["pose_stamped", "correction", "scale"]
        for kwargs in config["object_meshes"]["object_kwargs"].values():
            for key in keys_to_hash:
                if key in kwargs:
                    hash_algorithm.update(
                        json.dumps(kwargs[key], sort_keys=True).encode("utf-8")
                    )

        # Base link pose
        if include_robot:
            for side in ["left", "right"]:
                position, orientation = arrays_from_pose_msg(
                    self.get_frame_pose_stamped(f"{side}_base_link").pose
                )
                # Round to correct for floating point inaccuracies
                hash_algorithm.update(position.round(4).tobytes())
                hash_algorithm.update(orientation.round(4).tobytes())

        return hash_algorithm.hexdigest()

    def save_planning_scene(self, path: str):
        """Save the planning scene to a file."""
        self.log(f"Saving planning scene to {path}")
        with self.planning_scene_ro() as scene:
            if not scene.save_geometry_to_file(path):
                raise RuntimeError("Could not save planning scene to file")

    def load_planning_scene(self, path: str):
        """Load the planning scene from a file."""
        self.log(f"Loading planning scene from {path}")
        with self.planning_scene_rw() as scene:
            if not scene.load_geometry_from_file(path):
                raise RuntimeError("Could not load planning scene from file")
            scene.current_state.update()

    def save_collision_matrix(self, path: str):
        """Save the collision matrix to a file."""
        self.log(f"Saving collision matrix to {path}")
        self.collision_matrix_df.to_csv(path)

    def load_collision_matrix(self, path: str):
        """Load the collision matrix from a file."""
        self.log(f"Loading collision matrix from {path}")
        matrix_df = pd.read_csv(path, index_col=0)

        true_pairs = []
        false_pairs = []

        # Iterate through upper triangle of matrix to avoid duplicates
        for i in range(len(matrix_df.index)):
            for j in range(i, len(matrix_df.columns)):
                obj1 = matrix_df.index[i]
                obj2 = matrix_df.columns[j]
                if matrix_df.iloc[i, j]:
                    true_pairs.append((obj1, obj2))
                else:
                    false_pairs.append((obj1, obj2))

        self.allow_collision(*zip(*true_pairs))
        self.disallow_collision(*zip(*false_pairs))

    def save_grid_objects(self, path: str):
        """Save the grid object poses to a file."""
        self.log(f"Saving grid objects to {path}")

        to_save: list[dict] = []

        for grid_object in self.grid_objects_by_id.values():
            to_save.append(
                {
                    "id": grid_object.object_id,
                    "grid_idx": list(grid_object.grid_idx),
                    "pose_stamped": {
                        "frame_id": grid_object.pose_stamped.header.frame_id,
                        "position": [
                            grid_object.pose_stamped.pose.position.x,
                            grid_object.pose_stamped.pose.position.y,
                            grid_object.pose_stamped.pose.position.z,
                        ],
                        "orientation": [
                            grid_object.pose_stamped.pose.orientation.w,
                            grid_object.pose_stamped.pose.orientation.x,
                            grid_object.pose_stamped.pose.orientation.y,
                            grid_object.pose_stamped.pose.orientation.z,
                        ],
                    },
                }
            )

        with open(path, "w") as f:
            yaml.dump(to_save, f)

    def load_grid_objects(self, path: str):
        """Load the grid object poses from a file."""
        self.log(f"Loading grid object poses from {path}")

        with open(path, "r") as f:
            to_load: list[dict] = yaml.safe_load(f)

        for kwargs in to_load:
            object_id = kwargs["id"]
            x, y = kwargs["grid_idx"]
            pose_stamped = pose_stamped_msg(
                frame_id=kwargs["pose_stamped"]["frame_id"],
                position=kwargs["pose_stamped"]["position"],
                orientation=kwargs["pose_stamped"]["orientation"],
            )
            grid_object = GridObject(
                object_id=object_id, grid_idx=(x, y), pose_stamped=pose_stamped
            )
            self.grid_objects_by_id[object_id] = grid_object
            self.grid_objects_by_idx[(x, y)] = grid_object

    ###########################################################################
    ########## Collisions #####################################################
    ###########################################################################

    def get_collision_object(self, object_id: str) -> CollisionObject:
        """Get a collision object from the planning scene."""
        with self.planning_scene_ro() as scene:
            collision_objects: list[CollisionObject] = (
                scene.planning_scene_message.world.collision_objects
            )
            for x in collision_objects:
                if x.id == object_id:
                    return deepcopy(x)
            raise ValueError(f"Collision object {object_id} not found")

    # def check_collision(self, group_name: str) -> CollisionResult:
    #     """Check if an object is colliding with the planning scene."""
    #     self.log(f"Checking collision for group {group_name}")
    #
    #     request = CollisionRequest()
    #     request.joint_model_group_name = group_name
    #     request.contacts = True
    #     request.max_contacts = 100
    #     request.max_contacts_per_pair = 1
    #     request.cost = False
    #     request.verbose = True
    #
    #     with self.planning_scene_ro() as scene:
    #         result = CollisionResult()
    #         scene.check_collision(request, result)
    #         return result
    #
    def is_state_colliding(self, group_name: str) -> bool:
        """Check if the current state of the planning scene is colliding."""

        with self.planning_scene_ro() as scene:
            return scene.is_state_colliding(group_name)

    def is_path_valid(self, trajectory: RobotTrajectory):
        """Validate the given robot trajectory.

        Args:
            trajectory: The robot trajectory to validate.

        Raises:
            TrajectoryError: If the trajectory is invalid.
        """
        group_name = trajectory.joint_model_group_name

        with self.planning_scene_ro() as scene:
            return scene.is_path_valid(
                trajectory,
                joint_model_group_name=group_name,
                verbose=True,
                invalid_index=[],  # DON'T USE THIS, for GIL reasons
            )

    def _parse_collision_matrix_entry(
        self, success: bool, allowed_collision_type: str
    ) -> bool:
        """Parse the collision matrix entry for two collision objects."""
        assert success or allowed_collision_type == "NEVER", (
            "Inconsistent collision matrix entry"
        )
        if allowed_collision_type == "ALWAYS":
            return True
        elif allowed_collision_type == "NEVER":
            return False
        else:
            raise ValueError(
                f"Invalid allowed collision type: {allowed_collision_type}"
            )

    def is_collision_allowed(self, id_0: str, id_1: str) -> bool:
        """Check if collision is allowed between two collision objects."""
        with self.planning_scene_ro() as scene:
            matrix: AllowedCollisionMatrix = scene.allowed_collision_matrix
            success, allowed_collision_type = matrix.get_entry(id_0, id_1)
            return self._parse_collision_matrix_entry(
                success, allowed_collision_type
            )

    def _modify_collision_matrix(
        self,
        id_0: str | Iterable[str],
        id_1: str | Iterable[str],
        allow: bool | Iterable[bool],
    ) -> list[tuple[str, str]]:
        """Modify the collision matrix

        Accepts:
        - two collision object ids
        - one collision object id and a list of collision object ids to modify
            collisions with (order agnostic)
        - two lists of collision object ids representing pairs of collision objects
            to modify collisions with

        Args:
            id_0: The id of the first collision object or a list of collision object ids.
            id_1: The id of the second collision object or a list of collision object ids.
            allow: Whether to allow or disallow collisions.

        Returns:
            The pairs of collision objects that were modified.
        """
        # Convert single collision object ids to lists and check that the number of ids match
        if isinstance(id_0, str) and isinstance(id_1, str):
            ids_0 = [id_0]
            ids_1 = [id_1]
        elif isinstance(id_0, str):
            ids_1 = list(id_1)
            ids_0 = [id_0] * len(ids_1)
        elif isinstance(id_1, str):
            ids_0 = list(id_0)
            ids_1 = [id_1] * len(ids_0)
        else:
            ids_0 = list(id_0)
            ids_1 = list(id_1)
            if len(ids_0) != len(ids_1):
                raise ValueError("Number of ids 0 and ids 1 must match")

        if isinstance(allow, bool):
            allows = [allow] * len(ids_0)
        else:
            allows = list(allow)
            if len(ids_0) != len(allows):
                raise ValueError("Number of ids 0 and allow must match")

        # Modify the collision matrix
        modified: list[tuple[str, str]] = []
        with self.planning_scene_rw() as scene:
            matrix: AllowedCollisionMatrix = scene.allowed_collision_matrix
            for x, y in zip(ids_0, ids_1):
                success, allowed_collision_type = matrix.get_entry(x, y)
                allowed = self._parse_collision_matrix_entry(
                    success, allowed_collision_type
                )
                if allowed != allow:
                    matrix.set_entry(x, y, allow)
                    modified.append((x, y))
                #     self.log(
                #         f"{'Allowing' if allow else 'Disallowing'} "
                #         f"collision between {x} and {y}",
                #         severity="DEBUG",
                #     )
                # else:
                #     self.log(
                #         f"Collision between {x} and {y} is already "
                #         f"{'allowed' if allow else 'disallowed'}",
                #         severity="DEBUG",
                #     )

            scene.current_state.update()

        return modified

    def allow_collision(
        self, id_0: str | Iterable[str], id_1: str | Iterable[str]
    ) -> list[tuple[str, str]]:
        """Modify the collision matrix to allow collisions

        Accepts either a single pair of collision objects or multiple pairs of collision objects.

        See Also:
            `_modify_collision_matrix` for argument and return value details
        """
        return self._modify_collision_matrix(id_0, id_1, allow=True)

    def disallow_collision(
        self, id_0: str | Iterable[str], id_1: str | Iterable[str]
    ) -> list[tuple[str, str]]:
        """Modify the collision matrix to disallow collisions

        Accepts either a single pair of collision objects or multiple pairs of collision objects.

        See Also:
            `_modify_collision_matrix` for argument and return value details
        """
        return self._modify_collision_matrix(id_0, id_1, allow=False)

    def process_add_collision_object(
        self,
        collision_object: CollisionObject,
        *,
        color: Optional[
            ObjectColor | str | Iterable[float] | Mapping[str, float]
        ] = None,
        allowed_collision_ids: Optional[Iterable[str]] = None,
    ):
        """Process a collision object.

        Adds the collision object to the planning scene and saves the init kwargs.

        Args:
            collision_object: The collision object to process.
            color: The color of the collision object.
            allowed_collision_ids: The ids of the collision objects that are allowed to collide with this object.
        """
        self.log(
            f"Processing collision object: {collision_object.id}",
            severity="DEBUG",
        )

        if collision_object.operation != CollisionObject.ADD:
            raise ValueError("CollisionObject operation must be ADD")

        if (
            collision_object.id in self.collision_object_ids
            or collision_object in self.attached_collision_object_ids
        ):
            raise ValueError(
                f"CollisionObject ID {collision_object.id} already exists in planning scene"
            )

        # Process color
        if color is not None:
            if isinstance(color, ObjectColor):
                if color.id != collision_object.id:
                    raise ValueError(
                        f"Object color id {color.id} does not match collision object id {collision_object.id}"
                    )
            else:
                color = object_color_msg(collision_object.id, color)

        # Add collision object to the planning scene
        self.planning_scene_monitor.process_collision_object(
            collision_object, color
        )

        # Allow collision with provided ids
        if allowed_collision_ids is not None:
            self.allow_collision(collision_object.id, allowed_collision_ids)

    def add_plane_collision_object(
        self,
        object_id: str,
        *,
        coef: list[float],
        pose_stamped: PoseStamped | Mapping[str, Any],
        allowed_collision_ids: Optional[list[str]] = None,
    ):
        """Add a plane collision object to the planning scene.

        Args:
            object_id: The id for the collision object.
            coef: The coefficients of the plane.
            header_frame_id: The frame id of the header. If not specified, the
                planning frame will be used.
        """
        self.log(f"Adding plane collision object: {object_id}")
        if not isinstance(pose_stamped, PoseStamped):
            pose_stamped = self.create_pose_stamped(**pose_stamped)

        collision_object = add_plane_collision_object_msg(
            object_id=object_id, coef=coef, pose_stamped=pose_stamped
        )

        self.process_add_collision_object(
            collision_object=collision_object,
            allowed_collision_ids=allowed_collision_ids,
        )

    def add_primitive_collision_object(
        self,
        object_id: str,
        *,
        type: str,
        dimensions: list[float],
        pose_stamped: PoseStamped | Mapping[str, Any],
        subframes: Optional[Mapping[str, Pose | Mapping[str, Any]]] = None,
        color: Optional[str | Iterable[float] | Mapping[str, float]] = None,
        allowed_collision_ids: Optional[list[str]] = None,
    ):
        """Add a primitive collision object to the planning scene.

        Args:
            object_id: The id for the collision object.
            type: The type of the primitive.
            dimensions: The dimensions of the primitive.
            pose_stamped: The stamped pose of the collision object.
            grid_idx: The index of the collision object in the grid.
            color: The color of the collision object.
            allowed_collision_ids: The ids of the collision objects that are allowed to collide with this object.
        """
        self.log(f"Adding primitive collision object: {object_id}")

        if not isinstance(pose_stamped, PoseStamped):
            pose_stamped = self.create_pose_stamped(**pose_stamped)

        # Create subframe names and poses from kwargs
        subframe_names: list[str] = []
        subframe_poses: list[Pose] = []
        if subframes is not None:
            for name, pose in subframes.items():
                if not isinstance(pose, Pose):
                    pose = pose_msg(**pose)
                subframe_names.append(name)
                subframe_poses.append(pose)

        collision_object = add_primitive_collision_object_msg(
            object_id=object_id,
            pose_stamped=pose_stamped,
            type=type,
            dimensions=dimensions,
            subframe_names=subframe_names,
            subframe_poses=subframe_poses,
        )

        self.process_add_collision_object(
            collision_object=collision_object,
            color=color,
            allowed_collision_ids=allowed_collision_ids,
        )

    def add_mesh_collision_object(
        self,
        object_id: str,
        path: str,
        *,
        scale: Optional[float] = None,
        correction: Optional[Pose | Mapping[str, Any]] = None,
        simplification: Optional[
            Literal[
                "convex_hull",
                "quadratic_decimation",
                "bounding_primitive",
                "bounding_box",
                "bounding_sphere",
                "bounding_cylinder",
            ]
        ] = None,
        pose_stamped: PoseStamped | Mapping[str, Any],
        subframes: Optional[Mapping[str, Pose | Mapping[str, Any]]] = None,
        color: Optional[str | Iterable[float] | Mapping[str, float]] = None,
        allowed_collision_ids: Optional[list[str]] = None,
    ):
        """Add a mesh collision object at a given path to the planning scene.

        Args:
            object_id: The id for the collision object.
            path: The path to the mesh file.
            pose_stamped: The pose of the collision object.
            scale: The scale of the mesh.
            correction: The correction to apply to the mesh.
            simplification: The simplification method to use.
            additional_subframe_names: The names of the additional subframes.
            additional_subframe_poses: The poses of the additional subframes.
            color: The color of the collision object.
        """
        self.log(f"Adding mesh collision object: {object_id}")

        # Create pose stamped from kwargs
        if not isinstance(pose_stamped, PoseStamped):
            pose_stamped = self.create_pose_stamped(**pose_stamped)

        # Load geometry
        geometry = load_geometry(path, scale)

        # Simplify geometry
        match simplification:
            case "convex_hull":
                geometry = simplify_convex_hull(geometry)
            case "quadratic_decimation":
                geometry = simplify_quadratic_decimation(geometry)
            case _ if simplification is None or simplification.startswith(
                "bounding_"
            ):
                pass
            case _:
                raise ValueError(
                    f"Invalid simplification type: {simplification}"
                )

        # Apply correction
        if correction is not None:
            if not isinstance(correction, Pose):
                correction = pose_msg(**correction)
            tf = matrix_from_pose_msg(correction)
            geometry = transform_geometry(geometry, tf)

        # Create subframe names and poses from kwargs
        subframe_names: list[str] = []
        subframe_poses: list[Pose] = []
        if subframes is not None:
            for name, pose in subframes.items():
                if not isinstance(pose, Pose):
                    pose = pose_msg(**pose)
                subframe_names.append(name)
                subframe_poses.append(pose)

        # Create collision object
        if simplification is not None and simplification.startswith(
            "bounding_"
        ):
            collision_object = add_primitive_collision_object_msg_from_mesh(
                object_id=object_id,
                pose_stamped=pose_stamped,
                mesh=geometry,
                primitive_type=simplification,  # type: ignore
                subframe_names=subframe_names,
                subframe_poses=subframe_poses,
            )
        else:
            collision_object = add_mesh_collision_object_msg(
                object_id=object_id,
                pose_stamped=pose_stamped,
                mesh=geometry,
                subframe_names=subframe_names,
                subframe_poses=subframe_poses,
            )

        self.process_add_collision_object(
            collision_object=collision_object,
            color=color,
            allowed_collision_ids=allowed_collision_ids,
        )

    def add_grid_mesh_collision_objects(
        self,
        *,
        path: str,
        common_kwargs: dict[str, Any],
        object_kwargs: dict[str, dict[str, Any]],
    ):
        """Add object meshes as collision objects to the planning scene.

        Loads meshes from a directory and adds them using the global
        pose_stamped specified for each object.

        Args:
            path: The directory path to the object meshes.
            common_kwargs: The common kwargs for the object meshes.
            object_kwargs: The object kwargs for the object meshes, keyed
                by grid index (e.g., "0,0").
        """
        # Get object meshes paths
        if not os.path.isdir(path):
            if not os.path.exists(path):
                raise FileNotFoundError(
                    f"Object meshes path {path} does not exist"
                )
            raise NotADirectoryError(
                f"Object meshes path {path} is not a directory"
            )
        paths = glob(os.path.join(path, "*.stl")) + glob(
            os.path.join(path, "*.dae")
        )

        object_id_to_path: dict[str, str] = {}
        for mesh_path in paths:
            object_id = os.path.splitext(os.path.basename(mesh_path))[0]
            object_id_to_path[object_id] = mesh_path

        existing = set(self.collision_object_ids)
        assert len(existing) == len(self.collision_object_ids)

        for idx, overrides in object_kwargs.items():
            x, y = map(int, idx.split(","))

            # Skip "empty" grid positions (assign None to object_id)
            object_id = overrides.pop("object_id", None)
            if object_id is None:
                self.log(
                    f"Skipping object at index {(x, y)} because it does not have an id"
                )
                continue

            # Check for identical objects
            if object_id in existing:
                raise ValueError(
                    f"Object ID {object_id} already exists in planning scene"
                )

            # Merge allowed collision ids
            if (
                "allowed_collision_ids" in common_kwargs
                and "allowed_collision_ids" in overrides
            ):
                overrides["allowed_collision_ids"].extend(
                    common_kwargs["allowed_collision_ids"]
                )

            # Merge per-object kwargs with common kwargs
            kwargs: dict[str, Any] = deepcopy(common_kwargs)
            kwargs.update(overrides)

            pose_stamped = kwargs.pop("pose_stamped")
            if not isinstance(pose_stamped, PoseStamped):
                pose_stamped = self.create_pose_stamped(**pose_stamped)

            try:
                mesh_path = object_id_to_path[object_id]
            except KeyError:
                raise ValueError(
                    f"Object mesh {object_id} not found in {path}"
                )

            self.add_mesh_collision_object(
                object_id=object_id,
                path=mesh_path,
                pose_stamped=pose_stamped,
                **kwargs,
            )

            grid_object = GridObject(
                object_id=object_id, grid_idx=(x, y), pose_stamped=pose_stamped
            )
            self.grid_objects_by_id[object_id] = grid_object
            self.grid_objects_by_idx[(x, y)] = grid_object

    def attach_collision_object(
        self,
        object_id: str,
        link_name: str,
        *,
        touch_links: Optional[list[str]] = None,
    ):
        """Attach an object to the robot."""
        self.log(f"Attaching collision object {object_id}", severity="DEBUG")
        attached_collision_object = attached_collision_object_msg(
            object_id=object_id,
            link_name=link_name,
            operation="ADD",
            touch_links=touch_links,
        )
        self.planning_scene_monitor.process_attached_collision_object(
            attached_collision_object
        )

    def detach_collision_object(self, object_id: str, link_name: str = ""):
        """Detach an object from the robot."""
        self.log(f"Detaching collision object {object_id}", severity="DEBUG")
        attached_collision_object = attached_collision_object_msg(
            object_id=object_id,
            operation="REMOVE",
            link_name=link_name,
        )
        self.planning_scene_monitor.process_attached_collision_object(
            attached_collision_object
        )

    def detach_all_collision_objects(self):
        """Detach all collision objects from the robot."""
        self.log("Detaching all collision objects", severity="DEBUG")
        for object_id in self.attached_collision_object_ids:
            self.detach_collision_object(object_id)
        assert len(self.attached_collision_object_ids) == 0

    def remove_collision_object(self, object_id: str):
        """Remove a collision object from the planning scene."""
        self.log(f"Removing collision object: {object_id}")
        collision_object = CollisionObject(
            id=object_id, operation=CollisionObject.REMOVE
        )
        self.planning_scene_monitor.process_collision_object(collision_object)

    def remove_all_collision_objects(self):
        """Remove all collision objects from the planning scene."""
        self.log("Removing all collision objects", severity="DEBUG")

        self.detach_all_collision_objects()

        with self.planning_scene_rw() as scene:
            scene.remove_all_collision_objects()
            scene.current_state.update()

        self.grid_objects_by_id = {}
        self.grid_objects_by_idx = {}

        assert len(self.collision_object_ids) == 0

    def move_collision_object(self, object_id: str, pose_stamped: PoseStamped):
        """Move a collision object."""
        self.log(f"Moving collision object: {object_id}", severity="DEBUG")
        if object_id in self.attached_collision_object_ids:
            self.detach_collision_object(object_id)

        collision_object = CollisionObject()
        collision_object.header.frame_id = pose_stamped.header.frame_id
        collision_object.id = object_id
        collision_object.pose = pose_stamped.pose
        collision_object.operation = CollisionObject.MOVE

        self.planning_scene_monitor.process_collision_object(collision_object)

    ###########################################################################
    ########## Logging ########################################################
    ###########################################################################

    def log_planning_scene(
        self, severity: SeverityString | LoggingSeverity = "INFO"
    ):
        """Log the planning scene."""
        if not isinstance(severity, LoggingSeverity):
            severity = LoggingSeverity[severity]

        if self.log_level < severity:
            return

        self.log("Logging planning scene", severity=severity)
        with self.planning_scene_ro() as scene:
            planning_scene_msg: PlanningSceneMsg = scene.planning_scene_message

            for collision_object in planning_scene_msg.world.collision_objects:
                collision_object.meshes = []
            for (
                attached_collision_object
            ) in planning_scene_msg.robot_state.attached_collision_objects:
                attached_collision_object.object.meshes = []

            self.log_ros_msg(
                planning_scene_msg,
                title="Planning Scene Msg:",
                severity=severity,
            )

    def log_collision_matrix(
        self, severity: SeverityString | LoggingSeverity = "INFO"
    ):
        """Log the collision matrix."""
        if not isinstance(severity, LoggingSeverity):
            severity = LoggingSeverity[severity]

        if self.log_level < severity:
            return

        self.log(
            f"Allowed collision matrix: \n{self.collision_matrix_df.to_string()}",
            severity=severity,
        )

    def log_collision_objects(
        self, severity: SeverityString | LoggingSeverity = "INFO"
    ):
        """Log the collision objects."""
        if not isinstance(severity, LoggingSeverity):
            severity = LoggingSeverity[severity]

        if self.log_level < severity:
            return

        self.log("Logging collision objects", severity=severity)
        with self.planning_scene_ro() as scene:
            planning_scene_msg: PlanningSceneMsg = scene.planning_scene_message
            for collision_object in planning_scene_msg.world.collision_objects:
                collision_object.meshes = []
                self.log(
                    f"Collision object id: {collision_object.id}",
                    severity=severity,
                )
                self.log(
                    f"Collision object: {collision_object}",
                    severity=severity,
                )
                self.log("=" * 80, severity=severity)

            for (
                attached_collision_object
            ) in planning_scene_msg.robot_state.attached_collision_objects:
                attached_collision_object.object.meshes = []
                self.log(
                    f"Attached collision object id: {attached_collision_object.object.id}",
                    severity=severity,
                )
                self.log(
                    f"Attached collision object: {attached_collision_object}",
                    severity=severity,
                )
                self.log("=" * 80, severity=severity)

    ###########################################################################
    ########## Destroy ########################################################
    ###########################################################################

    def destroy_interface(self):
        """Shut down MoveItPy"""
        self.log("Destroying PlanAndExecuteInterface")
        if hasattr(self, "moveit_py"):
            self.moveit_py.shutdown()
        super().destroy_interface()
