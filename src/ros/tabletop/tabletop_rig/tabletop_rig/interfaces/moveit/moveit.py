"""Top-level MoveIt planning scene management interface.

This module provides the MoveItInterface, the top of the planning scene
management hierarchy (BaseInterface → PlanningSceneInterface →
PlanAndExecuteInterface → ObjectManipulationInterface → MoveItInterface).

Key Capabilities:
- Loading/initializing collision objects from YAML configuration
- Adding/removing world and attached collision objects
- Managing the allowed collision matrix for selective collision checking
- Supporting mesh, plane, and primitive collision geometries
- Frame transform queries and reference frame conversions
- Named robot state and pose management
- Planning scene persistence (save/load to/from disk)
- Exclusive region management for multi-robot coordination
- Scene hashing for trajectory cache validation

The interface wraps MoveItPy's PlanningSceneMonitor and provides convenient
methods for common planning scene operations. All public methods include
proper error handling and logging.

Grid objects track experiment objects on a 2D grid; exclusive regions gate
access to restricted areas with collision walls.
"""

import hashlib
import json
import os
from collections.abc import Iterable, Mapping
from copy import deepcopy
from dataclasses import dataclass
from glob import glob
from typing import (
    Any,
    Literal,
    NamedTuple,
    Optional,
)

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
from rclpy.exceptions import ParameterNotDeclaredException
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
    get_joint_group_positions,
    matrix_from_pose_msg,
    object_color_msg,
    pose_msg,
    pose_msg_from_matrix,
    pose_stamped_msg,
)


class GridObject(NamedTuple):
    """Tracks a collision object placed on the experiment grid.

    Attributes:
        object_id: Unique identifier for the collision object.
        grid_idx: Grid position as (row, col) integer tuple.
        pose_stamped: Current pose of the object with frame and timestamp.
    """

    object_id: str
    grid_idx: tuple[int, int]
    pose_stamped: PoseStamped


@dataclass
class ExclusiveRegion:
    """Tracks an exclusive region that gates access to a restricted area.

    An exclusive region uses collision walls to restrict robot access. Only
    one robot (identified by group_name) may acquire the region at a time,
    allowing its collision objects to pass through the walls.

    Attributes:
        region_id: Unique identifier for the exclusive region.
        collision_ids: List of wall collision object IDs that form the region.
        acquired: True if currently held by a robot group.
        group_name: Name of the planning group holding the region (or None).
        modified_collisions: List of (robot_id, wall_id) pairs that were
            allowed when acquired (or None if not acquired).
    """

    region_id: str
    collision_ids: list[str]
    acquired: bool
    group_name: str | None
    modified_collisions: list[tuple[str, str]] | None


class MoveItInterface(BaseInterface):
    """Top-level interface for MoveIt planning scene management.

    Manages all aspects of the MoveIt planning scene: collision objects
    (planes, primitives, meshes), attached objects, frame transforms,
    robot states, and the allowed collision matrix. Supports persistence
    via caching, grid object tracking, and exclusive regions for
    multi-robot coordination.

    Initialization loads the planning scene from either a cached file or
    by building it from YAML configuration, then caches it for future
    sessions if the configuration hash matches.

    Attributes:
        moveit_py: The MoveItPy instance providing planning scene access.
        grid_objects_by_id: Map of object_id → GridObject for grid objects.
        grid_objects_by_idx: Map of (row, col) → GridObject for grid objects.
        _exclusive_regions: Map of region_id → ExclusiveRegion state.
    """

    moveit_py: MoveItPy
    grid_objects_by_id: dict[str, GridObject]
    grid_objects_by_idx: dict[tuple[int, int], GridObject]
    _exclusive_regions: dict[str, ExclusiveRegion]

    def __init__(
        self,
        node: BaseNode,
        name: str,
        *,
        parameter_fallback_prefix: Optional[str] = None,
    ) -> None:
        """Initialize the MoveIt interface and load the planning scene.

        Creates the MoveItPy instance, initializes the planning scene from
        cache or configuration, sets up exclusive regions, and applies link
        padding. On first run, saves the initialized scene to cache.

        Args:
            node: Parent ROS node for parameter access and logging.
            name: Interface name (used for logging and parameter prefixes).
            parameter_fallback_prefix: Optional prefix for fallback parameter
                lookup (e.g., 'common_moveit_interface').
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

        self._exclusive_regions = {}

        # self._init_collision_detector()

        self._init_planning_scene()

        self._init_exclusive_regions()

        self._init_link_padding()

        # self._init_collision_detector()

        self.log(self.robot_model.get_model_info())

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
        scene_hash_path = os.path.join(cache_dir, "scene_hash.txt")
        grid_objects_path = os.path.join(cache_dir, "grid_objects.yaml")

        if config["use_saved_scene"]:
            if all(
                os.path.exists(path)
                for path in [
                    scene_path,
                    collision_matrix_path,
                    scene_hash_path,
                    grid_objects_path,
                ]
            ):
                with open(scene_hash_path, "r") as f:
                    saved_scene_hash = f.read().strip()

                if saved_scene_hash == self.scene_config_hash():
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

        # Add plane collision objects
        if "planes" in config["rig"]:
            for object_id, kwargs in config["rig"]["planes"].items():
                self.add_plane_collision_object(object_id=object_id, **kwargs)

        # Add primitive collision objects
        if "primitives" in config["rig"]:
            for object_id, kwargs in config["rig"]["primitives"].items():
                self.add_primitive_collision_object(
                    object_id=object_id, **kwargs
                )

        # Add rig mesh collision objects
        if "meshes" in config["rig"]:
            for object_id, kwargs in config["rig"]["meshes"].items():
                self.add_mesh_collision_object(object_id=object_id, **kwargs)

        # Add dynamic object meshes
        self.add_grid_mesh_collision_objects(**config["grid_objects"])

        # Save planning scene to file
        os.makedirs(cache_dir, exist_ok=True)
        self.save_planning_scene(scene_path)
        self.save_collision_matrix(collision_matrix_path)
        self.save_grid_objects(grid_objects_path)
        with open(scene_hash_path, "w") as f:
            f.write(self.scene_config_hash())

    def _init_link_padding(self):
        """Set the link padding for the planning scene."""
        config: dict[str, Any] = self.param("link_padding")
        with self.psm.read_write() as scene:
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
        with self.psm.read_write() as scene:
            scene.allocate_collision_detector("bullet")

    ###########################################################################
    ########## MoveItPy Convenience Methods and Properties ####################
    ###########################################################################

    @property
    def psm(self) -> PlanningSceneMonitor:
        """Get the planning scene monitor."""
        return self.moveit_py.get_planning_scene_monitor()

    # @contextlib.contextmanager
    # def planning_scene_rw(
    #     self,
    # ) -> Generator[PlanningScene, None, None]:
    #     """Get the planning scene in read-write mode."""
    #     with self.psm.read_write() as scene:
    #         yield scene
    #
    # @contextlib.contextmanager
    # def planning_scene_ro(
    #     self,
    # ) -> Generator[PlanningScene, None, None]:
    #     """Get the planning scene in read-only mode."""
    #     with self.psm.read_only() as scene:
    #         yield scene

    def get_planning_scene_copy(self) -> PlanningScene:
        """Get a deep copy of the current planning scene.

        Returns:
            A PlanningScene object representing the current state.
        """
        with self.psm.read_only() as scene:
            return deepcopy(scene)

    @property
    def planning_frame(self) -> str:
        """Get the reference frame for planning (always 'world').

        Returns:
            The planning frame ID string.
        """
        with self.psm.read_only() as scene:
            planning_frame = scene.planning_frame
            assert planning_frame == "world"
            return planning_frame

    @property
    def collision_object_ids(self) -> list[str]:
        """Get all world collision object IDs from the planning scene.

        Returns:
            List of collision object IDs (empty if none exist).
        """
        with self.psm.read_only() as scene:
            collision_objects: list[CollisionObject] = (
                scene.planning_scene_message.world.collision_objects
            )
            return [x.id for x in collision_objects]

    @property
    def collision_objects(self) -> dict[str, CollisionObject]:
        """Get all world collision objects from the planning scene.

        Returns:
            Dict mapping object_id → CollisionObject (deep copies).
        """
        with self.psm.read_only() as scene:
            collision_objects: list[CollisionObject] = (
                scene.planning_scene_message.world.collision_objects
            )
            return {x.id: deepcopy(x) for x in collision_objects}

    @property
    def attached_collision_object_ids(self) -> list[str]:
        """Get all attached collision object IDs from the planning scene.

        Returns:
            List of attached object IDs (empty if none attached).
        """
        with self.psm.read_only() as scene:
            attached_collision_objects: list[AttachedCollisionObject] = (
                scene.planning_scene_message.robot_state.attached_collision_objects
            )
            return [x.object.id for x in attached_collision_objects]

    @property
    def attached_collision_objects(
        self,
    ) -> dict[str, AttachedCollisionObject]:
        """Get all attached collision objects from the planning scene.

        Returns:
            Dict mapping object_id → AttachedCollisionObject (deep copies).
        """
        with self.psm.read_only() as scene:
            attached_collision_objects: list[AttachedCollisionObject] = (
                scene.planning_scene_message.robot_state.attached_collision_objects
            )
            return {
                x.object.id: deepcopy(x) for x in attached_collision_objects
            }

    @property
    def collision_matrix_df(self) -> pd.DataFrame:
        """Get the allowed collision matrix as a pandas DataFrame.

        Rows and columns are indexed by collision object IDs. Values are
        boolean (True = collision allowed, False = collision disallowed).

        Returns:
            A square DataFrame with object IDs as index and columns.
        """
        with self.psm.read_only() as scene:
            msg: AllowedCollisionMatrixMsg = (
                scene.planning_scene_message.allowed_collision_matrix
            )
            object_ids = list(msg.entry_names)
            matrix = np.array([row.enabled for row in msg.entry_values])

        matrix_df = pd.DataFrame(matrix, columns=object_ids, index=object_ids)  # type: ignore

        return matrix_df  # pyright: ignore[reportReturnType]

    @property
    def robot_model(self) -> RobotModel:
        """Get the robot model from URDF and SRDF.

        Returns:
            The RobotModel instance (from MoveItPy).
        """
        return self.moveit_py.get_robot_model()

    def get_current_state(self) -> RobotState:
        """Get the current robot state from the planning scene.

        Waits for the planning scene monitor to have a fresh robot state
        within the configured timeout, then returns a deep copy.

        Returns:
            A RobotState representing the robot's current joint positions.

        Raises:
            Logs a warning if current state unavailable within timeout.
        """
        self.log("Getting current state from planning scene", severity="DEBUG")

        # TODO: Should probably use this
        now = self.node.get_clock().now()
        wait_time = self.param("current_state_wait_time")

        if not self.psm.wait_for_current_robot_state(now, wait_time):
            self.log(
                f"Could not get current robot state in "
                f"{wait_time} seconds, using existing state",
                severity="WARN",
            )

        with self.psm.read_only() as scene:
            return deepcopy(scene.current_state)

    def get_joint_names(self, group_name: str) -> list[str]:
        """Get the active joint names for a planning group.

        Args:
            group_name: The name of the planning group.

        Returns:
            List of active joint names in the group.
        """
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
        """Get the names of all SRDF-defined target states for a group.

        Args:
            group_name: The name of the planning group.

        Returns:
            List of named target state names (e.g., 'home', 'ready').
        """
        return self.get_planning_component(group_name).named_target_states

    def get_target_state(
        self, target_name: str, group_name: str
    ) -> RobotState:
        """Get a RobotState for a named SRDF target.

        Looks up the target in the SRDF and creates a RobotState with the
        named joint positions, keeping other joints at their current values.

        Args:
            target_name: Name of the SRDF-defined target (e.g., 'home').
            group_name: Planning group to which the target belongs.

        Returns:
            A RobotState with the target joint positions applied.
        """
        joint_positions: dict[str, float] = self.get_planning_component(
            group_name
        ).get_named_target_state_values(target_name)
        assert set(joint_positions.keys()) == set(
            self.get_joint_names(group_name)
        )
        with self.psm.read_only() as scene:
            robot_state = deepcopy(scene.current_state)

        robot_state.joint_positions = joint_positions
        robot_state.update()
        return robot_state

    def get_joint_state_target(
        self,
        joint_positions: Mapping[str, float],
        group_name: str,
        *,
        relative: bool = False,
        base_state: Optional[RobotState] = None,
    ) -> RobotState:
        """Build a target RobotState from a partial joint-position mapping.

        Joints absent from `joint_positions` keep their value from
        `base_state`, so a partial mapping only moves the joints it names.
        Provided joints are set to the given absolute position, or added to
        the base position when `relative` is True.

        This is the joint-space analogue of `get_target_state` (which resolves
        an SRDF-named target) and backs the `JointState` / `JointStateDelta`
        planning goals.

        Args:
            joint_positions: Mapping of joint name -> absolute position (or
                delta, when `relative` is True). May cover any subset of the
                group's active joints.
            group_name: Planning group whose active joints define the full
                target state; every key must belong to this group.
            relative: If True, treat each value as a delta added to the base
                position; otherwise treat it as an absolute target.
            base_state: State used to fill unprovided joints (and as the base
                for deltas). Defaults to the current robot state.

        Returns:
            A new RobotState with the group's joints set accordingly.

        Raises:
            ValueError: If any provided joint is not an active joint of the
                group.
        """
        group_joint_names = set(self.get_joint_names(group_name))
        unknown = set(joint_positions) - group_joint_names
        if unknown:
            raise ValueError(
                f"Joint(s) {sorted(unknown)} are not active joints of group "
                f"'{group_name}'. Active joints: {sorted(group_joint_names)}"
            )

        if base_state is None:
            base_state = self.get_current_state()

        # Start from the full set of group joint positions so unprovided
        # joints stay where they are, then apply the requested overrides.
        target_positions = get_joint_group_positions(base_state, group_name)
        for joint, value in joint_positions.items():
            if relative:
                target_positions[joint] += value
            else:
                target_positions[joint] = value

        robot_state = deepcopy(base_state)
        robot_state.joint_positions = target_positions
        robot_state.update()
        return robot_state

    ###########################################################################
    ########## Poses and Frame Transformations ################################
    ###########################################################################

    def create_pose_stamped(
        self, *, frame_id: Optional[str] = None, **kwargs: Any
    ) -> PoseStamped:
        """Create a PoseStamped message from keyword arguments.

        Defaults to planning frame (world) if frame_id not specified.
        Passes remaining kwargs (position, orientation, etc.) to
        pose_stamped_msg utility.

        Args:
            frame_id: Reference frame ID (defaults to planning frame).
            **kwargs: Additional arguments passed to pose_stamped_msg
                (e.g., position=[x,y,z], orientation=[w,x,y,z]).

        Returns:
            A PoseStamped message with the specified frame and pose.
        """
        if frame_id is None:
            frame_id = self.planning_frame
        return pose_stamped_msg(frame_id=frame_id, **kwargs)

    def get_frame_transform(self, frame_id: str) -> np.ndarray:
        """Get the 4x4 transform matrix for a frame relative to planning frame.

        Args:
            frame_id: The frame ID to get the transform for.

        Returns:
            A 4x4 numpy array representing the homogeneous transform.

        Raises:
            ValueError: If the frame is unknown to the planning scene.
        """
        with self.psm.read_only() as scene:
            if not scene.knows_frame_transform(frame_id):
                raise ValueError(f"Frame transform to {frame_id} is undefined")
            tf = scene.get_frame_transform(frame_id)
            assert (
                frame_id == scene.planning_frame
                or not (tf == identity_matrix()).all()
            )
            return tf

    def change_reference_frame(
        self, pose_stamped: PoseStamped, new_frame_id: str
    ) -> PoseStamped:
        """Transform a pose to a different reference frame.

        Uses planning scene frame transforms to convert the pose from its
        current frame to the new frame. Logs a warning if already in the
        target frame.

        Args:
            pose_stamped: The pose in its original frame.
            new_frame_id: The target frame ID.

        Returns:
            A new PoseStamped in the target frame.

        Raises:
            ValueError: If either frame is unknown to the planning scene.
        """
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
        """Get a frame's pose relative to the planning frame.

        Args:
            frame_id: The frame to get the pose for.

        Returns:
            A PoseStamped in the planning frame (world).

        Raises:
            ValueError: If the frame is unknown to the planning scene.
        """
        return pose_stamped_msg(
            pose=pose_msg_from_matrix(self.get_frame_transform(frame_id)),
            frame_id=self.planning_frame,
        )

    def get_link_pose_stamped(
        self, link: str, frame_id: Optional[str] = None
    ) -> PoseStamped:
        """Get the current pose of a robot link.

        Gets the link's pose from the current robot state. If a frame_id is
        provided, transforms the pose to that frame.

        Args:
            link: The name of the robot link.
            frame_id: Optional target frame ID. If None, uses planning frame.

        Returns:
            A PoseStamped for the link in the requested frame.

        Raises:
            ValueError: If the frame_id is unknown to the planning scene.
        """
        link_pose = self.get_current_state().get_pose(link)

        pose_stamped = pose_stamped_msg(
            pose=link_pose, frame_id=self.planning_frame
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

    def _grid_object_id_to_path(self, mesh_dir: str) -> dict[str, str]:
        if not os.path.isdir(mesh_dir):
            if not os.path.exists(mesh_dir):
                raise FileNotFoundError(
                    f"Object meshes path {mesh_dir} does not exist"
                )
            raise NotADirectoryError(
                f"Object meshes path {mesh_dir} is not a directory"
            )
        paths = glob(os.path.join(mesh_dir, "*.stl")) + glob(
            os.path.join(mesh_dir, "*.dae")
        )

        object_id_to_path: dict[str, str] = {}
        for path in paths:
            object_id = os.path.splitext(os.path.basename(path))[0]
            object_id_to_path[object_id] = path

        return object_id_to_path

    def scene_config_hash(self) -> str:
        """Hash the planning scene configuration and mesh files.

        Computes MD5 hash of the planning_scene parameter and all mesh file
        contents referenced in the config. Used to invalidate cached scenes.

        Returns:
            Hex string of the MD5 hash.
        """
        config = self.param("planning_scene")

        hash_algorithm = hashlib.md5()

        hash_algorithm.update(
            json.dumps(config, sort_keys=True).encode("utf-8")
        )

        mesh_paths: list[str] = []

        # Rig meshes
        if "meshes" in config["rig"]:
            for kwargs in config["rig"]["meshes"].values():
                mesh_paths.append(kwargs["path"])

        # Grid object meshes
        mesh_dir = config["grid_objects"]["mesh_dir"]
        object_id_to_path = self._grid_object_id_to_path(mesh_dir)

        for kwargs in config["grid_objects"]["object_kwargs"].values():
            object_id = kwargs["object_id"]
            mesh_paths.append(object_id_to_path[object_id])

        for path in sorted(mesh_paths):
            with open(path, "rb") as f:
                while chunk := f.read(8192):
                    hash_algorithm.update(chunk)

        return hash_algorithm.hexdigest()

    def scene_hash(self, include_robot: bool) -> str:
        """Hash the current planning scene state.

        Computes MD5 hash of all collision object poses and optionally the
        robot base link transforms. Used for trajectory cache invalidation.

        Args:
            include_robot: If True, includes robot base link poses in hash.

        Returns:
            Hex string of the MD5 hash.
        """
        config = self.param("planning_scene")

        hash_algorithm = hashlib.md5()

        # Plane collision objects
        if "planes" in config["rig"]:
            keys_to_hash = ["pose_stamped", "coef"]
            for object_id, kwargs in sorted(config["rig"]["planes"].items()):
                hash_algorithm.update(object_id.encode("utf-8"))
                for key in keys_to_hash:
                    if key in kwargs:
                        hash_algorithm.update(
                            json.dumps(kwargs[key], sort_keys=True).encode(
                                "utf-8"
                            )
                        )

        # Primitive collision objects
        if "primitives" in config["rig"]:
            keys_to_hash = ["pose_stamped", "type", "dimensions"]
            for object_id, kwargs in sorted(
                config["rig"]["primitives"].items()
            ):
                hash_algorithm.update(object_id.encode("utf-8"))
                for key in keys_to_hash:
                    if key in kwargs:
                        hash_algorithm.update(
                            json.dumps(kwargs[key], sort_keys=True).encode(
                                "utf-8"
                            )
                        )

        # Rig mesh collision objects
        if "meshes" in config["rig"]:
            keys_to_hash = ["pose_stamped", "correction", "scale"]
            for object_id, kwargs in sorted(config["rig"]["meshes"].items()):
                hash_algorithm.update(object_id.encode("utf-8"))
                with open(kwargs["path"], "rb") as f:
                    while chunk := f.read(8192):
                        hash_algorithm.update(chunk)
                for key in keys_to_hash:
                    if key in kwargs:
                        hash_algorithm.update(
                            json.dumps(kwargs[key], sort_keys=True).encode(
                                "utf-8"
                            )
                        )

        # Dynamic collision objects
        keys_to_hash = ["pose_stamped"]
        for _, kwargs in sorted(
            config["grid_objects"]["object_kwargs"].items()
        ):
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
        """Save the planning scene geometry to a file.

        Args:
            path: Absolute path to write the scene file to.

        Raises:
            RuntimeError: If the save operation fails.
        """
        self.log(f"Saving planning scene to {path}")
        with self.psm.read_only() as scene:
            if not scene.save_geometry_to_file(path):
                raise RuntimeError("Could not save planning scene to file")

    def load_planning_scene(self, path: str):
        """Load planning scene geometry from a file.

        Args:
            path: Absolute path to the scene file to load.

        Raises:
            RuntimeError: If the load operation fails.
        """
        self.log(f"Loading planning scene from {path}")
        with self.psm.read_write() as scene:
            if not scene.load_geometry_from_file(path):
                raise RuntimeError("Could not load planning scene from file")
            scene.current_state.update()

    def save_collision_matrix(self, path: str):
        """Save the collision matrix to a CSV file.

        Args:
            path: Absolute path to write the CSV to.
        """
        self.log(f"Saving collision matrix to {path}")
        self.collision_matrix_df.to_csv(path)

    def load_collision_matrix(self, path: str):
        """Load the collision matrix from a CSV file.

        Reads the CSV and applies the allowed/disallowed collision pairs
        to the planning scene.

        Args:
            path: Absolute path to the CSV file to load.
        """
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
        """Save grid object poses to a YAML file.

        Args:
            path: Absolute path to write the YAML to.
        """
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
        """Load grid object poses from a YAML file.

        Args:
            path: Absolute path to the YAML file to load.
        """
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
        """Get a collision object by ID from the planning scene.

        Args:
            object_id: The ID of the collision object to retrieve.

        Returns:
            A deep copy of the CollisionObject message.

        Raises:
            ValueError: If no collision object with the given ID exists.
        """
        with self.psm.read_only() as scene:
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
    #     with self.psm.read_only() as scene:
    #         result = CollisionResult()
    #         scene.check_collision(request, result)
    #         return result
    #

    def is_state_valid(
        self, robot_state: RobotState, group_name: str, verbose: bool = True
    ) -> bool:
        """Check if a robot state is valid (no collisions, joint limits met).

        Args:
            robot_state: The RobotState to validate.
            group_name: Planning group for validation context.
            verbose: If True, logs details about any violations.

        Returns:
            True if the state is valid, False otherwise.
        """

        with self.psm.read_only() as scene:
            return scene.is_state_valid(
                robot_state, joint_model_group_name=group_name, verbose=verbose
            )

    def is_path_valid(self, trajectory: RobotTrajectory, verbose: bool = True):
        """Validate a trajectory for collisions and joint limits.

        Checks all waypoints in the trajectory, and optionally all states
        along straight-line interpolation between waypoints.

        Args:
            trajectory: The RobotTrajectory to validate.
            verbose: If True, logs details about any violations.

        Returns:
            True if the trajectory is valid, False otherwise.
        """
        group_name = trajectory.joint_model_group_name

        with self.psm.read_only() as scene:
            return scene.is_path_valid(
                trajectory, joint_model_group_name=group_name, verbose=verbose
            )

    def _parse_collision_matrix_entry(
        self, entry_found: bool, allowed_collision_type: str
    ) -> bool:
        """Parse collision matrix lookup result into boolean allowed flag.

        Internal helper for querying the allowed collision matrix.

        Args:
            entry_found: True if an entry exists in the matrix.
            allowed_collision_type: The type value ('ALWAYS', 'NEVER',
                'UNKNOWN').

        Returns:
            True if collision is allowed, False otherwise.

        Raises:
            ValueError: If allowed_collision_type is invalid.
        """
        if not entry_found:
            assert allowed_collision_type in ("UNKNOWN", "NEVER"), (
                "Inconsistent collision matrix entry | "
                f"entry_found: {entry_found}, "
                f"allowed_collision_type: {allowed_collision_type}"
            )
            return False
        elif allowed_collision_type == "ALWAYS":
            return True
        elif allowed_collision_type in "NEVER":
            return False
        else:
            raise ValueError(
                f"Invalid allowed collision type: {allowed_collision_type}"
            )

    def is_collision_allowed(self, id_0: str, id_1: str) -> bool:
        """Check if collision is allowed between two objects.

        Args:
            id_0: First collision object ID.
            id_1: Second collision object ID.

        Returns:
            True if collision is allowed, False if disallowed.
        """
        with self.psm.read_only() as scene:
            matrix: AllowedCollisionMatrix = scene.allowed_collision_matrix
            entry_found, allowed_collision_type = matrix.get_entry(id_0, id_1)
            return self._parse_collision_matrix_entry(
                entry_found, allowed_collision_type
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
        with self.psm.read_write() as scene:
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

        self.log(f"Modified collision pairs: {modified}", severity="DEBUG")

        return modified

    def allow_collision(
        self, id_0: str | Iterable[str], id_1: str | Iterable[str]
    ) -> list[tuple[str, str]]:
        """Allow collision between object(s) in the collision matrix.

        Accepts a single pair or multiple pairs of collision object IDs.
        Flexible argument forms via _modify_collision_matrix:
        - allow_collision('obj1', 'obj2'): single pair
        - allow_collision('obj1', ['obj2', 'obj3']): one-to-many
        - allow_collision(['obj1', 'obj2'], ['obj3', 'obj4']): many-to-many

        Args:
            id_0: First collision object ID(s).
            id_1: Second collision object ID(s).

        Returns:
            List of (id_0, id_1) pairs actually modified.
        """
        self.log(
            f"Disallowing collision between {id_0} and {id_1}",
            severity="DEBUG",
        )
        return self._modify_collision_matrix(id_0, id_1, allow=True)

    def disallow_collision(
        self, id_0: str | Iterable[str], id_1: str | Iterable[str]
    ) -> list[tuple[str, str]]:
        """Disallow collision between object(s) in the collision matrix.

        Accepts a single pair or multiple pairs of collision object IDs.
        Flexible argument forms via _modify_collision_matrix:
        - disallow_collision('obj1', 'obj2'): single pair
        - disallow_collision('obj1', ['obj2', 'obj3']): one-to-many
        - disallow_collision(['obj1', 'obj2'], ['obj3', 'obj4']): many-to-many

        Args:
            id_0: First collision object ID(s).
            id_1: Second collision object ID(s).

        Returns:
            List of (id_0, id_1) pairs actually modified.
        """
        self.log(
            f"Allowing collision between {id_0} and {id_1}",
            severity="DEBUG",
        )
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
        """Add a collision object to the planning scene with color and ACM.

        Validates the object (must be ADD operation, ID must be unique),
        processes color if provided, adds to scene via PSM, and allows
        collisions with the specified object IDs.

        Args:
            collision_object: The CollisionObject to add (operation=ADD).
            color: Optional color (ObjectColor msg, string name, RGBA list,
                or dict). Defaults to no color.
            allowed_collision_ids: Optional list of object IDs that are
                allowed to collide with this object.

        Raises:
            ValueError: If operation is not ADD or ID already exists.
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
        self.psm.process_collision_object(collision_object, color)

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
            object_id: Unique ID for the collision object.
            coef: Plane coefficients [a, b, c, d] for equation ax+by+cz+d=0.
            pose_stamped: Pose or dict (position/orientation) in the planning
                frame. If dict, passed to create_pose_stamped.
            allowed_collision_ids: Optional list of object IDs allowed to
                collide with this plane.
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
            object_id: Unique ID for the collision object.
            type: Primitive type (e.g., 'BOX', 'SPHERE', 'CYLINDER').
            dimensions: Primitive dimensions (box: [x,y,z], sphere: [r],
                cylinder: [r,h], etc.).
            pose_stamped: Pose or dict in the planning frame.
            subframes: Optional dict of subframe_name → Pose/dict.
            color: Optional color (string name, RGBA list, or dict).
            allowed_collision_ids: Optional list of object IDs allowed to
                collide with this primitive.
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
        """Add a mesh collision object from a file to the planning scene.

        Loads the mesh, optionally simplifies it, applies correction transform,
        and adds to the planning scene. Supports both mesh and primitive-based
        (bounding box, sphere, cylinder) collision representations.

        Args:
            object_id: Unique ID for the collision object.
            path: Absolute path to the mesh file (STL, DAE, etc.).
            scale: Optional scale factor for the mesh geometry.
            correction: Optional Pose/dict to apply to the mesh before adding.
            simplification: Optional simplification method. If starts with
                'bounding_', converts mesh to primitive. Otherwise, applies
                geometry simplification ('convex_hull', 'quadratic_decimation').
            pose_stamped: Pose or dict in the planning frame.
            subframes: Optional dict of subframe_name → Pose/dict.
            color: Optional color (string name, RGBA list, or dict).
            allowed_collision_ids: Optional list of object IDs allowed to
                collide with this mesh.
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
        mesh_dir: str,
        common_kwargs: dict[str, Any],
        object_kwargs: dict[str, dict[str, Any]],
    ):
        """Add grid objects (meshes) to the planning scene.

        Loads meshes from a directory and adds them as collision objects.
        Grid positions are tracked via the grid_objects_by_id and
        grid_objects_by_idx dicts. Objects with object_id=None are skipped.

        Args:
            mesh_dir: Directory containing mesh files (STL, DAE).
            common_kwargs: Common kwargs for all grid objects (scale,
                correction, simplification, subframes, color,
                allowed_collision_ids, pose_stamped).
            object_kwargs: Per-object kwargs keyed by grid index string
                (e.g., "0,0"). Merged with common_kwargs; per-object
                overrides common. Must include 'object_id' and
                'pose_stamped'.

        Raises:
            ValueError: If mesh_dir is invalid, object mesh not found, or
                object ID already exists in planning scene.
        """
        object_id_to_path = self._grid_object_id_to_path(mesh_dir)

        existing = set(self.collision_object_ids)

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
                path = object_id_to_path[object_id]
            except KeyError:
                raise ValueError(
                    f"Object mesh {object_id} not found in {mesh_dir}"
                )

            self.add_mesh_collision_object(
                object_id=object_id,
                path=path,
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
        """Attach a collision object to a robot link.

        Moves the object from the world to robot_state.attached_collision_objects.
        The object will move with the link and not collide with specified touch
        links (e.g., fingers, gripper).

        Args:
            object_id: ID of the collision object to attach.
            link_name: Name of the robot link to attach to.
            touch_links: Optional list of links that may touch the object
                without collision (e.g., end-effector fingers).
        """
        self.log(f"Attaching collision object {object_id}", severity="DEBUG")
        attached_collision_object = attached_collision_object_msg(
            object_id=object_id,
            link_name=link_name,
            operation="ADD",
            touch_links=touch_links,
        )
        self.psm.process_attached_collision_object(attached_collision_object)

    def detach_collision_object(self, object_id: str, link_name: str = ""):
        """Detach a collision object from a robot link.

        Moves the object from robot_state.attached_collision_objects back
        to the world.

        Args:
            object_id: ID of the attached collision object to detach.
            link_name: Name of the link the object is attached to (unused
                for detach operation, included for API clarity).
        """
        self.log(f"Detaching collision object {object_id}", severity="DEBUG")
        attached_collision_object = attached_collision_object_msg(
            object_id=object_id,
            operation="REMOVE",
            link_name=link_name,
        )
        self.psm.process_attached_collision_object(attached_collision_object)

    def detach_all_collision_objects(self):
        """Detach all attached collision objects from the robot.

        Detaches each object individually and verifies all are detached.
        """
        self.log("Detaching all collision objects", severity="DEBUG")
        for object_id in self.attached_collision_object_ids:
            self.detach_collision_object(object_id)
        assert len(self.attached_collision_object_ids) == 0

    def remove_collision_object(self, object_id: str):
        """Remove a collision object from the planning scene.

        Args:
            object_id: ID of the collision object to remove.
        """
        self.log(f"Removing collision object: {object_id}")
        collision_object = CollisionObject(
            id=object_id, operation=CollisionObject.REMOVE
        )
        self.psm.process_collision_object(collision_object)

    def remove_all_collision_objects(self):
        """Remove all collision objects from the planning scene.

        Detaches all attached objects first, then removes all world objects.
        Clears grid object tracking dicts and verifies empty state.
        """
        self.log("Removing all collision objects", severity="DEBUG")

        self.detach_all_collision_objects()

        with self.psm.read_write() as scene:
            scene.remove_all_collision_objects()
            scene.current_state.update()

        self.grid_objects_by_id = {}
        self.grid_objects_by_idx = {}

        assert len(self.collision_object_ids) == 0

    def move_collision_object(self, object_id: str, pose_stamped: PoseStamped):
        """Move a collision object to a new pose.

        If the object is attached, detaches it first. Then updates its pose
        in the planning scene.

        Args:
            object_id: ID of the collision object to move.
            pose_stamped: Target pose in the planning frame.
        """
        self.log(f"Moving collision object: {object_id}", severity="DEBUG")
        if object_id in self.attached_collision_object_ids:
            self.detach_collision_object(object_id)

        collision_object = CollisionObject()
        collision_object.header.frame_id = pose_stamped.header.frame_id
        collision_object.id = object_id
        collision_object.pose = pose_stamped.pose
        collision_object.operation = CollisionObject.MOVE

        self.psm.process_collision_object(collision_object)

    ###########################################################################
    ########## Exclusive Regions ##############################################
    ###########################################################################

    def _init_exclusive_regions(self):
        """Populate exclusive-region tracking from parameters.

        Loads exclusive region definitions from the 'exclusive_regions'
        parameter. Each region is a set of collision walls that gate access
        to a restricted area. Validates that all referenced collision_ids
        exist in the planning scene.

        Does not add the wall primitives to the scene (they are added during
        _init_planning_scene as regular collision objects).
        """
        try:
            regions: dict[str, Any] = self.param("exclusive_regions")
        except ParameterNotDeclaredException:
            return

        for region_id, config in regions.items():
            collision_ids: list[str] = config["collision_ids"]
            unknown = set(collision_ids) - set(self.collision_object_ids)
            if len(unknown) > 0:
                raise ValueError(
                    f"'exclusive_regions.{region_id}.collision_ids' parameter "
                    f"contains unkown collision ojects: {unknown}. "
                    f"Available: '{self.collision_object_ids}'"
                )
            self._exclusive_regions[region_id] = ExclusiveRegion(
                region_id=region_id,
                collision_ids=collision_ids,
                acquired=False,
                group_name=None,
                modified_collisions=None,
            )

    def acquire_exclusive_region(
        self,
        region_id: str,
        *,
        group_name: str,
        robot_collision_ids: list[str],
        region_collision_ids: list[str],
    ) -> None:
        """Acquire exclusive access to a region for a planning group.

        Allows each robot collision object to pass through each region wall
        by adding collision allowances to the ACM. Only one group may hold
        a region at a time.

        This is non-blocking: if the region is already held, `RuntimeError`
        is raised immediately rather than waiting for release.

        Args:
            region_id: The region to acquire (must match a key under
                `planning_scene.exclusive_regions` in config).
            group_name: Planning group name acquiring the region. Used to
                validate matching releases.
            robot_collision_ids: Collision object IDs that should be allowed
                to pass through the region's walls while it is held. Must be
                non-empty.
            region_collision_ids: Subset of the region's wall IDs that should
                have allowances added. Must be non-empty and all must belong
                to the region.

        Raises:
            ValueError: If region_id is unknown, group_name is invalid,
                or collision IDs are invalid/empty.
            RuntimeError: If the region is already held by any group.
        """
        self.log(
            f"Acquiring exclusive region '{region_id}' for '{group_name}'"
        )

        if region_id not in self._exclusive_regions:
            raise ValueError(
                f"Unknown exclusive region: '{region_id}'. "
                f"Known regions: {list(self._exclusive_regions.keys())}"
            )

        group_names = self.robot_model.joint_model_group_names
        if group_name not in group_names:
            raise ValueError(
                f"Unknown group name: '{group_name}'. "
                f"Known group names: {group_names}"
            )

        if len(robot_collision_ids) == 0:
            raise ValueError("'robot_collision_ids' must be non-empty")

        if len(region_collision_ids) == 0:
            raise ValueError("'scene_collision_ids' must be non-empty")

        region = self._exclusive_regions[region_id]

        unknown = set(region_collision_ids) - set(region.collision_ids)
        if len(unknown) > 0:
            raise ValueError(
                f"Unknown 'region_collision_ids' for exclusive region "
                f"'{region_id}': {unknown}. Available: '{region.collision_ids}'"
            )

        if region.acquired:
            raise RuntimeError(
                f"Exclusive region '{region_id}' has already been acquire"
            )

        assert region.modified_collisions is None
        assert region.group_name is None

        to_allow = [
            (x, y) for x in robot_collision_ids for y in region_collision_ids
        ]
        modified = self.allow_collision(*zip(*to_allow))

        region.modified_collisions = modified
        region.group_name = group_name
        region.acquired = True

    def release_exclusive_region(
        self,
        region_id: str,
        *,
        group_name: str,
    ) -> None:
        """Release exclusive access to a region.

        Reverts all collision allowances added by the corresponding
        `acquire_exclusive_region` call, returning the region to its initial
        state (walls block all traffic).

        Args:
            region_id: The region to release (must match acquisition).
            group_name: Planning group name releasing the region. Must match
                the group that acquired it.

        Raises:
            ValueError: If region_id is unknown.
            RuntimeError: If the region is not currently held, or is held
                by a different group.
        """
        if region_id not in self._exclusive_regions:
            raise ValueError(
                f"Unknown exclusive region: '{region_id}'. "
                f"Known regions: {list(self._exclusive_regions)}"
            )

        region = self._exclusive_regions[region_id]

        if not region.acquired:
            raise RuntimeError(
                f"Exclusive region '{region_id}' has not been acquired, can't release"
            )

        assert region.group_name is not None
        assert region.modified_collisions is not None

        if group_name != region.group_name:
            raise RuntimeError(
                f"Exclusive region '{region_id}' held by another group_name, can't release"
            )

        self.disallow_collision(*zip(*region.modified_collisions))

        region.modified_collisions = None
        region.group_name = None
        region.acquired = False

    ###########################################################################
    ########## Logging ########################################################
    ###########################################################################

    def log_planning_scene(
        self, severity: SeverityString | LoggingSeverity = "INFO"
    ):
        """Log the complete planning scene state (objects, state, ACM).

        Clears mesh geometry from logged messages for readability.

        Args:
            severity: Log level (defaults to INFO).
        """
        if not isinstance(severity, LoggingSeverity):
            severity = LoggingSeverity[severity]

        if self.log_level < severity:
            return

        self.log("Logging planning scene", severity=severity)
        with self.psm.read_only() as scene:
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
        """Log the allowed collision matrix as a DataFrame.

        Args:
            severity: Log level (defaults to INFO).
        """
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
        """Log all collision objects and attached objects.

        Clears mesh geometry from logged messages for readability.

        Args:
            severity: Log level (defaults to INFO).
        """
        if not isinstance(severity, LoggingSeverity):
            severity = LoggingSeverity[severity]

        if self.log_level < severity:
            return

        self.log("Logging collision objects", severity=severity)
        with self.psm.read_only() as scene:
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
        """Clean up MoveItPy resources and shutdown the interface.

        Calls moveit_py.shutdown() and invokes parent cleanup.
        """
        self.log("Destroying PlanAndExecuteInterface")
        if hasattr(self, "moveit_py"):
            self.moveit_py.shutdown()
        super().destroy_interface()
