import glob
import hashlib
import json
import os
from collections.abc import Iterable, Mapping
from copy import deepcopy
from typing import Any, ContextManager, Literal, Optional

import numpy as np
import pandas as pd
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.collision_detection import (  # type: ignore[reportMissingModuleSource]
    AllowedCollisionMatrix,
    CollisionRequest,
    CollisionResult,
)
from moveit.core.planning_scene import (  # type: ignore[reportMissingModuleSource]
    PlanningScene,
)
from moveit.planning import MoveItPy, PlanningSceneMonitor
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
from trimesh.transformations import identity_matrix

from tabletop_py.utils.mesh import (
    load_geometry,
    simplify_convex_hull,
    simplify_quadratic_decimation,
    transform_geometry,
)
from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.nodes.base import BaseNode
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


class PlanningSceneInterface(BaseInterface):
    def __init__(
        self, node: BaseNode, logger_name: str = "moveit_scene_interface"
    ):
        """Initializes the MoveItSceneInterface

        Initializes MoveItPy and planning scene

        Args:
            node: Parent ROS node for accessing parameters and communicating with other nodes
            logger_name: Name given to child logger of parent node
        """
        super().__init__(node, logger_name)

        self.moveit_py = MoveItPy("moveit_py", provide_planning_service=True)

        self._init_planning_scene()

        self._init_attached_object()

        self._init_link_padding()

        self._init_collision_detector()

        self.log("MoveIt scene interface initialized")

    def _init_planning_scene(self):
        """Setup the planning scene

        Adds plane, primitive, and mesh collision objects from the planning
        scene configuration.
        """
        self.log("Initializing planning scene")

        self.remove_all_collision_objects()

        config: dict[str, Any] = self.node.get_parameter_wrapper(
            "planning_scene"
        )

        cache_dir = os.path.expandvars(os.path.expanduser(config["dir"]))
        if not os.path.isabs(cache_dir):
            raise ValueError(
                f"Planning scene cache directory must be absolute: {cache_dir}"
            )

        scene_path = os.path.join(cache_dir, "scene.txt")
        collision_matrix_path = os.path.join(cache_dir, "collision_matrix.csv")
        config_path = os.path.join(cache_dir, "config.yaml")
        scene_hash_path = os.path.join(cache_dir, "scene_hash.txt")
        grid_object_poses_path = os.path.join(
            cache_dir, "grid_object_poses.yaml"
        )

        if config["use_saved_scene"]:
            if all(
                os.path.exists(path)
                for path in [
                    scene_path,
                    collision_matrix_path,
                    config_path,
                    scene_hash_path,
                    grid_object_poses_path,
                ]
            ):
                with open(config_path, "r") as f:
                    saved_config = yaml.safe_load(f)
                with open(scene_hash_path, "r") as f:
                    saved_scene_hash = f.read().strip()
                if (
                    saved_config == config
                    and saved_scene_hash == self.scene_hash
                ):
                    self.load_planning_scene(scene_path)
                    self.load_collision_matrix(collision_matrix_path)
                    self.load_grid_object_poses(grid_object_poses_path)
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
        self.save_grid_object_poses(grid_object_poses_path)
        with open(scene_hash_path, "w") as f:
            f.write(self.scene_hash)
        with open(config_path, "w") as f:
            yaml.dump(orig_config, f)

    def _init_attached_object(self):
        """Initialize the attached object."""
        object_id = None
        idx = None

        try:
            object_id = self.node.get_parameter_wrapper(
                "initial_attached_object"
            )
        except ParameterNotDeclaredException:
            pass

        try:
            idx = self.node.get_parameter_wrapper(
                "initial_attached_object_idx"
            )
        except ParameterNotDeclaredException:
            pass

        if object_id is not None:
            if idx is not None:
                raise ValueError(
                    "Cannot specify both initial_attached_object and initial_attached_object_idx"
                )
            if object_id not in self.collision_object_ids:
                raise ValueError(
                    f"Initial attached object {object_id} not found in collision object ids"
                )
            self.log(
                f"Moving and attaching initial object {object_id} from name"
            )
        elif idx is not None:
            object_id = self.object_grid[*idx]
            if object_id is None:
                raise ValueError(f"No object at index {idx}")
            assert object_id in self.collision_object_ids
            self.log(
                f"Moving and attaching initial object {object_id} from index {idx}"
            )
        else:
            self.log("No initial attached object specified")
            return

        assert isinstance(object_id, str)
        self.move_collision_object(object_id, self.eef_pose_stamped())
        self.attach_collision_object(
            object_id, self.default_pose_link, touch_links=self.touch_links
        )

    def _init_link_padding(self):
        """Set the link padding for the planning scene."""
        config: dict[str, Any] = self.node.get_parameter_wrapper(
            "link_padding"
        )
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
    ########## Parameter Convenience Properties ###############################
    ###########################################################################

    @property
    def default_group_name(self) -> str:
        """Get the planning group name from the parameter server."""
        return self.node.get_parameter_wrapper("planning.defaults.group_name")

    @property
    def default_pose_link(self) -> str:
        """Get the planning link from the parameter server."""
        return self.node.get_parameter_wrapper("planning.defaults.pose_link")

    @property
    def allowed_object_mount_collisions(self) -> list[tuple[str, str]]:
        """Get the allowed object mount collisions from the parameter server."""
        return [
            (id_0, id_1)
            for id_0, id_1 in self.node.get_parameter_wrapper(
                "object_manipulation.allowed_collisions"
            ).items()
        ]

    @property
    def touch_links(self) -> list[str]:
        """Get the touch links from the parameter server."""
        return self.node.get_parameter_wrapper(
            "object_manipulation.touch_links"
        )

    @property
    def object_grid(self) -> np.ndarray:
        """Get the object grid config from the parameters."""
        object_kwargs = self.node.get_parameter_wrapper(
            "planning_scene.object_meshes.object_kwargs"
        )

        object_grid = np.empty((10, 3), dtype=object)
        for idx, kwargs in object_kwargs.items():
            x, y = idx.split(",")
            object_grid[int(x), int(y)] = kwargs["object_id"]

        return object_grid

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

        # Reorder the matrix to put robot collision links first
        robot_collision_links = self.node.get_parameter_wrapper(
            "planning_scene.robot_collision_links"
        )
        collision_object_ids = set(object_ids) - set(robot_collision_links)
        columns = robot_collision_links + list(collision_object_ids)
        matrix_df = matrix_df.loc[columns, columns]

        return matrix_df

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

    def get_frame_pose_stamped(
        self, frame_id: str, **kwargs: Any
    ) -> PoseStamped:
        """Get the frame pose relative to the planning frame for a given frame id."""
        return self.create_pose_stamped(
            pose=pose_msg_from_matrix(self.get_frame_transform(frame_id)),
            **kwargs,
        )

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

    def eef_pose_stamped(self, frame_id: Optional[str] = None) -> PoseStamped:
        """Get the current end-effector pose."""
        with self.planning_scene_ro() as scene:
            eef_pose = scene.current_state.get_pose(self.default_pose_link)

        pose_stamped = self.create_pose_stamped(
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

    @property
    def scene_hash(self) -> str:
        """Get the hash of the rig, for consistency purposes.

        Returns:
            The hash of the rig.
        """
        config = self.node.get_parameter_wrapper("planning_scene")

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
        hash_algorithm.update(
            json.dumps(
                config["object_meshes"]["grid_origin"], sort_keys=True
            ).encode("utf-8")
        )
        keys_to_hash = ["rel_pose", "correction", "scale"]
        for kwargs in config["object_meshes"]["object_kwargs"].values():
            for key in keys_to_hash:
                if key in kwargs:
                    hash_algorithm.update(
                        json.dumps(kwargs[key], sort_keys=True).encode("utf-8")
                    )

        # Base link pose
        position, _ = arrays_from_pose_msg(
            self.get_frame_pose_stamped("base_link").pose
        )
        hash_algorithm.update(position.tobytes())

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

    def save_grid_object_poses(self, path: str):
        """Save the grid object poses to a file."""
        self.log(f"Saving grid object poses to {path}")
        grid_object_poses = {}
        for object_id, pose_stamped in self.grid_object_poses.items():
            grid_object_poses[object_id] = {
                "frame_id": pose_stamped.header.frame_id,
                "position": [
                    pose_stamped.pose.position.x,
                    pose_stamped.pose.position.y,
                    pose_stamped.pose.position.z,
                ],
                "orientation": [
                    pose_stamped.pose.orientation.w,
                    pose_stamped.pose.orientation.x,
                    pose_stamped.pose.orientation.y,
                    pose_stamped.pose.orientation.z,
                ],
            }
        with open(path, "w") as f:
            yaml.dump(grid_object_poses, f)

    def load_grid_object_poses(self, path: str):
        """Load the grid object poses from a file."""
        self.log(f"Loading grid object poses from {path}")
        with open(path, "r") as f:
            grid_object_poses = yaml.safe_load(f)

        self.grid_object_poses = {}
        for object_id, kwargs in grid_object_poses.items():
            pose_stamped = pose_stamped_msg(
                frame_id=kwargs["frame_id"],
                position=kwargs["position"],
                orientation=kwargs["orientation"],
            )
            self.grid_object_poses[object_id] = pose_stamped

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

    def get_exactly_one_attached_object_id(self) -> str:
        """Get the ID of the exactly one attached collision object.

        Returns:
            The ID of the attached collision object.

        Raises:
            RuntimeError: If there is not exactly one attached collision object
        """
        attached_collision_object_ids = self.attached_collision_object_ids
        if len(attached_collision_object_ids) != 1:
            raise RuntimeError(
                f"Expected exactly one attached collision object, but got {len(attached_collision_object_ids)}"
            )
        return attached_collision_object_ids[0]

    def check_collision(
        self, group_name: Optional[str] = None
    ) -> CollisionResult:
        """Check if an object is colliding with the planning scene."""
        if group_name is None:
            group_name = self.default_group_name

        self.log(f"Checking collision for group {group_name}")

        request = CollisionRequest()
        request.joint_model_group_name = group_name
        request.contacts = True
        request.max_contacts = 100
        request.max_contacts_per_pair = 1
        request.cost = False
        request.verbose = True

        with self.planning_scene_ro() as scene:
            result = CollisionResult()
            scene.check_collision(request, result)
            return result

    def is_state_colliding(self, group_name: Optional[str] = None) -> bool:
        """Check if the current state of the planning scene is colliding."""
        if group_name is None:
            group_name = self.default_group_name

        with self.planning_scene_ro() as scene:
            return scene.is_state_colliding(group_name)

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
        self, id_0: str | Iterable[str], id_1: str | Iterable[str], allow: bool
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

        # Modify the collision matrix
        modified: list[tuple[str, str]] = []
        with self.planning_scene_rw() as scene:
            matrix: AllowedCollisionMatrix = scene.allowed_collision_matrix
            for x, y in zip(ids_0, ids_1):
                success, allowed_collision_type = matrix.get_entry(x, y)
                allowed = self._parse_collision_matrix_entry(
                    success, allowed_collision_type
                )
                if allowed == allow:
                    # self.log(
                    #     f"Collision between {x} and {y} is already "
                    #     f"{'allowed' if allow else 'disallowed'}",
                    #     severity="DEBUG",
                    # )
                    pass
                else:
                    # self.log(
                    #     f"{'Allowing' if allow else 'Disallowing'} "
                    #     f"collision between {x} and {y}",
                    #     severity="DEBUG",
                    # )
                    matrix.set_entry(x, y, allow)
                    modified.append((x, y))

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

    def process_init_collision_object(
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

        self.process_init_collision_object(
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

        collision_object = add_primitive_collision_object_msg(
            object_id=object_id,
            pose_stamped=pose_stamped,
            type=type,
            dimensions=dimensions,
        )

        self.process_init_collision_object(
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
        additional_subframe_names: Optional[list[str]] = None,
        additional_subframe_poses: Optional[list[Pose]] = None,
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
        # Create pose stamped
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

        # Add subframes
        subframe_names = ["default"]
        subframe_poses = [Pose()]

        if (
            additional_subframe_names is not None
            or additional_subframe_poses is not None
        ):
            if (
                additional_subframe_names is None
                or additional_subframe_poses is None
            ):
                raise ValueError(
                    "Both additional subframe names and poses must be provided if one is provided"
                )
            if len(additional_subframe_names) != len(
                additional_subframe_poses
            ):
                raise ValueError(
                    "Number of additional subframe names and poses must match"
                )
            subframe_names.extend(additional_subframe_names)
            subframe_poses.extend(additional_subframe_poses)

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

        self.process_init_collision_object(
            collision_object=collision_object,
            color=color,
            allowed_collision_ids=allowed_collision_ids,
        )

    def add_grid_mesh_collision_objects(
        self,
        *,
        path: str,
        grid_origin: PoseStamped | Mapping[str, Any],
        common_kwargs: dict[str, Any],
        object_kwargs: dict[str, dict[str, Any]],
    ):
        """Add grid object meshes as collision objects to the planning scene.

        Loads meshes from a directory and adds them in a grid
        pattern based on the their index and the origin and delta.

        Args:
            path: The directory path to the object meshes.
            origin: The origin of the object meshes.
            delta: The delta of the object meshes in the x, y, and z directions.
            common_kwargs: The common kwargs for the object meshes.
            object_kwargs: The object kwargs for the object meshes.
        """
        # Get object origin and delta to calculate object position from index
        if not isinstance(grid_origin, PoseStamped):
            grid_origin = self.create_pose_stamped(**grid_origin)
        grid_origin_matrix = matrix_from_pose_msg(grid_origin.pose)
        origin_matrix = self.get_frame_transform(self.planning_frame)

        # Get object meshes paths
        if not os.path.isdir(path):
            if not os.path.exists(path):
                raise FileNotFoundError(
                    f"Object meshes path {path} does not exist"
                )
            raise NotADirectoryError(
                f"Object meshes path {path} is not a directory"
            )
        paths = glob.glob(os.path.join(path, "*.stl")) + glob.glob(
            os.path.join(path, "*.dae")
        )

        object_id_to_path: dict[str, str] = {}
        for mesh_path in paths:
            object_id = os.path.splitext(os.path.basename(mesh_path))[0]
            object_id_to_path[object_id] = mesh_path

        self.grid_object_poses: dict[str, PoseStamped] = {}

        for idx, overrides in object_kwargs.items():
            # Skip if object already exists in the planning scene
            object_id = overrides.pop("object_id", None)
            if object_id is None:
                self.log(
                    f"Skipping object at index {idx} because it does not have an id"
                )
                continue

            if object_id in self.collision_object_ids:
                self.log(
                    f"Skipping object mesh {object_id} because it already exists in the planning scene"
                )
                continue

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

            # Calculate global pose from relative pose and grid origin
            rel_pose = kwargs.pop("rel_pose")
            if not isinstance(rel_pose, Pose):
                rel_pose = pose_msg(**rel_pose)
            rel_pose_stamped = pose_stamped_msg(pose=rel_pose)

            pose_stamped = change_reference_frame_pose_stamped(
                old_pose_stamped=rel_pose_stamped,
                old_frame_transform=grid_origin_matrix,
                new_frame_transform=origin_matrix,
                new_frame_id=self.planning_frame,
            )

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

            self.grid_object_poses[object_id] = pose_stamped

    def attach_collision_object(
        self,
        object_id: str,
        link_name: str,
        *,
        touch_links: Optional[list[str]] = None,
    ):
        """Attach an object to the robot."""
        self.log(f"Attaching collision object {object_id}")
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
        self.log(f"Detaching collision object {object_id}")
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
        self.log("Removing all collision objects")

        self.detach_all_collision_objects()

        with self.planning_scene_rw() as scene:
            scene.remove_all_collision_objects()
            scene.current_state.update()

        assert len(self.collision_object_ids) == 0

    def move_collision_object(self, object_id: str, pose_stamped: PoseStamped):
        """Move a collision object."""
        self.log(f"Moving collision object: {object_id}")
        if object_id in self.attached_collision_object_ids:
            self.detach_collision_object(object_id)

        collision_object = CollisionObject()
        collision_object.header.frame_id = pose_stamped.header.frame_id
        collision_object.id = object_id
        collision_object.pose = pose_stamped.pose
        collision_object.operation = CollisionObject.MOVE

        self.planning_scene_monitor.process_collision_object(collision_object)

    def add_manually_attached_collision_object(self, object_id: str):
        """Add a manually attached collision object to the planning scene."""
        self.log(f"Adding manually attached collision object: {object_id}")
        mesh_dir = self.node.get_parameter_wrapper(
            "planning_scene.object_meshes.path"
        )
        mesh_paths = glob.glob(os.path.join(mesh_dir, f"{object_id}.*"))
        if not mesh_paths:
            raise FileNotFoundError(
                f"Mesh file for {object_id} not found in {mesh_dir}"
            )
        elif len(mesh_paths) > 1:
            raise ValueError(
                f"Multiple mesh files found for {object_id}: {mesh_paths}"
            )
        mesh_path = mesh_paths[0]

        self.add_mesh_collision_object(
            object_id=object_id,
            path=mesh_path,
            pose_stamped=self.eef_pose_stamped(),
            **self.node.get_parameter_wrapper(
                "manually_attached_object_kwargs"
            ),
        )
        self.attach_collision_object(
            object_id, self.default_pose_link, touch_links=self.touch_links
        )

    ###########################################################################
    ########## Logging ########################################################
    ###########################################################################

    def log_planning_scene(self, severity: str = "INFO"):
        """Log the planning scene."""
        if self.log_level < LoggingSeverity[severity]:
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

    def log_collision_matrix(self, severity: str = "INFO"):
        """Log the collision matrix."""
        if self.log_level < LoggingSeverity[severity]:
            return

        self.log(
            f"Allowed collision matrix: \n{self.collision_matrix_df.to_string()}",
            severity=severity,
        )

    def log_collision_objects(self, severity: str = "INFO"):
        """Log the collision objects."""
        if self.log_level < LoggingSeverity[severity]:
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

    def destroy(self):
        """Clean up MoveItPy"""
        if hasattr(self, "moveit_py"):
            self.moveit_py.shutdown()

    def __del__(self):
        self.destroy()
