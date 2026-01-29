"""Object manipulation state machine for pick-and-place operations.

This module extends PlanAndExecuteInterface with high-level object manipulation
capabilities, implementing a state machine for fetching, presenting, and
returning objects during experiments.

State Machine Phases (ObjectPhase):
    PRE_FETCH -> PRE_ATTACH -> ATTACH -> POST_ATTACH -> POST_FETCH
    -> PRE_PRESENT -> PRESENT -> UNPRESENT
    -> PRE_RETURN -> PRE_DETACH -> DETACH -> POST_DETACH -> POST_RETURN -> IDLE

Key Operations:
    - fetch_object(): Pick up an object from its rest position
    - present_object(): Move to presentation position with held object
    - return_object(): Return object to its rest position and release
    - reset_object(): Return object after an interrupted operation

The interface manages object attachment state, collision allowances, and
coordinates with the planning scene for proper collision checking during
manipulation.

Configuration:
    Object manipulation parameters (waypoints, collision settings) are loaded
    from YAML files in the configured objects directory.
"""

import asyncio
import os
from collections.abc import Callable, Coroutine
from enum import IntEnum
from glob import glob
from typing import Any, Optional

import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.exceptions import ParameterNotDeclaredException

from tabletop_py.utils.common import KwargYamlLoader
from tabletop_rig.exceptions import (
    MoveitRecoverableError,
    ObjectManipulationError,
    PlanningError,
)
from tabletop_rig.interfaces.moveit.plan_and_execute import (
    PlanAndExecuteInterface,
)
from tabletop_rig.interfaces.moveit.requests import (
    ConcatPlanRequest,
    ObjectResetConfig,
    PlanGoalT,
    TrajectoryCacheKwargs,
)
from tabletop_rig.nodes.base import BaseNode
from tabletop_rig.utils.ros import (
    change_reference_frame_pose_stamped,
    matrix_from_pose_msg,
    pose_stamped_msg,
)


class ObjectPhase(IntEnum):
    """State machine phases for object manipulation.

    The phases track the current state of object manipulation, from
    idle through fetch, present, and return operations.

    Attributes:
        PRE_FETCH: Moving toward object for pickup.
        PRE_ATTACH: At object, preparing to grasp.
        ATTACH: Grasping/attaching object.
        POST_ATTACH: Object attached, preparing to move away.
        POST_FETCH: Moving away with object.
        PRE_PRESENT: Moving toward presentation position.
        PRESENT: At presentation position, object visible.
        UNPRESENT: Moving away from presentation.
        PRE_RETURN: Moving toward return position.
        PRE_DETACH: At return position, preparing to release.
        DETACH: Releasing object.
        POST_DETACH: Object released, moving away.
        POST_RETURN: Completing return motion.
        IDLE: No active manipulation, ready for next command.
    """

    PRE_FETCH = 0
    PRE_ATTACH = 1
    ATTACH = 2
    POST_ATTACH = 3
    POST_FETCH = 4
    PRE_PRESENT = 5
    PRESENT = 6
    UNPRESENT = 7
    PRE_RETURN = 8
    PRE_DETACH = 9
    DETACH = 10
    POST_DETACH = 11
    POST_RETURN = 12
    IDLE = 13


MOUNT_PHASES = set(
    (
        ObjectPhase.PRE_ATTACH,
        ObjectPhase.ATTACH,
        ObjectPhase.POST_ATTACH,
        ObjectPhase.POST_FETCH,
        ObjectPhase.PRE_DETACH,
        ObjectPhase.DETACH,
        ObjectPhase.POST_DETACH,
        ObjectPhase.POST_RETURN,
    )
)

# Mapping of phases to their corresponding pose offsets
PHASE_OFFSET_MAP = {
    ObjectPhase.PRE_FETCH: "pre_fetch",
    ObjectPhase.PRE_ATTACH: "pre_attach",
    ObjectPhase.ATTACH: "attach",
    ObjectPhase.POST_ATTACH: "post_attach",
    ObjectPhase.POST_FETCH: "post_fetch",
    ObjectPhase.PRE_RETURN: "post_fetch",
    ObjectPhase.PRE_DETACH: "post_attach",
    ObjectPhase.DETACH: "attach",
    ObjectPhase.POST_DETACH: "pre_attach",
    ObjectPhase.POST_RETURN: "pre_fetch",
}


def object_manipulation_lock_decorator(
    coro_fn: Callable[..., Coroutine],
) -> Callable[..., Coroutine]:
    """Decorator for methods that should be run with the object manipulation lock."""

    async def wrapper(
        self: "ObjectManipulationInterface", *args: Any, **kwargs: Any
    ):
        async with self.object_manipulation_lock:
            return await coro_fn(self, *args, **kwargs)

    return wrapper


class ResetLoader(KwargYamlLoader):
    def get_kwarg_constructors(self) -> dict[str, Callable]:
        return {
            "!PoseStamped": pose_stamped_msg,
            "!ConcatPlanRequest": ConcatPlanRequest,
            "!ObjectResetConfig": ObjectResetConfig,
        }


class ObjectManipulationInterface(PlanAndExecuteInterface):
    # TODO: Documentation
    def __init__(
        self,
        node: BaseNode,
        safe_to_execute_callback: Callable[[], bool],
        logger_name: str = "moveit_plan_interface",
    ):
        """Initializes the MoveItObjectInterface"""
        super().__init__(node, safe_to_execute_callback, logger_name)

        self._init_attached_object()

        self._init_reset_configs()

        self.object_manipulation_lock = asyncio.Lock()
        self._object_phase = ObjectPhase.IDLE

        self._object_reset: dict[str, bool] = {}
        for object_id in self.grid_object_poses.keys():
            self._object_reset[object_id] = True

        self.log("MoveIt object manipulation interface initialized")

    def _init_reset_configs(self):
        # Check that all grid objects have reset configurations
        grid_objects = set(self.grid_object_poses.keys())
        reset_objects = set(self.reset_config_map.keys())
        missing = grid_objects - reset_objects
        if len(missing) > 0:
            raise ValueError(
                "All grid objects must have associated reset "
                "configs in object_manipulation.reset_configs"
            )

        # Parse and save the configurations for each unique file
        self.reset_configs: dict[str, ObjectResetConfig] = {}

        unique_files: set[str] = set(self.reset_config_map.values())
        for filename in unique_files:
            with open(filename) as f:
                config: ObjectResetConfig = yaml.load(f, ResetLoader)

            if not isinstance(config, ObjectResetConfig):
                raise TypeError(
                    f"Incorrect parsing of object reset config file {filename}. "
                    f"Make sure to add the !ObjectResetConfig YAML tag to the "
                    f"beginning of the file, as well as the !ConcatPlanRequest "
                    f"and !PoseStamped tags where necessary "
                    f"(see object_reset/example.yaml)"  # TODO
                )

            if (
                config.object_allowed_collision_ids is not None
                and len(config.object_allowed_collision_ids) > 0
            ) or (
                config.additional_allowed_collisions is not None
                and len(config.additional_allowed_collisions) > 0
            ):
                if (
                    config.reset_request.planning_pipeline is not None
                    and config.reset_request.planning_pipeline != "linear"
                ):
                    raise ValueError(
                        "If 'object_allowed_collision_ids' or 'additional_allowed_collisions' "
                        "is provided, the 'reset_request' planning_pipeline must be linear or "
                        "not provided for object resetting (to prevent accidental collisions)"
                    )
                config.reset_request.planning_pipeline = "linear"

            self.reset_configs[filename] = config

    def _init_attached_object(self):
        """Initialize the attached object."""
        object_id = None
        idx = None

        try:
            object_id = self.node.param("initial_attached_object")
        except ParameterNotDeclaredException:
            pass

        try:
            idx = self.node.param("initial_attached_object_idx")
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
        self.move_collision_object(
            object_id, self.get_link_pose_stamped(self.default_pose_link)
        )
        self.attach_collision_object(
            object_id, self.default_pose_link, touch_links=self.touch_links
        )

    ###########################################################################
    ########## Parameter Convenience Properties ###############################
    ###########################################################################

    @property
    def reset_config_map(self) -> dict[str, str]:
        """Get the mapping of object ids to reset configuration filenames"""
        return self.node.param("object_manipulation.reset_config_map")

    @property
    def touch_links(self) -> list[str]:
        """Get the touch links from the parameter server."""
        return self.node.param("object_manipulation.touch_links")

    @property
    def mount_ids(self) -> list[str]:
        """Get the object mount ids from the parameter server."""
        return self.node.param("object_manipulation.mount_ids")

    @property
    def mount_allowed_collisions(self) -> list[tuple[str, str]]:
        """Get the allowed object mount collisions from the parameter server."""
        return [
            (id_0, id_1)
            for id_0, id_1 in self.node.param(
                "object_manipulation.mount_allowed_collisions"
            ).items()
        ]

        # ret = self.node.param("object_manipulation.mount_allowed_collisions")
        # print(ret)
        # return ret

    ###########################################################################
    ########## Object Manipulation Convenience Methods ########################
    ###########################################################################

    def _grid_object_pose_stamped_with_offset(
        self, object_id: str, offset: list[float]
    ) -> PoseStamped:
        """Get the initial pose of an object from the parameters with an offset."""
        old_pose_stamped = self.grid_object_poses[object_id]
        old_frame_transform = matrix_from_pose_msg(old_pose_stamped.pose)
        new_frame_id = old_pose_stamped.header.frame_id
        new_frame_transform = self.get_frame_transform(new_frame_id)

        pose_stamped = pose_stamped_msg(position=offset)

        return change_reference_frame_pose_stamped(
            old_pose_stamped=pose_stamped,
            old_frame_transform=old_frame_transform,
            new_frame_transform=new_frame_transform,
            new_frame_id=new_frame_id,
        )

    ###########################################################################
    ########## Fetch, present, and return #####################################
    ###########################################################################

    def _get_phase_goal(
        self,
        phase: ObjectPhase,
        object_id: str,
    ) -> PlanGoalT:
        """Get the goal for the given phase and object.

        Args:
            phase: The phase to get the goal for
            object_id: The ID of the object to get the goal for
            goal: The goal to use if the phase is present or unpresent

        Returns:
            The goal for the given phase and object.
        """
        match phase:
            case ObjectPhase.IDLE:
                return "idle"
            case ObjectPhase.PRE_PRESENT:
                return "pre_present"
            case ObjectPhase.UNPRESENT:
                return self.create_pose_stamped(
                    **self.node.param(
                        "object_manipulation.unpresent_pose_stamped"
                    )
                )
            case _:
                offset = self.node.param(
                    f"object_manipulation.phase_offsets.{PHASE_OFFSET_MAP[phase]}"
                )
                return self._grid_object_pose_stamped_with_offset(
                    object_id, offset
                )

    async def _execute_phase(
        self,
        object_id: str,
        phase: ObjectPhase,
        **kwargs: Any,
    ) -> list[TrajectoryCacheKwargs] | None:
        """Plan and execute a phase of the object manipulation process.

        This is a helper function for the object manipulation process.

        Args:
            object_id: The ID of the object to manipulate
            phase: The phase to manipulate the object in
            cache_trajectory: Whether to cache the trajectory after a single
                phase
            **kwargs: Additional keyword arguments to pass to `_plan_and_execute_cached()`

        Returns:
            A dictionary containing the kwargs to cache the trajectory, or None
            if the trajectory was found in the cache.
        """
        self.log(f"{phase.name} phase for object {object_id}")

        goal = self._get_phase_goal(phase, object_id)
        extra_kwargs = {}
        extra_kwargs["planning_pipeline"] = "linear"
        extra_kwargs["use_cache"] = False

        if phase in MOUNT_PHASES:
            self.allow_collision(*zip(*self.mount_allowed_collisions))

        if phase == ObjectPhase.DETACH:
            extra_kwargs["velocity_scaling_factor"] = self.node.param(
                "object_manipulation.detach_velocity_scaling_factor"
            )

        match phase:
            case (
                ObjectPhase.PRE_FETCH
                | ObjectPhase.PRE_PRESENT
                | ObjectPhase.PRE_RETURN
                | ObjectPhase.IDLE
            ):
                del extra_kwargs["planning_pipeline"]
                del extra_kwargs["use_cache"]
            case (
                ObjectPhase.PRE_ATTACH
                | ObjectPhase.ATTACH
                | ObjectPhase.POST_DETACH
                | ObjectPhase.POST_RETURN
            ):
                self.allow_collision(object_id, self.touch_links)
            case ObjectPhase.POST_ATTACH | ObjectPhase.DETACH:
                self.allow_collision(object_id, self.mount_ids)

        self.log(f"{phase.name} goal: {goal}", severity="DEBUG")

        cache_kwargs = None
        try:
            cache_kwargs = await self.plan_and_execute(
                goal=goal, **kwargs, **extra_kwargs
            )
        except PlanningError as e:
            match phase:
                case (
                    ObjectPhase.POST_FETCH
                    | ObjectPhase.PRESENT
                    | ObjectPhase.UNPRESENT
                    | ObjectPhase.PRE_DETACH
                ):
                    self.log(
                        f"Error while planning for {phase.name} phase with linear pipeline: {e}",
                        severity="WARN",
                    )
                    self.log(
                        f"Attempting to plan and execute {phase.name} phase with default pipeline",
                        severity="WARN",
                    )
                    await self.plan_and_execute(goal=goal, **kwargs)
                case _:
                    raise
        finally:
            if phase in MOUNT_PHASES:
                self.disallow_collision(*zip(*self.mount_allowed_collisions))
            match phase:
                case (
                    ObjectPhase.PRE_ATTACH
                    | ObjectPhase.ATTACH
                    | ObjectPhase.POST_DETACH
                    | ObjectPhase.POST_RETURN
                ):
                    self.disallow_collision(object_id, self.touch_links)
                case ObjectPhase.POST_ATTACH | ObjectPhase.DETACH:
                    self.disallow_collision(object_id, self.mount_ids)

        match phase:
            case ObjectPhase.ATTACH:
                self.attach_collision_object(
                    object_id,
                    self.default_pose_link,
                    touch_links=self.touch_links,
                )
            case ObjectPhase.DETACH:
                self.detach_collision_object(object_id)

        return cache_kwargs

    @object_manipulation_lock_decorator
    async def fetch_object(
        self, object_id: str, *, cache_trajectories: bool = True
    ):
        """Fetch an object from its mount.

        The robot moves to the object's mount, attaches the object, and moves
        to the object's post-fetch pose. It uses cached trajectories if
        available and only caches the planned trajectories if the full fetch
        process is successful. This addresses the issue of the robot getting
        "stuck" in a state that it cannot complete the full fetch process and
        caching trajectories that are unusable.

        Args:
            object_id: The ID of the object to fetch
            cache_trajectories: Whether to cache the trajectories after fetching
                the object

        Raises:
            ValueError: If the object ID is not a valid collision object
            PlanningError: If the planning fails
            ExecutionError: If the execution fails
        """
        self.log(f"Fetching object {object_id}")

        if len(self.attached_collision_object_ids) > 0:
            raise ObjectManipulationError(
                "Cannot fetch object while another object is attached"
            )

        # Check that the object ID is valid
        if object_id not in self.collision_object_ids:
            raise ValueError(f"{object_id} is not a valid collision object")

        # Iterate through the fetch phases, returning the object to its mount
        # if the fetch fails
        cache_kwargs: list[TrajectoryCacheKwargs] = []
        for i in range(ObjectPhase.PRE_FETCH, ObjectPhase.POST_FETCH + 1):
            # self.object_phase = ObjectPhase(i)
            kwargs = await self._execute_phase(
                object_id, ObjectPhase(i), cache_trajectory=False
            )
            if kwargs is not None and cache_trajectories:
                cache_kwargs.extend(kwargs)
        # TODO: Move restart logic
        # self.log(
        #     f"Error while fetching object: {e}",
        #     severity="ERROR",
        # )
        # self.log("Attempting to return object to mount", severity="WARN")
        # start_idx = ObjectPhase.IDLE - i  # type: ignore
        # for i in range(start_idx, ObjectPhase.IDLE + 1):
        #     await self._object_phase(
        #         object_id, ObjectPhase(i), cache_trajectory=False
        #     )
        # raise

        # Cache all trajectories if requested
        if cache_trajectories and len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    @object_manipulation_lock_decorator
    async def pre_present_object(self):
        """Present the currently attached object at the specified end goal.

        Args:
            goal: The goal to present the object at
        """
        object_id = self.get_exactly_one_attached_object_id()
        self.log(f"Pre-presenting object {object_id}")

        # Pre-present phase
        await self._execute_phase(object_id, ObjectPhase.PRE_PRESENT)

        # Set object reset state to False
        self._object_reset[object_id] = False

    @object_manipulation_lock_decorator
    async def reset_object(
        self, *, unpresent: bool = True, cache_trajectories: bool = True
    ):
        """Unpresent the currently attached object and move it to its pre-return pose."""
        object_id = self.get_exactly_one_attached_object_id()

        if self._object_reset[object_id]:
            self.log(f"No need to reset object {object_id}, skipping")
            return
        else:
            self.log(f"Resetting object {object_id}")

        # Retrieve reset config
        filename = self.reset_config_map[object_id]
        config = self.reset_configs[filename]
        assert config.reset_request.planning_pipeline == "linear"

        # Unpresent (for caching reasons)
        if unpresent:
            await self._execute_phase(object_id, ObjectPhase.UNPRESENT)

        cache_kwargs: list[TrajectoryCacheKwargs] = []

        # Plan and execute to start goal
        kwargs = await self.plan_and_execute(
            goal=config.start_goal, cache_trajectory=False
        )
        if kwargs is not None:
            cache_kwargs.extend(kwargs)

        # Plan and execute reset path with allowed collisions
        allowed_collisions: list[tuple[str, str]] = []
        if config.object_allowed_collision_ids is not None:
            allowed_collisions = [
                (object_id, aid) for aid in config.object_allowed_collision_ids
            ]
        if config.additional_allowed_collisions is not None:
            allowed_collisions.extend(config.additional_allowed_collisions)

        if len(allowed_collisions) > 0:
            self.allow_collision(*zip(*allowed_collisions))
        try:
            kwargs = await self.plan_and_execute(
                config.reset_request, cache_trajectory=False
            )
            if kwargs is not None:
                cache_kwargs.extend(kwargs)
        finally:
            if len(allowed_collisions) > 0:
                self.disallow_collision(*zip(*allowed_collisions))

        self._object_reset[object_id] = True

        # Cache all trajectories if requested
        if cache_trajectories and len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    @object_manipulation_lock_decorator
    async def return_object(self, *, cache_trajectories: bool = True):
        """Return an object to its original position.

        Args:
            cache_trajectories: Whether to cache the trajectories after
                returning the object

        Raises:
            RuntimeError: If exactly one object is not attached
            PlanningError: If the planning fails
            ExecutionError: If the execution fails
        """
        object_id = self.get_exactly_one_attached_object_id()
        if object_id not in self.grid_object_poses:
            raise RuntimeError(
                f"Object {object_id} is not in the grid, cannot return it"
            )

        if not self._object_reset[object_id]:
            raise ObjectManipulationError(
                f"Object {object_id} has not been reset yet, cannot return"
            )

        self.log(f"Returning object {object_id}")

        cache_kwargs: list[TrajectoryCacheKwargs] = []

        # Iterate through the unpresenting and returning phases
        # for i in range(ObjectPhase.PRE_RETURN, ObjectPhase.IDLE + 1):
        for i in range(ObjectPhase.PRE_RETURN, ObjectPhase.POST_RETURN + 1):
            kwargs = await self._execute_phase(
                object_id, ObjectPhase(i), cache_trajectory=False
            )
            if kwargs is not None:
                cache_kwargs.extend(kwargs)

        # Cache all trajectories if requested
        if cache_trajectories and len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    ###########################################################################
    ########## Attach object manually #########################################
    ###########################################################################

    def add_manually_attached_collision_object(self, object_id: str):
        """Add a manually attached collision object to the planning scene."""
        self.log(f"Adding manually attached collision object: {object_id}")
        mesh_dir = self.node.param("planning_scene.object_meshes.path")
        mesh_paths = glob(os.path.join(mesh_dir, f"{object_id}.*"))
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
            pose_stamped=self.get_link_pose_stamped(self.default_pose_link),
            **self.node.param("manually_attached_object_kwargs"),
        )
        self.attach_collision_object(
            object_id, self.default_pose_link, touch_links=self.touch_links
        )

    ###########################################################################
    ########## Reset ##########################################################
    ###########################################################################

    async def reset_rig(self, end_goal: Optional[PlanGoalT] = None):
        """Move the robot out of collision if necessary and return any attached
        objects to their original positions.
        """
        self.log("Resetting rig")
        try:
            if len(self.attached_collision_object_ids) > 0:
                object_id = self.get_exactly_one_attached_object_id()
                if object_id in self.grid_object_poses:
                    await self.reset_object(unpresent=False)
                    await self.return_object()
            if end_goal is not None:
                await self.plan_and_execute(goal=end_goal)
        except MoveitRecoverableError as e:
            if self.node.param("simulate"):
                self.log(
                    f"Error while resetting rig: {type(e).__name__}: {e}",
                    severity="ERROR",
                )
                self.log(
                    "Clearing planning scene and moving out of collision (simulation only)",
                    severity="WARN",
                )
                await self.clear_scene_and_reset(end_goal)
            else:
                raise
