"""Object manipulation state machine for pick-and-place operations.

This module extends PlanAndExecuteInterface with high-level object manipulation
capabilities, implementing a state machine for fetching, presenting, and
returning objects during experiments.

State Machine Transition (ManipulationState):
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
import functools
import os
from collections.abc import Callable
from enum import Enum
from glob import glob
from typing import Any, Optional, cast

import yaml
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from rclpy.exceptions import ParameterNotDeclaredException
from statemachine import Event, State, StateMachine
from statemachine.states import States

from tabletop_py.utils.common import KwargYamlLoader
from tabletop_rig.exceptions import (
    MoveitRecoverableError,
    ObjectManipulationError,
)
from tabletop_rig.interfaces.moveit.plan_and_execute import (
    PlanAndExecuteInterface,
)
from tabletop_rig.interfaces.moveit.requests import (
    ConcatPlanRequest,
    ObjectResetConfig,
    PlanGoalT,
    PlanRequest,
    TrajectoryCacheKwargs,
)
from tabletop_rig.nodes.base import BaseNode
from tabletop_rig.utils.ros import (
    change_reference_frame_pose_stamped,
    matrix_from_pose_msg,
    pose_stamped_msg,
)


class ManipulationState(Enum):
    """State machine states for object manipulation.

    The states track the current state of object manipulation, from
    idle through fetch, present, and return operations.

    Attributes:
        IDLE: No active manipulation, ready for next command.
        PRE_FETCH: Moving toward object mount fetch position
            (below and behind object mount).
        PRE_ATTACH: Moving up (w.r.t. object mount) to contact dovetail.
        ATTACH: Moving forward (w.r.t. object mount) into dovetail.
        POST_ATTACH: Moving forward (w.r.t. object mount) out of object mount.
        POST_FETCH: Moving down (w.r.t. object mount) out of object grid.
        PRESENT: Moving toward presentation state/pose.
        RESET: Resetting the object (user-defined ObjectResetConfig)
        PRE_RETURN: Moving toward object mount return position
            (below and in front of object mount).
        PRE_DETACH: Moving up (w.r.t. object mount) in front of object mount.
        DETACH: Moving back (w.r.t. object mount) into object mount.
        POST_DETACH: Moving back (w.r.t. object mount) out of dovetail.
        POST_RETURN: Moving down (w.r.t. object mount) away from dovetail.
    """

    IDLE = 0
    PRE_FETCH = 1
    PRE_ATTACH = 2
    ATTACH = 3
    POST_ATTACH = 4
    POST_FETCH = 5
    FETCHED = 6
    NEEDS_RESET = 7
    RESETTED = 8
    MANUALLY_ATTACHED = 9


def dummy_match(state):
    match state:
        case (
            ManipulationState.IDLE
            | ManipulationState.PRE_FETCH
            | ManipulationState.PRE_ATTACH
            | ManipulationState.ATTACH
            | ManipulationState.POST_ATTACH
            | ManipulationState.POST_FETCH
            | ManipulationState.FETCHED
            | ManipulationState.NEEDS_RESET
            | ManipulationState.RESETTED
            | ManipulationState.MANUALLY_ATTACHED
        ):
            pass


class ObjectManipulationStateMachine(StateMachine):
    model: "ObjectManipulationInterface"

    _ = States.from_enum(
        ManipulationState,
        initial=ManipulationState.IDLE,
        use_enum_instance=True,
    )

    fetch_step = (
        _.IDLE.to(_.PRE_FETCH, validators=["validate_no_object_attached"])
        | _.PRE_FETCH.to(
            _.PRE_ATTACH, validators=["validate_no_object_attached"]
        )
        | _.PRE_ATTACH.to(_.ATTACH, validators=["validate_no_object_attached"])
        | _.ATTACH.to(_.POST_ATTACH, validators=["validate_object_attached"])
        | _.POST_ATTACH.to(
            _.POST_FETCH, validators=["validate_object_attached"]
        )
        | _.POST_FETCH.to(_.FETCHED, validators=["validate_object_attached"])
    )

    reset_step = _.NEEDS_RESET.to(
        _.RESETTED, validators=["validate_object_attached"]
    )

    return_step = (
        _.FETCHED.to(_.POST_FETCH, validators=["validate_object_attached"])
        | _.RESETTED.to(_.POST_FETCH, validators=["validate_object_attached"])
        | _.POST_FETCH.to(
            _.POST_ATTACH, validators=["validate_object_attached"]
        )
        | _.POST_ATTACH.to(_.ATTACH, validators=["validate_object_attached"])
        | _.ATTACH.to(_.PRE_ATTACH, validators=["validate_object_attached"])
        | _.PRE_ATTACH.to(
            _.PRE_FETCH, validators=["validate_no_object_attached"]
        )
    )

    plan_and_execute = (
        _.IDLE.to.itself(validators=["validate_no_object_attached"])
        | _.PRE_FETCH.to(_.IDLE, validators=["validate_no_object_attached"])
        | _.POST_FETCH.to(
            _.NEEDS_RESET, validators=["validate_object_attached"]
        )
        | _.FETCHED.to(_.NEEDS_RESET, validators=["validate_object_attached"])
        | _.NEEDS_RESET.to.itself(validators=["validate_object_attached"])
        | _.RESETTED.to(_.NEEDS_RESET, validators=["validate_object_attached"])
    )

    execute = (
        _.IDLE.to.itself(validators=["validate_no_object_attached"])
        | _.PRE_FETCH.to.itself(validators=["validate_no_object_attached"])
        | _.POST_FETCH.to(
            _.NEEDS_RESET, validators=["validate_object_attached"]
        )
        | _.FETCHED.to(_.NEEDS_RESET, validators=["validate_object_attached"])
        | _.NEEDS_RESET.to.itself(validators=["validate_object_attached"])
        | _.RESETTED.to(_.NEEDS_RESET, validators=["validate_object_attached"])
    )

    manually_attach = _.IDLE.to(
        _.MANUALLY_ATTACHED, validators=["validate_no_object_attached"]
    )

    manually_detach = _.MANUALLY_ATTACHED.to(
        _.IDLE, validators=["validate_object_attached"]
    )


def object_manipulation_decorator(coro_fn):
    """Decorator for methods that should be run with the object manipulation lock."""

    @functools.wraps(coro_fn)
    async def wrapper(self: "ObjectManipulationInterface", *args, **kwargs):
        if self._manipulation_lock.locked():
            raise ObjectManipulationError(
                f"Cannot call {coro_fn.__name__} while another operation is in progress"
            )
        async with self._manipulation_lock:
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
    ):
        """Initializes the MoveItObjectInterface"""
        super().__init__(node, safe_to_execute_callback)

        self._init_attached_object()

        self._init_reset_configs()

        self._init_object_manipulation_state()

        self.log("MoveIt object manipulation interface initialized")

    def _init_reset_configs(self):
        # Check that all grid objects have reset configurations
        grid_objects = set(self.grid_objects.keys())
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

    def _init_object_manipulation_state(self):
        self._manipulation_lock = asyncio.Lock()

        if self.attached_object_id is None:
            self._manipulation_state = ManipulationState.IDLE
        else:
            self._manipulation_state = ManipulationState.MANUALLY_ATTACHED

        self._sm = ObjectManipulationStateMachine(
            model=self, state_field="_manipulation_state"
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
    def allowed_mount_collisions(self) -> list[tuple[str, str]]:
        """Get the allowed object mount collisions from the parameter server."""
        return [
            (id_0, id_1)
            for id_0, id_1 in self.node.param(
                "object_manipulation.allowed_mount_collisions"
            ).items()
        ]

    @property
    def attached_object_id(self) -> str | None:
        """Get the ID of the attached collision object

        Returns:
            The ID of the attached collision object or None if no object is attached

        Raises:
            RuntimeError: If there is not exactly one attached collision object
        """
        attached_ids = self.attached_collision_object_ids
        if len(attached_ids) == 0:
            return None
        elif len(attached_ids) == 1:
            return attached_ids[0]
        else:
            raise RuntimeError(
                f"Expected exactly one attached collision object, but got {len(attached_ids)}"
            )

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
        old_pose_stamped = self.grid_objects[object_id].pose_stamped
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

    async def on_transition(
        self, event: Event, state: State, source: State, target: State
    ):
        assert state == source
        assert state.value == self._manipulation_state
        self.log(
            f"Handling {event.name} event for transition from target state {source.name} to {target.name}"
        )

    def _get_fetch_return_plan_request(
        self,
        object_id: str,
        *,
        source_state: ManipulationState,
        target_state: ManipulationState,
    ) -> PlanRequest:
        """Get the goal for the given state and object.

        Args:
            state: The state to get the goal for
            object_id: The ID of the object to get the goal for
            goal: The goal to use if the state is present or unpresent

        Returns:
            The goal for the given state and object.
        """
        match target_state:
            case ManipulationState.IDLE:
                goal = "idle"
            case (
                ManipulationState.PRE_FETCH
                | ManipulationState.PRE_ATTACH
                | ManipulationState.ATTACH
                | ManipulationState.POST_ATTACH
                | ManipulationState.POST_FETCH
            ):
                offset = self.node.param(
                    f"object_manipulation.state_offsets.{target_state.name.lower()}"
                )
                goal = self._grid_object_pose_stamped_with_offset(
                    object_id, offset
                )
            case ManipulationState.FETCHED:
                goal = "fetched"
            case _:
                raise AssertionError(f"Unsupported state {target_state}")

        request = PlanRequest(goal=goal)
        match source_state, target_state:
            case (
                (_, ManipulationState.PRE_ATTACH)
                | (_, ManipulationState.ATTACH)
                | (ManipulationState.ATTACH, ManipulationState.POST_ATTACH)
                | (ManipulationState.PRE_ATTACH, ManipulationState.PRE_FETCH)
            ):
                request.planning_pipeline = "linear"
                request.use_cache = False

        if (source_state, target_state) == (
            ManipulationState.POST_ATTACH,
            ManipulationState.ATTACH,
        ):
            PlanRequest.velocity_scaling_factor = self.node.param(
                "object_manipulation.detach_velocity_scaling_factor"
            )

        return request

    def _get_fetch_return_allowed_collisions(
        self, object_id: str, *, source_state: ManipulationState, target_state
    ) -> list[tuple[str, str]]:
        allowed_collisions: list[tuple[str, str]] = []
        match source_state, target_state:
            case (
                (_, ManipulationState.PRE_ATTACH)
                | (_, ManipulationState.ATTACH)
                | (_, ManipulationState.POST_ATTACH)
                | (ManipulationState.POST_ATTACH, ManipulationState.POST_FETCH)
                | (ManipulationState.PRE_ATTACH, ManipulationState.PRE_FETCH)
            ):
                allowed_collisions.extend(self.allowed_mount_collisions)

        match source_state, target_state:
            case (_, ManipulationState.PRE_ATTACH) | (
                ManipulationState.PRE_ATTACH,
                _,
            ):
                allowed_collisions.extend(
                    [(object_id, x) for x in self.touch_links]
                )
            case (ManipulationState.ATTACH, ManipulationState.POST_ATTACH) | (
                ManipulationState.POST_ATTACH,
                ManipulationState.ATTACH,
            ):
                allowed_collisions.extend(
                    [(object_id, x) for x in self.mount_ids]
                )

        return allowed_collisions

    async def _on_fetch_return_step(
        self, object_id: str, *, event: Event, source: State, target: State
    ) -> list[TrajectoryCacheKwargs] | None:
        """Plan and execute a state transition of the object manipulation process.

        This is a helper function for the object manipulation process.

        Args:
            object_id: The ID of the object to manipulate
            next_state: The object manipulation state to transition to
            cache_trajectory: Whether to cache the trajectory after a single state transition
            **kwargs: Additional keyword arguments to pass to `_plan_and_execute_cached()`

        Returns:
            A dictionary containing the kwargs to cache the trajectory, or None
            if the trajectory was found in the cache.
        """
        self.log(f"Executing {event.name} event")
        source_state = cast(ManipulationState, source.value)
        target_state = cast(ManipulationState, target.value)

        request = self._get_fetch_return_plan_request(
            object_id, source_state=source_state, target_state=target_state
        )
        collisions_to_allow = self._get_fetch_return_allowed_collisions(
            object_id, source_state=source_state, target_state=target_state
        )

        allowed_collisions = self.allow_collision(*zip(*collisions_to_allow))
        try:
            cache_kwargs = await self._plan_and_execute(
                request, cache_trajectory=False
            )
        finally:
            if len(allowed_collisions) > 0:
                self.disallow_collision(*zip(*allowed_collisions))

        if target_state == ManipulationState.ATTACH:
            if event == self._sm.fetch_step:
                self.attach_collision_object(
                    object_id,
                    self.default_pose_link,
                    touch_links=self.touch_links,
                )
            else:
                assert event == self._sm.return_step
                self.detach_collision_object(object_id)

        return cache_kwargs

    async def on_fetch_step(
        self, object_id: str, *, event: Event, source: State, target: State
    ) -> list[TrajectoryCacheKwargs] | None:
        assert event == self._sm.fetch_step
        return await self._on_fetch_return_step(
            object_id, event=event, source=source, target=target
        )

    async def on_return_step(
        self, object_id: str, *, event: Event, source: State, target: State
    ) -> list[TrajectoryCacheKwargs] | None:
        assert event == self._sm.return_step
        return await self._on_fetch_return_step(
            object_id, event=event, source=source, target=target
        )

    async def on_reset(
        self, object_id: str, *, state: State, target: State
    ) -> list[TrajectoryCacheKwargs] | None:
        self.log(f"Resetting object {object_id}")

        # Retrieve reset config
        filename = self.reset_config_map[object_id]
        config = self.reset_configs[filename]
        assert config.reset_request.planning_pipeline == "linear"
        cache_kwargs: list[TrajectoryCacheKwargs] = []

        # Plan and execute to start goal
        kwargs = await self._plan_and_execute(
            goal=config.start_goal, cache_trajectory=False
        )
        if kwargs is not None:
            cache_kwargs.extend(kwargs)

        # Plan and execute reset path with allowed collisions
        allowed_collisions: list[tuple[str, str]] = []
        try:
            if config.object_allowed_collision_ids is not None:
                allowed_collisions.extend(
                    self.allow_collision(
                        object_id, config.object_allowed_collision_ids
                    )
                )

            if config.additional_allowed_collisions is not None:
                allowed_collisions.extend(
                    self.allow_collision(
                        *zip(*config.additional_allowed_collisions)
                    )
                )

            kwargs = await self._plan_and_execute(
                config.reset_request, cache_trajectory=False
            )
            if kwargs is not None:
                cache_kwargs.extend(kwargs)
        finally:
            if len(allowed_collisions) > 0:
                self.disallow_collision(*zip(*allowed_collisions))

        # Cache all trajectories if requested
        if len(cache_kwargs) > 0:
            return cache_kwargs

    def validate_no_object_attached(self, *, source: State, target: State):
        attached_object_id = self.attached_object_id
        if attached_object_id is not None:
            raise ObjectManipulationError(
                f"No object should be attached when transitioning from {source.name} to {target.name}, found {attached_object_id} attached"
            )

    def validate_object_attached(self, *, source: State, target: State):
        attached_object_id = self.attached_object_id
        if attached_object_id is None:
            raise ObjectManipulationError(
                f"One object should be attached when transitioning from {source.name} to {target.name}, found no attached objects"
            )

    @object_manipulation_decorator
    async def fetch_object(
        self, object_id: str, *, cache_trajectories: bool = True
    ) -> None:
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

        cache_kwargs: list[TrajectoryCacheKwargs] = []

        while self._sm.current_state != self._sm.FETCHED:
            kwargs = await self._sm.fetch_step(
                object_id, cache_trajectories=cache_trajectories
            )
            if cache_trajectories and kwargs is not None:
                cache_kwargs.extend(kwargs)

        if cache_trajectories and len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    @object_manipulation_decorator
    async def present_object(self):
        """Present the currently attached object at the specified end goal.

        Args:
            goal: The goal to present the object at
        """
        object_id = self.attached_object_id
        if object_id is None:
            raise ObjectManipulationError("No attached object to present")

        self.log(f"Pre-presenting object {object_id}")

        await self._sm.plan_and_execute(goal="present")

    @object_manipulation_decorator
    async def reset_object(
        self, object_id: str, *, cache_trajectories: bool = True
    ):
        """Unpresent the currently attached object and perform the reset procedure"""
        cache_kwargs: list[TrajectoryCacheKwargs] = []

        while self._sm.current_state != self._sm.RESETTED:
            kwargs = await self._sm.return_step(
                object_id, cache_trajectories=cache_trajectories
            )
            if cache_trajectories and kwargs is not None:
                cache_kwargs.extend(kwargs)

        if cache_trajectories and len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    @object_manipulation_decorator
    async def return_object(
        self, object_id: str, *, cache_trajectories: bool = True
    ):
        """Return an object to its original position.

        Args:
            cache_trajectories: Whether to cache the trajectories after
                returning the object

        Raises:
            RuntimeError: If exactly one object is not attached
            PlanningError: If the planning fails
            ExecutionError: If the execution fails
        """
        self.log(f"Returning object {object_id}")

        cache_kwargs: list[TrajectoryCacheKwargs] = []

        while self._sm.current_state != self._sm.POST_FETCH:
            kwargs = await self._sm.return_step(
                object_id, cache_trajectories=cache_trajectories
            )
            if cache_trajectories and kwargs is not None:
                cache_kwargs.extend(kwargs)

        if cache_trajectories and len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    @object_manipulation_decorator
    async def plan_and_execute(
        self,
        request: PlanRequest | ConcatPlanRequest | None = None,
        cache_trajectory: bool = True,
        **kwargs: Any,
    ) -> list[TrajectoryCacheKwargs] | None:
        return await self._plan_and_execute(
            request, cache_trajectory, **kwargs
        )

    @object_manipulation_decorator
    async def execute(
        self, trajectory: RobotTrajectory | list[RobotTrajectory]
    ):
        return await self._execute(trajectory)

    ###########################################################################
    ########## Attach object manually #########################################
    ###########################################################################

    def add_manually_attached_object(self, object_id: str):
        """Add a manually attached collision object to the planning scene."""
        self.log(f"Adding manually attached object: {object_id}")

        if self._manipulation_lock.locked():
            raise ObjectManipulationError(
                "Attempted to manually attach object while another manipulation is in progress"
            )
        if self.attached_object_id is not None:
            raise ObjectManipulationError(
                "Attempted to manually attach object while another object is already attached"
            )

        # Get mesh path
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
        assert object_id not in self.grid_objects

        self._manipulation_state = ManipulationState.MANUALLY_ATTACHED

    ###########################################################################
    ########## Reset ##########################################################
    ###########################################################################

    async def reset_rig(self, end_goal: Optional[PlanGoalT] = None):
        """Move the robot out of collision if necessary and return any attached
        objects to their original positions.
        """
        self.log("Resetting rig")
        try:
            object_id = self.attached_object_id
            if object_id is not None and object_id in self.grid_objects:
                await self.reset_object(object_id)
                await self.return_object(object_id)
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
                self._init_object_manipulation_state()
            else:
                raise
