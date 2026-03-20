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
from enum import IntEnum
from glob import glob
from typing import Any, Optional

import yaml
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from rclpy.exceptions import ParameterNotDeclaredException

from tabletop_py.utils.common import KwargYamlLoader
from tabletop_rig.exceptions import (
    ExecutionError,
    ExecutionInterruptedError,
    ExecutionRejectedError,
    MoveitRecoverableError,
    ObjectManipulationError,
    ObjectMismatchError,
    StateTransitionError,
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


class State(IntEnum):
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
    PRE_RETURN = 9
    PRE_DETACH = 10
    DETACH = 11
    POST_DETACH = 12
    POST_RETURN = 13
    MANUALLY_ATTACHED = 99


FETCH_OR_RETURN_STATES = set(
    (
        State.IDLE,
        State.PRE_FETCH,
        State.PRE_ATTACH,
        State.ATTACH,
        State.POST_ATTACH,
        State.POST_FETCH,
        State.FETCHED,
        State.PRE_RETURN,
        State.PRE_DETACH,
        State.DETACH,
        State.POST_DETACH,
        State.POST_RETURN,
    )
)

OBJECT_DETACHED_STATES = set(
    (
        State.IDLE,
        State.PRE_FETCH,
        State.PRE_ATTACH,
        State.DETACH,
        State.POST_DETACH,
        State.POST_RETURN,
    )
)
GRID_OBJECT_ATTACHED_STATES = set(
    (
        State.ATTACH,
        State.POST_ATTACH,
        State.POST_FETCH,
        State.FETCHED,
        State.NEEDS_RESET,
        State.RESETTED,
        State.PRE_RETURN,
        State.PRE_DETACH,
        # State.MANUALLY_ATTACHED,
    )
)

ALLOWED_MOUNT_COLLISION_STATES = set(
    (
        State.PRE_ATTACH,
        State.ATTACH,
        State.POST_ATTACH,
        State.POST_FETCH,
        State.PRE_DETACH,
        State.DETACH,
        State.POST_DETACH,
        State.POST_RETURN,
    )
)

# Mapping of manipulation states to their corresponding pose offsets
STATE_OFFSET_MAP = {
    State.PRE_FETCH: "pre_fetch",
    State.PRE_ATTACH: "pre_attach",
    State.ATTACH: "attach",
    State.POST_ATTACH: "post_attach",
    State.POST_FETCH: "post_fetch",
    State.PRE_RETURN: "post_fetch",
    State.PRE_DETACH: "post_attach",
    State.DETACH: "attach",
    State.POST_DETACH: "pre_attach",
    State.POST_RETURN: "pre_fetch",
}


def manipulation_lock_and_validate(coro_fn):
    """Decorator for methods that should be run with the object manipulation lock."""

    @functools.wraps(coro_fn)
    async def wrapper(self: "ObjectManipulationInterface", *args, **kwargs):
        if self._manipulation_lock.locked():
            raise ObjectManipulationError(
                f"Cannot call {coro_fn.__name__} while another operation is in progress"
            )
        async with self._manipulation_lock:
            self._validate_attached_object_state()
            try:
                return await coro_fn(self, *args, **kwargs)
            finally:
                self._validate_attached_object_state()

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
    _manipulation_state: State
    _current_manipulation_id: str | None
    _manipulation_lock: asyncio.Lock
    _reset_configs: dict[str, ObjectResetConfig]

    def __init__(
        self,
        node: BaseNode,
        safe_to_execute_callback: Callable[[], bool],
    ):
        """Initializes the MoveItObjectInterface"""
        super().__init__(node, safe_to_execute_callback)

        self._init_reset_configs()

        self._current_manipulation_id = self._init_attached_object()

        if self._current_manipulation_id is not None:
            self._manipulation_state = State.NEEDS_RESET
        else:
            self._manipulation_state = State.IDLE

        self._manipulation_lock = asyncio.Lock()

        self.log("MoveIt object manipulation interface initialized")

    def _init_reset_configs(self):
        # Check that all grid objects have reset configurations
        grid_objects = set(self.grid_objects_by_id.keys())
        reset_objects = set(self.reset_config_map.keys())
        missing = grid_objects - reset_objects
        if len(missing) > 0:
            raise ValueError(
                "All grid objects must have associated reset "
                "configs in object_manipulation.reset_configs"
            )

        # Parse and save the configurations for each unique file
        self._reset_configs = {}

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

            self._reset_configs[filename] = config

    def _init_attached_object(self) -> str | None:
        """Initialize the attached object."""
        object_id = None
        idx = None

        try:
            object_id = self.node.param("initial_attached_object_id")
        except ParameterNotDeclaredException:
            pass

        try:
            idx = self.node.param("initial_attached_object_idx")
        except ParameterNotDeclaredException:
            pass

        if object_id is not None:
            if idx is not None:
                raise ValueError(
                    "Cannot specify both initial_attached_object_id and initial_attached_object_idx"
                )
            if object_id not in self.grid_objects_by_id:
                raise ValueError(
                    f"initial_attached_object_id parameter ({object_id}) not an existing grid object"
                )
        elif idx is not None:
            x, y = idx
            if (x, y) not in self.grid_objects_by_idx:
                raise ValueError(
                    f"initial_attached_object_idx parameter {object_id} not found in existing grid objects"
                )
            object_id = self.grid_objects_by_idx[(x, y)].object_id
        else:
            self.log("No initial attached object specified")
            return None

        assert isinstance(object_id, str)
        assert object_id in self.collision_object_ids

        self.move_collision_object(
            object_id, self.get_link_pose_stamped(self.default_pose_link)
        )
        self.attach_collision_object(
            object_id, self.default_pose_link, touch_links=self.touch_links
        )

        return object_id

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
        old_pose_stamped = self.grid_objects_by_id[object_id].pose_stamped
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

    def _validate_attached_object_state(self):
        assert (self._manipulation_state == State.IDLE) == (
            self._current_manipulation_id is None
        ), (
            f"_current_manipulation_id should be None (got "
            f"{self._current_manipulation_id}) iff _manipulation_state is IDLE "
            f"(got {self._manipulation_state.name})"
        )

        attached_object_id = self.attached_object_id
        if attached_object_id is None:
            assert self._manipulation_state in OBJECT_DETACHED_STATES, (
                f"If object is not attached, we can only be in "
                f"{(x.name for x in OBJECT_DETACHED_STATES)} states, "
                f"got: {self._manipulation_state.name}"
            )
        else:
            assert attached_object_id == self._current_manipulation_id

            if attached_object_id in self.grid_objects_by_id:
                assert (
                    self._manipulation_state in GRID_OBJECT_ATTACHED_STATES
                ), (
                    f"If grid object is attached, we can only be in "
                    f"{(x.name for x in GRID_OBJECT_ATTACHED_STATES)} states, "
                    f"got: {self._manipulation_state.name}"
                )
            else:
                assert self._manipulation_state == State.MANUALLY_ATTACHED, (
                    f"If non-grid object is attached, we can only be in "
                    f"MANUALLY_ATTACHED state, got: {self._manipulation_state.name}"
                )

    def _validate_target_object(self, object_id: str, *, is_grid_object: bool):
        if is_grid_object and object_id not in self.grid_objects_by_id:
            raise ValueError(
                f"'{object_id}' is not a valid collision object in the object grid"
            )

        if (
            self._current_manipulation_id is not None
            and object_id != self._current_manipulation_id
        ):
            raise ObjectMismatchError(
                f"Cannot manipulate object {object_id} because object "
                f"{self._current_manipulation_id} is already being manipulated"
            )

        # attached_object_id = self.attached_object_id
        # if attached_object_id is not None and object_id != attached_object_id:
        #     raise ObjectMismatchError(
        #         f"Cannot manipulate object {object_id} because object {attached_object_id} is already attached"
        #     )

    ###########################################################################
    ########## Object Manipulation State Transition Logic #####################
    ###########################################################################

    def _get_state_goal(
        self,
        state: State,
        object_id: str,
    ) -> PlanGoalT:
        """Get the goal for the given state and object.

        Args:
            state: The state to get the goal for
            object_id: The ID of the object to get the goal for
            goal: The goal to use if the state is present or unpresent

        Returns:
            The goal for the given state and object.
        """
        match state:
            case State.IDLE:
                return "idle"
            case State.FETCHED:
                return "fetched"  # TODO
            case _:
                offset = self.node.param(
                    f"object_manipulation.state_offsets.{STATE_OFFSET_MAP[state]}"
                )
                return self._grid_object_pose_stamped_with_offset(
                    object_id, offset
                )

    async def _fetch_or_return_transition(
        self,
        object_id: str,
        next_state: State,
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
        self.log(
            f"Transition from {self._manipulation_state.name} to {next_state.name} state for object {object_id}"
        )

        assert (
            self._manipulation_state == State.RESETTED
            or self._manipulation_state in FETCH_OR_RETURN_STATES
        )
        assert next_state in FETCH_OR_RETURN_STATES

        goal = self._get_state_goal(next_state, object_id)
        request = PlanRequest(goal=goal)

        match next_state:
            case (
                State.PRE_ATTACH
                | State.ATTACH
                | State.POST_ATTACH
                | State.DETACH
                | State.POST_DETACH
                | State.POST_RETURN
            ):
                request.planning_pipeline = "linear"
                request.use_cache = False

        if next_state == State.DETACH:
            PlanRequest.velocity_scaling_factor = self.node.param(
                "object_manipulation.detach_velocity_scaling_factor"
            )

        # try:
        #     if next_state in ALLOWED_MOUNT_COLLISION_STATES:
        #         allowed_collisions.extend(
        #             self.allow_collision(*zip(*self.allowed_mount_collisions))
        #         )
        #
        #     match next_state:
        #         case (
        #             State.PRE_ATTACH
        #             | State.ATTACH
        #             | State.POST_DETACH
        #             | State.POST_RETURN
        #         ):
        #             allowed_collisions.extend(
        #                 self.allow_collision(object_id, self.touch_links)
        #             )
        #         case State.POST_ATTACH | State.DETACH:
        #             allowed_collisions.extend(
        #                 self.allow_collision(object_id, self.mount_ids)
        #             )

        collisions_to_allow: list[tuple[str, str]] = []
        modified_collisions: list[tuple[str, str]] = []

        if next_state in ALLOWED_MOUNT_COLLISION_STATES:
            collisions_to_allow.extend(
                self.allow_collision(*zip(*self.allowed_mount_collisions))
            )
        match next_state:
            case (
                State.PRE_ATTACH
                | State.ATTACH
                | State.POST_DETACH
                | State.POST_RETURN
            ):
                collisions_to_allow.extend(
                    [(object_id, x) for x in self.touch_links]
                )
            case State.POST_ATTACH | State.DETACH:
                collisions_to_allow.extend(
                    [(object_id, x) for x in self.mount_ids]
                )

        if len(collisions_to_allow) > 0:
            modified_collisions = self.allow_collision(
                *zip(*collisions_to_allow)
            )
        try:
            cache_kwargs = await self._plan_and_execute(
                request, cache_trajectory=False
            )
        finally:
            if len(modified_collisions) > 0:
                self.disallow_collision(*zip(*modified_collisions))

        match next_state:
            case State.ATTACH:
                self.attach_collision_object(
                    object_id,
                    self.default_pose_link,
                    touch_links=self.touch_links,
                )
            case State.DETACH:
                self.detach_collision_object(object_id)

        return cache_kwargs

    @manipulation_lock_and_validate
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

        self._validate_target_object(object_id, is_grid_object=True)

        match self._manipulation_state:
            case (
                State.IDLE
                | State.PRE_FETCH
                | State.PRE_ATTACH
                | State.ATTACH
                | State.POST_ATTACH
                | State.POST_FETCH
            ):
                next_state = State(self._manipulation_state + 1)
            case State.FETCHED:
                self.log(
                    "Already at FETCHED state, skipping fetch", severity="WARN"
                )
                return
            case State.RESETTED:
                next_state = State.FETCHED
            case (
                State.PRE_RETURN
                | State.PRE_DETACH
                | State.DETACH
                | State.POST_DETACH
                | State.POST_RETURN
            ):
                return_progress = self._manipulation_state - State.PRE_RETURN
                next_state = State(State.POST_FETCH - return_progress)
            case unexpected if isinstance(unexpected, State):
                raise StateTransitionError(
                    f"Cannot fetch object from current state ({unexpected})"
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        assert State.PRE_FETCH <= next_state and next_state <= State.FETCHED

        cache_kwargs: list[TrajectoryCacheKwargs] = []

        # Iterate through the fetch states
        while self._manipulation_state != State.FETCHED:
            kwargs = await self._fetch_or_return_transition(
                object_id, next_state
            )

            self._manipulation_state = next_state
            next_state = State(self._manipulation_state + 1)

            if self._manipulation_state != State.IDLE:
                self._current_manipulation_id = object_id

            self._validate_attached_object_state()
            # self._validate_target_object(object_id, is_grid_object=True)

            if cache_trajectories and kwargs is not None:
                cache_kwargs.extend(kwargs)

        # Cache all trajectories if requested
        if cache_trajectories and len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    @manipulation_lock_and_validate
    async def present_object(self, object_id: str, *, cache_trajectory=True):
        """Present the currently attached object at the specified end goal.

        Args:
            goal: The goal to present the object at
        """
        self.log(f"Presenting object {object_id}")

        self._validate_target_object(object_id, is_grid_object=True)

        match self._manipulation_state:
            case State.FETCHED | State.RESETTED:
                pass
            case unexpected if isinstance(unexpected, State):
                raise StateTransitionError(
                    f"Cannot present object from current state ({unexpected})"
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        try:
            await self._plan_and_execute(
                goal="present", cache_trajectory=cache_trajectory
            )
        except (asyncio.CancelledError, ExecutionError):
            self._manipulation_state = State.NEEDS_RESET
            raise
        else:
            self._manipulation_state = State.NEEDS_RESET

    @manipulation_lock_and_validate
    async def reset_object(
        self, object_id: str, *, cache_trajectories: bool = True
    ):
        """Unpresent the currently attached object and perform the reset procedure"""
        self.log(f"Resetting object {object_id}")

        self._validate_target_object(object_id, is_grid_object=True)

        match self._manipulation_state:
            case State.NEEDS_RESET:
                pass
            # case State.POST_FETCH | State.FETCHED | State.RESETTED:
            #     self.log(
            #         f"No need to reset object {object_id} from current state ({self._manipulation_state}), skipping",
            #         severity="WARN",
            #     )
            #     return
            case unexpected if isinstance(unexpected, State):
                raise ObjectManipulationError(
                    f"Cannot reset object from current state ({unexpected})"
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        # Retrieve reset config
        filename = self.reset_config_map[object_id]
        config = self._reset_configs[filename]
        assert config.reset_request.planning_pipeline == "linear"

        cache_kwargs: list[TrajectoryCacheKwargs] = []

        # Plan and execute to start goal
        kwargs = await self._plan_and_execute(
            goal=config.start_goal, cache_trajectory=False
        )
        if cache_trajectories and kwargs is not None:
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
            if cache_trajectories and kwargs is not None:
                cache_kwargs.extend(kwargs)
        finally:
            if len(allowed_collisions) > 0:
                self.disallow_collision(*zip(*allowed_collisions))

        self._manipulation_state = State.RESETTED

        # Cache all trajectories if requested
        if len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    @manipulation_lock_and_validate
    async def return_object(
        self, object_id: str, *, cache_trajectories: bool = True
    ) -> None:
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

        self._validate_target_object(object_id, is_grid_object=True)

        match self._manipulation_state:
            case State.IDLE:
                self.log(
                    "Already at IDLE state, skipping return", severity="WARN"
                )
                return
            case (
                State.PRE_FETCH
                | State.PRE_ATTACH
                | State.ATTACH
                | State.POST_ATTACH
                | State.POST_FETCH
            ):
                fetch_progress = self._manipulation_state - State.PRE_FETCH
                next_state = State(State.POST_RETURN - fetch_progress)
            case State.FETCHED | State.RESETTED:
                next_state = State.PRE_RETURN
            case (
                State.PRE_RETURN
                | State.PRE_DETACH
                | State.DETACH
                | State.POST_DETACH
                | State.POST_RETURN
            ):
                next_state = State(
                    (self._manipulation_state + 1) % (State.POST_RETURN + 1)
                )
            case unexpected if isinstance(unexpected, State):
                raise ObjectManipulationError(
                    f"Cannot fetch object from current state ({unexpected})"
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        assert next_state == State.IDLE or (
            State.PRE_RETURN <= next_state and next_state <= State.POST_RETURN
        )

        cache_kwargs: list[TrajectoryCacheKwargs] = []

        # Iterate through the fetch states
        while self._manipulation_state != State.IDLE:
            kwargs = await self._fetch_or_return_transition(
                object_id, next_state
            )

            self._manipulation_state = next_state
            next_state = State(
                (self._manipulation_state + 1) % (State.POST_RETURN + 1)
            )

            if self._manipulation_state == State.IDLE:
                self._current_manipulation_id = None

            self._validate_attached_object_state()
            # self._validate_target_object(object_id, is_grid_object=True)

            if cache_trajectories and kwargs is not None:
                cache_kwargs.extend(kwargs)

        # Cache all trajectories if requested
        if cache_trajectories and len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    @manipulation_lock_and_validate
    async def plan_and_execute(
        self,
        request: PlanRequest | ConcatPlanRequest | None = None,
        cache_trajectory: bool = True,
        **kwargs: Any,
    ) -> list[TrajectoryCacheKwargs] | None:
        """TODO"""

        match self._manipulation_state:
            case State.IDLE | State.MANUALLY_ATTACHED:
                needs_reset = False
            case State.FETCHED | State.RESETTED | State.NEEDS_RESET:
                needs_reset = True
            case unexpected if isinstance(unexpected, State):
                raise StateTransitionError(
                    f"Cannot plan_and_execute from current state ({unexpected})"
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        try:
            result = await self._plan_and_execute(
                request, cache_trajectory, **kwargs
            )
        except (asyncio.CancelledError, ExecutionError):
            if needs_reset:
                self._manipulation_state = State.NEEDS_RESET
            raise
        else:
            if needs_reset:
                self._manipulation_state = State.NEEDS_RESET

        return result

    @manipulation_lock_and_validate
    async def execute(
        self, trajectory: RobotTrajectory | list[RobotTrajectory]
    ):
        """TODO"""

        match self._manipulation_state:
            case State.IDLE | State.MANUALLY_ATTACHED:
                needs_reset = False
            case State.FETCHED | State.RESETTED | State.NEEDS_RESET:
                needs_reset = True
            case unexpected if isinstance(unexpected, State):
                raise StateTransitionError(
                    f"Cannot execute from current state ({unexpected})"
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        try:
            await self._execute(trajectory)
        except (asyncio.CancelledError, ExecutionError):
            if needs_reset:
                self._manipulation_state = State.NEEDS_RESET
            raise
        else:
            if needs_reset:
                self._manipulation_state = State.NEEDS_RESET

    ###########################################################################
    ########## Attach object manually #########################################
    ###########################################################################

    @manipulation_lock_and_validate
    async def add_manually_attached_object(self, object_id: str):
        """Add a manually attached collision object to the planning scene."""
        self.log(f"Adding manually attached object: {object_id}")

        if object_id in self.grid_objects_by_id:
            raise NotImplementedError(
                "Need to implement manually attached grid objects (maybe)"
            )

        self._validate_target_object(object_id, is_grid_object=False)

        match self._manipulation_state:
            case State.IDLE:
                pass
            case unexpected if isinstance(unexpected, State):
                raise StateTransitionError(
                    f"Cannot manually attach object from current state ({unexpected})"
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
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
            object_id,
            path=mesh_path,
            pose_stamped=self.get_link_pose_stamped(self.default_pose_link),
            **self.node.param("manually_attached_object_kwargs"),
        )
        self.attach_collision_object(
            object_id, self.default_pose_link, touch_links=self.touch_links
        )

        self._current_manipulation_id = object_id
        self._manipulation_state = State.MANUALLY_ATTACHED

    @manipulation_lock_and_validate
    async def remove_manually_attached_object(self, object_id: str):
        """Add a manually attached collision object to the planning scene."""
        self.log(f"Adding manually attached object: {object_id}")

        if object_id in self.grid_objects_by_id:
            raise NotImplementedError(
                "Need to implement manually attached grid objects (maybe)"
            )

        self._validate_target_object(object_id, is_grid_object=False)

        match self._manipulation_state:
            case State.MANUALLY_ATTACHED:
                pass
            case unexpected if isinstance(unexpected, State):
                raise StateTransitionError(
                    f"Cannot manually detach object from current state ({unexpected})"
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        self.detach_collision_object(object_id)
        self.remove_collision_object(object_id)

        self._current_manipulation_id = None
        self._manipulation_state = State.IDLE

    ###########################################################################
    ########## Reset ##########################################################
    ###########################################################################

    @manipulation_lock_and_validate
    async def reset_rig(self, end_goal: Optional[PlanGoalT] = None):
        """Move the robot out of collision if necessary and return any attached
        objects to their original positions.
        """
        self.log("Resetting rig")

        try:
            if self._manipulation_state not in (
                State.IDLE,
                State.MANUALLY_ATTACHED,
            ):
                assert self._current_manipulation_id is not None
                if self._manipulation_state == State.DETACH:
                    try:
                        await self._fetch_or_return_transition(
                            self._current_manipulation_id, State.ATTACH
                        )
                        self._manipulation_state = State.ATTACH
                        await self._fetch_or_return_transition(
                            self._current_manipulation_id, State.POST_ATTACH
                        )
                        self._manipulation_state = State.POST_ATTACH
                    except (
                        ExecutionRejectedError,
                        ExecutionInterruptedError,
                    ) as e:
                        if self.node.param("simulate"):
                            raise
                        else:
                            raise RuntimeError(
                                "Object seems stuck, aborting"
                            ) from e
                elif self._manipulation_state == State.NEEDS_RESET:
                    await ObjectManipulationInterface.reset_object.__wrapped__(  # pyright: ignore[reportFunctionMemberAccess]
                        self,
                        self._current_manipulation_id,
                        cache_trajectories=False,
                    )

                await ObjectManipulationInterface.return_object.__wrapped__(  # pyright: ignore[reportFunctionMemberAccess]
                    self,
                    self._current_manipulation_id,
                    cache_trajectories=False,
                )
            if end_goal is not None:
                await ObjectManipulationInterface.plan_and_execute.__wrapped__(  # pyright: ignore[reportFunctionMemberAccess]
                    self, goal=end_goal, cache_trajectory=False
                )
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
                self._manipulation_state = State.IDLE
                self._current_manipulation_id = None
            else:
                raise
