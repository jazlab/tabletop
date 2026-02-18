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
from collections.abc import Callable, Coroutine
from enum import IntEnum
from glob import glob
from typing import Any, Literal, Optional

import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.exceptions import ParameterNotDeclaredException

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
    RESET = 7
    PRE_RETURN = 8
    PRE_DETACH = 9
    DETACH = 10
    POST_DETACH = 11
    POST_RETURN = 12
    MANUALLY_ATTACHED = 99


ALL_STATES = set(
    (
        State.IDLE,
        State.PRE_FETCH,
        State.PRE_ATTACH,
        State.ATTACH,
        State.POST_ATTACH,
        State.POST_FETCH,
        State.FETCHED,
        State.RESET,
        State.PRE_RETURN,
        State.PRE_DETACH,
        State.DETACH,
        State.POST_DETACH,
        State.POST_RETURN,
        State.MANUALLY_ATTACHED,
    )
)

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
OBJECT_ATTACHED_STATES = set(
    (
        State.ATTACH,
        State.POST_ATTACH,
        State.POST_FETCH,
        State.FETCHED,
        State.RESET,
        State.PRE_RETURN,
        State.PRE_DETACH,
        State.MANUALLY_ATTACHED,
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


def dummy_match(state):
    match state:
        case (
            State.IDLE
            | State.PRE_FETCH
            | State.PRE_ATTACH
            | State.ATTACH
            | State.POST_ATTACH
            | State.POST_FETCH
            | State.FETCHED
            | State.RESET
            | State.PRE_RETURN
            | State.PRE_DETACH
            | State.DETACH
            | State.POST_DETACH
            | State.POST_RETURN
            | State.MANUALLY_ATTACHED
        ):
            pass


def check_state(cur_state, next_state):
    match cur_state:
        case State.IDLE:
            assert next_state in (
                State.PRE_FETCH,
                State.MANUALLY_ATTACHED,
            )
        case (
            State.PRE_FETCH
            | State.PRE_ATTACH
            | State.ATTACH
            | State.POST_ATTACH
            | State.POST_FETCH
            | State.FETCHED
            | State.RESET
        ):
            pass
        case (
            State.IDLE
            | State.PRE_FETCH
            | State.PRE_ATTACH
            | State.ATTACH
            | State.POST_ATTACH
            | State.POST_FETCH
            | State.FETCHED
            | State.RESET
            | State.PRE_RETURN
            | State.PRE_DETACH
            | State.DETACH
            | State.POST_DETACH
            | State.POST_RETURN
            | State.MANUALLY_ATTACHED
        ):
            pass


def object_manipulation_decorator(
    coro_fn: Callable[..., Coroutine],
) -> Callable[..., Coroutine]:
    """Decorator for methods that should be run with the object manipulation lock."""

    @functools.wraps(coro_fn)
    async def wrapper(self: "ObjectManipulationInterface", *args, **kwargs):
        if self._manipulation_lock.locked():
            raise ObjectManipulationError(
                "Attempted to manipulate object while another manipulation is in progress"
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

        self._reset_object_manipulation_state()

        self.log("MoveIt object manipulation interface initialized")

    def _reset_object_manipulation_state(self):
        self._manipulation_lock = asyncio.Lock()
        self._manipulation_state = State.IDLE

        self._object_reset: dict[str, bool] = {}
        for object_id in self.grid_objects.keys():
            self._object_reset[object_id] = True

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

    @object_manipulation_decorator
    async def plan_and_execute(  # type: ignore  # TODO
        self,
        request: PlanRequest | ConcatPlanRequest | None = None,
        cache_trajectory: bool = True,
        **kwargs: Any,
    ) -> list[TrajectoryCacheKwargs] | None:
        if (
            self.attached_object_id is not None
            and self._manipulation_state
            not in (
                State.FETCHED,
                State.RESET,
                State.MANUALLY_ATTACHED,
            )
        ):
            raise ObjectManipulationError(
                f"Cannot plan_and_execute with attached object in any other state than FETCHED, RESET, and MANUALLY_ATTACHED, got {self._manipulation_state}"
            )

        try:
            return await super().plan_and_execute(
                request, cache_trajectory, **kwargs
            )
        finally:
            if self._manipulation_state == State.RESET:
                self._manipulation_state = State.FETCHED

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
            self._manipulation_state == State.RESET
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

        allowed_collisions: list[tuple[str, str]] = []
        try:
            if next_state in ALLOWED_MOUNT_COLLISION_STATES:
                allowed_collisions.extend(
                    self.allow_collision(*zip(*self.allowed_mount_collisions))
                )

            match next_state:
                case (
                    State.PRE_ATTACH
                    | State.ATTACH
                    | State.POST_DETACH
                    | State.POST_RETURN
                ):
                    allowed_collisions.extend(
                        self.allow_collision(object_id, self.touch_links)
                    )
                case State.POST_ATTACH | State.DETACH:
                    allowed_collisions.extend(
                        self.allow_collision(object_id, self.mount_ids)
                    )

            cache_kwargs = await super().plan_and_execute(
                request, cache_trajectory=False
            )
        finally:
            if len(allowed_collisions) > 0:
                self.disallow_collision(*zip(*allowed_collisions))

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

    async def _reset_transition(
        self, object_id: str, next_state: State
    ) -> list[TrajectoryCacheKwargs] | None:
        assert self._manipulation_state == State.FETCHED
        assert next_state == State.RESET

        if self._object_reset[object_id]:
            self.log(f"No need to reset object {object_id}, skipping")
            return
        else:
            self.log(f"Resetting object {object_id}")

        # Retrieve reset config
        filename = self.reset_config_map[object_id]
        config = self.reset_configs[filename]
        assert config.reset_request.planning_pipeline == "linear"
        cache_kwargs: list[TrajectoryCacheKwargs] = []

        # Plan and execute to start goal
        kwargs = await super().plan_and_execute(
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

            kwargs = await super().plan_and_execute(
                config.reset_request, cache_trajectory=False
            )
            if kwargs is not None:
                cache_kwargs.extend(kwargs)
        finally:
            if len(allowed_collisions) > 0:
                self.disallow_collision(*zip(*allowed_collisions))

        self._object_reset[object_id] = True

        # Cache all trajectories if requested
        if len(cache_kwargs) > 0:
            return cache_kwargs

    def _check_attached_object_state(self, object_id: str):
        attached_object_id = self.attached_object_id
        if attached_object_id is None:
            if self._manipulation_state not in OBJECT_DETACHED_STATES:
                raise AssertionError(
                    f"If object is not attached, we can only be in "
                    f"{(x.name for x in OBJECT_DETACHED_STATES)} states, "
                    f"got: {self._manipulation_state.name}"
                )
        else:
            if object_id != attached_object_id:
                raise ObjectManipulationError(
                    f"Cannot manipulate object {object_id} because object {attached_object_id} is already attached"
                )
            if self._manipulation_state not in OBJECT_ATTACHED_STATES:
                raise AssertionError(
                    f"If object is attached, we can only be in "
                    f"{(x.name for x in OBJECT_ATTACHED_STATES)} states, "
                    f"got: {self._manipulation_state.name}"
                )

    async def _transition_to_state(
        self,
        object_id: str,
        final_state: State,
        *,
        cache_trajectories: bool = True,
    ):
        """TODO"""
        if final_state not in (
            State.FETCHED,
            State.IDLE,
            State.RESET,
        ):
            raise ValueError(
                "final_state can only be one of FETCHED, RESET, or IDLE"
            )

        cache_kwargs: list[TrajectoryCacheKwargs] = []

        while self._manipulation_state != final_state:
            self._check_attached_object_state(object_id)

            handler: (
                Literal["fetch_or_return", "reset", "pass", "impossible"]
                | None
            ) = None
            next_state: State | None = None

            match self._manipulation_state:
                case (
                    State.IDLE
                    | State.PRE_FETCH
                    | State.PRE_ATTACH
                    | State.ATTACH
                    | State.POST_ATTACH
                    | State.POST_FETCH
                ):
                    match final_state:
                        case State.FETCHED:
                            next_state = State(self._manipulation_state + 1)
                            handler = "fetch_or_return"
                        case State.IDLE:
                            assert self._manipulation_state != State.IDLE
                            fetch_progress = (
                                self._manipulation_state - State.PRE_FETCH
                            )
                            next_state = State(
                                State.POST_RETURN - fetch_progress - 1
                            )
                            handler = "pass"
                        case State.RESET:
                            handler = "impossible"

                case State.FETCHED:
                    match final_state:
                        case State.FETCHED:
                            raise AssertionError
                        case State.IDLE:
                            handler = "impossible"
                        case State.RESET:
                            next_state = State.RESET
                            handler = "reset"

                case State.RESET:
                    match final_state:
                        case State.FETCHED:
                            next_state = State.FETCHED
                            handler = "fetch_or_return"
                        case State.IDLE:
                            next_state = State.PRE_RETURN
                            handler = "fetch_or_return"
                        case State.RESET:
                            raise AssertionError

                case (
                    State.PRE_RETURN
                    | State.PRE_DETACH
                    | State.DETACH
                    | State.POST_DETACH
                    | State.POST_RETURN
                ):
                    match final_state:
                        case State.FETCHED:
                            return_progress = (
                                self._manipulation_state - State.PRE_RETURN
                            )
                            next_state = State(
                                State.POST_FETCH - return_progress - 1
                            )
                            handler = "pass"
                        case State.IDLE:
                            next_state = State(self._manipulation_state + 1)
                            handler = "fetch_or_return"
                        case State.RESET:
                            handler = "impossible"

                case State.MANUALLY_ATTACHED:
                    # TODO: Add handling for manually attached objects here
                    raise RuntimeError(
                        "Cannot perform object manipulation if there is a manually attached object"
                    )
                case _:
                    raise RuntimeError(
                        f"Unknown manipulator state {self._manipulation_state}"
                    )

            assert handler is not None
            if handler == "impossible":
                raise ObjectManipulationError(
                    f"Cannot reach final_state {final_state.name} from {self._manipulation_state.name}"
                )

            assert next_state is not None
            match handler:
                case "pass":
                    pass
                case "fetch_or_return":
                    kwargs = await self._fetch_or_return_transition(
                        object_id, next_state
                    )
                    if cache_trajectories and kwargs is not None:
                        cache_kwargs.extend(kwargs)
                case "reset":
                    kwargs = await self._reset_transition(
                        object_id, next_state
                    )
                    if cache_trajectories and kwargs is not None:
                        cache_kwargs.extend(kwargs)
                case _:
                    raise AssertionError(f"Unkown handler: {handler}")

            self._manipulation_state = next_state

        self._check_attached_object_state(object_id)

        # Cache all trajectories if requested
        if cache_trajectories and len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    @object_manipulation_decorator
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

        attached_object_id = self.attached_object_id
        if attached_object_id is not None:
            raise ObjectManipulationError(
                "Cannot fetch object while another object is attached"
            )

        # Check that the object ID is valid
        if object_id not in self.grid_objects:
            raise ValueError(
                f"'{object_id}' is not a valid collision object in the object grid"
            )

        # Iterate through the fetch states, returning the object to its mount
        # if the fetch fails
        cache_kwargs: list[TrajectoryCacheKwargs] = []
        for i in range(State.PRE_FETCH, State.FETCHED + 1):
            kwargs = await self._fetch_or_return_transition(
                object_id, State(i)
            )
            self._manipulation_state = State(i)
            if cache_trajectories and kwargs is not None:
                cache_kwargs.extend(kwargs)

        self._object_reset[object_id] = False
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

        # Pre-present state
        await super().plan_and_execute(goal="present")

        # Set object reset state to False
        self._object_reset[object_id] = False

    @object_manipulation_decorator
    async def reset_object(
        self, *, unpresent: bool = True, cache_trajectories: bool = True
    ):
        """Unpresent the currently attached object and perform the reset procedure"""
        object_id = self.attached_object_id
        if object_id is None:
            raise ObjectManipulationError("No attached object to reset")

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
            # await super().plan_and_execute(goal="present")
            pass
            # await self._transition_state(object_id, State.PRESENT)

        cache_kwargs: list[TrajectoryCacheKwargs] = []

        # Plan and execute to start goal
        kwargs = await super().plan_and_execute(
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

            kwargs = await super().plan_and_execute(
                config.reset_request, cache_trajectory=False
            )
            if cache_trajectories and kwargs is not None:
                cache_kwargs.extend(kwargs)
        finally:
            if len(allowed_collisions) > 0:
                self.disallow_collision(*zip(*allowed_collisions))

        self._object_reset[object_id] = True

        # Cache all trajectories if requested
        if len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    @object_manipulation_decorator
    async def return_object(self, *, cache_trajectories: bool = True) -> None:
        """Return an object to its original position.

        Args:
            cache_trajectories: Whether to cache the trajectories after
                returning the object

        Raises:
            RuntimeError: If exactly one object is not attached
            PlanningError: If the planning fails
            ExecutionError: If the execution fails
        """
        object_id = self.attached_object_id
        if object_id is None:
            raise ObjectManipulationError("No attached object to reset")
        if object_id not in self.grid_objects:
            raise RuntimeError(
                f"Object {object_id} is not in the grid, cannot return it"
            )

        if not self._object_reset[object_id]:
            raise ObjectManipulationError(
                f"Object {object_id} has not been reset yet, cannot return"
            )

        self.log(f"Returning object {object_id}")

        cache_kwargs: list[TrajectoryCacheKwargs] = []

        # Iterate through the unpresenting and returning states
        # for i in range(ObjectPhase.PRE_RETURN, ObjectPhase.IDLE + 1):
        for i in range(State.PRE_RETURN, State.POST_RETURN + 1):
            kwargs = await self._fetch_or_return_transition(
                object_id, State(i)
            )
            if kwargs is not None:
                cache_kwargs.extend(kwargs)

        # Cache all trajectories if requested
        if cache_trajectories and len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

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

        self._manipulation_state = State.MANUALLY_ATTACHED

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
                await self.reset_object(unpresent=False)
                await self.return_object()
            if end_goal is not None:
                await super().plan_and_execute(goal=end_goal)
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
                self._reset_object_manipulation_state()
            else:
                raise
