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
import inspect
import os
import traceback
from collections.abc import Callable
from enum import IntEnum
from glob import glob
from typing import Any, Optional

import numpy as np
import yaml
from geometry_msgs.msg import PoseStamped, WrenchStamped
from moveit.core.robot_state import (  # type: ignore[reportMissingModuleSource]
    RobotState,
)
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.exceptions import ParameterNotDeclaredException
from sensor_msgs.msg import JointState

from tabletop_py.utils.common import KwargYamlLoader
from tabletop_rig.exceptions import (
    ExecutionInterruptedError,
    ExecutionStoppedError,
    ObjectManipulationError,
    ObjectMismatchError,
    PlanningError,
    StateTransitionError,
)
from tabletop_rig.interfaces.moveit.moveit import MoveItInterface
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
    PRESENTED = 7
    NEEDS_RESET = 8
    PRE_RESET = 9
    RESETTED = 10
    PRE_RETURN = 11
    PRE_DETACH = 12
    DETACH = 13
    POST_DETACH = 14
    POST_RETURN = 15
    MANUALLY_ATTACHED = 98
    UNINITIALIZED = 99


_FETCH_OR_RETURN_STATES = set(
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

_OBJECT_DETACHED_STATES = set(
    (
        State.IDLE,
        State.PRE_FETCH,
        State.PRE_ATTACH,
        State.DETACH,
        State.POST_DETACH,
        State.POST_RETURN,
        State.UNINITIALIZED,
    )
)
_GRID_OBJECT_ATTACHED_STATES = set(
    (
        State.ATTACH,
        State.POST_ATTACH,
        State.POST_FETCH,
        State.FETCHED,
        State.PRESENTED,
        State.NEEDS_RESET,
        State.PRE_RESET,
        State.RESETTED,
        State.PRE_RETURN,
        State.PRE_DETACH,
    )
)

# MOUNT_COLLISIONS_ALLOWED_STATES = set(
#     (
#         State.PRE_ATTACH,
#         State.ATTACH,
#         State.POST_ATTACH,
#         # State.POST_FETCH,
#         # State.PRE_DETACH,
#         State.DETACH,
#         State.POST_DETACH,
#         State.POST_RETURN,
#     )
# )

# Mapping of manipulation states to their corresponding pose offsets
_STATE_GOAL_NAME_MAP = {
    State.IDLE: "idle",
    State.PRE_FETCH: "pre_fetch",
    State.PRE_ATTACH: "pre_attach",
    State.ATTACH: "attach",
    State.POST_ATTACH: "post_attach",
    State.POST_FETCH: "post_fetch",
    State.FETCHED: "fetched",
    State.PRESENTED: "present",
    State.NEEDS_RESET: "fetched",
    State.PRE_RETURN: "post_fetch",
    State.PRE_DETACH: "post_attach",
    State.DETACH: "attach",
    State.POST_DETACH: "pre_attach",
    State.POST_RETURN: "pre_fetch",
}


def validate_and_lock(coro_fn):
    if inspect.iscoroutinefunction(coro_fn):

        @functools.wraps(coro_fn)
        async def async_wrapper(
            self: "ObjectManipulationInterface", *args, **kwargs
        ):
            if self._manipulation_lock.locked():
                raise ObjectManipulationError(
                    "Robot cannot cannot acquire manipulation "
                    "lock while while another operation is in progress",
                    group_name=self.group_name,
                )

            async with self._manipulation_lock:
                self._validate_manipulation_state()
                try:
                    await coro_fn(self, *args, **kwargs)
                finally:
                    self._validate_manipulation_state()

        return async_wrapper


class ResetLoader(KwargYamlLoader):
    def get_kwarg_constructors(self) -> dict[str, Callable]:
        return {
            "!PoseStamped": pose_stamped_msg,
            "!ConcatPlanRequest": ConcatPlanRequest,
            "!ObjectResetConfig": ObjectResetConfig,
        }


class ObjectManipulationInterface(PlanAndExecuteInterface):
    """TODO"""

    _manipulation_state: State
    _current_manipulation_id: str | None
    _manipulation_lock: asyncio.Lock
    _reachable_object_ids: set[str]
    _reset_configs: dict[str, ObjectResetConfig]
    _post_fetch_states: dict[str, RobotState]

    def __init__(
        self,
        node: BaseNode,
        name: str,
        *,
        moveit_interface: MoveItInterface,
        safe_to_execute_condition: Callable[[], bool],
        simulate: bool,
        parameter_fallback_prefix: Optional[str] = None,
    ):
        """Initializes the ObjectInterface"""
        super().__init__(
            node,
            name,
            moveit_interface=moveit_interface,
            safe_to_execute_condition=safe_to_execute_condition,
            parameter_fallback_prefix=parameter_fallback_prefix,
        )

        self._init_reachable_indices()

        self._init_reset_configs()

        self._manipulation_state = State.UNINITIALIZED
        self._current_manipulation_id = None
        self._manipulation_lock = asyncio.Lock()

        self._simulate = simulate

        self._post_fetch_states = {}

        self.log("MoveIt object manipulation interface initialized")

    def _init_reachable_indices(self):
        self._reachable_object_ids = set()

        indices: list[str] = self.param("reachable_grid_indices")
        for idx in indices:
            x, y = map(int, idx.split(","))

            if (x, y) not in self._moveit.grid_objects_by_idx:
                raise ValueError(
                    f"Parameter reachable_grid_indices "
                    f"contains index {(x, y)}, which is not in the existing "
                    f"object grid"
                )

            object_id = self._moveit.grid_objects_by_idx[(x, y)].object_id
            self._reachable_object_ids.add(object_id)

    def _init_reset_configs(self):
        # Check that all grid objects have reset configurations

        reset_config_map: dict[str, str] = self.param("reset_config_map")
        reset_objects = set(reset_config_map.keys())

        missing = self.reachable_object_ids - reset_objects
        if len(missing) > 0:
            raise ValueError(
                f"All grid objects must have associated reset "
                f"configs in object_manipulation.reset_configs, "
                f"missing {missing}"
            )

        # Parse and save the configurations for each unique file
        self._reset_configs = {}

        for object_id in self.reachable_object_ids:
            filename = reset_config_map[object_id]

            if filename is None:
                continue

            if filename not in self._reset_configs:
                with open(filename) as f:
                    config = yaml.load(f, ResetLoader)

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
                            f"Error in reset config ({filename}): "
                            f"If 'object_allowed_collision_ids' or "
                            f"'additional_allowed_collisions' is provided, "
                            f"'reset_request.planning_pipeline' must be linear "
                            f"or not provided (for safe object resetting to "
                            f"prevent accidental collisions)"
                        )
                    config.reset_request.planning_pipeline = "linear"

                self._reset_configs[filename] = config

    def _init_attached_object(self) -> str | None:
        """Initialize the attached object."""
        object_id = None
        idx = None

        try:
            object_id = self.param("initial_attached_object_id")
        except ParameterNotDeclaredException:
            pass

        try:
            idx = self.param("initial_attached_object_idx")
        except ParameterNotDeclaredException:
            pass

        if object_id is not None:
            if idx is not None:
                raise ValueError(
                    "Cannot specify both "
                    "initial_attached_object_id and "
                    "initial_attached_object_idx"
                )
            if object_id not in self._moveit.grid_objects_by_id:
                raise ValueError(
                    f"initial_attached_object_id parameter "
                    f"({object_id}) not an existing grid object"
                )
        elif idx is not None:
            x, y = idx
            if (x, y) not in self._moveit.grid_objects_by_idx:
                raise ValueError(
                    f"initial_attached_object_idx parameter "
                    f"{object_id} not found in existing grid objects"
                )
            object_id = self._moveit.grid_objects_by_idx[(x, y)].object_id
        else:
            self.log("No initial attached object specified")
            return None

        assert isinstance(object_id, str)
        assert object_id in self._moveit.collision_object_ids

        self._moveit.move_collision_object(
            object_id, self._moveit.get_link_pose_stamped(self.attach_link)
        )
        self._moveit.attach_collision_object(
            object_id, self.attach_link, touch_links=self.touch_links
        )

        return object_id

    ###########################################################################
    ########## User-Accessible Read-Only Properties ###########################
    ###########################################################################

    @property
    def reachable_object_ids(self) -> set[str]:
        """Get the reachable objects for this robot"""
        return self._reachable_object_ids

    ###########################################################################
    ########## Parameter Convenience Properties and Methods ###################
    ###########################################################################

    @property
    def mount_collision_ids(self) -> list[str]:
        """Get the object mount ids from the parameter server."""
        return self.param("mount_collision_ids")

    @property
    def attach_link(self) -> str:
        return self.default_pose_link

    @property
    def touch_links(self) -> list[str]:
        """Get the touch links from the parameter server."""
        return self.param("touch_links")

    @property
    def allowed_mount_collisions(self) -> list[tuple[str, str]]:
        """Get the allowed object mount collisions from the parameter server."""
        return [
            (id_0, id_1)
            for id_0, id_1 in self.param("allowed_mount_collisions").items()
        ]

    def _get_reset_config(self, object_id: str) -> ObjectResetConfig | None:
        """Get the mapping of object ids to reset configuration filenames"""
        filename = self.param(f"reset_config_map.{object_id}")
        if filename is None:
            return None
        return self._reset_configs[filename]

    ###########################################################################
    ########## Object Manipulation Convenience Methods ########################
    ###########################################################################

    def _get_attached_object_id(self) -> str | None:
        """Get the ID of the attached collision object

        Returns:
            The ID of the attached collision object or None if no object is attached

        Raises:
            RuntimeError: If there is not exactly one attached collision object
        """
        attached_objects = self._moveit.attached_collision_objects
        attach_link = self.attach_link

        attached_ids: list[str] = []
        for object_id, attached_object in attached_objects.items():
            if attached_object.link_name == attach_link:
                attached_ids.append(object_id)

        if len(attached_ids) == 0:
            return None
        elif len(attached_ids) == 1:
            return attached_ids[0]
        else:
            raise RuntimeError(
                f"Expected exactly one collision object to be attached to "
                f"{attach_link} of joint model group {self.group_name} but "
                f"got {len(attached_ids)} ({attached_ids})"
            )

    def _grid_object_pose_stamped_with_offset(
        self, object_id: str, offset: list[float]
    ) -> PoseStamped:
        """Get the initial pose of an object from the parameters with an offset."""
        old_pose_stamped = self._moveit.grid_objects_by_id[
            object_id
        ].pose_stamped
        old_frame_transform = matrix_from_pose_msg(old_pose_stamped.pose)
        new_frame_id = old_pose_stamped.header.frame_id
        assert new_frame_id == self._moveit.planning_frame
        new_frame_transform = self._moveit.get_frame_transform(new_frame_id)

        pose_stamped = pose_stamped_msg(position=offset)

        return change_reference_frame_pose_stamped(
            old_pose_stamped=pose_stamped,
            old_frame_transform=old_frame_transform,
            new_frame_transform=new_frame_transform,
            new_frame_id=new_frame_id,
        )

    def _validate_manipulation_state(self):
        # Check that joint model group name exists
        assert (
            self._manipulation_state in (State.IDLE, State.UNINITIALIZED)
        ) == (self._current_manipulation_id is None), (
            f"_current_manipulation_id should be None (got "
            f"{self._current_manipulation_id}) iff _manipulation_state is IDLE "
            f"(got {self._manipulation_state.name})"
        )

        attached_object_id = self._get_attached_object_id()
        if attached_object_id is None:
            assert self._manipulation_state in _OBJECT_DETACHED_STATES, (
                f"If object is not attached, we can only be in "
                f"{(x.name for x in _OBJECT_DETACHED_STATES)} states, "
                f"got: {self._manipulation_state.name}"
            )
        else:
            assert attached_object_id == self._current_manipulation_id

            if attached_object_id in self._moveit.grid_objects_by_id:
                assert (
                    self._manipulation_state in _GRID_OBJECT_ATTACHED_STATES
                ), (
                    f"If grid object is attached, we can only be in "
                    f"{(x.name for x in _GRID_OBJECT_ATTACHED_STATES)} states, "
                    f"got: {self._manipulation_state.name}"
                )
            else:
                assert self._manipulation_state == State.MANUALLY_ATTACHED, (
                    f"If non-grid object is attached, we can only be in "
                    f"MANUALLY_ATTACHED state, got: {self._manipulation_state.name}"
                )

    def _validate_target_object(
        self, object_id: str, *, expect_grid_object: bool
    ):
        # Check that grid object exists and is reachable
        if expect_grid_object:
            if object_id not in self._moveit.grid_objects_by_id:
                raise ValueError(f"'{object_id}' is not a valid grid object")
            if object_id not in self._reachable_object_ids:
                raise ValueError(
                    f"'{object_id}' is not reachable by robot {self.group_name}"
                )
        else:
            if object_id in self._moveit.grid_objects_by_id:
                raise ValueError(f"'{object_id}' should not be grid object")

        # Check that target grid object is consistent with manipulation id
        if (
            self._current_manipulation_id is not None
            and object_id != self._current_manipulation_id
        ):
            raise ObjectMismatchError(
                f"Cannot manipulate object {object_id} "
                f"because object {self._current_manipulation_id} "
                f"is already being manipulated",
                group_name=self.group_name,
            )

    # @asynccontextmanager
    # async def _validate_and_lock(self, *, wait: bool):
    #     if not wait and self._manipulation_lock.locked():
    #         raise ObjectManipulationError(
    #             "Robot cannot cannot acquire manipulation "
    #             "lock while while another operation is in progress",
    #             group_name=self.group_name,
    #         )
    #
    #     async with self._manipulation_lock:
    #         self._validate_manipulation_state()
    #         try:
    #             yield
    #         finally:
    #             self._validate_manipulation_state()

    ###########################################################################
    ########## Object Manipulation State Transition Logic #####################
    ###########################################################################

    def _get_state_goal(
        self,
        state: State,
        object_id: Optional[str],
    ) -> PlanGoalT:
        """Get the goal for the given state and object.

        Args:
            state: The state to get the goal for
            object_id: The ID of the object to get the goal for
            goal: The goal to use if the state is present or unpresent

        Returns:
            The goal for the given state and object.
        """
        if state == State.PRE_RETURN and object_id in self._post_fetch_states:
            return self._post_fetch_states[object_id]

        goal_name = _STATE_GOAL_NAME_MAP[state]
        param_name = f"manipulation_state_goals.object_overrides.{object_id}.{goal_name}"
        goal_config: dict[str, Any]
        if object_id is not None:
            try:
                goal_config = self.param(param_name)
            except ParameterNotDeclaredException:
                param_name = f"manipulation_state_goals.{goal_name}"
                goal_config = self.param(param_name)
        else:
            param_name = f"manipulation_state_goals.{goal_name}"
            goal_config = self.param(param_name)

        goal_type: str = goal_config["type"]
        match goal_type:
            case "offset":
                assert object_id is not None
                offset: list[float] = goal_config["value"]
                return self._grid_object_pose_stamped_with_offset(
                    object_id, offset
                )
            case "named_target_state":
                named_target_state: str = goal_config["value"]
                assert (
                    named_target_state
                    in self._moveit.get_named_target_states(self.group_name)
                )
                return named_target_state
            case "joint_positions":
                joint_positions: dict[str, float] = goal_config["value"]
                assert set(joint_positions.keys()) == set(
                    self._moveit.get_joint_names(self.group_name)
                )
                robot_state = self._moveit.get_current_state()
                robot_state.joint_positions = joint_positions
                robot_state.update()
                return robot_state
            case "pose_stamped":
                pose_stamped: dict[str, Any] = goal_config["value"]
                return pose_stamped_msg(**pose_stamped)
            case _:
                raise ValueError(
                    f"Unknown manipulation_state_goal type for parameter "
                    f"{param_name}: {goal_type}"
                )

    async def _fetch_or_return_transition(
        self, object_id: str, next_state: State
    ) -> list[TrajectoryCacheKwargs] | None:
        """Plan and execute a state transition of the object manipulation process.

        This is a helper function for the object manipulation process.

        Args:
            object_id: The ID of the object to manipulate
            group_name: Joint model group name to use
            next_state: The object manipulation state to transition to

        Returns:
            A dictionary containing the kwargs to cache the trajectory, or None
            if the trajectory was found in the cache.
        """
        self.log(
            f"Transitioning from "
            f"{self._manipulation_state.name} to "
            f"{next_state.name} state for object {object_id}"
        )

        assert (
            self._manipulation_state == State.RESETTED
            or self._manipulation_state in _FETCH_OR_RETURN_STATES
        )
        assert next_state in _FETCH_OR_RETURN_STATES

        if next_state == State.IDLE and self.param("skip_idle_on_return"):
            return None
        if next_state == State.FETCHED and self.param("skip_fetched_on_fetch"):
            return None

        goal = self._get_state_goal(next_state, object_id)
        request = PlanRequest(goal=goal)

        if next_state in (
            State.PRE_ATTACH,
            State.ATTACH,
            State.POST_ATTACH,
            State.POST_FETCH,
            State.PRE_DETACH,
            State.DETACH,
            State.POST_DETACH,
            State.POST_RETURN,
        ):
            request.planning_pipeline = "linear"
            request.use_cache = False

        if next_state in (State.DETACH, State.POST_DETACH):
            request.velocity_scaling_factor = self.param(
                "detach_velocity_scaling_factor"
            )

        collisions_to_allow: list[tuple[str, str]] = []
        modified_collisions: list[tuple[str, str]] = []

        if next_state in (
            State.PRE_ATTACH,
            State.ATTACH,
            State.POST_ATTACH,
            # State.POST_FETCH,
            # State.PRE_DETACH,
            State.DETACH,
            State.POST_DETACH,
            State.POST_RETURN,
        ):
            collisions_to_allow.extend(self.allowed_mount_collisions)

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
                    [(object_id, x) for x in self.mount_collision_ids]
                )

        if len(collisions_to_allow) > 0:
            modified_collisions = self._moveit.allow_collision(
                *zip(*collisions_to_allow)
            )
        try:
            cache_kwargs = await self.plan_and_execute(
                request, cache_trajectories=False
            )
        finally:
            if len(modified_collisions) > 0:
                self._moveit.disallow_collision(*zip(*modified_collisions))

        match next_state:
            case State.POST_FETCH:
                self._post_fetch_states[object_id] = (
                    self._moveit.get_current_state()
                )
            case State.ATTACH:
                self._moveit.move_collision_object(
                    object_id,
                    self._moveit.get_link_pose_stamped(self.attach_link),
                )
                self._moveit.attach_collision_object(
                    object_id,
                    self.attach_link,
                    touch_links=self.touch_links,
                )
            case State.DETACH:
                self._moveit.detach_collision_object(object_id)
                self._moveit.move_collision_object(
                    object_id,
                    self._moveit.grid_objects_by_id[object_id].pose_stamped,
                )

        if request.planning_pipeline == "linear":
            return None
        else:
            return cache_kwargs

    async def _fetch_object_impl(
        self,
        object_id: str,
        *,
        cache_trajectories: bool = True,
    ) -> None:
        self.log(f"Fetching object {object_id}")

        self._validate_target_object(object_id, expect_grid_object=True)

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
                    "Already at FETCHED state, skipping fetch",
                    severity="WARN",
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
                    f"Cannot fetch object from current state: {unexpected.name}",
                    group_name=self.group_name,
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        assert State.PRE_FETCH <= next_state and next_state <= State.FETCHED

        cache_kwargs: list[TrajectoryCacheKwargs] = []

        # Iterate through the fetch states
        while self._manipulation_state != State.FETCHED:
            try:
                kwargs = await self._fetch_or_return_transition(
                    object_id, next_state
                )
            except PlanningError:
                if next_state != State.POST_FETCH:
                    raise

                # If post-fetch fails, try moving to fetched
                self.log(
                    "Failed to plan to POST_FETCH, skipping to FETCHED",
                    severity="WARN",
                )
                next_state = State.FETCHED
                try:
                    kwargs = await self._fetch_or_return_transition(
                        object_id, next_state
                    )
                except (ExecutionInterruptedError, ExecutionStoppedError):
                    # If execution is interrupted here, we set the manipulation
                    # state to POST_FETCH so that we don't get stuck trying to
                    # return the object
                    self._manipulation_state = State.POST_FETCH
                    raise

            self._manipulation_state = next_state
            next_state = State(self._manipulation_state + 1)

            if self._manipulation_state != State.IDLE:
                self._current_manipulation_id = object_id

            self._validate_manipulation_state()

            if cache_trajectories and kwargs is not None:
                cache_kwargs.extend(kwargs)

        # Cache all trajectories if requested
        if cache_trajectories and len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    async def _present_object_impl(
        self, object_id: str, *, cache_trajectories=True
    ):
        self.log(f"Presenting object {object_id}")

        self._validate_target_object(object_id, expect_grid_object=True)

        match self._manipulation_state:
            case State.FETCHED | State.RESETTED:
                next_state = State.PRESENTED
            case unexpected if isinstance(unexpected, State):
                raise StateTransitionError(
                    f"Cannot present object "
                    f"from current state: {unexpected.name}",
                    group_name=self.group_name,
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        goal = self._get_state_goal(next_state, object_id)
        await self.plan_and_execute(
            goal=goal,
            cache_trajectories=cache_trajectories,
        )
        self._manipulation_state = next_state

    async def _unpresent_object_impl(
        self, object_id: str, *, cache_trajectories=True
    ):
        self.log(f"Unpresenting object {object_id}")

        self._validate_target_object(object_id, expect_grid_object=True)

        match self._manipulation_state:
            case State.PRESENTED:
                next_state = State.NEEDS_RESET
            case unexpected if isinstance(unexpected, State):
                raise StateTransitionError(
                    f"Cannot unpresent object from current state: {unexpected.name}",
                    group_name=self.group_name,
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        goal = self._get_state_goal(next_state, object_id)
        await self.plan_and_execute(
            goal=goal,
            cache_trajectories=cache_trajectories,
        )
        self._manipulation_state = next_state

    async def _reset_object_impl(
        self,
        object_id: str,
        *,
        cache_trajectories: bool = True,
    ):
        self.log(f"Resetting object {object_id}")

        self._validate_target_object(object_id, expect_grid_object=True)

        match self._manipulation_state:
            case State.NEEDS_RESET:
                pre_reset_allow_collisions = False
            case State.PRE_RESET:
                pre_reset_allow_collisions = True
            # case State.POST_FETCH | State.FETCHED | State.RESETTED:
            #     self.log(
            #         f"No need to reset object {object_id} from current state ({self._manipulation_state}), skipping",
            #         severity="WARN",
            #     )
            #     return
            case unexpected if isinstance(unexpected, State):
                raise StateTransitionError(
                    f"Cannot reset object from current state: {unexpected.name}",
                    group_name=self.group_name,
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        # Retrieve reset config
        config = self._get_reset_config(object_id)
        if config is None:
            self._manipulation_state = State.RESETTED
            return
        assert config.reset_request.planning_pipeline == "linear"

        cache_kwargs: list[TrajectoryCacheKwargs] = []

        # Plan and execute to start goal
        if not pre_reset_allow_collisions:
            kwargs = await self.plan_and_execute(
                goal=config.start_goal,
                cache_trajectories=False,
            )
            if cache_trajectories and kwargs is not None:
                cache_kwargs.extend(kwargs)

            self._manipulation_state = State.PRE_RESET

        # Plan and execute reset path with allowed collisions
        collisions_to_allow: list[tuple[str, str]] = []
        modified_collisions: list[tuple[str, str]] = []

        if config.object_allowed_collision_ids is not None:
            collisions_to_allow.extend(
                [(object_id, x) for x in config.object_allowed_collision_ids]
            )

        if config.additional_allowed_collisions is not None:
            collisions_to_allow.extend(config.additional_allowed_collisions)

        if len(collisions_to_allow) > 0:
            modified_collisions = self._moveit.allow_collision(
                *zip(*collisions_to_allow)
            )
        try:
            if pre_reset_allow_collisions:
                await self.plan_and_execute(
                    goal=config.start_goal,
                    group_name=self.group_name,
                    planning_pipeline="linear",
                    cache_trajectories=False,
                )

            kwargs = await self.plan_and_execute(
                config.reset_request, cache_trajectories=False
            )
        finally:
            if len(modified_collisions) > 0:
                self._moveit.disallow_collision(*zip(*modified_collisions))

        if cache_trajectories and kwargs is not None:
            cache_kwargs.extend(kwargs)

        self._manipulation_state = State.RESETTED

        # Cache all trajectories if requested
        if len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    async def _return_object_impl(
        self,
        object_id: str,
        *,
        cache_trajectories: bool = True,
    ) -> None:
        self.log(f"Returning object {object_id}")

        self._validate_target_object(object_id, expect_grid_object=True)

        match self._manipulation_state:
            case State.IDLE:
                self.log(
                    "Already at IDLE state, skipping return",
                    severity="WARN",
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
                raise StateTransitionError(
                    f"Cannot return object "
                    f"from current state: {unexpected.name}",
                    group_name=self.group_name,
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

            self._validate_manipulation_state()

            if cache_trajectories and kwargs is not None:
                cache_kwargs.extend(kwargs)

        # Cache all trajectories if requested
        if cache_trajectories and len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    async def _plan_and_move_impl(
        self,
        request: PlanRequest | ConcatPlanRequest | None = None,
        *,
        cache_trajectories: bool = True,
        **kwargs: Any,
    ) -> None:
        match self._manipulation_state:
            case (
                State.IDLE
                | State.MANUALLY_ATTACHED
                | State.FETCHED
                | State.PRESENTED
                | State.RESETTED
            ):
                pass
            case unexpected if isinstance(unexpected, State):
                raise StateTransitionError(
                    f"Cannot plan_and_move from current state: {unexpected.name}",
                    group_name=self.group_name,
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        await self.plan_and_execute(
            request,
            cache_trajectories=cache_trajectories,
            **kwargs,
        )
        # No state update needed

    async def _move_impl(
        self, trajectory: RobotTrajectory | list[RobotTrajectory]
    ):
        match self._manipulation_state:
            case (
                State.IDLE
                | State.MANUALLY_ATTACHED
                | State.FETCHED
                | State.PRESENTED
                | State.RESETTED
            ):
                pass
            case unexpected if isinstance(unexpected, State):
                raise StateTransitionError(
                    f"Cannot move from current state: {unexpected.name}",
                    group_name=self.group_name,
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        await self.execute(trajectory)

        # No state update needed

    async def _manually_attach_object_impl(self, object_id: str):
        self.log(f"Manually attaching object {object_id}")

        self._validate_target_object(object_id, expect_grid_object=False)

        match self._manipulation_state:
            case State.IDLE:
                pass
            case unexpected if isinstance(unexpected, State):
                raise StateTransitionError(
                    f"Cannot manually attach object "
                    f"from current state: {unexpected.name}",
                    group_name=self.group_name,
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        # Get mesh path
        mesh_dir = self.param("manually_attach.mesh_dir")
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

        self._moveit.add_mesh_collision_object(
            object_id,
            path=mesh_path,
            pose_stamped=self._moveit.get_link_pose_stamped(self.attach_link),
            **self.param("manually_attach.collision_object_kwargs"),
        )
        self._moveit.attach_collision_object(
            object_id, self.attach_link, touch_links=self.touch_links
        )

        self._current_manipulation_id = object_id
        self._manipulation_state = State.MANUALLY_ATTACHED

    async def _manually_detach_object_impl(self, object_id: str):
        self.log(f"Manually detaching object {object_id}")

        self._validate_target_object(object_id, expect_grid_object=False)

        match self._manipulation_state:
            case State.MANUALLY_ATTACHED:
                pass
            case unexpected if isinstance(unexpected, State):
                raise StateTransitionError(
                    f"Cannot manually detach object "
                    f"from current state: {unexpected.name}",
                    group_name=self.group_name,
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        self._moveit.detach_collision_object(object_id)
        self._moveit.remove_collision_object(object_id)

        self._current_manipulation_id = None
        self._manipulation_state = State.IDLE

    async def _test_object_attached(self):
        self.log("Testing if an object is attached")

        assert self._manipulation_state == State.UNINITIALIZED

        config: dict[str, Any] = self.param("test_object_attached")
        goal: str = config["goal"]
        object_id: str = config["object_id"]
        topic: str = config["topic"]
        num_samples: int = config["num_samples"]
        joint_name: str = config["joint_name"]
        effort_threshold: float = config["effort_threshold"]
        greater_than: bool = config["greater_than"]

        if joint_name not in self._moveit.get_current_state().joint_efforts:
            raise RuntimeError(f"Unknown joint_name: {joint_name}")

        done_event = asyncio.Event()
        loop = asyncio.get_running_loop()
        sample_count = 0
        joint_efforts: dict[str, list[float]] = {}

        def joint_state_callback(msg: JointState):
            try:
                nonlocal joint_efforts
                nonlocal num_samples
                nonlocal joint_name
                nonlocal loop
                nonlocal done_event
                nonlocal sample_count

                if sample_count < num_samples:
                    for i, joint in enumerate(msg.name):
                        joint_efforts.setdefault(joint, []).append(
                            msg.effort[i]
                        )
                    sample_count += 1
                elif not done_event.is_set():
                    loop.call_soon_threadsafe(done_event.set)
            except BaseException as e:
                traceback.print_exception(e)
                raise

        try:
            self._manipulation_state = State.IDLE
            await self._manually_attach_object_impl(object_id)
            try:
                await self._plan_and_move_impl(
                    goal=goal, cache_trajectories=False
                )
            finally:
                await self._manually_detach_object_impl(object_id)

            # Sleep to allow robot to stop
            await asyncio.sleep(1)

            sub = self.node.create_subscription(
                JointState,
                topic,
                joint_state_callback,
                10,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
            try:
                await done_event.wait()
            finally:
                self.node.destroy_subscription(sub)

            avgs = {}
            stds = {}
            for joint, efforts in joint_efforts.items():
                efforts = np.array(efforts)
                avg = float(efforts.mean())
                std = float(efforts.std())
                self.log(
                    f"Joint effort for joint {joint}: {avg:.4f} +- {std:.4f}"
                )
                avgs[joint] = avg
                stds[joint] = std

            avg_effort = avgs[joint_name]
            if greater_than:
                is_attached = avg_effort > effort_threshold
            else:
                is_attached = avg_effort < effort_threshold

            initial_object_id = self._init_attached_object()

            if is_attached and initial_object_id is None:
                raise RuntimeError(
                    "Attached object detected but initial_object not provided"
                )
            elif not is_attached and initial_object_id is not None:
                raise RuntimeError(
                    f"No attached object detected but initial_object "
                    f"({initial_object_id}) was provided"
                )

            return initial_object_id
        finally:
            self._manipulation_state = State.UNINITIALIZED

    async def _test_object_attached_eef(self):
        self.log("Testing if an object is attached")

        assert self._manipulation_state == State.UNINITIALIZED

        dim_to_idx = {"x": 0, "y": 1, "z": 2}

        config: dict[str, Any] = self.param("test_object_attached")
        goal: str = config["goal"]
        object_id: str = config["object_id"]
        topic: str = config["topic"]
        num_samples: int = config["num_samples"]
        force_idx = dim_to_idx[config["force_dim"]]
        torque_idx = dim_to_idx[config["torque_dim"]]
        force_threshold: float = config["force_threshold"]
        torque_threshold: float = config["torque_threshold"]

        done_event = asyncio.Event()
        loop = asyncio.get_running_loop()
        sample_count = 0
        forces: list[list[float]] = []
        torques: list[list[float]] = []

        def wrench_callback(msg: WrenchStamped):
            try:
                nonlocal forces
                nonlocal torques
                nonlocal num_samples
                nonlocal loop
                nonlocal done_event
                nonlocal sample_count

                if sample_count < num_samples:
                    force = msg.wrench.force
                    torque = msg.wrench.torque
                    forces.append([force.x, force.y, force.z])
                    torques.append([torque.x, torque.y, torque.z])
                    sample_count += 1
                elif not done_event.is_set():
                    loop.call_soon_threadsafe(done_event.set)
            except BaseException as e:
                traceback.print_exception(e)
                raise

        try:
            self._manipulation_state = State.IDLE
            await self._manually_attach_object_impl(object_id)
            await self._plan_and_move_impl(goal=goal, cache_trajectories=False)
            await self._manually_detach_object_impl(object_id)

            # Sleep to allow robot to stop
            await asyncio.sleep(1)

            sub = self.node.create_subscription(
                WrenchStamped,
                topic,
                wrench_callback,
                10,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
            try:
                await done_event.wait()
            finally:
                self.node.destroy_subscription(sub)

            avg_forces = np.mean(np.array(forces), axis=0)
            avg_torques = np.mean(np.array(torques), axis=0)

            self.log(f"Avg forces: {avg_forces}, Avg torques: {avg_torques}")

            avg_force = float(np.absolute(avg_forces[force_idx]))
            avg_torque = float(np.absolute(avg_torques[torque_idx]))

            is_attached = (avg_force > force_threshold) or (
                avg_torque > torque_threshold
            )
            initial_object_id = self._init_attached_object()

            if is_attached and initial_object_id is None:
                raise RuntimeError(
                    "Attached object detected but initial_object not provided"
                )
            elif not is_attached and initial_object_id is not None:
                raise RuntimeError(
                    f"No attached object detected but initial_object "
                    f"({initial_object_id}) was provided"
                )

            return initial_object_id

        finally:
            self._manipulation_state = State.UNINITIALIZED

    async def _reset_manipulation_impl(self, *, reset_to_idle: bool):
        self.log("Resetting object manipulation")

        # Test if object is attached and
        if self._manipulation_state == State.UNINITIALIZED:
            if self._simulate or not self.param("test_object_attached.enable"):
                self._current_manipulation_id = self._init_attached_object()
            else:
                self._current_manipulation_id = (
                    await self._test_object_attached()
                )

            if self._current_manipulation_id is None:
                self._manipulation_state = State.IDLE
            else:
                self._manipulation_state = State.NEEDS_RESET

        # Reset and return object if attached
        if self._manipulation_state not in (
            State.IDLE,
            State.MANUALLY_ATTACHED,
        ):
            object_id = self._current_manipulation_id
            assert object_id is not None

            # Handle case where robot may be stuck in detach state
            # if self._manipulation_state == State.DETACH:
            #     await self._fetch_or_return_transition(object_id, State.ATTACH)
            #     self._manipulation_state = State.ATTACH
            #     await self._fetch_or_return_transition(
            #         object_id, State.POST_ATTACH
            #     )
            #     self._manipulation_state = State.POST_ATTACH
            #     # except (
            #     #     ExecutionRejectedError,
            #     #     ExecutionInterruptedError,
            #     # ) as e:
            #     #     if self._simulate:
            #     #         raise
            #     #     else:
            #     #         raise RuntimeError(
            #     #             "Object seems stuck, aborting"
            #     #         ) from e

            # "Unreturn" object and move to idle to try and get a better plan
            if self._manipulation_state in (
                State.PRE_RETURN,
                State.PRE_DETACH,
                State.DETACH,
            ):
                await self._fetch_object_impl(object_id)
                goal = self._get_state_goal(State.IDLE, object_id=None)
                await self.plan_and_execute(
                    goal=goal, cache_trajectories=False
                )

            # Unpresent object if needed
            elif self._manipulation_state == State.PRESENTED:
                await self._unpresent_object_impl(object_id)

            # Reset object if needed
            if self._manipulation_state in (
                State.NEEDS_RESET,
                State.PRE_RESET,
            ):
                try:
                    await self._reset_object_impl(object_id)
                except PlanningError:
                    goal = self._get_state_goal(State.IDLE, object_id=None)
                    await self.plan_and_execute(
                        goal=goal, cache_trajectories=False
                    )
                    await self._reset_object_impl(object_id)

            # Return object
            await self._return_object_impl(object_id)

        if reset_to_idle:
            goal = self._get_state_goal(State.IDLE, object_id=None)
            await self._plan_and_move_impl(goal=goal, cache_trajectories=False)

    ###########################################################################
    ########## Object Manipulation User Interface #############################
    ###########################################################################

    @property
    def current_manipulation_id(self) -> str | None:
        return self._current_manipulation_id

    @validate_and_lock
    async def fetch_object(
        self,
        object_id: str,
        *,
        cache_trajectories: bool = True,
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
        # async with self._validate_and_lock(wait=False):
        await self._fetch_object_impl(
            object_id, cache_trajectories=cache_trajectories
        )

    @validate_and_lock
    async def present_object(self, object_id: str, *, cache_trajectories=True):
        """Present the object to the present pose

        Args:
            goal: The goal to present the object at
        """
        # async with self._validate_and_lock(wait=False):
        await self._present_object_impl(
            object_id, cache_trajectories=cache_trajectories
        )

    @validate_and_lock
    async def unpresent_object(
        self, object_id: str, *, cache_trajectories=True
    ):
        """Unpresent the currently attached object

        Args:
            goal: The goal to present the object at
        """
        # async with self._validate_and_lock(wait=False):
        await self._unpresent_object_impl(
            object_id, cache_trajectories=cache_trajectories
        )

    @validate_and_lock
    async def reset_object(
        self,
        object_id: str,
        *,
        cache_trajectories: bool = True,
    ):
        """Perform the reset procedure for an object"""
        # async with self._validate_and_lock(wait=False):
        await self._reset_object_impl(
            object_id, cache_trajectories=cache_trajectories
        )

    @validate_and_lock
    async def return_object(
        self,
        object_id: str,
        *,
        cache_trajectories: bool = True,
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
        # async with self._validate_and_lock(wait=False):
        await self._return_object_impl(
            object_id, cache_trajectories=cache_trajectories
        )

    @validate_and_lock
    async def manually_attach_object(self, object_id: str):
        """Manually attach collision object to the robot end effector."""
        # async with self._validate_and_lock(wait=False):
        await self._manually_attach_object_impl(object_id)

    @validate_and_lock
    async def manually_detach_object(self, object_id: str):
        """Manually detach collision object from the robot end effector."""
        # async with self._validate_and_lock(wait=False):
        await self._manually_detach_object_impl(object_id)

    @validate_and_lock
    async def plan_and_move(
        self,
        request: PlanRequest | ConcatPlanRequest | None = None,
        *,
        cache_trajectories: bool = True,
        **kwargs: Any,
    ) -> None:
        """TODO"""
        # async with self._validate_and_lock(wait=False):
        await self._plan_and_move_impl(
            request, cache_trajectories=cache_trajectories, **kwargs
        )

    @validate_and_lock
    async def move(self, trajectory: RobotTrajectory | list[RobotTrajectory]):
        """TODO"""
        # async with self._validate_and_lock(wait=False):
        await self._move_impl(trajectory)

    ###########################################################################
    ########## Reset ##########################################################
    ###########################################################################

    @validate_and_lock
    async def reset_manipulation(self, *, reset_to_idle: bool = False) -> None:
        """Reset robot manipulation state.

        If manipulating a grid object (aka not a manually attached object),
        reset and return the grid object to its mount if necessary.

        Then, move to idle position.
        """
        # TODO: Maybe change wait to True here
        await self._reset_manipulation_impl(reset_to_idle=reset_to_idle)
