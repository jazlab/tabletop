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
import pickle
import traceback
from collections.abc import Callable
from copy import copy
from enum import IntEnum
from glob import glob
from typing import Any, NamedTuple, Optional

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
    JointStateDeltaDict,
    JointStateDict,
    ObjectResetConfig,
    PlanGoalT,
    PlanRequest,
    TrajectoryCacheKwargs,
)
from tabletop_rig.nodes.base import BaseNode
from tabletop_rig.utils.ros import (
    change_reference_frame_pose_stamped,
    constraints_msg,
    get_joint_group_positions,
    matrix_from_pose_msg,
    pose_stamped_msg,
)

_PICKLE_PROTOCOL = pickle.HIGHEST_PROTOCOL


class ManipulationState(IntEnum):
    """State machine states for object manipulation operations.

    Represents the current phase of a pick-place-return cycle for grid
    objects, plus special states for manual attachment and initialization.

    Main Sequences:
        FETCH: IDLE → PRE_FETCH → PRE_ATTACH → ATTACH → POST_ATTACH
            → POST_FETCH → FETCHED (sequential; each state can skip if
            already past)
        PRESENT: FETCHED → PRESENTED (atomic)
        RESET: User-defined waypoints via ObjectResetConfig (NEEDS_RESET
            → PRE_RESET → RESETTED)
        RETURN: FETCHED/RESETTED → PRE_RETURN → PRE_DETACH → DETACH
            → POST_DETACH → POST_RETURN → IDLE (reverse of fetch; can
            skip to IDLE if planning fails)

    State Definitions:
        IDLE: No object attached, ready for fetch/present/move commands.
        PRE_FETCH: Moving to fetch approach pose (below/behind mount).
        PRE_ATTACH: Moving to dovetail contact pose (up w.r.t. mount).
        ATTACH: Inserting into dovetail (forward w.r.t. mount); object
            attached in this state.
        POST_ATTACH: Withdrawing from mount (forward w.r.t. mount).
        POST_FETCH: Moving away from grid (down w.r.t. mount).
        FETCHED: Ready to present; object held above grid.
        PRESENTED: Holding object at presentation pose (locked via
            exclusive region).
        NEEDS_RESET: Object presented; unpresenting failed or reset
            requested. Requires reset path execution.
        PRE_RESET: Reset sequence started; awaiting completion.
        RESETTED: Reset sequence complete; ready for return.
        PRE_RETURN: Moving to return approach (above/in front of mount).
        PRE_DETACH: Moving to dovetail insertion pose (up w.r.t. mount).
        DETACH: Inserting into dovetail for return (back w.r.t. mount);
            object still attached.
        POST_DETACH: Withdrawing from dovetail (back w.r.t. mount);
            object detached in this state.
        POST_RETURN: Moving away from grid (down w.r.t. mount).
        MANUALLY_ATTACHED: Non-grid object manually attached to end
            effector (no state transitions; only reachable via
            manually_attach_object()).
        UNINITIALIZED: Initial state; not yet tested for attached object.
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
        ManipulationState.PRE_FETCH,
        ManipulationState.PRE_ATTACH,
        ManipulationState.ATTACH,
        ManipulationState.POST_ATTACH,
        ManipulationState.POST_FETCH,
        ManipulationState.FETCHED,
        ManipulationState.PRE_RETURN,
        ManipulationState.PRE_DETACH,
        ManipulationState.DETACH,
        ManipulationState.POST_DETACH,
        ManipulationState.POST_RETURN,
    )
)

_OBJECT_DETACHED_STATES = set(
    (
        ManipulationState.IDLE,
        ManipulationState.PRE_FETCH,
        ManipulationState.PRE_ATTACH,
        ManipulationState.DETACH,
        ManipulationState.POST_DETACH,
        ManipulationState.POST_RETURN,
        ManipulationState.UNINITIALIZED,
    )
)
_GRID_OBJECT_ATTACHED_STATES = set(
    (
        ManipulationState.ATTACH,
        ManipulationState.POST_ATTACH,
        ManipulationState.POST_FETCH,
        ManipulationState.FETCHED,
        ManipulationState.PRESENTED,
        ManipulationState.NEEDS_RESET,
        ManipulationState.PRE_RESET,
        ManipulationState.RESETTED,
        ManipulationState.PRE_RETURN,
        ManipulationState.PRE_DETACH,
    )
)


# Mapping of manipulation states to their corresponding pose offsets
_STATE_GOAL_NAME_MAP = {
    ManipulationState.IDLE: "idle",
    ManipulationState.PRE_FETCH: "pre_fetch",
    ManipulationState.PRE_ATTACH: "pre_attach",
    ManipulationState.ATTACH: "attach",
    ManipulationState.POST_ATTACH: "post_attach",
    ManipulationState.POST_FETCH: "post_fetch",
    ManipulationState.FETCHED: "fetched",
    ManipulationState.PRESENTED: "present",
    ManipulationState.NEEDS_RESET: "fetched",
    ManipulationState.PRE_RETURN: "post_fetch",
    ManipulationState.PRE_DETACH: "post_attach",
    ManipulationState.DETACH: "attach",
    ManipulationState.POST_DETACH: "pre_attach",
    ManipulationState.POST_RETURN: "pre_fetch",
}


class PersistentState(NamedTuple):
    manipulation_state: ManipulationState
    manipulation_id: str | None
    saved_return_state_positions: dict[
        str, tuple[ManipulationState, dict[str, float]]
    ]


class ResetLoader(KwargYamlLoader):
    def get_kwarg_constructors(self) -> dict[str, Callable]:
        return {
            "!PoseStamped": pose_stamped_msg,
            "!Constraints": constraints_msg,
            "!ConcatPlanRequest": ConcatPlanRequest,
            "!ObjectResetConfig": ObjectResetConfig,
            "!JointStateDict": JointStateDict,
            "!JointStateDeltaDict": JointStateDeltaDict,
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


class ObjectManipulationInterface(PlanAndExecuteInterface):
    """High-level object manipulation state machine for pick-and-place ops.

    Extends PlanAndExecuteInterface with automated state-machine-based
    picking, presenting, and returning of objects from a grid mount.
    Coordinates collision allowances, attachment/detachment, and reset
    sequences defined in per-object YAML configuration files.

    State Machine (ManipulationState):
        Fetch sequence (PRE_FETCH → ... → FETCHED)
        Present sequence (FETCHED → PRESENTED)
        Reset sequence (user-defined waypoints via ObjectResetConfig)
        Return sequence (FETCHED/PRESENTED → ... → IDLE)

    Parameters loaded from config with fallback prefixes:
        '<name>_manipulation_interface.' → 'common_manipulation_interface.'

    Attributes:
        _manipulation_state: Current state in fetch/present/return cycle.
        _current_manipulation_id: Object currently being manipulated.
        _manipulation_lock: Async lock for serializing operations.
        _reachable_object_ids: Set of grid object IDs the robot can reach.
        _reset_configs: Cached ObjectResetConfig per grid object.
        _saved_return_states: Cached states for returning to fetch pose.
        _last_pre_reset_state: State before reset sequence (for recovery).
        _persistent_state_path: Absolute path for persisting state on exit.
    """

    _manipulation_state: ManipulationState
    _current_manipulation_id: str | None
    _manipulation_lock: asyncio.Lock
    _reachable_object_ids: set[str]
    _reset_configs: dict[str, ObjectResetConfig]
    _saved_return_states: dict[str, tuple[ManipulationState, RobotState]]
    _last_pre_reset_state: RobotState | None
    _persistent_state_path: str

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

        self._simulate = simulate

        self._init_reachable_indices()

        self._init_reset_configs()

        self._manipulation_state = ManipulationState.UNINITIALIZED
        self._current_manipulation_id = None
        self._manipulation_lock = asyncio.Lock()

        self._saved_return_states = {}
        self._last_pre_reset_state = None

        path: str = self.param("persistent_state_path")
        path = os.path.expandvars(os.path.expanduser(path))
        if not os.path.isabs(path):
            raise ValueError(
                f"'persistent_state_path' parameter must be absolute: {path}"
            )
        self._persistent_state_path = path

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

                # TODO: I warned you
                if (
                    config.object_allowed_collision_ids is not None
                    and len(config.object_allowed_collision_ids) > 0
                ) or (
                    config.additional_allowed_collisions is not None
                    and len(config.additional_allowed_collisions) > 0
                ):
                    if config.reset_request.planning_pipeline not in (
                        "linear",
                        "ptp",
                    ):
                        raise ValueError(
                            f"Error in reset config ({filename}): "
                            f"If 'object_allowed_collision_ids' or "
                            f"'additional_allowed_collisions' is provided, "
                            f"'reset_request.planning_pipeline' must be 'linear' "
                            f"or 'ptp' (for safe object resetting to "
                            f"prevent accidental collisions)"
                        )

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

    @property
    def manipulation_state(self) -> ManipulationState:
        """Get the manipulation state for this robot"""
        return self._manipulation_state

    @property
    def manipulation_id(self) -> str | None:
        """Get the manipulation object id for this robot"""
        return self._current_manipulation_id

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
            self._manipulation_state
            in (ManipulationState.IDLE, ManipulationState.UNINITIALIZED)
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
                assert (
                    self._manipulation_state
                    == ManipulationState.MANUALLY_ATTACHED
                ), (
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
        state: ManipulationState,
        object_id: Optional[str],
    ) -> PlanGoalT:
        """Get the planning goal for the given manipulation state.

        Looks up goal configuration from parameters, supporting per-object
        overrides. Goal types include offsets from object mount position,
        named target states, joint configurations, and Cartesian poses.
        Saves and restores return-path states (PRE_RETURN, PRE_DETACH)
        so the robot can retrace its fetch path.

        Args:
            state: ManipulationState to get goal for.
            object_id: Grid object ID (or None for IDLE/non-grid goals).

        Returns:
            Goal as PlanGoalT (RobotState, PoseStamped, str, or
            list[Constraints]).

        Raises:
            ParameterNotDeclaredException: If goal config not found.
            ValueError: If goal type is unknown or validation fails.
        """
        if (
            state
            in (ManipulationState.PRE_RETURN, ManipulationState.PRE_DETACH)
            and object_id in self._saved_return_states
        ):
            saved_state, goal = self._saved_return_states[object_id]
            assert state == saved_state
            return goal

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
        self, object_id: str, next_state: ManipulationState
    ) -> list[TrajectoryCacheKwargs] | None:
        """Plan and execute one step in the fetch/return state machine.

        Helper for _fetch_object_impl and _return_object_impl. Manages
        collision allowances during object-mount interactions, saves/
        restores return-path states, and attaches/detaches objects at
        the appropriate states. Uses linear planning pipeline for
        Cartesian mount approach/withdraw motions.

        Starting State: Current _manipulation_state (IDLE, RESETTED, or
            a fetch/return state).
        Resulting State: next_state (set by caller after this returns).

        Args:
            object_id: Grid object ID being manipulated.
            next_state: Target ManipulationState to transition toward.

        Returns:
            List of TrajectoryCacheKwargs if trajectory was planned
            (cacheable). None if trajectory was cached or request had
            use_cache=False.

        Raises:
            ExecutionInterruptedError, ExecutionStoppedError: On execution
                failure.
            StateTransitionError: If next_state is invalid for current
                state.
        """
        self.log(
            f"Transitioning from "
            f"{self._manipulation_state.name} to "
            f"{next_state.name} state for object {object_id}"
        )

        assert (
            self._manipulation_state
            in (ManipulationState.IDLE, ManipulationState.RESETTED)
            or self._manipulation_state in _FETCH_OR_RETURN_STATES
        )
        assert (
            next_state == ManipulationState.IDLE
            or next_state in _FETCH_OR_RETURN_STATES
        )

        if (
            next_state == ManipulationState.IDLE
            and self.param("skip_idle_on_return")
        ) or (
            next_state == ManipulationState.FETCHED
            and self.param("skip_fetched_on_fetch")
        ):
            return None

        goal = self._get_state_goal(next_state, object_id)
        request = PlanRequest(goal=goal)

        if next_state in (
            ManipulationState.PRE_ATTACH,
            ManipulationState.ATTACH,
            ManipulationState.POST_ATTACH,
            ManipulationState.POST_FETCH,
            ManipulationState.PRE_DETACH,
            ManipulationState.DETACH,
            ManipulationState.POST_DETACH,
            ManipulationState.POST_RETURN,
        ):
            if isinstance(goal, RobotState):
                assert next_state in (
                    ManipulationState.PRE_RETURN,
                    ManipulationState.PRE_DETACH,
                )
            else:
                assert isinstance(goal, PoseStamped)
                request.planning_pipeline = "linear"
                request.use_cache = False

        if next_state in (
            ManipulationState.DETACH,
            ManipulationState.POST_DETACH,
        ):
            request.velocity_scaling_factor = self.param(
                "detach_velocity_scaling_factor"
            )

        collisions_to_allow: list[tuple[str, str]] = []
        modified_collisions: list[tuple[str, str]] = []

        if next_state in (
            ManipulationState.PRE_ATTACH,
            ManipulationState.ATTACH,
            ManipulationState.POST_ATTACH,
            ManipulationState.DETACH,
            ManipulationState.POST_DETACH,
            ManipulationState.POST_RETURN,
        ):
            collisions_to_allow.extend(self.allowed_mount_collisions)

        match next_state:
            case (
                ManipulationState.PRE_ATTACH
                | ManipulationState.ATTACH
                | ManipulationState.POST_DETACH
                | ManipulationState.POST_RETURN
            ):
                collisions_to_allow.extend(
                    [(object_id, x) for x in self.touch_links]
                )
            case ManipulationState.POST_ATTACH | ManipulationState.DETACH:
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
            case ManipulationState.POST_ATTACH:
                self._saved_return_states[object_id] = (
                    ManipulationState.PRE_DETACH,
                    self._moveit.get_current_state(),
                )
            case ManipulationState.POST_FETCH:
                self._saved_return_states[object_id] = (
                    ManipulationState.PRE_RETURN,
                    self._moveit.get_current_state(),
                )
            case ManipulationState.PRE_RETURN | ManipulationState.PRE_DETACH:
                if object_id in self._saved_return_states:
                    del self._saved_return_states[object_id]
            case ManipulationState.ATTACH:
                self._moveit.move_collision_object(
                    object_id,
                    self._moveit.get_link_pose_stamped(self.attach_link),
                )
                self._moveit.attach_collision_object(
                    object_id,
                    self.attach_link,
                    touch_links=self.touch_links,
                )
            case ManipulationState.DETACH:
                self._moveit.detach_collision_object(object_id)
                self._moveit.move_collision_object(
                    object_id,
                    self._moveit.grid_objects_by_id[object_id].pose_stamped,
                )

        if not request.use_cache:
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
                ManipulationState.IDLE
                | ManipulationState.PRE_FETCH
                | ManipulationState.PRE_ATTACH
                | ManipulationState.ATTACH
                | ManipulationState.POST_ATTACH
                | ManipulationState.POST_FETCH
            ):
                next_state = ManipulationState(self._manipulation_state + 1)
            case ManipulationState.FETCHED:
                self.log(
                    "Already at FETCHED state, skipping fetch",
                    severity="WARN",
                )
                return
            case ManipulationState.RESETTED:
                next_state = ManipulationState.FETCHED
            case (
                ManipulationState.PRE_RETURN
                | ManipulationState.PRE_DETACH
                | ManipulationState.DETACH
                | ManipulationState.POST_DETACH
                | ManipulationState.POST_RETURN
            ):
                return_progress = (
                    self._manipulation_state - ManipulationState.PRE_RETURN
                )
                next_state = ManipulationState(
                    ManipulationState.POST_FETCH - return_progress
                )
            case unexpected if isinstance(unexpected, ManipulationState):
                raise StateTransitionError(
                    f"Cannot fetch object from current state: {unexpected.name}",
                    group_name=self.group_name,
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        assert (
            ManipulationState.PRE_FETCH <= next_state
            and next_state <= ManipulationState.FETCHED
        )

        cache_kwargs: list[TrajectoryCacheKwargs] = []

        # Iterate through the fetch states
        while self._manipulation_state != ManipulationState.FETCHED:
            try:
                kwargs = await self._fetch_or_return_transition(
                    object_id, next_state
                )
            except PlanningError:
                if next_state != ManipulationState.POST_FETCH:
                    raise

                # If post-fetch fails, try moving to fetched
                self.log(
                    "Failed to plan to POST_FETCH, skipping to FETCHED",
                    severity="WARN",
                )
                next_state = ManipulationState.FETCHED
                try:
                    kwargs = await self._fetch_or_return_transition(
                        object_id, next_state
                    )
                except (ExecutionInterruptedError, ExecutionStoppedError):
                    # If execution is interrupted here, we set the manipulation
                    # state to POST_FETCH so that we don't get stuck trying to
                    # return the object
                    self._manipulation_state = ManipulationState.POST_FETCH
                    raise

            self._manipulation_state = next_state
            next_state = ManipulationState(self._manipulation_state + 1)

            if self._manipulation_state != ManipulationState.IDLE:
                self._current_manipulation_id = object_id

            self._validate_manipulation_state()

            if cache_trajectories and kwargs is not None:
                cache_kwargs.extend(kwargs)

        # Cache all trajectories if requested
        if cache_trajectories and len(cache_kwargs) > 0:
            self.cache_trajectories(cache_kwargs)

    def _acquire_presentation_region(self, object_id) -> None:
        region_id: str = self.param("presentation_region.region_id")
        robot_collision_ids: list[str] = self.param(
            "presentation_region.robot_collision_ids"
        )
        robot_collision_ids.append(object_id)
        region_collision_ids: list[str] = self.param(
            "presentation_region.region_collision_ids"
        )
        self._moveit.acquire_exclusive_region(
            region_id,
            group_name=self.group_name,
            robot_collision_ids=robot_collision_ids,
            region_collision_ids=region_collision_ids,
        )

    def _release_presentation_region(self) -> None:
        region_id: str = self.param("presentation_region.region_id")
        self._moveit.release_exclusive_region(
            region_id, group_name=self.group_name
        )

    async def _present_object_impl(
        self, object_id: str, *, cache_trajectories=True
    ):
        self.log(f"Presenting object {object_id}")

        self._validate_target_object(object_id, expect_grid_object=True)

        match self._manipulation_state:
            case ManipulationState.FETCHED | ManipulationState.RESETTED:
                next_state = ManipulationState.PRESENTED
            case unexpected if isinstance(unexpected, ManipulationState):
                raise StateTransitionError(
                    f"Cannot present object "
                    f"from current state: {unexpected.name}",
                    group_name=self.group_name,
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        self._acquire_presentation_region(object_id)
        try:
            goal = self._get_state_goal(next_state, object_id)
            await self.plan_and_execute(
                goal=goal,
                cache_trajectories=cache_trajectories,
            )
        except BaseException:
            # State stays at FETCHED/RESETTED on failure, so the reset
            # path won't take the PRESENTED -> unpresent branch and won't
            # release the region for us. Release here to avoid stranding
            # the lock; recovery of the robot's pose is the reset path's
            # responsibility.
            try:
                self._release_presentation_region()
            except Exception as e:
                self.log(
                    f"Failed to release presentation region "
                    f"after present failure: {e}",
                    severity="ERROR",
                )
            raise

        self._manipulation_state = next_state

    async def _unpresent_object_impl(
        self, object_id: str, *, cache_trajectories=True
    ):
        self.log(f"Unpresenting object {object_id}")

        self._validate_target_object(object_id, expect_grid_object=True)

        match self._manipulation_state:
            case ManipulationState.PRESENTED:
                next_state = ManipulationState.NEEDS_RESET
            case unexpected if isinstance(unexpected, ManipulationState):
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

        # Release only after the move succeeds. If the move failed, state
        # stays at PRESENTED and reset_manipulation will re-call this
        # method, which will eventually release on success.
        self._release_presentation_region()

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
            case ManipulationState.NEEDS_RESET:
                assert self._last_pre_reset_state is None
                reset_interrupted = False
            case ManipulationState.PRE_RESET:
                assert self._last_pre_reset_state is not None
                reset_interrupted = True
            case unexpected if isinstance(unexpected, ManipulationState):
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
            self._manipulation_state = ManipulationState.RESETTED
            return

        cache_kwargs: list[TrajectoryCacheKwargs] = []

        # Plan and execute reset path with allowed collisions
        collisions_to_allow: list[tuple[str, str]] = []

        if config.object_allowed_collision_ids is not None:
            collisions_to_allow.extend(
                [(object_id, x) for x in config.object_allowed_collision_ids]
            )

        if config.additional_allowed_collisions is not None:
            collisions_to_allow.extend(config.additional_allowed_collisions)

        # Plan and execute to start goal using default planning pipeline
        # unless the previous reset attempt was interrupted and collisions
        # were allowed during the resetting
        pre_reset_complete = False
        if not reset_interrupted or len(collisions_to_allow) == 0:
            kwargs = await self.plan_and_execute(
                goal=config.start_goal,
                cache_trajectories=False,
            )
            if cache_trajectories and kwargs is not None:
                cache_kwargs.extend(kwargs)

            self._last_pre_reset_state = self._moveit.get_current_state()
            self._manipulation_state = ManipulationState.PRE_RESET
            pre_reset_complete = True

        # Plan and execute reset path with allowed collisions
        if len(collisions_to_allow) > 0:
            modified_collisions = self._moveit.allow_collision(
                *zip(*collisions_to_allow)
            )
        else:
            modified_collisions = []
        try:
            if not pre_reset_complete:
                assert self._last_pre_reset_state is not None
                await self.plan_and_execute(
                    goal=self._last_pre_reset_state,
                    planning_pipeline="linear",
                    use_cache=False,
                    cache_trajectories=False,
                )

            reset_request = copy(config.reset_request)
            reset_request.use_cache = False
            await self.plan_and_execute(
                config.reset_request, cache_trajectories=False
            )
        finally:
            if len(modified_collisions) > 0:
                self._moveit.disallow_collision(*zip(*modified_collisions))

        self._last_pre_reset_state = None
        self._manipulation_state = ManipulationState.RESETTED

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
            case ManipulationState.IDLE:
                self.log(
                    "Already at IDLE state, skipping return",
                    severity="WARN",
                )
                return
            case (
                ManipulationState.PRE_FETCH
                | ManipulationState.PRE_ATTACH
                | ManipulationState.ATTACH
                | ManipulationState.POST_ATTACH
                | ManipulationState.POST_FETCH
            ):
                fetch_progress = (
                    self._manipulation_state - ManipulationState.PRE_FETCH
                )
                next_state = ManipulationState(
                    ManipulationState.POST_RETURN - fetch_progress
                )
            case ManipulationState.FETCHED | ManipulationState.RESETTED:
                next_state = ManipulationState.PRE_RETURN
            case (
                ManipulationState.PRE_RETURN
                | ManipulationState.PRE_DETACH
                | ManipulationState.DETACH
                | ManipulationState.POST_DETACH
                | ManipulationState.POST_RETURN
            ):
                next_state = ManipulationState(
                    (self._manipulation_state + 1)
                    % (ManipulationState.POST_RETURN + 1)
                )
            case unexpected if isinstance(unexpected, ManipulationState):
                raise StateTransitionError(
                    f"Cannot return object "
                    f"from current state: {unexpected.name}",
                    group_name=self.group_name,
                )
            case unexpected:
                raise AssertionError(
                    f"Unexpected state type ({type(unexpected).__name__}) with value: {unexpected}"
                )

        assert next_state == ManipulationState.IDLE or (
            ManipulationState.PRE_RETURN <= next_state
            and next_state <= ManipulationState.POST_RETURN
        )

        cache_kwargs: list[TrajectoryCacheKwargs] = []

        # Iterate through the return states
        while self._manipulation_state != ManipulationState.IDLE:
            if (
                next_state
                in (ManipulationState.PRE_RETURN, ManipulationState.PRE_DETACH)
                and object_id in self._saved_return_states
            ):
                next_state, _ = self._saved_return_states[object_id]
                assert next_state in (
                    ManipulationState.PRE_RETURN,
                    ManipulationState.PRE_DETACH,
                )
                assert self._manipulation_state != next_state

            kwargs = await self._fetch_or_return_transition(
                object_id, next_state
            )

            self._manipulation_state = next_state
            next_state = ManipulationState(
                (self._manipulation_state + 1)
                % (ManipulationState.POST_RETURN + 1)
            )

            if self._manipulation_state == ManipulationState.IDLE:
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
                ManipulationState.IDLE
                | ManipulationState.MANUALLY_ATTACHED
                | ManipulationState.FETCHED
                | ManipulationState.PRESENTED
                | ManipulationState.RESETTED
            ):
                pass
            case unexpected if isinstance(unexpected, ManipulationState):
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
                ManipulationState.IDLE
                | ManipulationState.MANUALLY_ATTACHED
                | ManipulationState.FETCHED
                | ManipulationState.PRESENTED
                | ManipulationState.RESETTED
            ):
                pass
            case unexpected if isinstance(unexpected, ManipulationState):
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
            case ManipulationState.IDLE:
                pass
            case unexpected if isinstance(unexpected, ManipulationState):
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
        self._manipulation_state = ManipulationState.MANUALLY_ATTACHED

    async def _manually_detach_object_impl(self, object_id: str):
        self.log(f"Manually detaching object {object_id}")

        self._validate_target_object(object_id, expect_grid_object=False)

        match self._manipulation_state:
            case ManipulationState.MANUALLY_ATTACHED:
                pass
            case unexpected if isinstance(unexpected, ManipulationState):
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
        self._manipulation_state = ManipulationState.IDLE

    async def _test_object_attached(self):
        self.log("Testing if an object is attached")

        assert self._manipulation_state == ManipulationState.UNINITIALIZED

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
            self._manipulation_state = ManipulationState.IDLE
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
            self._manipulation_state = ManipulationState.UNINITIALIZED

    async def _test_object_attached_eef(self):
        self.log("Testing if an object is attached")

        assert self._manipulation_state == ManipulationState.UNINITIALIZED

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
            self._manipulation_state = ManipulationState.IDLE
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
            self._manipulation_state = ManipulationState.UNINITIALIZED

    def _load_persistent_state(self) -> PersistentState | None:
        path = self._persistent_state_path
        if not os.path.exists(path):
            return None

        try:
            with open(path, "rb") as f:
                payload: PersistentState = pickle.load(f)
            assert isinstance(payload, PersistentState)
            assert isinstance(payload.manipulation_state, ManipulationState)
            assert payload.manipulation_id is None or isinstance(
                payload.manipulation_id, str
            )
        except Exception as e:
            raise ValueError(
                f"Failed to load persistent manipulation state from "
                f"{path}: {e}. Please delete {path} and make sure the "
                f"robot is in idle state with no objects attached before "
                f"trying again"
            )
        os.remove(path)
        return payload

    def _save_persistent_state(self) -> None:
        if self._manipulation_state == ManipulationState.UNINITIALIZED:
            return

        path = self._persistent_state_path
        assert not os.path.exists(path)

        saved_return_state_positions: dict[
            str, tuple[ManipulationState, dict[str, float]]
        ] = {}

        for object_id, (
            manipulation_state,
            robot_state,
        ) in self._saved_return_states.items():
            positions = get_joint_group_positions(robot_state, self.group_name)
            saved_return_state_positions[object_id] = (
                manipulation_state,
                positions,
            )

        payload = PersistentState(
            self._manipulation_state,
            self._current_manipulation_id,
            saved_return_state_positions,
        )

        try:
            with open(path, "wb") as f:
                pickle.dump(payload, f, protocol=_PICKLE_PROTOCOL)
            self.log(
                f"Saved persistent state {payload} to {path}",
                severity="INFO",
            )
        except Exception as e:
            self.log(
                f"Failed to save persistent state to {path}: {e}",
                severity="ERROR",
            )

    async def _reset_manipulation_impl(
        self, *, reset_to_idle: bool, cache_trajectories: bool = True
    ):
        self.log("Resetting object manipulation")

        # Test if object is attached and
        if self._manipulation_state == ManipulationState.UNINITIALIZED:
            # persistent_state = self._load_persistent_state()
            # if persistent_state is None:
            #     self._manipulation_state = ManipulationState.IDLE
            #     self._current_manipulation_id = None
            # else:
            #     self._manipulation_state = persistent_state.manipulation_state
            #     self._current_manipulation_id = (
            #         persistent_state.manipulation_id
            #     )
            #     if self._manipulation_state == ManipulationState.PRESENTED:
            #         self._acquire_presentation_region(
            #             self._current_manipulation_id
            #         )

            if self._simulate or not self.param("test_object_attached.enable"):
                self._current_manipulation_id = self._init_attached_object()
            else:
                self._current_manipulation_id = (
                    await self._test_object_attached()
                )

            # TODO: Check if robot is in presentation region
            if self._current_manipulation_id is None:
                self._manipulation_state = ManipulationState.IDLE
            else:
                self._manipulation_state = ManipulationState.NEEDS_RESET

        # Reset and return object if attached
        if self._manipulation_state not in (
            ManipulationState.IDLE,
            ManipulationState.MANUALLY_ATTACHED,
        ):
            object_id = self._current_manipulation_id
            assert object_id is not None

            # Handle case where robot may be stuck in detach state
            # if self._manipulation_state == ManipulationState.DETACH:
            #     await self._fetch_or_return_transition(object_id, ManipulationState.ATTACH)
            #     self._manipulation_state = ManipulationState.ATTACH
            #     await self._fetch_or_return_transition(
            #         object_id, ManipulationState.POST_ATTACH
            #     )
            #     self._manipulation_state = ManipulationState.POST_ATTACH
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

            # Unpresent object if needed
            if self._manipulation_state == ManipulationState.PRESENTED:
                await self._unpresent_object_impl(
                    object_id, cache_trajectories=cache_trajectories
                )

            # Reset object if needed
            if self._manipulation_state in (
                ManipulationState.NEEDS_RESET,
                ManipulationState.PRE_RESET,
            ):
                try:
                    await self._reset_object_impl(
                        object_id, cache_trajectories=cache_trajectories
                    )
                except PlanningError:
                    goal = self._get_state_goal(
                        ManipulationState.IDLE, object_id=None
                    )
                    await self.plan_and_execute(
                        goal=goal, cache_trajectories=False
                    )
                    await self._reset_object_impl(
                        object_id, cache_trajectories=cache_trajectories
                    )

            # Return object
            assert (
                self._manipulation_state
                in (ManipulationState.IDLE, ManipulationState.RESETTED)
                or self._manipulation_state in _FETCH_OR_RETURN_STATES
            )
            try:
                await self._return_object_impl(
                    object_id, cache_trajectories=cache_trajectories
                )
            except PlanningError:
                if self._manipulation_state == ManipulationState.POST_RETURN:
                    # Almost made it, good enough
                    self._manipulation_state = ManipulationState.IDLE
                    self._current_manipulation_id = None
                else:
                    # "Unreturn" object and move to idle to try and get a better plan
                    await self._fetch_object_impl(
                        object_id, cache_trajectories=False
                    )
                    goal = self._get_state_goal(
                        ManipulationState.IDLE, object_id=None
                    )
                    await self.plan_and_execute(
                        goal=goal, cache_trajectories=False
                    )
                    await self._return_object_impl(
                        object_id, cache_trajectories=cache_trajectories
                    )

        if reset_to_idle:
            goal = self._get_state_goal(ManipulationState.IDLE, object_id=None)
            await self._plan_and_move_impl(
                goal=goal, cache_trajectories=cache_trajectories
            )

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
        """Pick up an object from its grid mount (IDLE → FETCHED).

        Executes fetch state machine: PRE_FETCH → PRE_ATTACH → ATTACH
        → POST_ATTACH → POST_FETCH → FETCHED. If already partially
        through fetch, resumes from current state. If in return states,
        reverses direction. Caches only after full success.

        Starting State: IDLE, RESETTED, or any fetch/return state.
        Resulting State: FETCHED (or partially through if exception).

        Args:
            object_id: Reachable grid object ID.
            cache_trajectories: Cache planned trajectories after fetch.

        Raises:
            ValueError: object_id not reachable or not a grid object.
            StateTransitionError: Invalid starting state.
            PlanningError: Planning failed (e.g., POST_FETCH fallback to
                FETCHED may occur).
            ExecutionError: Execution failed (state advanced before
                exception if applicable).
        """
        await self._fetch_object_impl(
            object_id, cache_trajectories=cache_trajectories
        )

    @validate_and_lock
    async def present_object(self, object_id: str, *, cache_trajectories=True):
        """Move fetched object to presentation pose (FETCHED → PRESENTED).

        Acquires exclusive presentation region lock, moves to presentation
        pose, and releases lock only on success. Failure leaves state at
        FETCHED and lock not released; reset_manipulation will retry.

        Starting State: FETCHED or RESETTED.
        Resulting State: PRESENTED.

        Args:
            object_id: Reachable grid object ID being presented.
            cache_trajectories: Cache planned trajectory after present.

        Raises:
            StateTransitionError: Not in FETCHED or RESETTED state.
            PlanningError: Planning failed.
            ExecutionError: Execution failed; state reverts to FETCHED,
                lock not released.
        """
        await self._present_object_impl(
            object_id, cache_trajectories=cache_trajectories
        )

    @validate_and_lock
    async def unpresent_object(
        self, object_id: str, *, cache_trajectories=True
    ):
        """Return presented object to fetch pose (PRESENTED → NEEDS_RESET).

        Releases exclusive presentation region lock after successful move.

        Starting State: PRESENTED.
        Resulting State: NEEDS_RESET.

        Args:
            object_id: Reachable grid object ID being unpresented.
            cache_trajectories: Cache planned trajectory after unpresent.

        Raises:
            StateTransitionError: Not in PRESENTED state.
            PlanningError: Planning failed.
            ExecutionError: Execution failed; state reverts to PRESENTED,
                lock not released.
        """
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
        """Execute user-defined reset path (NEEDS_RESET/PRE_RESET → RESETTED).

        Applies collision allowances defined in ObjectResetConfig and
        executes multi-waypoint reset sequence. If interrupted mid-reset,
        can resume by retrying reset_object. On failure, attempts fallback
        to IDLE pose then retries.

        Starting State: NEEDS_RESET or PRE_RESET.
        Resulting State: RESETTED.

        Args:
            object_id: Grid object ID to reset.
            cache_trajectories: Cache planned trajectories after reset.

        Raises:
            StateTransitionError: Not in NEEDS_RESET or PRE_RESET state.
            PlanningError: Planning failed twice (after fallback retry).
            ExecutionError: Execution failed.
        """
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
        """Return held object to mount and detach (FETCHED/RESETTED → IDLE).

        Executes return state machine in reverse of fetch: PRE_RETURN →
        PRE_DETACH → DETACH → POST_DETACH → POST_RETURN → IDLE. If
        intermediate state, resumes from current position. On planning
        failure, attempts fallback strategies: skip to IDLE, or fetch
        object back and retry return with IDLE intermediate pose.

        Starting State: IDLE (noop), FETCHED, RESETTED, or any fetch/
            return state.
        Resulting State: IDLE.

        Args:
            object_id: Reachable grid object ID being returned.
            cache_trajectories: Cache planned trajectories after return.

        Raises:
            StateTransitionError: Invalid starting state.
            PlanningError: Planning failed even after fallback strategies.
            ExecutionError: Execution failed; state partially advanced.
        """
        await self._return_object_impl(
            object_id, cache_trajectories=cache_trajectories
        )

    @validate_and_lock
    async def manually_attach_object(self, object_id: str):
        """Manually attach a non-grid object to end effector.

        Loads mesh from manually_attach.mesh_dir, adds collision object,
        and attaches it. Used for objects not in the grid mount.

        Starting State: IDLE.
        Resulting State: MANUALLY_ATTACHED.

        Args:
            object_id: Non-grid object ID (must have mesh in
                manually_attach.mesh_dir).

        Raises:
            StateTransitionError: Not in IDLE state.
            FileNotFoundError: Mesh not found for object_id.
            ValueError: Multiple meshes found for object_id.
        """
        await self._manually_attach_object_impl(object_id)

    @validate_and_lock
    async def manually_detach_object(self, object_id: str):
        """Manually detach a manually-attached non-grid object.

        Starting State: MANUALLY_ATTACHED.
        Resulting State: IDLE.

        Args:
            object_id: Non-grid object ID being detached.

        Raises:
            StateTransitionError: Not in MANUALLY_ATTACHED state.
        """
        await self._manually_detach_object_impl(object_id)

    @validate_and_lock
    async def plan_and_move(
        self,
        request: PlanRequest | ConcatPlanRequest | None = None,
        *,
        cache_trajectories: bool = True,
        **kwargs: Any,
    ) -> None:
        """Plan and execute a trajectory without changing manipulation state.

        Valid while holding fetched/presented/reset objects or no object.
        Does not update _manipulation_state; only moves the robot.

        Starting State: IDLE, MANUALLY_ATTACHED, FETCHED, PRESENTED,
            or RESETTED.
        Resulting State: Unchanged.

        Args:
            request: PlanRequest/ConcatPlanRequest or None.
            cache_trajectories: Cache planned trajectory.
            **kwargs: Additional arguments to plan() (goal, goals, etc.).

        Raises:
            StateTransitionError: Invalid current state.
            PlanningError: Planning failed.
            ExecutionError: Execution failed.
        """
        await self._plan_and_move_impl(
            request, cache_trajectories=cache_trajectories, **kwargs
        )

    @validate_and_lock
    async def move(self, trajectory: RobotTrajectory | list[RobotTrajectory]):
        """Execute a pre-planned trajectory without replanning.

        Valid while holding fetched/presented/reset objects or no object.
        Does not update _manipulation_state.

        Starting State: IDLE, MANUALLY_ATTACHED, FETCHED, PRESENTED,
            or RESETTED.
        Resulting State: Unchanged.

        Args:
            trajectory: Trajectory or list of trajectories to execute.

        Raises:
            StateTransitionError: Invalid current state.
            ExecutionError: Execution failed.
        """
        await self._move_impl(trajectory)

    ###########################################################################
    ########## Reset ##########################################################
    ###########################################################################

    @validate_and_lock
    async def reset_manipulation(
        self, *, reset_to_idle: bool = False, cache_trajectories: bool = True
    ) -> None:
        """Recover manipulation state and optionally move to idle.

        Tests if an object is attached (or loads persistent state on
        first call), then unpresents/resets/returns it if needed. If
        reset_to_idle, also moves to the idle configuration.

        Starting State: UNINITIALIZED (first call) or any valid state.
        Resulting State: IDLE or UNINITIALIZED (if no object attached).

        Args:
            reset_to_idle: Move to idle configuration after returning
                object (if attached).
            cache_trajectories: Cache planned trajectories during reset.

        Raises:
            PlanningError: Planning failed even after fallback strategies
                (returns to IDLE pose, then retries).
            ExecutionError: Execution failed.
        """
        await self._reset_manipulation_impl(
            reset_to_idle=reset_to_idle, cache_trajectories=cache_trajectories
        )

    # def destroy_interface(self):
    #     self._save_persistent_state()
    #     return super().destroy_interface()
