"""Main commander node orchestrating all experimental apparatus.

This module provides the Commander node, the central control point for the
tabletop experimental rig. It aggregates all interface components and provides
a high-level API for running behavioral experiments.

The Commander coordinates:
- Robot motion planning and execution (MoveIt interface)
- Safety monitoring (Teensy interface)
- Subject feedback (sound, reward, smartglass)
- Response time measurement (Flic buttons)
- Gaze tracking (Eyelink interface)
- Robot state recovery (dashboard interface)

The node supports running custom experiment coroutines that can be
specified at launch time, enabling flexible experiment protocols.

Usage:
    ros2 run tabletop_rig commander --ros-args --params-file config.yaml

    With custom experiment:
    ros2 run tabletop_rig commander --coro-module my_experiments \\
        --coro-name run_trial --coro-config config.yaml

Example:
    async with Commander() as commander:
        await commander.fetch_object("object_1")
        await commander.present_object("object_1")
        response_time = await commander.flic_response_time(
            "object_1", timeout=10.0
        )
        await commander.return_object("object_1")
"""

import argparse
import asyncio
import concurrent.futures
import functools
import importlib
import inspect
import threading
import traceback
from collections.abc import Callable, Coroutine, Mapping
from types import TracebackType
from typing import Any, Literal, Optional, Self

import rclpy
import rclpy.utilities
from mingus.containers import Note
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from rclpy.executors import (
    Executor,
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.signals import SignalHandlerOptions
from tabletop_interfaces.msg import TeensySensor
from ur_dashboard_msgs.msg import SafetyMode

from tabletop_rig.exceptions import (
    ActionClientError,
    ExecutionError,
    ExecutionInterruptedError,
    ExecutionRejectedError,
    ExecutionStoppedError,
    ManipulationContextExitedError,
    MoveitRecoverableError,
    NotSafeToExecuteError,
    RigExecutionSafetyError,
    RigSafetyError,
    ServiceClientError,
    URSafetyViolationError,
)
from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.interfaces.eyelink import EyelinkInterface
from tabletop_rig.interfaces.flic import FlicInterface
from tabletop_rig.interfaces.moveit.moveit import MoveItInterface
from tabletop_rig.interfaces.moveit.object_manipulation import (
    ManipulationState,
    ObjectManipulationInterface,
)
from tabletop_rig.interfaces.sound import SoundInterface
from tabletop_rig.interfaces.teensy import TeensyInterface
from tabletop_rig.interfaces.ur import URInterface
from tabletop_rig.nodes.base import BaseNode

_FATAL_UR_SAFETY_MODES = {
    getattr(SafetyMode, name): name
    for name in (
        "PROTECTIVE_STOP",
        "SAFEGUARD_STOP",
        "SYSTEM_EMERGENCY_STOP",
        "ROBOT_EMERGENCY_STOP",
        "VIOLATION",
        "FAULT",
        "AUTOMATIC_MODE_SAFEGUARD_STOP",
        "SYSTEM_THREE_POSITION_ENABLING_STOP",
    )
    if hasattr(SafetyMode, name)
}


def ensure_context(fn):
    if inspect.iscoroutinefunction(fn):

        @functools.wraps(fn)
        async def async_wrapper(
            self: "ManipulationContextManager | Commander", *args, **kwargs
        ):
            if not self._entered_context:
                raise RuntimeError(
                    f"{type(self).__name__} context manager not yet entered."
                )
            if self._exited_context:
                raise RuntimeError(
                    f"{type(self).__name__} context manager already exited."
                )
            return await fn(self, *args, **kwargs)

        return async_wrapper
    else:

        @functools.wraps(fn)
        def wrapper(
            self: "ManipulationContextManager | Commander", *args, **kwargs
        ):
            if not self._entered_context:
                raise RuntimeError(
                    f"{type(self).__name__} context manager not yet entered."
                )
            if self._exited_context:
                raise RuntimeError(
                    f"{type(self).__name__} context manager already exited."
                )
            return fn(self, *args, **kwargs)

        return wrapper


def handle_interruptions(coro_fn):
    """Decorator for methods requiring safety-checked robot execution.

    Wraps coroutines that execute robot motions to handle common errors:
    - NotSafeToExecuteError: Locks arms and waits for safety
    - Laser-related execution errors: Restores the real UR controller and
      retries from measured state
    - Non-laser real execution errors: Latches a fatal rig-wide fault

    Args:
        coro_fn: Async method to wrap.

    Returns:
        Wrapped method with retry logic.
    """

    @functools.wraps(coro_fn)
    async def wrapper(self: "ManipulationContextManager", *args, **kwargs):
        self._raise_if_rig_safety_faulted()
        max_attempts = self.param("interruptions.max_attempts")
        if max_attempts < 1:
            raise ValueError(
                "'interruptions.max_attempts' parameter must be at least 1"
            )
        remaining = max_attempts

        async def wait_until_safe(phase: str) -> None:
            """Apply the safety gate immediately before each motion attempt."""
            self._raise_if_rig_safety_faulted()
            if self._safe_to_execute_condition():
                return
            assert self._manipulator.presentation_region_active
            self.log(
                f"Not safe to execute {phase} '{coro_fn.__name__}'. "
                f"Locking arms and waiting for safety.",
                severity="WARN",
            )
            await self._teensy.lock_arms_and_wait(
                condition=self._safe_to_execute_condition
            )
            self._raise_if_rig_safety_faulted()

        excs: list[Exception] = []
        while remaining > 0:
            await self._recover_after_laser_stop()
            # The beam can break while the UR dashboard is being restored or
            # between attempts. Re-apply the stable-clear gate immediately
            # before every retry instead of relying on the original call's
            # one-time check.
            await wait_until_safe("before attempting")
            attempt_laser_generation = self._laser_break_generation_value()
            try:
                return await coro_fn(self, *args, **kwargs)
            except ExecutionError as e:
                # A real controller execution failure leaves the measured arm
                # state indeterminate. Stop the whole rig instead of retrying
                # this motion or running cleanup paths from a stale state. A
                # laser-related failure is recoverable even when the beam has
                # already cleared: the generation counter preserves that
                # evidence independently of the controller-restart flag.
                self._raise_if_rig_safety_faulted()
                laser_event_observed = (
                    self._laser_stop_pending()
                    or not self._safe_to_execute_condition()
                    or self._laser_break_generation_value()
                    != attempt_laser_generation
                )
                if not self._simulate and not laser_event_observed:
                    raise self._rig_execution_fault_handler(e) from e
                excs.append(e)
                self.log(
                    f"Caught exception while running '{coro_fn.__name__}' | "
                    f"{type(e).__name__}: {e}",
                    severity="WARN",
                )
                self.log(
                    f"Traceback: \n {' '.join(traceback.format_tb(e.__traceback__))}",
                    severity="DEBUG",
                )

                remaining -= 1

                if remaining <= 0:
                    break

                await self._recover_after_laser_stop()
                if isinstance(
                    e,
                    (
                        NotSafeToExecuteError,
                        ExecutionInterruptedError,
                        ExecutionStoppedError,
                    ),
                ):
                    await wait_until_safe("during interruption handling for")

                if isinstance(
                    e,
                    (
                        ExecutionRejectedError,
                        ExecutionInterruptedError,
                        ExecutionStoppedError,
                    ),
                ):
                    ready = await self._ur.is_ready()
                    if not ready:
                        self.log(
                            f"Dashboard not ready during interrupting handling "
                            f"while running '{coro_fn.__name__}'. Resetting UR "
                            f"interface.",
                            severity="WARN",
                        )
                        await self._ur.reset()

                self.log(
                    f"Reset successful, retrying '{coro_fn.__name__}' {remaining} more times",
                    severity="WARN",
                )

        if len(excs) == 1:
            raise excs[0]
        if len(excs) > 1:
            raise ExceptionGroup(
                f"Failed to reset commander after {max_attempts} attempts",
                excs,
            )

    return wrapper


class ManipulationContextManager(BaseInterface):
    """Manages robot manipulation state for a single arm with safety.

    Aggregates the MoveIt interface, UR dashboard interface, and
    object manipulation interface for coordinated control of a single
    robot arm. Handles safety checks and error recovery.

    Attributes:
        _ur: UR robot interface.
        _manipulator: Object manipulation interface.
        _teensy: Safety and I/O interface (shared).
        _initial_reset: Whether the initial reset has been performed.
        _entered_context: Whether the context manager has been entered.
        _exited_context: Whether the context manager has been exited.
    """

    def __init__(
        self,
        node: BaseNode,
        name: str,
        *,
        simulate: bool,
        moveit_interface: MoveItInterface,
        teensy_interface: TeensyInterface,
        rig_safety_fault_getter: Callable[[], RigSafetyError | None],
        rig_execution_fault_handler: Callable[
            [ExecutionError], RigSafetyError
        ],
        ur_interface_name: str,
        manipulation_interface_name: str,
        parameter_fallback_prefix: Optional[str] = None,
    ):
        self._ur = URInterface(
            node,
            ur_interface_name,
            simulate=simulate,
            parameter_fallback_prefix="common_ur_interface",
        )
        self._manipulator = ObjectManipulationInterface(
            node,
            manipulation_interface_name,
            simulate=simulate,
            moveit_interface=moveit_interface,
            safe_to_execute_condition=self._safe_to_execute_condition,
            parameter_fallback_prefix="common_manipulation_interface",
        )
        super().__init__(
            node, name, parameter_fallback_prefix=parameter_fallback_prefix
        )

        self._teensy = teensy_interface
        self._simulate = simulate
        self._rig_safety_fault_getter = rig_safety_fault_getter
        self._rig_execution_fault_handler = rig_execution_fault_handler
        self._laser_stop_requested = threading.Event()
        self._laser_state_lock = threading.Lock()
        self._laser_is_broken: bool | None = None
        self._laser_break_generation = 0

        self._initial_reset = False
        self._entered_context = False

        # For compatibility with ensure_context, unused because this context
        # manager is reentrant
        self._exited_context = False

    def _safe_to_execute_condition(self) -> bool:
        return self._rig_safety_fault_getter() is None and (
            not self._manipulator.presentation_region_active
            or self._teensy.safe_to_execute
        )

    def _raise_if_rig_safety_faulted(self) -> None:
        fault = self._rig_safety_fault_getter()
        if fault is not None:
            raise fault

    def _laser_stop_pending(self) -> bool:
        """Whether this arm's UR program was stopped for a laser event."""
        return self._laser_stop_requested.is_set()

    def observe_laser_state(self, is_broken: bool) -> int:
        """Record every clear-to-broken edge and return its generation.

        The generation is deliberately independent of whether this arm is
        executing. A break can occur while its UR program is being restored,
        when there is no active trajectory for the callback to cancel.
        """
        with self._laser_state_lock:
            if is_broken and self._laser_is_broken is not True:
                self._laser_break_generation += 1
            self._laser_is_broken = is_broken
            return self._laser_break_generation

    def _laser_break_generation_value(self) -> int:
        """Return the latest observed laser-break generation."""
        with self._laser_state_lock:
            return self._laser_break_generation

    def _complete_laser_recovery_if_unchanged(self, generation: int) -> bool:
        """Acknowledge recovery only if no newer break was observed."""
        with self._laser_state_lock:
            if self._laser_break_generation != generation:
                return False
            self._laser_stop_requested.clear()
            return True

    def request_laser_safety_stop(self) -> bool:
        """Stop this arm once and remember that dashboard recovery is needed.

        Returns True only for the callback that issued the stop, preventing the
        100 Hz Teensy stream from flooding the dashboard service and logs.
        """
        if self._laser_stop_requested.is_set():
            return False
        self._laser_stop_requested.set()
        if self._simulate:
            self._manipulator.stop_execution()
        else:
            self._ur.stop_program()
        return True

    async def _recover_after_laser_stop(self) -> None:
        """Restore the UR external-control program after the laser clears.

        The laser callback intentionally stops the dashboard program, which
        deactivates the scaled trajectory controller. Waiting for the beam to
        clear is therefore insufficient: the program must be replayed before
        the next trajectory goal is submitted.
        """
        max_recovery_attempts = self.param(
            "interruptions.max_controller_recovery_attempts"
        )
        if max_recovery_attempts < 1:
            raise ValueError(
                "'interruptions.max_controller_recovery_attempts' parameter "
                "must be at least 1"
            )

        recovery_attempt = 0
        while self._laser_stop_requested.is_set():
            if not self._safe_to_execute_condition():
                await self._teensy.lock_arms_and_wait(
                    condition=self._safe_to_execute_condition
                )
            self._raise_if_rig_safety_faulted()

            # Snapshot after the stable-clear wait. If the generation changes
            # at any point during dashboard recovery, the restored program is
            # stopped again and the complete recovery is repeated.
            recovery_generation = self._laser_break_generation_value()

            if self._simulate:
                if self._safe_to_execute_condition() and (
                    self._complete_laser_recovery_if_unchanged(
                        recovery_generation
                    )
                ):
                    return
                continue

            recovery_attempt += 1
            if recovery_attempt > max_recovery_attempts:
                error = ExecutionStoppedError(
                    "Laser controller recovery did not remain safe and ready "
                    f"after {max_recovery_attempts} attempts",
                    group_name=self._manipulator.group_name,
                )
                raise self._rig_execution_fault_handler(error) from error

            self.log(
                "Safety laser clear and stable; verifying UR program and "
                "controller readiness before resuming motion",
                severity="WARN",
            )
            if not await self._ur.is_ready():
                await self._ur.reset()

            controller_ready = await self._ur.is_ready()
            safety_ready = self._safe_to_execute_condition()
            generation_unchanged = (
                self._laser_break_generation_value() == recovery_generation
            )
            if (
                controller_ready
                and safety_ready
                and generation_unchanged
                and self._complete_laser_recovery_if_unchanged(
                    recovery_generation
                )
            ):
                return

            # reset() may have replayed external_control.urp before a newer
            # break was observed. Stop it again before waiting/retrying so no
            # trajectory can be accepted from a stale recovery cycle.
            if controller_ready:
                self._ur.stop_program()
            reasons = []
            if not controller_ready:
                reasons.append("UR controller not ready")
            if not safety_ready:
                reasons.append("safety conditions not stable")
            if not generation_unchanged:
                reasons.append("newer laser break observed")
            self.log(
                "Laser recovery must restart: " + ", ".join(reasons),
                severity="WARN",
            )

    @property
    @ensure_context
    def current_manipulation_id(self) -> str | None:
        """Get the ID of the currently manipulated object.

        Returns:
            Object ID if manipulating, None otherwise.
        """
        return self._manipulator.current_manipulation_id

    @property
    @ensure_context
    def manipulation_state(self) -> ManipulationState:
        """Get the current manipulation state (idle/fetched/presented).

        Returns:
            Current ManipulationState enum value.
        """
        return self._manipulator.manipulation_state

    @ensure_context
    async def manually_attach_object(self, object_id: str) -> None:
        """Attach a non-grid object to the robot end-effector.

        Used when the robot already has an object grasped and the planning
        scene needs updating.

        Args:
            object_id: ID of the collision object to attach.
        """
        await self._manipulator.manually_attach_object(object_id)

    @ensure_context
    async def manually_detach_object(self, object_id: str) -> None:
        """Detach a non-grid object from the robot end-effector

        Used when a previously manually attached object has been
        detached by hand and the planning scene needs updating.

        Args:
            object_id: ID of the currently attached collision object to detach.
        """
        await self._manipulator.manually_detach_object(object_id)

    @ensure_context
    async def plan(self, *args, **kwargs) -> RobotTrajectory:
        """Plan a trajectory to the specified goal.

        Generates a motion plan without executing it. Useful for
        previewing trajectories or caching plans for later execution.

        Args:
            *args: Positional arguments passed to MoveItInterface.plan.
            **kwargs: Keyword arguments passed to MoveItInterface.plan.

        Returns:
            Planned trajectory, or None if planning failed.
        """
        trajectory, _ = await self._manipulator.plan(*args, **kwargs)
        return trajectory

    @ensure_context
    @handle_interruptions
    async def move(self, *args, **kwargs) -> None:
        """Execute a previously planned trajectory.

        Args:
            *args: Positional arguments passed to MoveItInterface.execute.
            **kwargs: Keyword arguments passed to MoveItInterface.execute.

        Raises:
            ExecutionError: If trajectory execution fails.
            NotSafeToExecuteError: If safety conditions not met.
        """
        await self._manipulator.move(*args, **kwargs)

    @ensure_context
    @handle_interruptions
    async def plan_and_move(self, *args, **kwargs) -> None:
        """Plan and execute a motion to the specified goal.

        Combines planning and execution in a single call. Uses the
        trajectory cache when available.

        Args:
            *args: Positional arguments passed to MoveItInterface.
            **kwargs: Keyword arguments passed to MoveItInterface.

        Raises:
            PlanningError: If motion planning fails.
            ExecutionError: If trajectory execution fails.
            NotSafeToExecuteError: If safety conditions not met.
        """
        await self._manipulator.plan_and_move(*args, **kwargs)

    @ensure_context
    @handle_interruptions
    async def fetch_object(self, object_id: str):
        """Fetch an object from its mount.

        The robot moves to the object's mount, attaches the object, and moves
        to the object's stagin area.

        Args:
            object_id: The ID of the object to fetch

        Raises:
            ValueError: If the object ID is not a valid collision object
            PlanningError: If the planning fails
            ExecutionError: If the execution fails
        """
        await self._manipulator.fetch_object(object_id)

    @ensure_context
    @handle_interruptions
    async def present_object(self, object_id: str):
        """Move object from staging area to presentation area.

        Args:
            object_id: The ID of the object to present.

        Raises:
            PlanningError: If motion planning fails.
            ExecutionError: If motion execution fails.
            NotSafeToExecuteError: If safety conditions not met.
        """
        await self._manipulator.present_object(object_id)

    @ensure_context
    @handle_interruptions
    async def unpresent_object(self, object_id: str):
        """Move object from presentation area back to staging area.

        Args:
            object_id: The ID of the object to unpresent.

        Raises:
            PlanningError: If motion planning fails.
            ExecutionError: If motion execution fails.
            NotSafeToExecuteError: If safety conditions not met.
        """
        await self._manipulator.unpresent_object(object_id)

    @ensure_context
    @handle_interruptions
    async def reset_object(self, object_id: str):
        """Reset the object using its associated reset configuration.

        Args:
            object_id: The ID of the object to reset.

        Raises:
            RuntimeError: If exactly one object is not attached.
            PlanningError: If motion planning fails.
            ExecutionError: If motion execution fails.
            NotSafeToExecuteError: If safety conditions not met.
        """
        await self._manipulator.reset_object(object_id)

    @ensure_context
    @handle_interruptions
    async def return_object(self, object_id: str):
        """Return the currently attached object to its mount.

        Raises:
            RuntimeError: If exactly one object is not attached
            PlanningError: If the planning fails
            ExecutionError: If the execution fails
        """
        await self._manipulator.return_object(object_id)

    @ensure_context
    @handle_interruptions
    async def reset_manipulation(self, *, reset_to_idle: bool = False) -> None:
        """Reset robot manipulation state.

        If manipulating a grid object (aka not a manually attached object),
        reset and return the grid object to its mount if necessary.

        Then, move to idle position.
        """
        await self._manipulator.reset_manipulation(reset_to_idle=reset_to_idle)

    async def _reset(
        self,
        *,
        reset_to_idle: bool,
    ) -> None:
        """Reset the robot to a known good state.

        Performs a full reset sequence: locks arms, waits for safety,
        resets the UR dashboard, and optionally moves to a goal pose.
        Retries automatically on recoverable errors.

        Args:
            reset_to_idle: Whether or not to return to idle state after
                resetting

        Raises:
            asyncio.TimeoutError: If reset not completed within timeout.
        """
        self.log("Resetting manipulation")
        self._raise_if_rig_safety_faulted()

        max_attempts = self.param("reset.max_attempts")
        if max_attempts < 1:
            raise ValueError(
                "'reset.max_attempts' parameter must be at least 1"
            )

        remaining = max_attempts

        excs: list[Exception] = []
        while remaining > 0:
            try:
                self._raise_if_rig_safety_faulted()
                if self._manipulator.presentation_region_active:
                    await self._teensy.set_smartglass(reveal=False)

                if not self._safe_to_execute_condition():
                    assert self._manipulator.presentation_region_active
                    self.log(
                        "Not safe to execute during manipulation context reset. "
                        "Locking arms and waiting for safety.",
                        severity="WARN",
                    )
                    await self._teensy.lock_arms_and_wait(
                        condition=self._safe_to_execute_condition
                    )

                ready = await self._ur.is_ready()
                if not ready:
                    self.log(
                        "Dashboard not ready during manipulation context reset. "
                        "Resetting UR interface for before retrying",
                        severity="WARN",
                    )
                    await self._ur.reset()

                await self._manipulator.reset_manipulation(
                    reset_to_idle=reset_to_idle, cache_trajectories=False
                )

                return
            except (
                ServiceClientError,
                ActionClientError,
                MoveitRecoverableError,
            ) as e:
                assert (
                    not isinstance(e, MoveitRecoverableError)
                    or e.group_name == self._manipulator.group_name
                )

                excs.append(e)
                self.log(
                    f"Caught exception while resetting manipulation context | "
                    f"{type(e).__name__}: {e}",
                    severity="WARN",
                )
                self.log(
                    f"Traceback: \n {' '.join(traceback.format_tb(e.__traceback__))}",
                    severity="DEBUG",
                )

                remaining -= 1

                if remaining > 0:
                    self.log(f"Retrying {remaining} more times")

        if len(excs) == 1:
            raise excs[0]
        elif len(excs) > 1:
            raise ExceptionGroup(
                f"Failed to reset manipulation after {remaining} attempts",
                excs,
            )

        assert False

    def _parse_group_names(self, exc: Exception) -> set[str]:
        group_names: set[str] = set()
        if isinstance(exc, MoveitRecoverableError):
            group_names.add(exc.group_name)
        elif isinstance(exc, ExceptionGroup):
            for e in exc.exceptions:
                group_names |= self._parse_group_names(e)
        else:
            assert False

        return group_names

    async def __aenter__(self) -> Self:
        """Enter the async context manager.

        Initializes MoveIt and resets the commander to the idle state.

        Returns:
            The Commander instance.
        """
        self.log("Entering manipulation context manager", severity="DEBUG")

        if self._entered_context:
            raise RuntimeError(
                "Cannot reenter manipulation context until it has been exited"
            )

        if not self._initial_reset:
            await self._reset(reset_to_idle=False)
            self._initial_reset = True

        self._entered_context = True

        return self

    async def __aexit__(
        self,
        exc_type: Optional[type[BaseException]],
        exc_value: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> bool | None:
        """Exit the async context manager.

        Handles recoverable errors by resetting the commander.
        Always cleans up MoveIt resources.

        Args:
            exc_type: Exception type if an exception occurred.
            exc_value: Exception instance if an exception occurred.
            exc_tb: Traceback if an exception occurred.

        Returns:
            True if a recoverable error was handled, False otherwise.
        """
        try:
            self.log("Exiting manipulation context manager", severity="DEBUG")

            if exc_type is not None:
                unrecoverable_excs: ExceptionGroup | None = None
                if isinstance(exc_value, ExceptionGroup):
                    exc_value, unrecoverable_excs = exc_value.split(
                        MoveitRecoverableError
                    )
                    if exc_value is None:
                        return False
                elif not isinstance(exc_value, MoveitRecoverableError):
                    return False

                self.log(
                    f"Caught recoverable exception(s) in manipulation context | "
                    f"{exc_type.__name__}: {exc_value}",
                    severity="ERROR",
                )
                self.log(
                    f"Traceback: \n {' '.join(traceback.format_tb(exc_tb))}",
                    severity="DEBUG",
                )
                if isinstance(exc_value, ExceptionGroup):
                    self.log("ExceptionGroup subexceptions:", severity="ERROR")
                    for e in exc_value.exceptions:
                        self.log(f"{type(e).__name__}: {e}", severity="ERROR")
                        self.log(
                            f"Traceback: \n {' '.join(traceback.format_tb(e.__traceback__))}",
                            severity="DEBUG",
                        )

                group_names = self._parse_group_names(exc_value)

                if self._manipulator.group_name in group_names:
                    group_names.remove(self._manipulator.group_name)
                    await self._reset(reset_to_idle=True)

                if len(group_names) > 0:
                    self.log(
                        f"Exception(s) caught in this manipulation "
                        f"context have different joint model group "
                        f"name! Expected {self._manipulator.group_name}, "
                        f"got {group_names}",
                        severity="WARN",
                    )
                    return False

                if unrecoverable_excs is not None:
                    return False

                raise ManipulationContextExitedError(
                    "Successfully handled recoverable errors",
                    recovered_error=exc_value,
                ) from exc_value

            return False
        finally:
            self._entered_context = False


class Commander(BaseNode):
    """Main commander node coordinating all experimental apparatus.

    The Commander is the top-level control node that aggregates all
    hardware interfaces and provides a unified API for running experiments.
    It handles safety interlocks, error recovery, and coordinates between
    subsystems.

    Use as an async context manager for automatic setup and cleanup:
        async with Commander() as commander:
            await commander.plan_and_move(...)

    Attributes:
        sound: Audio feedback interface.
        teensy: Microcontroller interface for safety and I/O.
        flic: Bluetooth button interface for response times.
        eyelink: Eye tracker interface for gaze monitoring.
        dashboard: UR robot dashboard interface.
        moveit: Motion planning and execution interface.
    """

    required_params: set[str] = BaseNode.required_params | {
        "simulate",
        "smooth_pursuit",
        "flic",
        "sound_interface",
        "teensy_interface",
        "common_ur_interface",
        "left_ur_interface",
        "right_ur_interface",
        "common_manipulation_interface",
        "left_manipulation_interface",
        "right_manipulation_interface",
        "common_manipulation_context_interface",
    }

    ###########################################################################
    ########## Initialization #################################################
    ###########################################################################

    def __init__(self):
        """Initialize the Commander node with all interfaces.

        Creates all hardware interface objects and connects to the
        MoveIt motion planning system.
        """
        super().__init__(
            "commander", automatically_declare_parameters_from_overrides=True
        )

        self._sound = SoundInterface(self, "sound_interface")
        self._teensy = TeensyInterface(
            self,
            "teensy_interface",
            additional_subscription_callback=self._teensy_sensor_callback,
        )
        self._flic = FlicInterface(self, "flic_interface")
        self._eyelink = EyelinkInterface(self, "eyelink_interface")

        self._moveit = MoveItInterface(self, "moveit_interface")

        self._rig_safety_fault_lock = threading.Lock()
        self._rig_safety_fault: RigSafetyError | None = None
        self._rig_safety_monitor_armed = False
        self._rig_safety_event: asyncio.Event | None = None
        self._rig_safety_loop: asyncio.AbstractEventLoop | None = None

        # self._urs: dict[str, URInterface] = {}
        # self._manipulators: dict[str, ObjectManipulationInterface] = {}
        self._manipulation_contexts: dict[str, ManipulationContextManager] = {}

        for robot_name, interface_names in self.param(
            "robot_interface_names"
        ).items():
            manipulation_context = ManipulationContextManager(
                self,
                interface_names["manipulation_context_interface_name"],
                simulate=self.param("simulate"),
                moveit_interface=self._moveit,
                teensy_interface=self._teensy,
                rig_safety_fault_getter=self._get_rig_safety_fault,
                rig_execution_fault_handler=functools.partial(
                    self._handle_rig_execution_fault, robot_name
                ),
                ur_interface_name=interface_names["ur_interface_name"],
                manipulation_interface_name=interface_names[
                    "manipulation_interface_name"
                ],
                parameter_fallback_prefix="common_manipulation_context_interface",
            )
            # self._urs[robot_name] = ur_interface
            # self._manipulators[robot_name] = manipulation_interface
            self._manipulation_contexts[robot_name] = manipulation_context

        # The io/status controller publishes safety changes immediately. Keep
        # subscriptions at Commander scope so one arm can abort the complete
        # dual-arm session without service polling or execution-timeout delay.
        self._safety_mode_subscriptions = []
        for robot_name, context in self._manipulation_contexts.items():
            ur_namespace = str(context._ur.param("namespace")).strip("/")
            topic = f"/{ur_namespace}_io_and_status_controller/safety_mode"
            self._safety_mode_subscriptions.append(
                self.create_subscription(
                    SafetyMode,
                    topic,
                    functools.partial(
                        self._ur_safety_mode_callback, robot_name
                    ),
                    10,
                )
            )

        self._initial_reset = False
        self._entered_context = False
        self._exited_context = False

        self.log("Commander initialized")

    def _get_rig_safety_fault(self) -> RigSafetyError | None:
        with self._rig_safety_fault_lock:
            return self._rig_safety_fault

    def _latch_rig_safety_fault(self, fault: RigSafetyError) -> RigSafetyError:
        """Latch one fatal fault and immediately stop both robot programs."""
        with self._rig_safety_fault_lock:
            if not self._rig_safety_monitor_armed:
                return fault
            if self._rig_safety_fault is not None:
                return self._rig_safety_fault
            self._rig_safety_fault = fault
            loop = self._rig_safety_loop
            event = self._rig_safety_event

        self.log(
            f"FATAL RIG SAFETY FAULT: {fault}. Automatic recovery is "
            "disabled for this session; inspect the robots and restart "
            "Commander explicitly.",
            severity="ERROR",
        )
        self.stop_all_execution()
        for context in self._manipulation_contexts.values():
            if not self.param("simulate"):
                context._ur.stop_program()

        if loop is not None and event is not None:
            loop.call_soon_threadsafe(event.set)
        return fault

    def stop_all_execution(self) -> None:
        """Immediately cancel active trajectory goals for both arms.

        This is deliberately synchronous and safe to call repeatedly so signal
        and safety paths can stop motion before awaiting coroutine cleanup.
        """
        for context in self._manipulation_contexts.values():
            context._manipulator.stop_execution()

    def _handle_rig_execution_fault(
        self, robot_name: str, error: ExecutionError
    ) -> RigSafetyError:
        """Make a real trajectory execution failure fatal rig-wide."""
        return self._latch_rig_safety_fault(
            RigExecutionSafetyError(robot_name, error)
        )

    @property
    def rig_safety_faulted(self) -> bool:
        """Whether this Commander session has latched a fatal rig fault."""
        return self._get_rig_safety_fault() is not None

    def raise_if_rig_safety_faulted(self) -> None:
        """Raise the latched fatal rig safety error, if present."""
        fault = self._get_rig_safety_fault()
        if fault is not None:
            raise fault

    async def wait_for_rig_safety_fault(self) -> None:
        """Wait until either arm reports a fatal safety fault, then raise it."""
        event = self._rig_safety_event
        if event is None:
            raise RuntimeError("Commander safety monitor is not active")
        self.raise_if_rig_safety_faulted()
        await event.wait()
        self.raise_if_rig_safety_faulted()
        raise RuntimeError("Safety fault event set without a latched fault")

    def _ur_safety_mode_callback(
        self, robot_name: str, msg: SafetyMode
    ) -> None:
        """Latch UR stop/fault modes and immediately stop both manipulators."""
        mode_name = _FATAL_UR_SAFETY_MODES.get(msg.mode)
        if mode_name is None:
            return

        self._latch_rig_safety_fault(
            URSafetyViolationError(robot_name, mode_name)
        )

    def _teensy_sensor_callback(self, msg: TeensySensor) -> None:
        """Handle Teensy sensor updates for safety monitoring.

        Immediately stops robot execution if safety conditions are
        violated (e.g., safety laser broken while robot is moving).

        Args:
            msg: Current sensor state from the Teensy.
        """
        # Record raw beam edges for every arm before deciding whether an
        # active trajectory needs to be stopped. This preserves breaks that
        # occur while a UR program is being restored and no trajectory is
        # currently marked executing.
        for context in self._manipulation_contexts.values():
            context.observe_laser_state(bool(msg.is_safety_laser_broken))

        if not self._teensy.safe_to_execute:
            for robot_name, context in self._manipulation_contexts.items():
                if (
                    context._manipulator.executing
                    and context._manipulator.presentation_region_active
                ):
                    if context.request_laser_safety_stop():
                        self.log(
                            f"Not safe to execute for {robot_name}, "
                            "stopping UR program once",
                            severity="WARN",
                        )

    def _safe_to_execute_condition(self) -> bool:
        return not self.rig_safety_faulted and (
            not any(
                x._manipulator.presentation_region_active
                for x in self._manipulation_contexts.values()
            )
            or self._teensy.safe_to_execute
        )

    ###########################################################################
    ########## User Interface #################################################
    ###########################################################################

    @ensure_context
    async def play_sound(
        self,
        note: Optional[Note | Mapping[str, Any]] = None,
        duration: Optional[float] = None,
    ):
        """Play a note for a given duration.

        Args:
            note: Note to play. If None, the default note is used.
            duration: Duration of the sound in seconds. If None, the default duration is used.
        """
        await self._sound.play(note, duration)

    @ensure_context
    async def release_arm(self, arm: Literal["left", "right", "both"]) -> None:
        """Release the specified arm lock(s).

        Args:
            arm: Which arm(s) to release - "left", "right", or "both".
        """
        await self._teensy.set_arm_lock(arm, lock=False)

    @ensure_context
    async def lock_arm(self, arm: Literal["left", "right", "both"]) -> None:
        """Lock the specified arm lock(s).

        Args:
            arm: Which arm(s) to lock - "left", "right", or "both".
        """
        await self._teensy.set_arm_lock(arm, lock=True)

    @ensure_context
    async def lock_arms_and_wait(
        self, timeout: Optional[float] = None
    ) -> bool:
        """Lock arms and wait for safety conditions to be met.

        Engages both arm lock solenoids and waits for the subject to
        place their arms in the locks and for the safety laser to
        be unbroken.

        Args:
            timeout: Maximum wait time in seconds, or None for default.

        Returns:
            True if safety conditions met within timeout.
        """
        return await self._teensy.lock_arms_and_wait(
            timeout, condition=self._safe_to_execute_condition
        )

    @ensure_context
    async def reveal_smartglass(self) -> None:
        """Make the smartglass transparent (subject can see through)."""
        await self._teensy.set_smartglass(reveal=True)

    @ensure_context
    async def occlude_smartglass(self) -> None:
        """Make the smartglass opaque (subject's view is blocked)."""
        await self._teensy.set_smartglass(reveal=False)

    @ensure_context
    async def stop_reward(self) -> None:
        """Stop any active reward delivery immediately."""
        await self._teensy.set_reward(activate=False)

    @ensure_context
    async def start_reward_and_wait(self, duration: float) -> None:
        """Deliver reward for the specified duration.

        Args:
            duration: Reward duration in seconds.
        """
        await self._teensy.start_reward_and_wait(duration)

    @ensure_context
    async def flic_response_time(
        self, object_id: str, *, timeout: Optional[float] = None
    ) -> float | None:
        """Measure response time using the Flic button.

        Waits for the subject to press the Flic button associated with
        the currently attached object and returns the ROS timestamp
        (converted to seconds) that the button was pressed.

        Args:
            timeout: Maximum wait time in seconds, or None for no timeout.

        Returns:
            ROS timestamp (converted to seconds) that button was pressed
            or None if the timeout was reached before a press.
        """
        bd_addr = self.param(f"flic.bd_addrs.{object_id}")

        return await self._flic.response_time(bd_addr, timeout)

    @ensure_context
    async def smooth_pursuit_and_reward(self) -> None:
        """Monitor smooth pursuit and provide contingent reward.

        Continuously monitors the subject's eye movements. When smooth
        pursuit is detected, plays a tone and accumulates reward credit.
        Periodically delivers reward if the pursuit ratio exceeds the
        threshold.

        Runs until cancelled externally.
        """
        interval_start_time: float | None = None
        last_smooth_pursuit = False
        pursuit_count = 0
        count = 0

        async def callback(smooth_pursuit: bool):
            """Consumer for the eyelink smooth pursuit queue."""
            nonlocal interval_start_time
            nonlocal count
            nonlocal pursuit_count
            nonlocal last_smooth_pursuit

            duration = self.param("smooth_pursuit.reward_duration")
            interval = self.param("smooth_pursuit.reward_interval")
            reward_threshold = self.param(
                "smooth_pursuit.reward_threshold_ratio"
            )

            count += 1
            if smooth_pursuit:
                pursuit_count += 1
                if not last_smooth_pursuit:
                    self.log("Smooth pursuit started", severity="INFO")
                    self._sound.start_note()
            elif last_smooth_pursuit:
                self.log("Smooth pursuit ended", severity="INFO")
                self._sound.stop_note()

            if interval_start_time is None:
                interval_start_time = self.ros_time()
            elif self.ros_time() - interval_start_time >= interval:
                if pursuit_count / count >= reward_threshold:
                    await self._teensy.set_reward(
                        activate=True, duration=duration
                    )

                interval_start_time = self.ros_time()
                pursuit_count = 0
                count = 0

            last_smooth_pursuit = smooth_pursuit

        try:
            await self._eyelink.smooth_pursuit(callback)
        finally:
            self._sound.stop_everything()
            try:
                await self._teensy.set_reward(activate=False)
            except Exception as e:
                self.log(f"Error stopping reward: {e}", severity="ERROR")

    @property
    @ensure_context
    def robot_names(self) -> list[str]:
        """Get list of available robot names.

        Returns:
            List of configured robot identifiers.
        """
        return list(self._manipulation_contexts.keys())

    @ensure_context
    def reachable_object_ids(self, robot_name: str) -> set[str]:
        """Get the IDs of reachable objects for a robot.

        Args:
            robot_name: Name of the robot.

        Returns:
            Set of object IDs this robot can manipulate.

        Raises:
            ValueError: If robot_name is not configured.
        """
        if robot_name not in self.robot_names:
            raise ValueError(
                f"Unsupported robot_name: {robot_name}. "
                f"Available: {self.robot_names}"
            )
        return self._manipulation_contexts[
            robot_name
        ]._manipulator.reachable_object_ids

    @ensure_context
    def manipulation_context(
        self, robot_name: str
    ) -> ManipulationContextManager:
        """Get the manipulation context for a specific robot.

        Args:
            robot_name: Name of the robot.

        Returns:
            ManipulationContextManager for the robot.

        Raises:
            ValueError: If robot_name is not configured.
        """
        if robot_name not in self.robot_names:
            raise ValueError(
                f"Unsupported robot_name: {robot_name}. "
                f"Available: {self.robot_names}"
            )
        return self._manipulation_contexts[robot_name]

    @ensure_context
    async def manipulation_preflight(
        self, *, num_seeds: int = 32
    ) -> dict[str, Any]:
        """Plan-only validation of manipulation branches for every mount.

        The implementation refuses to run unless Commander is in mock mode.
        """
        from tabletop_rig.manipulation_preflight import run_preflight

        return await run_preflight(self, num_seeds=num_seeds)

    @ensure_context
    def apply_manipulation_preflight(
        self,
        *,
        report_name: str = "manipulation_preflight.yaml",
        allow_real_use: bool = False,
    ) -> dict[str, Any]:
        """Apply mock branches in simulation and report compatible objects."""
        from tabletop_rig.manipulation_preflight import load_compatible_results

        results = load_compatible_results(
            self,
            report_name=report_name,
            allow_real_use=allow_real_use,
        )
        for context in self._manipulation_contexts.values():
            manipulator = context._manipulator
            manipulator._set_preflight_pre_fetch_branches(
                results["branches_by_group"].get(manipulator.group_name, {})
            )
        self.log(
            f"Applied manipulation preflight {report_name}: "
            f"{len(results['available_object_ids'])} PASS, "
            f"{len(results['unavailable_object_ids'])} UNAVAILABLE, "
            f"{len(results['stale_object_ids'])} stale/missing, "
            f"mode={'mock' if self.param('simulate') else 'real'}"
        )
        return results

    async def __aenter__(self) -> Self:
        """Enter the async context manager.

        Initializes MoveIt and resets the commander to the idle state.

        Returns:
            The Commander instance.
        """
        self.log("Entering commander context manager")

        if self._entered_context:
            raise RuntimeError("Commander context manager already entered")

        if self._exited_context:
            raise RuntimeError("Commander context manager already exited")

        self._rig_safety_loop = asyncio.get_running_loop()
        self._rig_safety_event = asyncio.Event()
        try:
            async with asyncio.TaskGroup() as tg:
                tg.create_task(
                    self._teensy.set_sync_pulse_solenoid(activate=True)
                )

                for context in self._manipulation_contexts.values():
                    tg.create_task(context._ur.reset())
        except BaseException:
            self._rig_safety_loop = None
            self._rig_safety_event = None
            self.destroy_node()
            raise

        with self._rig_safety_fault_lock:
            self._rig_safety_monitor_armed = True
        self._entered_context = True
        return self

    async def __aexit__(
        self,
        exc_type: Optional[type[BaseException]],
        exc_value: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> bool | None:
        """Exit the async context manager.

        Handles recoverable errors by resetting the commander.
        Always cleans up MoveIt resources.

        Args:
            exc_type: Exception type if an exception occurred.
            exc_value: Exception instance if an exception occurred.
            exc_tb: Traceback if an exception occurred.

        Returns:
            True if a recoverable error was handled, False otherwise.
        """
        self.log("Exiting commander context manager")
        with self._rig_safety_fault_lock:
            self._rig_safety_monitor_armed = False
        try:
            await self._teensy.set_sync_pulse_solenoid(activate=False)
        except BaseException as e:
            self.log(f"Failed to set sync pulse: {e!r}")

        try:
            self.destroy_node()
        except BaseException as e:
            self.log(f"Failed to destroy node: {e!r}")

        self._entered_context = False
        self._exited_context = True


async def _wait_for_task_or_safety_fault(
    commander: Commander, task: asyncio.Task
) -> None:
    """Run a Commander coroutine until it finishes or a UR fault occurs."""
    safety_task = asyncio.create_task(commander.wait_for_rig_safety_fault())
    try:
        done, _ = await asyncio.wait(
            (task, safety_task), return_when=asyncio.FIRST_COMPLETED
        )
        if safety_task in done:
            if not task.done():
                task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            await safety_task

        safety_task.cancel()
        try:
            await safety_task
        except asyncio.CancelledError:
            pass
        await task
    except asyncio.CancelledError:
        # Stop controller goals before allowing task cancellation cleanup to
        # run. Awaiting the task here keeps that cleanup inside the live
        # Commander context and prevents asyncio.run() from discovering an
        # orphan task during shutdown.
        commander.stop_all_execution()
        if not task.done():
            task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        raise
    finally:
        if not safety_task.done():
            safety_task.cancel()


async def debug_commander(
    commander: Commander, config: Optional[str] = None
) -> None:
    """Run the commander interactively for debugging.

    Sets a breakpoint and runs an infinite loop for
    interactive debugging via attached debugger.

    Args:
        commander: The Commander instance.
        config: Unused configuration path.
    """
    while True:
        # pose_stamped = commander.eef_pose_stamped()
        # old_frame_transform = commander.get_frame_transform(
        #     pose_stamped.header.frame_id
        # )
        # rel_pose = change_reference_frame_pose(
        #     old_pose=pose_stamped.pose,
        #     old_frame_transform=old_frame_transform,
        #     new_frame_transform=grid_origin_matrix,
        # )
        # position, euler = arrays_from_pose_msg(rel_pose, euler=True)
        # commander.log(
        #     f"Eef relative position: {position.round(4).tolist()}, euler: {euler.round(4).tolist()}"
        # )
        await asyncio.sleep(1)


async def asyncio_runner(
    commander: Commander,
    coro_fn: Callable[[Commander, Optional[str]], Coroutine],
    config: str | None,
    cancel_future: concurrent.futures.Future,
    max_workers: int | None,
) -> None:
    """Run an experiment coroutine with proper executor setup.

    Configures the asyncio event loop with a thread pool executor and
    runs the experiment coroutine. Handles cancellation when the ROS
    executor stops.

    Args:
        coro_fn: Async function to run (signature: commander, config).
        commander: The Commander instance.
        config: Configuration file path to pass to coro_fn.
        spin_future: Future for the ROS executor spin thread.
        max_workers: Number of thread pool workers.
    """
    tpe = concurrent.futures.ThreadPoolExecutor(max_workers=max_workers)
    loop = asyncio.get_event_loop()
    loop.set_default_executor(tpe)

    try:
        async with commander:
            task = asyncio.create_task(coro_fn(commander, config))

            def stop_and_cancel() -> None:
                commander.stop_all_execution()
                task.cancel()

            cancel_future.add_done_callback(
                lambda _: (
                    loop.call_soon_threadsafe(stop_and_cancel)
                    if loop.is_running()
                    else None
                )
            )
            await _wait_for_task_or_safety_fault(commander, task)
    except asyncio.CancelledError:
        pass
    except BaseException as e:
        print(
            f"Caught exception in task: \n "
            f"{' '.join(traceback.format_exception(e))}"
        )
        raise


def spin_notify(executor: Executor, cancel_future: concurrent.futures.Future):
    first_exc = None
    while executor.context.ok() and not executor._is_shutdown:
        try:
            executor.spin()
            assert not executor.context.ok() or executor._is_shutdown
        except Exception as e:
            print(
                f"Caught exception in executor: \n "
                f"{' '.join(traceback.format_exception(e))}"
            )
            if first_exc is None:
                first_exc = e
        finally:
            if not cancel_future.done():
                cancel_future.set_result(None)

    if not cancel_future.done():
        cancel_future.set_result(None)

    if first_exc is not None:
        raise first_exc


EXECUTOR_TYPE = "single-threaded"


def main_sync(args=None) -> None:
    """Entry point for the commander node.

    Parses command line arguments, optionally loads a custom experiment
    coroutine, and runs the commander node.

    Args:
        args: Command line arguments (uses sys.argv if None).

    Command line options:
        --coro-module: Python module containing the experiment coroutine.
        --coro-name: Name of the coroutine function to run.
        --coro-config: Configuration file path for the experiment.
        --max-workers: Number of thread pool workers.
        --debug: Enable debugpy for remote debugging.
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    executor = None
    spin_future = None

    try:
        # Parse non-ROS arguments
        parser = argparse.ArgumentParser()
        parser.add_argument("--coro-module", type=str, default=None)
        parser.add_argument("--coro-name", type=str, default=None)
        parser.add_argument("--coro-config", type=str, default=None)
        parser.add_argument("--max-workers", type=int, default=None)
        parser.add_argument("--debug", action="store_true", default=False)

        non_ros_args = rclpy.utilities.remove_ros_args(args)
        args, _ = parser.parse_known_args(non_ros_args)

        if args.coro_module is not None and args.coro_name is not None:
            print(
                f"Loading coroutine {args.coro_name} from module {args.coro_module} "
            )
            coro_fn: Callable[[Commander, Optional[str]], Coroutine] = getattr(
                importlib.import_module(args.coro_module),
                args.coro_name,
            )
        elif args.coro_name is not None or args.coro_module is not None:
            raise ValueError(
                "Both coro_module and coro_name must be provided when one is provided"
            )
        else:
            print(
                "No coroutine module or name provided, running in debug mode"
            )
            coro_fn = debug_commander
            args.coro_config = None

        if args.coro_config is not None:
            print(f"Config file: {args.coro_config}")

        if args.debug:
            import debugpy

            print("Debug mode enabled")
            debugpy.listen(1300)
            print("Waiting for debugger to attach")
            debugpy.wait_for_client()
            print("Debugger attached")

        match EXECUTOR_TYPE:
            case "events":
                executor = EventsExecutor()
            case "single-threaded":
                executor = SingleThreadedExecutor()
            case "multi-threaded":
                executor = MultiThreadedExecutor()
            case _:
                raise ValueError(f"Unsupported EXECUTOR_TYPE: {EXECUTOR_TYPE}")

        tpe = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        cancel_future = concurrent.futures.Future()
        spin_future = tpe.submit(spin_notify, executor, cancel_future)

        commander = Commander()
        executor.add_node(commander)

        asyncio.run(
            asyncio_runner(
                commander,
                coro_fn,
                args.coro_config,
                cancel_future,
                args.max_workers,
            )
        )
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            print("Shutting down executor")
            executor.shutdown()
        if spin_future is not None:
            print("Raising executor spin errors")
            spin_future.result()
        print("Shutting down rclpy")
        rclpy.try_shutdown()  # type: ignore


def main(args=None):
    """Entry point for the flic node."""
    try:
        main_sync(args)
        # rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        # print_signal_handler()
        # asyncio.run(main_async(args), debug=True)
        # asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("Keyboard interrupt")
