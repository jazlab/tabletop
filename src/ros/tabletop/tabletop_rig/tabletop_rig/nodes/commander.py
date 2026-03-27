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
        await commander.present_object()
        response_time = await commander.flic_response_time(timeout=10.0)
        await commander.return_object()
"""

import argparse
import asyncio
import concurrent.futures
import functools
import importlib
import inspect
import signal
import traceback
from collections.abc import Callable, Coroutine, Mapping
from contextlib import (
    AbstractAsyncContextManager,
    AbstractContextManager,
    AsyncExitStack,
)
from types import TracebackType
from typing import Any, Literal, Optional, Self

import rclpy
import rclpy.utilities
from mingus.containers import Note
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from rclpy.executors import (
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.experimental import EventsExecutor
from rclpy.signals import SignalHandlerOptions
from tabletop_interfaces.msg import TeensySensor

from tabletop_rig.exceptions import (
    ActionError,
    ExecutionError,
    ExecutionInterruptedError,
    MoveitRecoverableError,
    NotSafeToExecuteError,
    ObjectManipulationError,
    ServiceCallUnsuccessfulError,
)
from tabletop_rig.executors import AIOExecutor
from tabletop_rig.interfaces.dashboard import DashboardInterface
from tabletop_rig.interfaces.eyelink import EyelinkInterface
from tabletop_rig.interfaces.flic import FlicInterface
from tabletop_rig.interfaces.moveit.moveit import MoveItInterface
from tabletop_rig.interfaces.moveit.requests import PlanGoalT
from tabletop_rig.interfaces.sound import SoundInterface
from tabletop_rig.interfaces.teensy import TeensyInterface
from tabletop_rig.nodes.base import BaseNode


def ensure_context(fn):
    if inspect.iscoroutinefunction(fn):

        @functools.wraps(fn)
        async def async_wrapper(self: "Commander", *args, **kwargs):
            if not self._entered_context:
                raise RuntimeError(
                    "Commander context manager not entered, use 'async with commander:' before calling any Commander methods"
                )
            return await fn(self, *args, **kwargs)

        return async_wrapper
    else:

        @functools.wraps(fn)
        def wrapper(self: "Commander", *args, **kwargs):
            if not self._entered_context:
                raise RuntimeError(
                    "Commander context manager not entered, use 'async with commander:' before calling any Commander methods"
                )
            return fn(self, *args, **kwargs)

        return wrapper


def safe_execution(coro_fn):
    """Decorator for methods requiring safety-checked robot execution.

    Wraps coroutines that execute robot motions to handle common error
    cases with automatic recovery:
    - NotSafeToExecuteError: Locks arms and waits for safety
    - ExecutionInterruptedError: Resets dashboard and retries

    Args:
        coro_fn: Async method to wrap.

    Returns:
        Wrapped method with retry logic.
    """

    @functools.wraps(coro_fn)
    async def wrapper(self: "Commander", *args, **kwargs):
        max_retries = self.param("safe_execution.max_retries")
        if max_retries < 0:
            raise ValueError("safe_execution.max_retries must be at least 0")

        for i in range(max_retries + 1):
            try:
                return await coro_fn(self, *args, **kwargs)
            except NotSafeToExecuteError as e:
                if i == max_retries:
                    raise

                self.log(
                    f"Safe execution violated while running {coro_fn.__name__}: {e}. "
                    f"Locking arms and waiting for safety, then resetting dashboard "
                    f"before retrying",
                    severity="WARN",
                )
                await self.teensy.lock_arms_and_wait()
                await self.dashboard.reset()
                self.log(
                    f"Arms locked and safe to execute and dashboard reset, "
                    f"retrying {coro_fn.__name__}",
                    severity="WARN",
                )
            except ExecutionInterruptedError as e:
                if i == max_retries:
                    raise

                self.log(
                    f"Execution interrupted while running {coro_fn.__name__}: {e}. "
                    f"Resetting dashboard before retrying",
                    severity="WARN",
                )
                await self.dashboard.reset()
                self.log(
                    f"Dashboard reset, retrying {coro_fn.__name__}",
                    severity="WARN",
                )

    return wrapper


class Commander(BaseNode):
    """Main commander node coordinating all experimental apparatus.

    The Commander is the top-level control node that aggregates all
    hardware interfaces and provides a unified API for running experiments.
    It handles safety interlocks, error recovery, and coordinates between
    subsystems.

    Use as an async context manager for automatic setup and cleanup:
        async with Commander() as commander:
            await commander.plan_and_execute(...)

    Attributes:
        sound: Audio feedback interface.
        teensy: Microcontroller interface for safety and I/O.
        flic: Bluetooth button interface for response times.
        eyelink: Eye tracker interface for gaze monitoring.
        dashboard: UR robot dashboard interface.
        moveit: Motion planning and execution interface.
    """

    default_params: dict[str, Any] = BaseNode.default_params | {
        "wait_for_interfaces": True,
        "recovery.max_attempts": 5,
        "initial_attached_object_id": "null",
        "initial_attached_object_idx": "null",
    }
    required_params: set[str] = BaseNode.required_params | {
        "simulate",
        "dashboard.installation",
        "dashboard.program",
        "teensy.spin_period",
        "link_padding",
        "planning_scene.cache_dir",
        "planning_scene.use_saved_scene",
        "planning_scene.object_meshes",
        "planning_scene.rig_meshes",
        # "planning.defaults", TODO
        # "planning.pose_tolerance.position_tolerance",
        # "planning.pose_tolerance.orientation_tolerance",
        # "predefined_states.idle_state",
        "trajectory_cache.use_cached_trajectories",
        "trajectory_cache.freeze_cache",
        "trajectory_cache.kwargs",
        "object_manipulation.touch_links",
        "object_manipulation.mount_ids",
        "object_manipulation.allowed_mount_collisions",
        "object_manipulation.detach_velocity_scaling_factor",
        "object_manipulation.state_offsets.pre_fetch",
        "object_manipulation.state_offsets.pre_attach",
        "object_manipulation.state_offsets.attach",
        "object_manipulation.state_offsets.post_attach",
        "object_manipulation.state_offsets.post_fetch",
        # "object_manipulation.unpresent_pose_stamped",
        "smooth_pursuit.reward_duration",
        "smooth_pursuit.reward_interval",
        "smooth_pursuit.reward_threshold_ratio",
        "safe_execution.max_retries",
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

        self.sound = SoundInterface(self)
        self.teensy = TeensyInterface(
            self, additional_subscription_callback=self._teensy_sensor_callback
        )
        self.flic = FlicInterface(self)
        self.eyelink = EyelinkInterface(self)
        self.dashboard = DashboardInterface(self)
        self.moveit = MoveItInterface(
            self, safe_to_execute_callback=lambda: self.teensy.safe_to_execute
        )

        self._entered_context = False

        self.log("Commander initialized")

    def _teensy_sensor_callback(self, msg: TeensySensor) -> None:
        """Handle Teensy sensor updates for safety monitoring.

        Immediately stops robot execution if safety conditions are
        violated (e.g., safety laser broken while robot is moving).

        Args:
            msg: Current sensor state from the Teensy.
        """
        if not self.teensy.safe_to_execute and self.moveit.executing:
            self.log(
                "Not safe to execute, stopping execution",
                severity="WARN",
            )
            self.moveit.stop_execution()

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
        await self.sound.play(note, duration)

    @ensure_context
    async def release_arm(self, arm: Literal["left", "right", "both"]) -> None:
        """Release the specified arm lock(s).

        Args:
            arm: Which arm(s) to release - "left", "right", or "both".
        """
        await self.teensy.set_arm_lock(arm, lock=False)

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
        return await self.teensy.lock_arms_and_wait(timeout)

    @ensure_context
    async def reveal_smartglass(self) -> None:
        """Make the smartglass transparent (subject can see through)."""
        await self.teensy.set_smartglass(reveal=True)

    @ensure_context
    async def occlude_smartglass(self) -> None:
        """Make the smartglass opaque (subject's view is blocked)."""
        await self.teensy.set_smartglass(reveal=False)

    @ensure_context
    async def stop_reward(self) -> None:
        """Stop any active reward delivery immediately."""
        await self.teensy.set_reward(activate=False)

    @ensure_context
    async def start_reward_and_wait(self, duration: float) -> None:
        """Deliver reward for the specified duration.

        Args:
            duration: Reward duration in seconds.
        """
        await self.teensy.start_reward_and_wait(duration)

    @ensure_context
    async def flic_response_time(
        self, timeout: Optional[float] = None
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
        object_id = self.moveit.attached_object_id
        if object_id is None:
            raise ObjectManipulationError("No attached object to reset")

        bd_addr = self.param(f"flic.bd_addrs.{object_id}")

        return await self.flic.response_time(bd_addr, timeout)

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
                    self.sound.start_note()
            elif last_smooth_pursuit:
                self.log("Smooth pursuit ended", severity="INFO")
                self.sound.stop_note()

            if interval_start_time is None:
                interval_start_time = self.ros_time()
            elif self.ros_time() - interval_start_time >= interval:
                if pursuit_count / count >= reward_threshold:
                    await self.teensy.set_reward(
                        activate=True, duration=duration
                    )

                interval_start_time = self.ros_time()
                pursuit_count = 0
                count = 0

            last_smooth_pursuit = smooth_pursuit

        try:
            await self.eyelink.smooth_pursuit(callback)
        finally:
            self.sound.stop_everything()
            try:
                await self.teensy.set_reward(activate=False)
            except Exception as e:
                self.log(f"Error stopping reward: {e}", severity="ERROR")

    @ensure_context
    async def attach_object_manually(self, object_id: str) -> None:
        """Attach a non-grid object to the robot end-effector

        Used when the robot already has an object grasped and the planning
        scene needs updating.

        Args:
            object_id: ID of the collision object to attach.
        """
        await self.moveit.add_manually_attached_object(object_id)

    @ensure_context
    async def detach_object_manually(self, object_id: str) -> None:
        """Detach a non-grid object from the robot end-effector

        Used when a previously manually attached object has been
        detached by hand and the planning scene needs updating.

        Args:
            object_id: ID of the currently attached collision object to detach.
        """
        await self.moveit.remove_manually_attached_object(object_id)

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
        trajectory, _ = await self.moveit.plan(*args, **kwargs)
        return trajectory

    @ensure_context
    @safe_execution
    async def execute(self, *args, **kwargs) -> None:
        """Execute a previously planned trajectory.

        Args:
            *args: Positional arguments passed to MoveItInterface.execute.
            **kwargs: Keyword arguments passed to MoveItInterface.execute.

        Raises:
            ExecutionError: If trajectory execution fails.
            NotSafeToExecuteError: If safety conditions not met.
        """
        await self.moveit.execute(*args, **kwargs)

    @ensure_context
    @safe_execution
    async def plan_and_execute(self, *args, **kwargs) -> None:
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
        await self.moveit.plan_and_execute(*args, **kwargs)

    @ensure_context
    @safe_execution
    async def fetch_object(self, object_id: str):
        """Fetch an object from its mount.

        The robot moves to the object's mount, attaches the object, and moves
        to the object's post-fetch pose.

        Args:
            object_id: The ID of the object to fetch

        Raises:
            ValueError: If the object ID is not a valid collision object
            PlanningError: If the planning fails
            ExecutionError: If the execution fails
        """
        await self.moveit.fetch_object(object_id)

    @ensure_context
    @safe_execution
    async def present_object(self, object_id: str):
        """Move to present state with the currently attached object"""
        await self.moveit.present_object(object_id)

    @ensure_context
    @safe_execution
    async def reset_object(self, object_id: str):
        """Reset the currently attached object using its associated ObjectResetConfig

        Raises:
            RuntimeError: If exactly one object is not attached
            PlanningError: If the planning fails
            ExecutionError: If the execution fails
        """
        await self.moveit.reset_object(object_id)

    @ensure_context
    @safe_execution
    async def return_object(self, object_id: str):
        """Return the currently attached object to its mount.

        Raises:
            RuntimeError: If exactly one object is not attached
            PlanningError: If the planning fails
            ExecutionError: If the execution fails
        """
        await self.moveit.return_object(object_id)

    async def _reset_commander(
        self,
        timeout: Optional[float] = None,
        end_goal: Optional[PlanGoalT] = None,
    ) -> None:
        """Reset the robot to a known good state.

        Performs a full reset sequence: locks arms, waits for safety,
        resets the UR dashboard, and optionally moves to a goal pose.
        Retries automatically on recoverable errors.

        Args:
            timeout: Maximum total time for reset sequence.
            end_goal: Optional pose or state to move to after reset.

        Raises:
            asyncio.TimeoutError: If reset not completed within timeout.
        """
        self.log("Resetting commander")

        async with asyncio.timeout(timeout):
            max_attempts = self.param("recovery.max_attempts")
            if max_attempts < 1:
                raise ValueError(
                    "recover.max_attempts parameter must be at least 1"
                )

            excs: list[Exception] = []
            for _ in range(max_attempts):
                try:
                    if not self.teensy.safe_to_execute:
                        self.log(
                            "Cannot reset commander until safe to execute",
                            severity="WARN",
                        )
                        await self.teensy.lock_arms_and_wait()
                    await self.dashboard.reset()
                    await self.moveit.reset_rig(end_goal)
                    return
                except (
                    ServiceCallUnsuccessfulError,
                    ActionError,
                    MoveitRecoverableError,
                ) as e:
                    excs.append(e)
                    self.log(
                        "Caught exception while resetting commander:",
                        severity="WARN",
                    )
                    self.log(f"{type(e).__name__}: {e}", severity="WARN")
                    self.log(
                        f"Traceback: \n {' '.join(traceback.format_tb(e.__traceback__))}",
                        severity="DEBUG",
                    )
                    if isinstance(e, ExecutionError):
                        sleep_time = 1
                    else:
                        sleep_time = 1
                    self.log(
                        f"Sleeping for {sleep_time} seconds before retrying",
                        severity="WARN",
                    )
                    await asyncio.sleep(sleep_time)

            if len(excs) == 1:
                raise excs[0]
            if len(excs) > 1:
                raise ExceptionGroup(
                    f"Failed to reset commander after {max_attempts} attempts",
                    excs,
                )

    ###########################################################################
    ########## Context manager ################################################
    ###########################################################################

    async def _handle_recoverable_errors(
        self,
        exc_type: Optional[type[BaseException]],
        exc_value: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> bool | None:
        """Handles recoverable errors by resetting the commander.

        Args:
            exc_type: Exception type if an exception occurred.
            exc_value: Exception instance if an exception occurred.
            exc_tb: Traceback if an exception occurred.

        Returns:
            True if a recoverable error was handled, False otherwise.
        """
        self.log("Exiting commander context manager", severity="DEBUG")

        if exc_type is not None:
            # We use only the first exception raised if an exception group is met
            if isinstance(exc_value, ExceptionGroup):
                if len(exc_value.exceptions) != 1:
                    return False
                exc_value = exc_value.exceptions[0]

            if isinstance(exc_value, MoveitRecoverableError):
                # return False
                self.log(
                    "Caught exception while running commander:",
                    severity="ERROR",
                )
                self.log(f"{exc_type.__name__}: {exc_value}", severity="ERROR")
                self.log(
                    f"Traceback: \n {' '.join(traceback.format_tb(exc_tb))}",
                    severity="DEBUG",
                )
                # if exc_type is ExecutionError:
                #     self.log(
                #         "Sleeping for 5 seconds before resetting commander",
                #         severity="WARN",
                #     )
                #     await asyncio.sleep(5)
                await self._reset_commander(end_goal="idle")
                return True

        return False

    async def __aenter__(self) -> Self:
        """Enter the async context manager.

        Initializes MoveIt and resets the commander to the idle state.

        Returns:
            The Commander instance.
        """
        self.log("Entering commander context manager", severity="DEBUG")
        self._context_stack = await AsyncExitStack().__aenter__()
        try:
            for interface in self._interfaces:
                if isinstance(interface, AbstractAsyncContextManager):
                    await self._context_stack.enter_async_context(interface)
                elif isinstance(interface, AbstractContextManager):
                    self._context_stack.enter_context(interface)

            # await self._reset_commander(end_goal="idle")
            await self._reset_commander()
            self._context_stack.push_async_exit(
                self._handle_recoverable_errors
            )
        except BaseException as e:
            await self._context_stack.__aexit__(type(e), e, e.__traceback__)
            raise e

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
            return await self._context_stack.__aexit__(
                exc_type, exc_value, exc_tb
            )
        finally:
            self._entered_context = False


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
    del config

    commander.log("Running commander interactively")

    # grid_origin = commander.object_grid_origin_pose_stamped()
    # grid_origin_matrix = matrix_from_pose_msg(grid_origin.pose)
    # position, euler = arrays_from_pose_msg(grid_origin.pose, euler=True)
    # commander.log(
    #     f"Object grid origin position: {position.round(4)}, euler: {euler.round(4)}"
    # )

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
    coro_fn: Callable[[Commander, Optional[str]], Coroutine],
    commander: Commander,
    config: str | None,
    spin_future: concurrent.futures.Future,
    max_workers: int,
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

    task = asyncio.create_task(coro_fn(commander, config))
    spin_future.add_done_callback(
        lambda _: (
            loop.call_soon_threadsafe(task.cancel)
            if loop.is_running()
            else None
        )
    )

    await task


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
    try:
        # Parse non-ROS arguments
        parser = argparse.ArgumentParser()
        parser.add_argument("--coro-module", type=str, default=None)
        parser.add_argument("--coro-name", type=str, default=None)
        parser.add_argument("--coro-config", type=str, default=None)
        parser.add_argument("--max-workers", type=int, default=4)
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
            args.debug = True

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

        commander = Commander()
        executor.add_node(commander)

        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as tpe:
            spin_future = tpe.submit(executor.spin)
            try:
                asyncio.run(
                    asyncio_runner(
                        coro_fn,
                        commander,
                        args.coro_config,
                        spin_future,
                        args.max_workers,
                    )
                )
            finally:
                print("Shutting down commander")
                commander.destroy_node()
                print("Shutting down executor")
                executor.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down rclpy")
        rclpy.try_shutdown()  # type: ignore


# @pyinstrument.profile(async_mode="enabled")
async def main_async(args=None):
    """Async entry point for the Flic node.

    Initializes ROS2, creates the node, connects to the Flic server,
    and spins until shutdown or connection loss.

    Args:
        args: Command line arguments (passed to rclpy.init).
    """
    # rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    rclpy.init(args=args)

    # Parse non-ROS arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--coro-module", type=str, default=None)
    parser.add_argument("--coro-name", type=str, default=None)
    parser.add_argument("--coro-config", type=str, default=None)
    parser.add_argument("--max-workers", type=int, default=4)
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
        print("No coroutine module or name provided, running in debug mode")
        coro_fn = debug_commander
        args.coro_config = None
        args.debug = True

    if args.coro_config is not None:
        print(f"Config file: {args.coro_config}")

    if args.debug:
        import debugpy

        print("Debug mode enabled")
        debugpy.listen(1300)
        print("Waiting for debugger to attach")
        debugpy.wait_for_client()
        print("Debugger attached")

    try:
        executor = AIOExecutor()
        commander = Commander()
        executor.add_node(commander)

        p = None
        try:
            # future = executor.create_task(coro_fn(commander, args.coro_config))
            # await executor.spin_until_future_complete(future)
            async with asyncio.TaskGroup() as tg:
                spin_task = tg.create_task(executor.spin())
                user_task = tg.create_task(
                    coro_fn(commander, args.coro_config)
                )
                await asyncio.wait(
                    [spin_task, user_task], return_when=asyncio.FIRST_COMPLETED
                )
            # with pyinstrument.Profiler(async_mode="enabled") as p:
        finally:
            if p is not None:
                p.print(show_all=True, timeline=True)
            print("Destroying commander")
            commander.destroy_node()
            print("Shutting down executor")
            executor.shutdown()
    finally:
        print("Shutting down rclpy")
        rclpy.try_shutdown()  # type: ignore


def print_signal_handler():
    handler = signal.getsignal(signal.SIGINT)
    if handler is signal.default_int_handler:
        print(f"Default signal handler: {handler}")
    else:
        print(f"Non-default signal handler: {handler}")


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
