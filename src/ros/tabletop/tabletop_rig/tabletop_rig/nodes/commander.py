import argparse
import asyncio
import concurrent.futures
import importlib
import traceback
from collections.abc import Callable, Coroutine, Mapping
from types import TracebackType
from typing import Any, Literal, Optional, Self, TypeVar

import debugpy
import rclpy
import rclpy.utilities
from geometry_msgs.msg import PoseStamped
from mingus.containers import Note
from moveit.core.robot_trajectory import (  # type: ignore[reportMissingModuleSource]
    RobotTrajectory,
)
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from tabletop_interfaces.msg import TeensySensor

from tabletop_rig.exceptions import (
    ActionCallUnsuccessfulError,
    CommanderRecoverableError,
    ExecutionError,
    ExecutionInterruptedError,
    NotSafeToExecuteError,
    ServiceCallUnsuccessfulError,
)
from tabletop_rig.interfaces.dashboard import DashboardInterface
from tabletop_rig.interfaces.eyelink import EyelinkInterface
from tabletop_rig.interfaces.flic import FlicInterface
from tabletop_rig.interfaces.moveit.moveit import MoveItInterface
from tabletop_rig.interfaces.moveit.requests import PlanGoalT
from tabletop_rig.interfaces.sound import SoundInterface
from tabletop_rig.interfaces.teensy import TeensyInterface
from tabletop_rig.nodes.base import BaseNode

T = TypeVar("T", bound=Callable[..., Coroutine])


def safe_execution(coro_fn: T) -> T:
    """Decorator for methods that should be run with the object manipulation lock."""

    async def wrapper(self: "Commander", *args: Any, **kwargs: Any):
        max_retries = self.get_parameter_wrapper("safe_execution.max_retries")
        for i in range(max_retries):
            try:
                return await coro_fn(self, *args, **kwargs)
            except NotSafeToExecuteError as e:
                if i == max_retries - 1:
                    raise

                self.log(
                    f"Error while planning and executing: {e}. Locking arms and waiting for safety before retrying",
                    severity="WARN",
                )
                await self.teensy.lock_arms_and_wait()
                self.log(
                    "Arms locked and safe to execute, retrying plan_and_execute",
                    severity="WARN",
                )
            except ExecutionInterruptedError as e:
                if i == max_retries - 1:
                    raise

                self.log(
                    f"Error while planning and executing: {e}. Resetting dashboard before retrying",
                    severity="WARN",
                )
                await asyncio.sleep(2)
                await self.dashboard.reset()
                self.log(
                    "Dashboard reset, retrying plan_and_execute",
                    severity="WARN",
                )

    return wrapper  # type: ignore[reportReturnType]


class Commander(BaseNode):
    default_params: dict[str, Any] = BaseNode.default_params | {}
    required_params: set[str] = BaseNode.required_params | {
        "simulate",
        "max_workers",
        "dashboard.installation",
        "dashboard.program",
        "teensy.spin_period",
        "link_padding",
        "planning_scene.dir",
        "planning_scene.use_saved_scene",
        "planning_scene.object_meshes",
        "planning_scene.rig_meshes",
        "planning.defaults",
        "planning.pose_tolerance.position_tolerance",
        "planning.pose_tolerance.orientation_tolerance",
        "predefined_states.idle_state",
        "predefined_poses.pre_fetch_offset",
        "predefined_poses.pre_attach_offset",
        "predefined_poses.post_attach_offset",
        "predefined_poses.post_fetch_offset",
        "predefined_poses.pre_present_pose",
        "trajectory_cache.use_cached_trajectories",
        "trajectory_cache.freeze_cache",
        "trajectory_cache.kwargs",
        "object_manipulation.detach_velocity_scaling_factor",
        "object_manipulation.allowed_collisions",
        "object_manipulation.touch_links",
        "object_manipulation.mount_ids",
        "smooth_pursuit.reward_duration",
        "smooth_pursuit.reward_interval",
        "smooth_pursuit.reward_threshold_ratio",
        "safe_execution.max_retries",
    }

    ###########################################################################
    ########## Initialization #################################################
    ###########################################################################

    def __init__(self):
        """Initializes the Commander node.

        Sets up MoveItPy, trajectory execution manager, robot model, and planning scene monitor.
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
            self, lambda: self.teensy.safe_to_execute
        )

        self.log("Commander initialized")

    def _teensy_sensor_callback(self, msg: TeensySensor):
        """Additional callback for the Teensy sensor subscription.

        If Teensy interface indicates it is not safe to execute and the
        MoveIt inteface is executing, stop execution
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

    async def release_arm(self, arm: Literal["left", "right", "both"]):
        """Release the arm lock."""
        await self.teensy.set_arm_lock(arm, lock=False)

    async def lock_arms_and_wait(
        self, timeout: Optional[float] = None
    ) -> bool:
        """Lock arms and wait for safety laser to be unbroken"""
        return await self.teensy.lock_arms_and_wait(timeout)

    async def reveal_smartglass(self):
        """Reveal the smartglass."""
        await self.teensy.set_smartglass(reveal=True)

    async def occlude_smartglass(self):
        """Occlude the smartglass."""
        await self.teensy.set_smartglass(reveal=False)

    async def stop_reward(self):
        """Set the reward state."""
        await self.teensy.set_reward(activate=False)

    async def start_reward_and_wait(self, duration: float):
        """Start reward and wait for it to finish."""
        await self.teensy.start_reward_and_wait(duration)

    async def flic_response_time(
        self, timeout: Optional[float] = None
    ) -> float | None:
        """Wait for flic button press, then return response time, or None if timeout is reached."""
        object_id = self.moveit.get_exactly_one_attached_object_id()
        bd_addr = self.get_parameter_wrapper(f"flic.bd_addrs.{object_id}")
        return await self.flic.response_time(bd_addr, timeout)

    async def smooth_pursuit_and_reward(self):
        """Get Eyelink smooth pursuit state and reward if smooth pursuit is active"""
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

            duration = self.get_parameter_wrapper(
                "smooth_pursuit.reward_duration"
            )
            interval = self.get_parameter_wrapper(
                "smooth_pursuit.reward_interval"
            )
            reward_threshold = self.get_parameter_wrapper(
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

    def create_pose_stamped(
        self, *, frame_id: Optional[str] = None, **kwargs: Any
    ) -> PoseStamped:
        """Create a PoseStamped message from keyword arguments.

        Uses planning frame as default frame id if not specified.
        """
        return self.moveit.create_pose_stamped(frame_id=frame_id, **kwargs)

    async def plan(self, *args, **kwargs) -> RobotTrajectory | None:
        """Plan to a given goal

        Args:
            goal: The gaol to plan to
        """
        trajectory, _ = await self.moveit.plan(*args, **kwargs)
        return trajectory

    @safe_execution
    async def execute(self, *args, **kwargs):
        """Plan to a given goal

        Args:
            goal: The gaol to plan to
        """
        await self.moveit.execute(*args, **kwargs)

    @safe_execution
    async def plan_and_execute(self, *args, **kwargs):
        """Plan and execute to a given goal

        Args:
            goal: The gaol to plan to
        """
        await self.moveit.plan_and_execute(*args, **kwargs)

    @safe_execution
    async def fetch_object(self, object_id: str):
        """Fetch an object from its mount.

        The robot moves to the object's mount, attaches the object, and moves
        to the object's post-fetch pose.

        Args:
            object_id: The ID of the object to fetch
            cache_trajectories: Whether to cache the trajectories after fetching
                the object

        Raises:
            ValueError: If the object ID is not a valid collision object
            PlanningError: If the planning fails
            ExecutionError: If the execution fails
        """
        await self.moveit.fetch_object(object_id)

    @safe_execution
    async def pre_present_object(self):
        """Move to pre-present goal

        Args:
            goal: The goal at which to present the object
        """
        await self.moveit.pre_present_object()

    @safe_execution
    async def unpresent_object(self):
        """Unpresent the currently attached object"""
        await self.moveit.unpresent_object()

    @safe_execution
    async def return_object(self):
        """Return an object to its original position.

        Raises:
            RuntimeError: If exactly one object is not attached
            PlanningError: If the planning fails
            ExecutionError: If the execution fails
        """
        await self.moveit.return_object()

    def attach_object_manually(self, object_id: str):
        """Add a manually attached collision object to the end effector"""
        self.moveit.add_manually_attached_collision_object(object_id)

    # async def plan_and_execute(
    #     self, *args: Any, max_attempts: Optional[int] = None, **kwargs: Any
    # ) -> dict[str, Any] | None:
    #     """Plan and execute a trajectory
    #
    #     Attempts to call `_plan_and_execute_impl()` up to `max_attempts` times,
    #     retrying if the robot is not safe to execute or the execution is
    #     interrupted, waiting for safety before retrying.
    #
    #     See Also:
    #         `_plan_and_execute_impl()`: For parameter and implementation details.
    #     """
    #     if max_attempts is None:
    #         max_attempts = cast(
    #             int,
    #             self.get_parameter_wrapper("plan_and_execute.max_attempts"),
    #         )
    #
    #     if max_attempts < 1:
    #         raise ValueError(
    #             f"max_attempts should be greater than or equal to 1, got {max_attempts}"
    #         )
    #
    #     first_attempt = True
    #     for i in range(max_attempts):
    #         if i > 0:
    #             first_attempt = False
    #             kwargs["cache_trajectory"] = False
    #         try:
    #             response = await self._plan_and_execute_impl(*args, **kwargs)
    #             break
    #         except NotSafeToExecuteError as e:
    #             if i == max_attempts - 1:
    #                 raise
    #             self.log(
    #                 f"Error while planning and executing: {e}. Locking arms and waiting for safety before retrying",
    #                 severity="WARN",
    #             )
    #             await self._arm_lock_and_wait()
    #             self.log(
    #                 "Arms locked and safe to execute, retrying plan_and_execute",
    #                 severity="WARN",
    #             )
    #         except ExecutionInterruptedError as e:
    #             if i == max_attempts - 1:
    #                 raise
    #             self.log(
    #                 f"Error while planning and executing: {e}. Resetting dashboard before retrying",
    #                 severity="WARN",
    #             )
    #             await asyncio.sleep(2)
    #             await self.reset_dashboard()
    #             self.log(
    #                 "Dashboard reset, retrying plan_and_execute",
    #                 severity="WARN",
    #             )
    #
    #     if first_attempt:
    #         return response  # type: ignore
    #     else:
    #         return None

    # TODO: Move retry logic to Commander

    async def reset_commander(
        self,
        timeout: Optional[float] = None,
        end_goal: Optional[PlanGoalT] = None,
    ):
        """Reset the dashboard and the robot until successful or timeout.

        Args:
            goal: Optional pose to move to after resetting the robot
            timeout: Optional timeout for resetting the commander
        """
        self.log("Resetting commander")

        async with asyncio.timeout(timeout):
            while True:
                try:
                    if not self.teensy.safe_to_execute:
                        self.log(
                            "Cannot reset commander until safe to execute",
                            severity="WARN",
                        )
                        await self.teensy.lock_arms_and_wait()
                    await self.dashboard.reset(timeout)
                    await self.moveit.reset_rig(end_goal)
                    break
                except (
                    ServiceCallUnsuccessfulError,
                    ActionCallUnsuccessfulError,
                    CommanderRecoverableError,
                ) as e:
                    self.log(
                        "Caught exception while resetting commander:",
                        severity="WARN",
                    )
                    self.log(f"{type(e).__name__}: {e}", severity="WARN")
                    self.log(
                        f"Traceback: {traceback.format_exc()}",
                        severity="DEBUG",
                    )
                    if isinstance(e, ExecutionError):
                        sleep_time = 5
                    else:
                        sleep_time = 1
                    self.log(
                        f"Sleeping for {sleep_time} seconds before retrying",
                        severity="WARN",
                    )
                    await asyncio.sleep(sleep_time)

    ###########################################################################
    ########## Context manager ################################################
    ###########################################################################

    async def __aenter__(self) -> Self:
        """Enter the context manager."""
        self.log("Entering commander context manager", severity="DEBUG")
        self.moveit.__enter__()
        await self.reset_commander()
        return self

    async def __aexit__(
        self,
        exc_type: Optional[type[BaseException]],
        exc_value: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> bool:
        """Exit the context manager."""
        self.log("Exiting commander context manager", severity="DEBUG")
        try:
            if exc_type is not None:
                if isinstance(exc_value, CommanderRecoverableError):
                    self.log(
                        "Caught exception while running commander:",
                        severity="ERROR",
                    )
                    self.log(
                        f"{exc_type.__name__}: {exc_value}", severity="ERROR"
                    )
                    self.log(f"Traceback: {exc_tb}", severity="DEBUG")
                    if exc_type is ExecutionError:
                        self.log(
                            "Sleeping for 5 seconds before resetting commander",
                            severity="WARN",
                        )
                        await asyncio.sleep(5)
                    await self.reset_commander(end_goal="idle")
                    return True
            return False
        finally:
            self.moveit.__exit__(exc_type, exc_value, exc_tb)

    ###########################################################################
    ########## Destroy ########################################################
    ###########################################################################

    def destroy_node(self):
        if hasattr(self, "moveit"):
            self.moveit.destroy()
        super().destroy_node()

    def __del__(self):
        self.destroy_node()


async def debug_commander(commander: Commander, config: Optional[str] = None):
    """Run the commander node interactively with a debugger.

    Waits indefinitely
    """
    del config

    commander.log("Running commander interactively")

    debugpy.breakpoint()

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
):
    """Run a coroutine in an asyncio event loop.

    This function sets the default executor for the asyncio event loop to the
    thread pool executor provided. Used to run coroutines in a custom thread
    pool executor for performance reasons (e.g. more workers).

    Args:
        coro: The coroutine to run.
    """
    tpe = concurrent.futures.ThreadPoolExecutor(max_workers=max_workers)
    loop = asyncio.get_event_loop()
    loop.set_default_executor(tpe)

    task = asyncio.create_task(coro_fn(commander, config))
    spin_future.add_done_callback(
        lambda _: loop.call_soon_threadsafe(task.cancel)
        if loop.is_running()
        else None
    )

    await task


def main(args=None):
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

        if args.coro_module is not None or args.coro_name is not None:
            if args.coro_name is None or args.coro_module is None:
                raise ValueError(
                    "Both coro_module and coro_name must be provided when one is provided"
                )
            print(
                f"Loading coroutine {args.coro_name} from module {args.coro_module} "
            )
            coro_fn: Callable[[Commander, Optional[str]], Coroutine] = getattr(
                importlib.import_module(args.coro_module),
                args.coro_name,
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
            print("Debug mode enabled")
            debugpy.listen(1300)
            print("Waiting for debugger to attach")
            debugpy.wait_for_client()
            print("Debugger attached")

        executor: SingleThreadedExecutor | MultiThreadedExecutor = (
            SingleThreadedExecutor()
        )
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
