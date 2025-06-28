import argparse
import asyncio
import concurrent.futures
import glob
import hashlib
import importlib
import json
import os
import pickle
import threading
import traceback
from collections.abc import (
    Callable,
    Coroutine,
    Iterable,
    Mapping,
)
from copy import deepcopy
from enum import IntEnum
from types import TracebackType
from typing import Any, ContextManager, Literal, Optional, Self, cast

import debugpy
import numpy as np
import pandas as pd
import rclpy
import rclpy.utilities
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.collision_detection import (  # type: ignore
    AllowedCollisionMatrix,  # type: ignore
    CollisionRequest,
    CollisionResult,
)
from moveit.core.planning_interface import MotionPlanResponse  # type: ignore
from moveit.core.planning_scene import PlanningScene  # type: ignore
from moveit.core.robot_model import RobotModel  # type: ignore
from moveit.core.robot_state import RobotState  # type: ignore
from moveit.core.robot_trajectory import RobotTrajectory  # type: ignore
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
    PlanningComponent,
    PlanningSceneMonitor,
    PlanRequestParameters,
    TrajectoryExecutionManager,
)
from moveit_msgs.msg import AllowedCollisionMatrix as AllowedCollisionMatrixMsg
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    LinkPadding,
    ObjectColor,
)
from moveit_msgs.msg import (
    PlanningScene as PlanningSceneMsg,
)
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.executors import MultiThreadedExecutor
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.qos import QoSPresetProfiles
from std_srvs.srv import Trigger
from tabletop_interfaces.action import FlicResponseTime
from tabletop_interfaces.msg import TeensySensor
from tabletop_interfaces.srv import (
    SetArmLock,
    SetReward,
    SetSmartglass,
)
from tabletop_utils.mesh import (
    load_geometry,
    simplify_convex_hull,
    simplify_quadratic_decimation,
    transform_geometry,
)
from tabletop_utils.ros import (
    MOVEIT_ERROR_CODE_MAP,
    CommanderRecoverableError,
    ExecuteRequest,
    ExecutionError,
    MaxAttemptsReachedError,
    MaxExecutionAttemptsReachedError,
    MaxPlanningAttemptsReachedError,
    NotSafeToExecuteError,
    ObjectManipulationError,
    PlanningError,
    PlanningGoalT,
    PlanRequest,
    ServiceCallError,
    ServiceCallUnsuccessfulError,
    TrajectoryError,
    TrajectoryErrorCodes,
    add_mesh_collision_object_msg,
    add_plane_collision_object_msg,
    add_primitive_collision_object_msg,
    add_primitive_collision_object_msg_from_geometry,
    all_close_poses_stamped,
    all_close_robot_states,
    arrays_from_pose_msg,
    attached_collision_object_msg,
    change_reference_frame_pose_stamped,
    matrix_from_pose_msg,
    object_color_msg,
    pose_msg,
    pose_msg_from_matrix,
    pose_stamped_msg,
    robot_trajectory_copy,
)
from tabletop_utils.trajectory_cache import (
    FuzzyTrajectoryCache,
)
from tf_transformations import identity_matrix
from ur_dashboard_msgs.srv import Load

from tabletop_server.nodes.base import BaseNode


def asyncio_task_decorator[T](
    coro_fn: Callable[..., Coroutine[None, None, T]],
) -> Callable[..., asyncio.Task[T]]:
    """
    Decorator for methods that should be run in the current asyncio.TaskGroup.

    This decorator is designed for BaseNode methods. It will only work for
    methods whose first argument is `self` and whose class has an
    `asyncio.TaskGroup` attribute named `tg`.

    WARNING: If a task raises an exception, all tasks in the TaskGroup will
    be cancelled. As a result, you should not use this decorator for coroutines
    that are expected to raise exceptions (e.g. you cannot catch exceptions of tasks).

    Args:
        coro_fn: The coroutine function to decorate.

    Returns:
        The decorated function which returns an asyncio.Task.
    """

    def wrapper(*args: Any, **kwargs: Any) -> asyncio.Task:
        """Wrapper function that creates and returns an asyncio.Task.

        Args:
            self: The instance of the class.
            *args: Positional arguments for the coroutine function.
            **kwargs: Keyword arguments for the coroutine function.

        Returns:
            The created asyncio.Task.
        """
        coro = coro_fn(*args, **kwargs)
        return asyncio.create_task(coro)

    return wrapper


def object_manipulation_lock_decorator(
    coro_fn: Callable[..., Coroutine],
) -> Callable[..., Coroutine]:
    """Decorator for methods that should be run with the object manipulation lock."""

    async def wrapper(self: "Commander", *args: Any, **kwargs: Any):
        async with self.object_manipulation_lock:
            return await coro_fn(self, *args, **kwargs)

    return wrapper


def safe_to_execute_decorator(
    coro_fn: Callable[..., Coroutine],
) -> Callable[..., Coroutine]:
    """Decorator for methods that should be run with the safe to execute lock."""

    async def wrapper(self: "Commander", *args: Any, **kwargs: Any):
        while True:
            try:
                return await coro_fn(self, *args, **kwargs)
            except NotSafeToExecuteError as e:
                self.log(
                    f"Not safe to execute while running {coro_fn.__name__}: {e}",
                    severity="WARN",
                )
                self.log(
                    f"{coro_fn.__name__} called with args: {args} and kwargs: {kwargs}",
                    severity="WARN",
                )
                self.log(
                    "Locking arms and waiting for safe to execute",
                    severity="WARN",
                )
                await self._arm_lock_and_wait()
                self.log(
                    f"Arms locked and safe to execute, retrying {coro_fn.__name__}",
                    severity="WARN",
                )

    return wrapper


class ObjectPhase(IntEnum):
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


OBJECT_MOUNT_PHASES = [
    ObjectPhase.PRE_ATTACH,
    ObjectPhase.ATTACH,
    ObjectPhase.POST_ATTACH,
    ObjectPhase.POST_FETCH,
    ObjectPhase.PRE_DETACH,
    ObjectPhase.DETACH,
    ObjectPhase.POST_DETACH,
    ObjectPhase.POST_RETURN,
]


class Commander(BaseNode):
    default_params: dict[str, Any] = BaseNode.default_params | {}
    required_params: set[str] = BaseNode.required_params | {
        "simulate",
        "max_workers",
        "dashboard.installation",
        "dashboard.program",
        "teensy.spin_period",
        "planning.defaults",
        "planning.position_tolerance",
        "planning.orientation_tolerance",
        "planning.use_euler_tolerance",
        "execution.defaults",
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
        "link_padding",
        "planning_scene.dir",
        "planning_scene.use_saved_scene",
        "planning_scene.object_meshes",
        "planning_scene.rig_meshes",
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

        self.init_ros()

        self.moveit_py = MoveItPy("moveit_py", provide_planning_service=True)

        self.init_planning_scene()

        self.init_attached_object()

        self.init_link_padding()

        self.init_extra_attributes()

        self.log("Commander initialized")

    def init_extra_attributes(self):
        """Setup variables for the commander."""
        # Trajectory cache
        trajectory_cache_config = self.get_parameter_wrapper(
            "trajectory_cache.kwargs"
        )
        self.trajectory_cache = FuzzyTrajectoryCache(
            rig_hash=self.rig_hash, **trajectory_cache_config
        )

        # Whether the robot has been reset
        self.initial_reset = False

        # Execution lock
        self.execution_lock = threading.Lock()

        # Object manipulation lock
        self.object_manipulation_lock = asyncio.Lock()

    def init_ros(self):
        """Create services for the commander.

        Services:
        - /commander/get_frame_transform
        - /commander/add_collision_object
        - /commander/remove_collision_object
        - /commander/attach_collision_object
        - /commander/detach_collision_object
        - /commander/allow_collision
        - /commander/disallow_collision
        - /commander/plan_and_execute
        """
        # Subscribers
        self.teensy_sub = self.create_subscription(
            TeensySensor,
            "/teensy/sensor",
            self.teensy_sensor_callback,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._last_teensy_sensor = TeensySensor()
        self._safe_to_execute_count = 0
        self._safe_to_execute = False
        self._teensy_sensor_lock = threading.Lock()

        self.set_arm_lock_client = self.create_client(
            SetArmLock,
            "/teensy/set_arm_lock",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.set_reward_client = self.create_client(
            SetReward,
            "/teensy/set_reward",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.set_smartglass_client = self.create_client(
            SetSmartglass,
            "/teensy/set_smartglass",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.set_arm_lock_client.wait_for_service()
        self.set_reward_client.wait_for_service()
        self.set_smartglass_client.wait_for_service()

        # Action clients
        self.flic_response_time_client = ActionClient(
            self,
            FlicResponseTime,
            "/flic/response_time",
        )
        self.flic_response_time_client.wait_for_server()

        # Services
        # TODO: Add services
        # self.create_service(
        #     GetFrameTransform,
        #     "/commander/get_frame_transform",
        #     self.get_frame_transform_callback,
        # )
        # self.create_service(
        #     AddCollisionObject,
        #     "/commander/add_collision_object",
        #     self.add_collision_object_callback,
        # )
        # self.create_service(
        #     RemoveCollisionObject,
        #     "/commander/remove_collision_object",
        #     self.remove_collision_object_callback,
        # )
        # self.create_service(
        #     AttachCollisionObject,
        #     "/commander/attach_collision_object",
        #     self.attach_collision_object_callback,
        # )
        # self.create_service(
        #     DetachCollisionObject,
        #     "/commander/detach_collision_object",
        #     self.detach_collision_object_callback,
        # )
        # self.create_service(
        #     AllowCollision,
        #     "/commander/allow_collision",
        #     self.allow_collision_callback,
        # )
        # self.create_service(
        #     DisallowCollision,
        #     "/commander/disallow_collision",
        #     self.disallow_collision_callback,
        # )
        # self.create_service(
        #     PlanAndExecute,
        #     "/commander/plan_and_execute",
        #     self.plan_and_execute_callback,
        # )

    ###########################################################################
    ########## MoveItPy Interface #############################################
    ###########################################################################

    def get_planning_component(
        self, group_name: Optional[str] = None
    ) -> PlanningComponent:
        """Get the planning component for a given planning group name.

        Args:
            group_name: The name of the planning group. If None, the default planning group is used.

        Returns:
            The planning component for the specified group.
        """
        if group_name is None:
            group_name = self.default_group_name
        return self.moveit_py.get_planning_component(group_name)

    @property
    def planning_scene_monitor(self) -> PlanningSceneMonitor:
        """Get the planning scene monitor."""
        return self.moveit_py.get_planning_scene_monitor()

    @property
    def robot_model(self) -> RobotModel:
        """Get the robot model."""
        return self.moveit_py.get_robot_model()

    @property
    def trajectory_execution_manager(self) -> TrajectoryExecutionManager:
        """Get the trajectory execution manager."""
        return self.moveit_py.get_trajectory_execution_manager()

    ###########################################################################
    ########## Parameters #####################################################
    ###########################################################################

    @property
    def simulate(self) -> bool:
        """Get the simulation flag."""
        return self.get_parameter_wrapper("simulate")

    @property
    def default_group_name(self) -> str:
        """Get the planning group name from the parameter server."""
        return self.get_parameter_wrapper("planning.defaults.group_name")

    @property
    def default_pose_link(self) -> str:
        """Get the planning link from the parameter server."""
        return self.get_parameter_wrapper("planning.defaults.pose_link")

    @property
    def allowed_object_mount_collisions(self) -> list[tuple[str, str]]:
        """Get the allowed object mount collisions from the parameter server."""
        return [
            (id_0, id_1)
            for id_0, id_1 in self.get_parameter_wrapper(
                "object_manipulation.allowed_collisions"
            ).items()
        ]

    @property
    def touch_links(self) -> list[str]:
        """Get the touch links from the parameter server."""
        return self.get_parameter_wrapper("object_manipulation.touch_links")

    @property
    def object_mount_ids(self) -> list[str]:
        """Get the object mount ids from the parameter server."""
        return self.get_parameter_wrapper("object_manipulation.mount_ids")

    @property
    def use_cached_trajectories(self) -> bool:
        return self.get_parameter_wrapper(
            "trajectory_cache.use_cached_trajectories"
        )

    @property
    def freeze_trajectory_cache(self) -> bool:
        return self.get_parameter_wrapper("trajectory_cache.freeze_cache")

    @property
    def object_grid(self) -> np.ndarray:
        """Get the object grid config from the parameters."""
        object_kwargs = self.get_parameter_wrapper(
            "planning_scene.object_meshes.object_kwargs"
        )

        object_grid = np.empty((10, 3), dtype=object)
        for object_id, kwargs in object_kwargs.items():
            x, y = kwargs["idx"]
            object_grid[x, y] = object_id

        return object_grid

    ###########################################################################
    ########## Logging ########################################################
    ###########################################################################

    def log_plan_response(
        self,
        plan_response: MotionPlanResponse,
        attempt: int,
        max_attempts: int,
        severity: str = "INFO",
    ):
        """Log a motion plan response.

        Args:
            plan_response: The motion plan response to log.
            severity: The logging severity level.
        """
        if self.log_level < LoggingSeverity[severity]:
            return

        msg = []
        if plan_response:
            msg.append(f"Plan attempt {attempt + 1}/{max_attempts} succeeded")
            msg.append(f"planner id: {plan_response.planner_id}")
            msg.append(f"planning time: {plan_response.planning_time} s")
        else:
            msg.append(f"Plan attempt {attempt + 1}/{max_attempts} failed")
            msg.append(
                f"error code: {MOVEIT_ERROR_CODE_MAP[plan_response.error_code.val]}"
            )
        self.log(" | ".join(msg), severity=severity)

    def log_planning_scene(self, severity: str = "INFO"):
        """Log the planning scene."""
        if self.log_level < LoggingSeverity[severity]:
            return

        self.log("Logging planning scene", severity=severity)
        with self.planning_scene_read_only() as scene:
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
        with self.planning_scene_read_only() as scene:
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

        self.log("Collision object init kwargs:", severity=severity)
        for object_id, kwargs in self.collision_object_init_kwargs.items():
            self.log(
                f"Collision object id: {object_id}",
                severity=severity,
            )
            self.log(f"kwargs: {kwargs}", severity=severity)
            self.log("=" * 80, severity=severity)

    ###########################################################################
    ########## UR Dashboard Interface #########################################
    ###########################################################################

    async def dashboard_trigger(self, srv_name: str) -> Trigger.Response:
        """Call a dashboard client Trigger service (asynchronous)."""
        self.log(
            f"Triggering {srv_name} in UR Dashboard",
            severity="DEBUG",
        )
        response = await self.service_call_async(
            srv_request=Trigger.Request(), srv_type=Trigger, srv_name=srv_name
        )
        return cast(Trigger.Response, response)

    async def dashboard_load(
        self,
        srv_name: str,
        filename: str,
    ) -> Load.Response:
        """Load a program or installation on the robot dashboard (asynchronous)."""
        self.log(
            f"Loading {srv_name}: {filename} in UR Dashboard",
            severity="DEBUG",
        )
        response = await self.service_call_async(
            srv_request=Load.Request(filename=filename),
            srv_type=Load,
            srv_name=srv_name,
        )
        return cast(Load.Response, response)

    async def wait_for_dashboard(
        self,
        service_name: str,
        timeout: Optional[float] = None,
    ):
        """Wait for the dashboard to be initialized (asynchronous)."""
        self.log("Waiting for dashboard to be ready", severity="DEBUG")
        await self.wait_for_service_async(
            srv_type=Trigger,
            srv_name=f"/dashboard_client/{service_name}",
            timeout=timeout,
        )

    async def reset_dashboard(self, timeout: Optional[float] = None):
        """Call a sequence of dashboard client services to reset the dashboard (asynchronous)."""
        self.log("Resetting dashboard")
        config = self.get_parameter_wrapper("dashboard")
        async with asyncio.timeout(timeout):
            while True:
                # Timeout included in wait_for_dashboard to stop the thread
                # from waiting longer than timeout
                await self.wait_for_dashboard("close_popup", timeout)
                await self.dashboard_trigger("/dashboard_client/close_popup")
                await self.dashboard_trigger(
                    "/dashboard_client/close_safety_popup"
                )
                await self.dashboard_trigger(
                    "/dashboard_client/unlock_protective_stop"
                )
                await self.dashboard_load(
                    "/dashboard_client/load_program", config["program"]
                )
                await self.dashboard_trigger("/dashboard_client/brake_release")
                for _ in range(config["play_retries"]):
                    try:
                        await self.dashboard_trigger("/dashboard_client/play")
                        return
                    except ServiceCallUnsuccessfulError:
                        self.log(
                            f"Failed attempt to play dashboard program, "
                            f"retrying after {config['play_retry_delay']} seconds...",
                            severity="WARN",
                        )
                        await asyncio.sleep(config["play_retry_delay"])

    ###########################################################################
    ########## ROS Interface ##################################################
    ###########################################################################

    # Subscribers

    def teensy_sensor_callback(self, msg: TeensySensor):
        """Callback for the teensy sensor."""
        safe_to_execute_required_count = self.get_parameter_wrapper(
            "teensy.safe_to_execute_required_count"
        )
        with self._teensy_sensor_lock:
            self._last_teensy_sensor = msg

            if (
                msg.is_left_arm_locked
                and msg.is_right_arm_locked
                and not msg.is_safety_laser_broken
            ):
                self._safe_to_execute_count += 1
            else:
                self._safe_to_execute_count = 0

            self._safe_to_execute = (
                self._safe_to_execute_count > safe_to_execute_required_count
            )

        # If it is not safe to execute and the robot is executing, stop execution
        if not self.safe_to_execute and self.execution_lock.locked():
            self.log(
                "Arms not locked or safety laser broken, stopping execution",
                severity="WARN",
            )
            self.trajectory_execution_manager.stop_execution()

    @property
    def last_teensy_sensor(self) -> TeensySensor:
        """Get the last teensy sensor."""
        with self._teensy_sensor_lock:
            return deepcopy(self._last_teensy_sensor)

    @property
    def safe_to_execute(self) -> bool:
        """Get the is safe to execute state."""
        with self._teensy_sensor_lock:
            return self._safe_to_execute

    # Service client setters

    async def _set_arm_lock(
        self, arm: Literal["left", "right", "both"], lock: bool
    ) -> SetArmLock.Response:
        """Set the arm lock state."""
        if arm not in ["left", "right", "both"]:
            raise ValueError("Invalid arm: must be 'left', 'right', or 'both'")

        left = arm in ["left", "both"]
        right = arm in ["right", "both"]

        response = await self.service_call_async(
            srv_request=SetArmLock.Request(
                left_arm=left, right_arm=right, lock=lock
            ),
            srv_client=self.set_arm_lock_client,
        )
        return cast(SetArmLock.Response, response)

    @asyncio_task_decorator
    async def arm_release(
        self, arm: Literal["left", "right", "both"]
    ) -> SetArmLock.Response:
        """Release the arm lock."""
        return await self._set_arm_lock(arm, lock=False)

    async def _arm_lock_and_wait(
        self, timeout: Optional[float] = None
    ) -> bool:
        """Lock arms and wait for safety laser to be unbroken

        Args:
            timeout: Timeout in seconds. If None, the default timeout from
                parameters is used.

        Returns:
            True if arms were locked and safety laser was unbroken within the timeout,
            False otherwise.
        """
        self.log("Locking arms and waiting until safe to execute")
        await self._set_arm_lock("both", lock=True)

        spin_period = self.get_parameter_wrapper("teensy.spin_period")
        try:
            async with asyncio.timeout(timeout):
                while not self.safe_to_execute:
                    await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    @asyncio_task_decorator
    async def arm_lock_and_wait(self, timeout: Optional[float] = None) -> bool:
        """Lock arms and wait for safety laser to be unbroken"""
        return await self._arm_lock_and_wait(timeout)

    async def _set_reward(self, duration: float) -> SetReward.Response:
        """Set the reward state."""
        self.log(f"Delivering reward for {duration} s")
        if duration < 0:
            raise ValueError("Duration must be greater than 0!")

        duration_msg = Duration(seconds=duration).to_msg()
        response = await self.service_call_async(
            srv_request=SetReward.Request(duration=duration_msg),
            srv_client=self.set_reward_client,
        )
        return cast(SetReward.Response, response)

    @asyncio_task_decorator
    async def reward_and_wait(self, duration: float):
        """Start reward and wait for it to be active."""
        await self._set_reward(duration)

        spin_period = self.get_parameter_wrapper("teensy.spin_period")
        timeout = duration + spin_period
        try:
            async with asyncio.timeout(timeout):
                while self.last_teensy_sensor.is_reward_active:
                    await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    async def _set_smartglass(self, reveal: bool) -> SetSmartglass.Response:
        """Set the smartglass state."""
        self.log(f"Smartglass {'reveal' if reveal else 'occlude'}")
        response = await self.service_call_async(
            srv_request=SetSmartglass.Request(reveal=reveal),
            srv_client=self.set_smartglass_client,
        )
        return cast(SetSmartglass.Response, response)

    @asyncio_task_decorator
    async def smartglass_reveal(self) -> SetSmartglass.Response:
        """Reveal the smartglass."""
        return await self._set_smartglass(True)

    @asyncio_task_decorator
    async def smartglass_occlude(self) -> SetSmartglass.Response:
        """Occlude the smartglass."""
        return await self._set_smartglass(False)

    # TODO: Potential race condition (between monkey and get_flic lol)
    @asyncio_task_decorator
    async def flic_response_time(
        self, timeout: Optional[float] = None
    ) -> float | None:
        """Wait for flic button press, then return response time, or None if timeout is reached."""
        try:
            async with asyncio.timeout(timeout):
                goal_handle = cast(
                    ClientGoalHandle,
                    await self.flic_response_time_client.send_goal_async(
                        FlicResponseTime.Goal()
                    ),
                )
                if not goal_handle.accepted:
                    raise RuntimeError("Flic goal not accepted")

                try:
                    response = await goal_handle.get_result_async()
                    response_time = Duration.from_msg(
                        response.result.response_time
                    )
                    return response_time.nanoseconds / 1e9
                except asyncio.CancelledError:
                    goal_handle.cancel_goal_async()
                    raise
        except TimeoutError:
            return None

    ###########################################################################
    ########## Planning scene #################################################
    ###########################################################################

    def planning_scene_read_only(
        self,
    ) -> ContextManager[PlanningScene]:
        """Get the planning scene in read-only mode."""
        return self.planning_scene_monitor.read_only()

    def planning_scene_read_write(
        self,
    ) -> ContextManager[PlanningScene]:
        """Get the planning scene in read-write mode."""
        return self.planning_scene_monitor.read_write()

    def get_planning_scene_copy(self) -> PlanningScene:
        """Get a copy of the planning scene."""
        with self.planning_scene_read_only() as scene:
            return deepcopy(scene)

    def save_planning_scene(self, path: str):
        """Save the planning scene to a file."""
        self.log(f"Saving planning scene to {path}")
        with self.planning_scene_read_only() as scene:
            scene.save_geometry_to_file(path)

    def load_planning_scene(self, path: str):
        """Load the planning scene from a file."""
        self.log(f"Loading planning scene from {path}")
        with self.planning_scene_read_write() as scene:
            scene.load_geometry_from_file(path)
            scene.current_state.update()

    def save_object_init_kwargs(self, path: str):
        """Save the object init kwargs to a file."""
        self.log(f"Saving object init kwargs to {path}")
        with open(path, "wb") as f:
            pickle.dump(self.collision_object_init_kwargs, f)

    def load_object_init_kwargs(self, path: str):
        """Load the object init kwargs from a file."""
        self.log(f"Loading object init kwargs from {path}")
        with open(path, "rb") as f:
            self.collision_object_init_kwargs = pickle.load(f)

    @property
    def planning_frame(self) -> str:
        """Get the planning frame from the planning scene."""
        with self.planning_scene_read_only() as scene:
            planning_frame = scene.planning_frame
            assert planning_frame == "world"
            return planning_frame

    @property
    def current_state(self) -> RobotState:
        with self.planning_scene_read_only() as scene:
            return deepcopy(scene.current_state)

    @property
    def collision_object_ids(self) -> list[str]:
        """Get the collision object ids from the planning scene."""
        with self.planning_scene_read_only() as scene:
            collision_objects: list[CollisionObject] = (
                scene.planning_scene_message.world.collision_objects
            )
            return [x.id for x in collision_objects]

    @property
    def collision_objects(self) -> dict[str, CollisionObject]:
        """Get the collision objects from the planning scene."""
        with self.planning_scene_read_only() as scene:
            collision_objects: list[CollisionObject] = (
                scene.planning_scene_message.world.collision_objects
            )
            return {x.id: deepcopy(x) for x in collision_objects}

    def get_collision_object(self, object_id: str) -> CollisionObject:
        """Get a collision object from the planning scene."""
        with self.planning_scene_read_only() as scene:
            collision_objects: list[CollisionObject] = (
                scene.planning_scene_message.world.collision_objects
            )
            for x in collision_objects:
                if x.id == object_id:
                    return deepcopy(x)
            raise ValueError(f"Collision object {object_id} not found")

    @property
    def attached_collision_object_ids(self) -> list[str]:
        """Get the attached collision object ids from the planning scene."""
        with self.planning_scene_read_only() as scene:
            attached_collision_objects: list[AttachedCollisionObject] = (
                scene.planning_scene_message.robot_state.attached_collision_objects
            )
            return [x.object.id for x in attached_collision_objects]

    @property
    def attached_collision_objects(self) -> dict[str, AttachedCollisionObject]:
        """Get the attached collision objects from the planning scene."""
        with self.planning_scene_read_only() as scene:
            attached_collision_objects: list[AttachedCollisionObject] = (
                scene.planning_scene_message.robot_state.attached_collision_objects
            )
            return {
                x.object.id: deepcopy(x) for x in attached_collision_objects
            }

    @property
    def collision_matrix_df(self) -> pd.DataFrame:
        """Get the collision matrix as a pandas DataFrame."""
        with self.planning_scene_read_only() as scene:
            msg: AllowedCollisionMatrixMsg = (
                scene.planning_scene_message.allowed_collision_matrix
            )
            object_ids = list(msg.entry_names)
            matrix = np.array([row.enabled for row in msg.entry_values])
            matrix_df = pd.DataFrame(
                matrix,
                columns=object_ids,  # type: ignore[arg-type]
                index=object_ids,  # type: ignore[arg-type]
            )

            robot_link_ids = [
                "base_link_inertia",
                "shoulder_link",
                "upper_arm_link",
                "forearm_link",
                "wrist_1_link",
                "wrist_2_link",
                "wrist_3_link",
                "eef_link",
                "eef_sphere",
            ]
            collision_object_ids = set(object_ids) - set(robot_link_ids)
            columns = robot_link_ids + list(collision_object_ids)
            matrix_df = matrix_df.loc[columns, columns]

            return matrix_df

    def init_link_padding(self):
        """Set the link padding for the planning scene."""
        config: dict[str, Any] = self.get_parameter_wrapper("link_padding")
        with self.planning_scene_read_write() as scene:
            msg = PlanningSceneMsg(
                is_diff=True,
                link_padding=[
                    LinkPadding(link_name=name, padding=padding)
                    for name, padding in config.items()
                ],
            )
            if not scene.set_planning_scene_diff_msg(msg):
                raise RuntimeError("Failed to set link padding")

    def _get_exactly_one_attached_object_id(self) -> str:
        """Get the ID of the exactly one attached collision object.

        Returns:
            The ID of the attached collision object.

        Raises:
            RuntimeError: If there is not exactly one attached collision object
        """
        attached_collision_object_ids = self.attached_collision_object_ids
        if len(attached_collision_object_ids) != 1:
            raise RuntimeError(
                f"Expected exactly one attached collision object, "
                f"but got {len(attached_collision_object_ids)}"
            )
        return attached_collision_object_ids[0]

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

        with self.planning_scene_read_only() as scene:
            result = CollisionResult()
            scene.check_collision(request, result)
            return result

    def is_state_colliding(self, group_name: Optional[str] = None) -> bool:
        """Check if the current state of the planning scene is colliding."""
        if group_name is None:
            group_name = self.default_group_name

        with self.planning_scene_read_only() as scene:
            return scene.is_state_colliding(group_name)

    def _parse_collision_matrix_entry(
        self, success: bool, allowed_collision_type: str
    ) -> bool:
        """Parse the collision matrix entry for two collision objects."""
        assert (
            success or allowed_collision_type == "NEVER"
        ), "Inconsistent collision matrix entry"
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
        with self.planning_scene_read_only() as scene:
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
        with self.planning_scene_read_write() as scene:
            matrix: AllowedCollisionMatrix = scene.allowed_collision_matrix
            for x, y in zip(ids_0, ids_1):
                success, allowed_collision_type = matrix.get_entry(x, y)
                allowed = self._parse_collision_matrix_entry(
                    success, allowed_collision_type
                )
                if allowed == allow:
                    self.log(
                        f"Collision between {x} and {y} is already "
                        f"{'allowed' if allow else 'disallowed'}",
                        severity="DEBUG",
                    )
                else:
                    self.log(
                        f"{'Allowing' if allow else 'Disallowing'} "
                        f"collision between {x} and {y}",
                        severity="DEBUG",
                    )
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
        dynamic: bool = False,
        pose_stamped: Optional[PoseStamped] = None,
        subframe_names: Optional[list[str]] = None,
        subframe_poses: Optional[list[Pose]] = None,
        color: Optional[
            ObjectColor | str | Iterable[float] | Mapping[str, float]
        ] = None,
        allowed_collision_ids: Optional[Iterable[str]] = None,
    ):
        """Process a collision object.

        Adds the collision object to the planning scene and saves the init kwargs.

        Args:
            collision_object: The collision object to process.
            dynamic: Whether the collision object is dynamic.
            pose_stamped: The pose of the collision object (only used if dynamic).
            color: The color of the collision object.
            allowed_collision_ids: The ids of the collision objects that are allowed to collide with this object.
        """
        self.log(
            f"Processing collision object: {collision_object.id}",
            severity="DEBUG",
        )

        # Check that pose stamped is provided for dynamic collision objects
        if dynamic and pose_stamped is None:
            raise ValueError(
                "Pose stamped is required for dynamic collision objects"
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

        # Save collision object kwargs if requested
        if collision_object.id in self.collision_object_init_kwargs:
            raise ValueError(
                f"Collision object {collision_object.id} already has init kwargs"
            )
        self.collision_object_init_kwargs[collision_object.id] = deepcopy(
            {
                "dynamic": dynamic,
                "pose_stamped": pose_stamped,
                "subframe_names": subframe_names,
                "subframe_poses": subframe_poses,
                "color": color,
                "allowed_collision_ids": allowed_collision_ids,
            }
        )

    def add_plane_collision_object(
        self,
        object_id: str,
        *,
        coef: list[float],
        pose_stamped: PoseStamped | Mapping[str, Any],
        dynamic: bool,
        allowed_collision_ids: Optional[list[str]] = None,
    ):
        """Add a plane collision object to the planning scene.

        Args:
            object_id: The id for the collision object.
            coef: The coefficients of the plane.
            dynamic: Whether the collision object is dynamic.
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
            dynamic=dynamic,
            pose_stamped=pose_stamped,
            allowed_collision_ids=allowed_collision_ids,
        )

    def add_primitive_collision_object(
        self,
        object_id: str,
        *,
        type: str,
        dimensions: list[float],
        pose_stamped: PoseStamped | Mapping[str, Any],
        dynamic: bool,
        color: Optional[str | Iterable[float] | Mapping[str, float]] = None,
        allowed_collision_ids: Optional[list[str]] = None,
    ):
        """Add a primitive collision object to the planning scene.

        Args:
            object_id: The id for the collision object.
            type: The type of the primitive.
            dimensions: The dimensions of the primitive.
            pose_stamped: The stamped pose of the collision object.
            dynamic: Whether the collision object is dynamic.
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
            dynamic=dynamic,
            pose_stamped=pose_stamped,
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
        simplification: Optional[str] = None,
        pose_stamped: PoseStamped | Mapping[str, Any],
        dynamic: bool,
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
            dynamic: Whether the collision object is dynamic.
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
            case "bounding_primitive" | None:
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
        if simplification == "bounding_primitive":
            collision_object = (
                add_primitive_collision_object_msg_from_geometry(
                    object_id=object_id,
                    pose_stamped=pose_stamped,
                    geometry=geometry,
                    subframe_names=subframe_names,
                    subframe_poses=subframe_poses,
                )
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
            dynamic=dynamic,
            pose_stamped=pose_stamped,
            subframe_names=subframe_names,
            subframe_poses=subframe_poses,
            color=color,
            allowed_collision_ids=allowed_collision_ids,
        )

    def add_dynamic_mesh_collision_objects(
        self,
        *,
        path: str,
        origin: list[float],
        positions_relative: dict[str, list[float]],
        common_kwargs: dict[str, Any],
        object_kwargs: dict[str, Any],
    ):
        """Add dynamic (object) meshes as collision objects to the planning scene.

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
        origin_arr = np.array(origin)

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

        for object_id, overrides in object_kwargs.items():
            # Skip if object already exists in the planning scene
            if object_id in self.collision_object_ids:
                self.log(
                    f"Skipping object mesh {object_id} because it already exists in the planning scene"
                )
                continue

            # Get common and per-object configurations
            kwargs: dict[str, Any] = deepcopy(common_kwargs)

            if (
                "allowed_collision_ids" in kwargs
                and "allowed_collision_ids" in overrides
            ):
                overrides["allowed_collision_ids"].extend(
                    kwargs["allowed_collision_ids"]
                )

            # Get index from per-object configurations and calculate position
            x, y = overrides.pop("idx")
            rel_pos = np.array(positions_relative[f"{x},{y}"])
            position = origin_arr + rel_pos

            # Override common configurations with per-object configurations
            kwargs.update(overrides)

            # Create pose stamped from common and per-object configurations
            # This will not work if the common or per-object configurations
            # contain a pose_stamped configuration that designates a position
            pose_stamped_kwargs: dict[str, Any] = kwargs.pop(
                "pose_stamped", {}
            )
            pose_stamped = self.create_pose_stamped(
                position=position, **pose_stamped_kwargs
            )

            try:
                mesh_path = object_id_to_path[object_id]
            except KeyError:
                raise ValueError(
                    f"Object mesh {object_id} not found in {path}"
                )

            self.add_mesh_collision_object(
                object_id=object_id,
                path=object_id_to_path[object_id],
                pose_stamped=pose_stamped,
                dynamic=True,
                **kwargs,
            )

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

        with self.planning_scene_read_write() as scene:
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

    def reset_collision_object(self, object_id: str):
        """Reset a collision object."""
        self.log(f"Resetting collision object: {object_id}")

        if object_id in self.attached_collision_object_ids:
            self.detach_collision_object(object_id)

        # Remove collision object from planning scene and update allowed collision matrix
        self.move_collision_object(
            object_id, self.object_init_pose_stamped(object_id)
        )

    @property
    def rig_hash(self) -> str:
        """Get the hash of the rig, for consistency purposes.

        Returns:
            The hash of the rig.
        """
        config = self.get_parameter_wrapper("planning_scene")

        hash_algorithm = hashlib.md5()

        # Hash rig meshes
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

        # Hash plane collision objects
        keys_to_hash = ["pose_stamped", "coef"]
        for object_id, kwargs in config["planes"].items():
            hash_algorithm.update(object_id.encode("utf-8"))
            for key in keys_to_hash:
                if key in kwargs:
                    hash_algorithm.update(
                        json.dumps(kwargs[key], sort_keys=True).encode("utf-8")
                    )

        # Hash primitive collision objects
        keys_to_hash = ["pose_stamped", "type", "dimensions"]
        for object_id, kwargs in config["primitives"].items():
            hash_algorithm.update(object_id.encode("utf-8"))
            for key in keys_to_hash:
                if key in kwargs:
                    hash_algorithm.update(
                        json.dumps(kwargs[key], sort_keys=True).encode("utf-8")
                    )

        # Hash dynamic object meshes
        keys_to_hash = ["origin", "delta"]
        kwargs = config["object_meshes"]
        for key in keys_to_hash:
            if key in kwargs:
                hash_algorithm.update(
                    json.dumps(kwargs[key], sort_keys=True).encode("utf-8")
                )

        # Hash base link pose
        position, _ = arrays_from_pose_msg(
            self.get_frame_pose_stamped("base_link").pose
        )
        hash_algorithm.update(position.tobytes())

        return hash_algorithm.hexdigest()

    def init_planning_scene(self):
        """Setup the planning scene

        Adds plane, primitive, and mesh collision objects from the planning
        scene configuration.
        """
        self.log("Initializing planning scene")

        self.remove_all_collision_objects()

        self.collision_object_init_kwargs: dict[str, dict[str, Any]] = {}

        config: dict[str, Any] = self.get_parameter_wrapper("planning_scene")

        scene_path = os.path.join(config["dir"], "scene.txt")
        collision_matrix_path = os.path.join(
            config["dir"], "collision_matrix.csv"
        )
        config_path = os.path.join(config["dir"], "config.yaml")
        rig_hash_path = os.path.join(config["dir"], "rig_hash.txt")
        object_init_kwargs_path = os.path.join(
            config["dir"], "object_init_kwargs.pkl"
        )

        if config["use_saved_scene"]:
            if all(
                os.path.exists(path)
                for path in [
                    scene_path,
                    collision_matrix_path,
                    config_path,
                    object_init_kwargs_path,
                    rig_hash_path,
                ]
            ):
                with open(config_path, "r") as f:
                    saved_config = yaml.safe_load(f)
                with open(rig_hash_path, "r") as f:
                    saved_rig_hash = f.read().strip()
                if saved_config == config and saved_rig_hash == self.rig_hash:
                    self.log(
                        f"Loading planning scene from file {scene_path}",
                    )
                    self.load_planning_scene(scene_path)
                    self.load_collision_matrix(collision_matrix_path)
                    self.load_object_init_kwargs(object_init_kwargs_path)
                    # self.log_planning_scene(severity="DEBUG")
                    # self.log_collision_matrix(severity="DEBUG")
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

        # Add plane collision objects
        if "planes" in config:
            for object_id, kwargs in config["planes"].items():
                self.add_plane_collision_object(
                    object_id=object_id, dynamic=False, **kwargs
                )

        # Add primitive collision objects
        if "primitives" in config:
            for object_id, kwargs in config["primitives"].items():
                self.add_primitive_collision_object(
                    object_id=object_id, dynamic=False, **kwargs
                )

        # Add dynamic object meshes
        self.add_dynamic_mesh_collision_objects(**config["object_meshes"])

        # Add rig mesh collision objects
        for object_id, kwargs in config["rig_meshes"].items():
            self.add_mesh_collision_object(
                object_id=object_id, dynamic=False, **kwargs
            )

        # Log planning scene
        # self.log_planning_scene(severity="DEBUG")
        # self.log_collision_objects(severity="DEBUG")
        # self.log_collision_matrix(severity="DEBUG")

        # Save planning scene to file
        os.makedirs(config["dir"], exist_ok=True)
        self.save_planning_scene(scene_path)
        self.save_collision_matrix(collision_matrix_path)
        self.save_object_init_kwargs(object_init_kwargs_path)
        with open(rig_hash_path, "w") as f:
            f.write(self.rig_hash)
        with open(config_path, "w") as f:
            yaml.dump(orig_config, f)

    def init_attached_object(self):
        """Initialize the attached object."""
        object_id = None
        idx = None

        try:
            object_id = self.get_parameter_wrapper("initial_attached_object")
        except ParameterNotDeclaredException:
            pass

        try:
            idx = self.get_parameter_wrapper("initial_attached_object_idx")
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

    ###########################################################################
    ########## Poses and states ###############################################
    ###########################################################################

    def get_named_target_states(
        self, group_name: Optional[str] = None
    ) -> list[str]:
        """Get the named target states from the planning component."""
        return self.get_planning_component(group_name).named_target_states()

    def get_target_state(
        self, target_name: str, group_name: Optional[str] = None
    ) -> RobotState:
        """Get the named target state from the planning component."""
        if target_name == "idle":
            target_name = self.get_parameter_wrapper(
                "predefined_states.idle_state"
            )
        elif target_name == "pre_present":
            target_name = self.get_parameter_wrapper(
                "predefined_states.pre_present_state"
            )

        joint_state_dict = self.get_planning_component(
            group_name
        ).get_named_target_state_values(target_name)
        robot_state = self.current_state
        robot_state.joint_positions = joint_state_dict
        robot_state.update()
        return robot_state

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
        with self.planning_scene_read_only() as scene:
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
        with self.planning_scene_read_only() as scene:
            eef_pose = scene.current_state.get_pose(self.default_pose_link)

        pose_stamped = self.create_pose_stamped(
            pose=deepcopy(eef_pose), frame_id=self.planning_frame
        )

        # If a frame id is provided, change the reference frame
        if frame_id is not None and frame_id != self.planning_frame:
            pose_stamped = self.change_reference_frame(
                pose_stamped=pose_stamped,
                new_frame_id=frame_id,
            )

        return pose_stamped

    def object_init_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the initial pose of an object from the parameters."""
        return deepcopy(
            self.collision_object_init_kwargs[object_id]["pose_stamped"]
        )

    def _object_init_pose_stamped_with_offset(
        self, object_id: str, offset: list[float]
    ) -> PoseStamped:
        """Get the initial pose of an object from the parameters with an offset."""
        pose_stamped = self.object_init_pose_stamped(object_id)
        pose_stamped.pose.position.x += offset[0]
        pose_stamped.pose.position.y += offset[1]
        pose_stamped.pose.position.z += offset[2]
        return pose_stamped

    # Fetch poses

    def pre_fetch_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the pre-fetch pose of an object."""
        return self._object_init_pose_stamped_with_offset(
            object_id,
            self.get_parameter_wrapper("predefined_poses.pre_fetch_offset"),
        )

    def pre_attach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the pre-attach pose of an object."""
        return self._object_init_pose_stamped_with_offset(
            object_id,
            self.get_parameter_wrapper("predefined_poses.pre_attach_offset"),
        )

    def attach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the attach pose of an object."""
        return self.object_init_pose_stamped(object_id)

    def post_attach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the post-attach pose of an object."""
        return self._object_init_pose_stamped_with_offset(
            object_id,
            self.get_parameter_wrapper("predefined_poses.post_attach_offset"),
        )

    def post_fetch_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the post-fetch pose of an object."""
        return self._object_init_pose_stamped_with_offset(
            object_id,
            self.get_parameter_wrapper("predefined_poses.post_fetch_offset"),
        )

    def pre_present_pose_stamped(self, _: str) -> PoseStamped:
        """Get the pre-present pose."""
        return self.create_pose_stamped(
            **self.get_parameter_wrapper("predefined_poses.pre_present_pose")
        )

    # Return poses

    def unpresent_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the unpresent (pre-present) pose."""
        return self.pre_present_pose_stamped(object_id)

    def pre_return_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the pre-return (post-fetch) pose of an object."""
        return self.post_fetch_pose_stamped(object_id)

    def pre_detach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the pre-detach (post-attach) pose of an object."""
        return self.post_attach_pose_stamped(object_id)

    def detach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the detach (object init) pose of an object."""
        return self.object_init_pose_stamped(object_id)

    def post_detach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the post-detach (pre-attach) pose of an object."""
        return self.pre_attach_pose_stamped(object_id)

    def post_return_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the post-return (pre-fetch) pose of an object."""
        return self.pre_fetch_pose_stamped(object_id)

    def all_close_pose_stamped(
        self,
        pose_stamped1: PoseStamped,
        pose_stamped2: PoseStamped,
        position_tolerance: Optional[float] = None,
        orientation_tolerance: Optional[float] = None,
        use_euler_tolerance: Optional[bool] = None,
    ) -> bool:
        """Check if two pose stamped messages are all close.

        Performs a reference frame change if necessary.

        Args:
            pose_stamped1: The first pose stamped message.
            pose_stamped2: The second pose stamped message.
            position_tolerance: The tolerance for the position.
            orientation_tolerance: The tolerance for the orientation.
            use_euler_tolerance: Whether to use euler tolerance.
        Returns:
            True if the two pose stamped messages are all close, False otherwise.
        """
        if position_tolerance is None:
            position_tolerance = self.get_parameter_wrapper(
                "planning.position_tolerance"
            )
        if orientation_tolerance is None:
            orientation_tolerance = self.get_parameter_wrapper(
                "planning.orientation_tolerance"
            )
        if use_euler_tolerance is None:
            use_euler_tolerance = self.get_parameter_wrapper(
                "planning.use_euler_tolerance"
            )

        if pose_stamped1.header.frame_id != pose_stamped2.header.frame_id:
            pose_stamped2 = self.change_reference_frame(
                pose_stamped2, pose_stamped1.header.frame_id
            )

        return all_close_poses_stamped(
            pose_stamped1,
            pose_stamped2,
            position_tolerance,
            orientation_tolerance,
            use_euler_tolerance,
        )

    def all_close_robot_states(
        self,
        robot_state1: RobotState,
        robot_state2: RobotState,
        position_tolerance: Optional[float | dict[str, float]] = None,
        velocity_tolerance: Optional[float | dict[str, float]] = None,
        acceleration_tolerance: Optional[float | dict[str, float]] = None,
    ) -> bool:
        """Check if two robot states are all close."""
        if position_tolerance is None:
            position_tolerance = self.get_parameter_wrapper(
                "planning.position_tolerance"
            )
        assert position_tolerance is not None
        return all_close_robot_states(
            robot_state1,
            robot_state2,
            position_tolerance,
            velocity_tolerance,
            acceleration_tolerance,
        )

    ###########################################################################
    ########## Planning and execution #########################################
    ###########################################################################

    def get_empty_trajectory(self) -> RobotTrajectory:
        return RobotTrajectory(self.robot_model)

    def get_default_plan_request(self) -> PlanRequest:
        """Get the default plan request."""
        kwargs = self.get_parameter_wrapper("planning.defaults")
        return PlanRequest(
            goal=PoseStamped(), start_state=self.current_state, **kwargs
        )

    def create_plan_request(
        self, goal: PlanningGoalT, **kwargs: Any
    ) -> tuple[PlanRequest, dict[str, Any]]:
        """Parse the planning kwargs.

        Args:
            goal: The goal to plan for.
            **kwargs: Additional keyword arguments to override the default plan
                request.

        Returns:
            A tuple of the parsed kwargs and any unused kwargs.
        """
        self.log("Parsing plan args", severity="DEBUG")

        request = self.get_default_plan_request()

        # TODO: Implement pose_link functionality
        if "pose_link" in kwargs and kwargs["pose_link"] != request.pose_link:
            raise NotImplementedError(
                "pose_link functionality is not implemented"
            )

        # Override the default kwargs with the provided kwargs
        for key in request.__slots__:
            if key in kwargs:
                setattr(request, key, kwargs.pop(key))

        # Set the goal to the target state if the goal is a configuration name
        if isinstance(goal, str):
            goal = self.get_target_state(goal, request.group_name)
        elif isinstance(goal, PoseStamped):
            if goal.header.frame_id != self.planning_frame:
                goal = self.change_reference_frame(goal, self.planning_frame)
        request.goal = goal

        return request, kwargs

    def get_default_execute_request(self) -> ExecuteRequest:
        """Get the default execute request."""
        kwargs = self.get_parameter_wrapper("execution.defaults")
        return ExecuteRequest(trajectory=self.get_empty_trajectory(), **kwargs)

    def create_execute_request(
        self, trajectory: RobotTrajectory, **kwargs: Any
    ) -> tuple[ExecuteRequest, dict[str, Any]]:
        """Create an execute request.

        Args:
            trajectory: The robot trajectory to execute.
            **kwargs: Additional keyword arguments to override the default
                execute request.

        Returns:
            An execute request.
        """
        request = self.get_default_execute_request()

        request.trajectory = trajectory

        # Override the default kwargs with the provided kwargs
        for key in request.__slots__:
            if key in kwargs:
                setattr(request, key, kwargs.pop(key))

        return request, kwargs

    def _pre_plan(
        self, request: PlanRequest
    ) -> tuple[
        PlanningComponent,
        PlanRequestParameters | MultiPipelinePlanRequestParameters,
    ]:
        """Get the planning component and request parameters for the given kwargs.

        Args:
            request: The request to plan for.

        Returns:
            A tuple of the planning component and request parameters.
        """
        self.log(
            f"Preparing planning component and request parameters with request {request}",
            severity="DEBUG",
        )

        planning_component = self.get_planning_component(request.group_name)

        # Set start state
        if not planning_component.set_start_state(
            robot_state=request.start_state
        ):
            raise ValueError(f"Invalid start state: {request.start_state}")

        # Check that pose_link is the planning link
        if request.pose_link != self.default_pose_link:
            raise NotImplementedError(
                "pose_link functionality is not implemented"
            )

        # Set goal state
        goal_kwargs = {}
        if isinstance(request.goal, PoseStamped):
            goal_kwargs["pose_stamped_msg"] = request.goal
            goal_kwargs["pose_link"] = request.pose_link
        else:
            goal_kwargs["robot_state"] = request.goal

        if not planning_component.set_goal_state(**goal_kwargs):
            raise ValueError(f"Invalid goal: {request.goal}")

        # Set path constraints
        if request.path_constraints is not None:
            planning_component.set_path_constraints(request.path_constraints)

        # Plan
        if isinstance(request.planning_pipeline, str):
            request_params = PlanRequestParameters(
                self.moveit_py, request.planning_pipeline
            )
        else:
            assert isinstance(request.planning_pipeline, (list, tuple))
            request_params = MultiPipelinePlanRequestParameters(
                self.moveit_py, request.planning_pipeline
            )

        return planning_component, request_params

    def _plan_once_blocking(
        self,
        planning_component: PlanningComponent,
        request_params: PlanRequestParameters
        | MultiPipelinePlanRequestParameters,
        planning_scene: Optional[PlanningScene] = None,
    ) -> RobotTrajectory:
        """Plan a trajectory to the given waypoint once.

        Args:
            planning_component: The planning component to use.
            request_params: The request parameters to use.
        """
        self.log(
            f"Planning once with request params {request_params} and planning scene {planning_scene}",
            severity="DEBUG",
        )
        if isinstance(request_params, MultiPipelinePlanRequestParameters):
            plan_response = planning_component.plan(
                self.moveit_py,
                multi_plan_parameters=request_params,
                planning_scene=planning_scene,
            )
        else:
            plan_response = planning_component.plan(
                self.moveit_py,
                single_plan_parameters=request_params,
                planning_scene=planning_scene,
            )

        if not plan_response:
            raise PlanningError(plan_response.error_code)

        return plan_response.trajectory

    def _plan_blocking(
        self,
        *args: Any,
        request: Optional[PlanRequest] = None,
        cancel_event: Optional[threading.Event] = None,
        **kwargs: Any,
    ) -> RobotTrajectory | None:
        """
        Plan a trajectory to the given waypoint, retrying up to max_plan_attempts
        times until successful.

        Args:
            *args: Arguments to pass to `create_plan_request()`.
            request: The request to plan for. If not provided, the request is
                created from args and kwargs.
            cancel_event: An event that can be used to cancel planning.
            **kwargs: Keyword arguments to pass to `create_plan_request()`.

        Returns:
            The planned trajectory, or None if the goal is already reached.

        Raises:
            MaxPlanningAttemptsReachedError: If the maximum number of planning
                attempts is reached.
            asyncio.CancelledError: If the planning is cancelled by the cancel_event.

        See Also:
            `create_plan_request()`: For parameter details
        """
        # Parse the planning args if requested
        if request is None:
            request, unused_kwargs = self.create_plan_request(*args, **kwargs)
            if len(unused_kwargs) > 0:
                raise ValueError(f"Unused kwargs: {unused_kwargs}")
        elif len(args) > 0 or len(kwargs) > 0:
            raise ValueError(
                f"Additional arguments ({args}) or kwargs ({kwargs}) "
                "cannot be provided if plan_request is provided"
            )

        # Check if the goal is already reached
        if isinstance(request.goal, PoseStamped):
            if self.all_close_pose_stamped(
                request.goal, self.eef_pose_stamped()
            ):
                self.log("Already at goal, skipping planning")
                return None
        elif self.all_close_robot_states(request.goal, self.current_state):
            self.log("Already at start state, skipping planning")
            return None

        # Get the planning component and request parameters
        planning_component, request_params = self._pre_plan(request)

        # Plan until successful or max attempts reached
        errors: list[PlanningError] = []
        for i in range(request.max_plan_attempts):
            if cancel_event is not None and cancel_event.is_set():
                raise asyncio.CancelledError("Plan cancelled")
            try:
                trajectory = self._plan_once_blocking(
                    planning_component, request_params, request.planning_scene
                )
                self.log(
                    f"Planning attempt {i + 1}/{request.max_plan_attempts} succeeded",
                    severity="DEBUG",
                )
                return trajectory
            except PlanningError as e:
                self.log(
                    f"Planning attempt {i + 1}/{request.max_plan_attempts} failed: {e}",
                    severity="WARN",
                )
                errors.append(e)
        else:
            raise MaxPlanningAttemptsReachedError(errors)

    async def plan(self, *args: Any, **kwargs: Any) -> RobotTrajectory | None:
        """Asynchronously calls `plan()` method in a separate thread.

        See Also:
            `plan()`: For parameter details and synchronous implementation.
        """
        cancel_event = threading.Event()
        try:
            return await asyncio.to_thread(
                self._plan_blocking, *args, cancel_event=cancel_event, **kwargs
            )
        finally:
            cancel_event.set()

    def _apply_totg(
        self,
        trajectory: RobotTrajectory,
        *,
        velocity_scaling_factor: float,
        acceleration_scaling_factor: float,
        path_tolerance: float,
        resample_dt: float,
        min_angle_change: float,
    ) -> RobotTrajectory:
        """Apply time parameterization to the given robot trajectory.

        Args:
            trajectory: The robot trajectory to apply time parameterization to.
            velocity_scaling_factor: The velocity scaling factor to apply to the trajectory.
            acceleration_scaling_factor: The acceleration scaling factor to apply to the trajectory.
            path_tolerance: The path tolerance to apply to the trajectory.
            resample_dt: The resample time step to apply to the trajectory.
            min_angle_change: The minimum angle change to apply to the trajectory.

        Returns:
            The robot trajectory with time parameterization applied.

        Raises:
            PostProcessingError: If time parameterization fails.
        """
        self.log(
            "Applying time parameterization to trajectory with "
            f"velocity_scaling_factor {velocity_scaling_factor}, "
            f"acceleration_scaling_factor {acceleration_scaling_factor}, "
            f"path_tolerance {path_tolerance}, "
            f"resample_dt {resample_dt}, "
            f"min_angle_change {min_angle_change}",
            severity="DEBUG",
        )

        trajectory = robot_trajectory_copy(trajectory)

        old_num_waypoints = len(trajectory)
        old_duration = trajectory.duration
        old_path_length = trajectory.path_length

        if not trajectory.apply_totg_time_parameterization(
            velocity_scaling_factor=velocity_scaling_factor,
            acceleration_scaling_factor=acceleration_scaling_factor,
            path_tolerance=path_tolerance,
            resample_dt=resample_dt,
            min_angle_change=min_angle_change,
        ):
            raise TrajectoryError(TrajectoryErrorCodes.TOTG_FAILED)

        self.log(
            "Time parameterization applied successfully with: "
            f"number of waypoints {old_num_waypoints} -> {len(trajectory)}, "
            f"duration {old_duration} -> {trajectory.duration}, "
            f"path length {old_path_length} -> {trajectory.path_length}",
            severity="DEBUG",
        )

        return trajectory

    def _apply_smoothing(
        self,
        trajectory: RobotTrajectory,
        *,
        velocity_scaling_factor: float,
        acceleration_scaling_factor: float,
        mitigate_overshoot: bool,
        overshoot_threshold: float,
    ) -> RobotTrajectory:
        """Apply ruckig smoothing to the given robot trajectory.

        Args:
            trajectory: The robot trajectory to apply smoothing to.
            velocity_scaling_factor: The velocity scaling factor to apply to the trajectory.
            acceleration_scaling_factor: The acceleration scaling factor to apply to the trajectory.
            mitigate_overshoot: Whether to mitigate overshoot.
            overshoot_threshold: The overshoot threshold to apply to the trajectory.

        Returns:
            The robot trajectory with smoothing applied.

        Raises:
            PostProcessingError: If smoothing fails.
        """
        trajectory = robot_trajectory_copy(trajectory)

        old_num_waypoints = len(trajectory)
        old_duration = trajectory.duration
        old_path_length = trajectory.path_length

        self.log(
            f"Applying smoothing to trajectory with "
            f"velocity_scaling_factor {velocity_scaling_factor}, "
            f"acceleration_scaling_factor {acceleration_scaling_factor}, "
            f"mitigate_overshoot {mitigate_overshoot}, "
            f"overshoot_threshold {overshoot_threshold}",
            severity="DEBUG",
        )

        if not trajectory.apply_ruckig_smoothing(
            velocity_scaling_factor=velocity_scaling_factor,
            acceleration_scaling_factor=acceleration_scaling_factor,
            mitigate_overshoot=mitigate_overshoot,
            overshoot_threshold=overshoot_threshold,
        ):
            raise TrajectoryError(TrajectoryErrorCodes.SMOOTHING_FAILED)

        self.log(
            "Smoothing applied successfully with: "
            f"number of waypoints {old_num_waypoints} -> {len(trajectory)}, "
            f"duration {old_duration} -> {trajectory.duration}, "
            f"path length {old_path_length} -> {trajectory.path_length}",
            severity="DEBUG",
        )

        return trajectory

    def _validate_trajectory(self, trajectory: RobotTrajectory):
        """Validate the given robot trajectory.

        Args:
            trajectory: The robot trajectory to validate.
        """
        self.log("Validating trajectory", severity="DEBUG")

        group_name = trajectory.joint_model_group_name

        with self.planning_scene_read_only() as scene:
            if not scene.is_path_valid(
                trajectory,
                joint_model_group_name=group_name,
                verbose=True,
                invalid_index=[],
            ):
                raise TrajectoryError(TrajectoryErrorCodes.INVALID_TRAJECTORY)

    def _execute_once_blocking(self, trajectory_msg: RobotTrajectoryMsg):
        """Execute the given robot trajectory message once.

        Args:
            trajectory_msg: The robot trajectory message to execute.
        Returns:
            The status of the execution.
        """
        self.log("Executing trajectory once", severity="DEBUG")
        self.trajectory_execution_manager.push(trajectory_msg)
        if not self.safe_to_execute:
            raise NotSafeToExecuteError()

        with self.execution_lock:
            execution_status = (
                self.trajectory_execution_manager.execute_and_wait()
            )
        if not self.safe_to_execute:
            raise NotSafeToExecuteError(execution_status)

        if not execution_status:
            raise ExecutionError(execution_status)

        return execution_status

    def _execute_blocking(
        self,
        *args: Any,
        request: Optional[ExecuteRequest] = None,
        cancel_event: Optional[threading.Event] = None,
        **kwargs: Any,
    ):
        """Execute the given robot trajectory, retrying up to max_execution_attempts times
        until successful.

        Args:
            *args: Arguments to pass to `create_execute_request()`.
            request: The request to execute. If not provided, the request is
                created from args and kwargs.
            cancel_event: An event that can be used to cancel execution.
            **kwargs: Keyword arguments to pass to `create_execute_request()`.

        Returns:
            ExecutionStatus: The status of the execution.
        Raises:
            MaxAttemptsReachedError: If the maximum number of execution attempts
                is reached.
            asyncio.CancelledError: If the execution is cancelled by the cancel_event.
        See Also:
            `create_execute_request()`: For parameter details
        """
        if request is None:
            request, unused_kwargs = self.create_execute_request(
                *args, **kwargs
            )
            if len(unused_kwargs) > 0:
                raise ValueError(f"Unused kwargs: {unused_kwargs}")
        elif len(args) > 0 or len(kwargs) > 0:
            raise ValueError(
                f"Additional arguments ({args}) or kwargs ({kwargs}) "
                "cannot be provided if request is provided"
            )

        if request.apply_totg:
            request.trajectory = self._apply_totg(
                request.trajectory,
                velocity_scaling_factor=request.velocity_scaling_factor,
                acceleration_scaling_factor=request.acceleration_scaling_factor,
                path_tolerance=request.path_tolerance,
                resample_dt=request.resample_dt,
                min_angle_change=request.min_angle_change,
            )

        if request.apply_smoothing:
            request.trajectory = self._apply_smoothing(
                request.trajectory,
                velocity_scaling_factor=request.velocity_scaling_factor,
                acceleration_scaling_factor=request.acceleration_scaling_factor,
                mitigate_overshoot=request.mitigate_overshoot,
                overshoot_threshold=request.overshoot_threshold,
            )

        if request.validate_trajectory:
            self._validate_trajectory(request.trajectory)

        # Convert to trajectory message
        trajectory_msg = request.trajectory.get_robot_trajectory_msg()

        # Execute the trajectory until successful or max attempts reached
        errors: list[ExecutionError] = []
        for i in range(request.max_execution_attempts):
            if cancel_event is not None and cancel_event.is_set():
                raise asyncio.CancelledError("Execution cancelled")

            execution_status = self._execute_once_blocking(trajectory_msg)
            if execution_status:
                self.log(
                    f"Execution attempt {i + 1}/{request.max_execution_attempts} succeeded",
                    severity="DEBUG",
                )
                return
            else:
                self.log(
                    f"Execution attempt {i + 1}/{request.max_execution_attempts} "
                    f"failed with status {execution_status.status}",
                    severity="DEBUG",
                )
                errors.append(execution_status)
        else:
            raise MaxExecutionAttemptsReachedError(errors)

    async def execute(self, *args: Any, **kwargs: Any):
        """Execute the given robot trajectory in a separate thread.

        Retries up to max_execution_attempts times until successful.

        See Also:
            `_execute_blocking()`: For parameter details
        """
        cancel_event = threading.Event()
        try:
            await asyncio.to_thread(
                self._execute_blocking,
                *args,
                cancel_event=cancel_event,
                **kwargs,
            )
        finally:
            cancel_event.set()
            self.trajectory_execution_manager.stop_execution()

    def cache_trajectory(self, trajectory: RobotTrajectory, **kwargs: Any):
        """Cache the given trajectory.

        Args:
            trajectory: The trajectory to cache.
            **kwargs: Keyword arguments to pass to `FuzzyTrajectoryCache.cache_trajectory()`.
        """
        if not self.freeze_trajectory_cache:
            self.trajectory_cache.cache_trajectory(trajectory, **kwargs)
            self.log("Cached trajectory successfully")
        else:
            self.log("Cache is frozen, skipping cache")

    def robot_moved(self, old_state: RobotState) -> bool:
        """Check if the robot has moved.

        Args:
            old_state: The old state of the robot.

        Returns:
            True if the robot has moved, False otherwise.
        """
        tolerance = self.get_parameter_wrapper(
            "execution.robot_moved_tolerance"
        )
        return not self.all_close_robot_states(
            old_state, self.current_state, position_tolerance=tolerance
        )

    @safe_to_execute_decorator
    async def plan_and_execute(
        self, *args: Any, cache_trajectory: bool = True, **kwargs: Any
    ) -> dict[str, Any] | None:
        """Plan and execute a trajectory, using the cached trajectory if available.

        Args:
            *args: Arguments to pass to `create_plan_request()`.
            cache_trajectory: Whether to cache the planned trajectory.
            **kwargs: Keyword arguments to pass to `create_plan_request()`
                and `execute()`.

        Returns:
            A dictionary containing the kwargs to cache the trajectory, or None
            if the trajectory was found in the cache.

        Raises:
            MaxAttemptsReachedError: If the maximum number of plan or execute
                attempts is reached.
            TrajectoryError: If the trajectory post-processing fails or the
                trajectory is invalid.
        """
        self.log(
            "Planning and executing trajectory (with cache)", severity="DEBUG"
        )

        if "start_state" in kwargs:
            raise ValueError("start_state is not allowed in plan_and_execute")

        start_state = self.current_state

        # Parse the planning kwargs
        plan_request, execute_kwargs = self.create_plan_request(
            *args, start_state=start_state, **kwargs
        )

        # Attempt to execute the cached trajectory, otherwise plan and execute normally
        if self.use_cached_trajectories:
            try:
                # TODO: Refactor so caching happens in the plan() function
                trajectories = self.trajectory_cache.get_trajectories(
                    plan_request
                )
            except KeyError:
                self.log(
                    "No cached trajectory found, planning and executing normally"
                )
            else:
                self.log(
                    "Cached trajectories found, trying to execute in order of path length"
                )
                for trajectory in trajectories:
                    try:
                        await self.execute(trajectory, **execute_kwargs)
                        return
                    except (
                        MaxExecutionAttemptsReachedError,
                        TrajectoryError,
                    ) as e:
                        self.log(
                            f"Error while executing cached trajectory: {e}",
                            severity="WARN",
                        )
                        if self.robot_moved(start_state):
                            self.log(
                                "Robot moved, skipping cached trajectories"
                            )
                            break

                self.log(
                    "All cached trajectories failed, planning and executing normally"
                )
        else:
            self.log(
                "Not using cached trajectories, planning and executing normally"
            )

        # Reset the start state to the current state
        plan_request.start_state = self.current_state

        # Plan and execute normally
        trajectory = await self.plan(request=plan_request)
        if trajectory is None:
            return
        await self.execute(
            trajectory, validate_trajectory=False, **execute_kwargs
        )

        to_cache_kwargs = {
            "trajectory": trajectory,
            "request": plan_request,
            "true_end_state": self.current_state,
        }

        # Cache the trajectory if requested
        if cache_trajectory:
            self.cache_trajectory(**to_cache_kwargs)

        return to_cache_kwargs

    ###########################################################################
    ########## Fetch, present, and return #####################################
    ###########################################################################

    def _phase_to_goal(
        self,
        object_id: str,
        phase: ObjectPhase,
        goal: PlanningGoalT | None = None,
    ) -> PlanningGoalT:
        match phase:
            case ObjectPhase.PRESENT:
                if goal is None:
                    raise ValueError(
                        "Goal is required for present and unpresent phases"
                    )
                return goal
            case ObjectPhase.IDLE:
                return self.get_target_state("idle")
            case ObjectPhase.PRE_PRESENT:
                try:
                    return self.get_target_state("pre_present")
                except ParameterNotDeclaredException:
                    return self.pre_present_pose_stamped(object_id)
            case _:
                return getattr(self, f"{phase.name.lower()}_pose_stamped")(
                    object_id
                )

    async def _object_phase(
        self,
        object_id: str,
        phase: ObjectPhase,
        goal: PlanningGoalT | None = None,
        **kwargs: Any,
    ) -> dict[str, Any] | None:
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

        goal = self._phase_to_goal(object_id, phase, goal)
        extra_kwargs = {}
        extra_kwargs["planning_pipeline"] = "linear"

        if phase in OBJECT_MOUNT_PHASES:
            self.allow_collision(*zip(*self.allowed_object_mount_collisions))

        if phase == ObjectPhase.DETACH:
            extra_kwargs["velocity_scaling_factor"] = (
                self.get_parameter_wrapper(
                    "object_manipulation.detach_velocity_scaling_factor"
                )
            )

        match phase:
            case (
                ObjectPhase.PRE_FETCH
                | ObjectPhase.PRE_PRESENT
                | ObjectPhase.PRE_RETURN
                | ObjectPhase.IDLE
            ):
                del extra_kwargs["planning_pipeline"]
            case (
                ObjectPhase.PRE_ATTACH
                | ObjectPhase.ATTACH
                | ObjectPhase.POST_DETACH
                | ObjectPhase.POST_RETURN
            ):
                self.allow_collision(object_id, self.touch_links)
            case ObjectPhase.POST_ATTACH | ObjectPhase.DETACH:
                self.allow_collision(object_id, self.object_mount_ids)

        self.log(f"{phase.name} goal: {goal}", severity="DEBUG")

        try:
            to_cache_kwargs = await self.plan_and_execute(
                goal, **kwargs, **extra_kwargs
            )
        except (MaxExecutionAttemptsReachedError, TrajectoryError) as e:
            to_cache_kwargs = None
            match phase:
                case (
                    ObjectPhase.POST_FETCH
                    | ObjectPhase.PRESENT
                    | ObjectPhase.UNPRESENT
                    | ObjectPhase.PRE_DETACH
                ):
                    self.log(
                        f"Error while planning and executing {phase.name} phase with linear pipeline: {e}",
                        severity="WARN",
                    )
                    self.log(
                        f"Attempting to plan and execute {phase.name} phase with default pipeline",
                        severity="WARN",
                    )
                    await self.plan_and_execute(goal, **kwargs)
                case _:
                    raise
        finally:
            if phase in OBJECT_MOUNT_PHASES:
                self.disallow_collision(
                    *zip(*self.allowed_object_mount_collisions)
                )
            match phase:
                case (
                    ObjectPhase.PRE_ATTACH
                    | ObjectPhase.ATTACH
                    | ObjectPhase.POST_DETACH
                    | ObjectPhase.POST_RETURN
                ):
                    self.disallow_collision(object_id, self.touch_links)
                case ObjectPhase.POST_ATTACH | ObjectPhase.DETACH:
                    self.disallow_collision(object_id, self.object_mount_ids)

        match phase:
            case ObjectPhase.ATTACH:
                self.attach_collision_object(
                    object_id,
                    self.default_pose_link,
                    touch_links=self.touch_links,
                )
            case ObjectPhase.DETACH:
                self.detach_collision_object(object_id)

        return to_cache_kwargs

    @asyncio_task_decorator
    @object_manipulation_lock_decorator
    async def fetch_object(
        self, object_id: str, cache_trajectories: bool = True
    ):
        """Fetch an object from its mount.

        The robot moves to the object's mount, attaches the object, and moves
        to the object's post-fetch pose. It uses cached trajectories if
        available and only caches the planned trajectories if the full fetch
        process is successful. This addresses the issue of the robot getting
        "stuck" in a state that it cannot complete the full fetch process and
        caching trajectories that are unusable. If the fetch fails, the robot
        attempts to return the object to its mount.

        Args:
            object_id: The ID of the object to fetch
            cache_trajectories: Whether to cache the trajectories after fetching
                the object

        Raises:
            ValueError: If the object ID is not a valid collision object
            MaxAttemptsReachedError: If the fetch fails
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
        to_cache_kwargs: list[dict[str, Any]] = []
        try:
            for i in range(ObjectPhase.PRE_FETCH, ObjectPhase.POST_FETCH + 1):
                kwargs = await self._object_phase(
                    object_id, ObjectPhase(i), cache_trajectory=False
                )
                if kwargs is not None:
                    to_cache_kwargs.append(kwargs)
        except (MaxAttemptsReachedError, TrajectoryError) as e:
            self.log(
                f"Error while fetching object: {e}",
                severity="ERROR",
            )
            self.log("Attempting to return object to mount", severity="WARN")
            start_idx = ObjectPhase.IDLE - i
            for i in range(start_idx, ObjectPhase.IDLE + 1):
                await self._object_phase(
                    object_id, ObjectPhase(i), cache_trajectory=False
                )
            raise

        # Cache all trajectories if requested
        if cache_trajectories and len(to_cache_kwargs) > 0:
            for kwargs in to_cache_kwargs:
                self.cache_trajectory(**kwargs)
            self.log(
                f"Cached {len(to_cache_kwargs)} fetch trajectories successfully"
            )

    @asyncio_task_decorator
    @object_manipulation_lock_decorator
    async def present_object(self, goal: PlanningGoalT):
        """Present an object at the specified end goal.

        Args:
            goal: The goal to present the object at
        """
        object_id = self._get_exactly_one_attached_object_id()
        self.log(f"Presenting object {object_id}")

        # Pre-present phase
        await self._object_phase(object_id, ObjectPhase.PRE_PRESENT)

        # Move to end goal
        await self._object_phase(object_id, ObjectPhase.PRESENT, goal=goal)

    @asyncio_task_decorator
    @object_manipulation_lock_decorator
    async def unpresent_object(self):
        """Unpresent an object and move it to its pre-return pose."""
        object_id = self._get_exactly_one_attached_object_id()
        self.log(f"Unpresenting object {object_id}")

        # Unpresent phase
        await self._object_phase(object_id, ObjectPhase.UNPRESENT)

        # Pre-return phase
        self._pre_return_cache_kwargs = await self._object_phase(
            object_id, ObjectPhase.PRE_RETURN, cache_trajectory=False
        )

    @asyncio_task_decorator
    @object_manipulation_lock_decorator
    async def return_object(self, cache_trajectories: bool = True):
        """Return an object to its original position.

        Args:
            end_goal: The goal to move to after returning the object.
            cache_trajectories: Whether to cache the trajectories after
                returning the object

        Raises:
            RuntimeError: If exactly one object is not attached
            MaxAttemptsReachedError: If the return process fails
        """
        object_id = self._get_exactly_one_attached_object_id()
        self.log(f"Returning object {object_id}")

        to_cache_kwargs: list[dict[str, Any]] = []

        # Cache the unpresent trajectory if it exists
        if not hasattr(self, "_pre_return_cache_kwargs"):
            raise RuntimeError("Object was not unpresented before returning")

        if self._pre_return_cache_kwargs is not None:
            to_cache_kwargs.append(self._pre_return_cache_kwargs)
            del self._pre_return_cache_kwargs

        # Iterate through the unpresenting and returning phases
        for i in range(ObjectPhase.PRE_DETACH, ObjectPhase.IDLE + 1):
            kwargs = await self._object_phase(
                object_id, ObjectPhase(i), cache_trajectory=False
            )
            if kwargs is not None:
                to_cache_kwargs.append(kwargs)

        # Cache all trajectories if requested
        if cache_trajectories and len(to_cache_kwargs) > 0:
            for kwargs in to_cache_kwargs:
                self.cache_trajectory(**kwargs)
            self.log(
                f"Cached {len(to_cache_kwargs)} return trajectories successfully"
            )

    ###########################################################################
    ########## Reset rig #####################################################
    ###########################################################################

    async def _move_out_of_collision_simulation(
        self, end_goal: PlanningGoalT = "idle", **kwargs
    ):
        """Move the robot out of collision with the scene asynchronously.

        To be used only in simulation. With the real robot, the user should
        manually (via the teach pendant) move the robot away from the collision
        objects.

        Using this function will reset any attached dynamic collision objects
        to their initial poses and move the robot to the target pose, ignoring
        collisions.

        Args:
            end_goal: The goal to move to after moving out of collision.
            **kwargs: Keyword arguments to pass to `plan_and_execute()`.
        """
        self.log("Moving out of collision")
        if not self.simulate:
            raise RuntimeError("This function is only available in simulation")

        self.remove_all_collision_objects()
        await self.plan_and_execute(end_goal, **kwargs)
        self.init_planning_scene()

    async def reset_rig(self, end_goal: PlanningGoalT = "idle"):
        """Move the robot out of collision if necessary and return any attached
        objects to their original positions.
        """
        self.log("Resetting rig")
        if self.is_state_colliding():
            if self.simulate:
                await self._move_out_of_collision_simulation(end_goal)
            else:
                raise RuntimeError(
                    "Robot is in collision with the scene! "
                    "Please move the robot away from the collision objects manually."
                )
        else:
            try:
                if len(self.attached_collision_object_ids) > 0:
                    self._pre_return_cache_kwargs = await self._object_phase(
                        self._get_exactly_one_attached_object_id(),
                        ObjectPhase.PRE_RETURN,
                    )
                    await self.return_object()
                else:
                    await self.plan_and_execute(end_goal)
            except (MaxAttemptsReachedError, TrajectoryError):
                if self.simulate:
                    self.log(
                        "Max attempts reached while resetting rig.",
                        severity="WARN",
                    )
                    await self._move_out_of_collision_simulation(end_goal)
                else:
                    raise

    async def reset_commander(self, timeout: Optional[float] = None):
        """Reset the dashboard and the robot until successful or timeout.

        Args:
            goal: Optional pose to move to after resetting the robot
            timeout: Optional timeout for resetting the commander
        """
        self.log("Resetting commander")

        async with asyncio.timeout(timeout):
            while True:
                try:
                    if not self.safe_to_execute:
                        self.log(
                            "Cannot reset commander until safe to execute",
                            severity="WARN",
                        )
                        await self._arm_lock_and_wait()
                    await self.reset_dashboard(timeout)
                    await self.reset_rig()
                    break
                except (
                    ServiceCallError,
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
                    if isinstance(e, MaxExecutionAttemptsReachedError):
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
        self.trajectory_cache.__enter__()
        if not self.initial_reset:
            await self.reset_commander()
            self.initial_reset = True
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
                if isinstance(
                    exc_value, (TimeoutError, CommanderRecoverableError)
                ):
                    self.log(
                        "Caught exception while running commander:",
                        severity="ERROR",
                    )
                    self.log(
                        f"{exc_type.__name__}: {exc_value}", severity="ERROR"
                    )
                    self.log(f"Traceback: {exc_tb}", severity="DEBUG")
                    if exc_type is NotSafeToExecuteError:
                        self.log(
                            "Not safe to execute, waiting for safe to execute",
                            severity="WARN",
                        )
                        await self._arm_lock_and_wait()
                        return True
                    if exc_type is MaxExecutionAttemptsReachedError:
                        self.log(
                            "Sleeping for 5 seconds before resetting commander",
                            severity="WARN",
                        )
                        await asyncio.sleep(5)
                    await self.reset_commander()
                    return True
            return False
        finally:
            self.trajectory_cache.__exit__(exc_type, exc_value, exc_tb)

    ###########################################################################
    ########## Asyncio schedule ###############################################
    ###########################################################################

    def schedule(self, *coros: Coroutine) -> asyncio.Task | list[asyncio.Task]:
        """Schedule coroutines to run.

        Args:
            *coros: Coroutines to schedule.

        Returns:
            List of scheduled tasks.
        """
        tasks = []
        for coro in coros:
            tasks.append(asyncio.create_task(coro))
        return tasks[0] if len(tasks) == 1 else tasks

    ###########################################################################
    ########## Destroy ########################################################
    ###########################################################################

    def destroy_node(self):
        if hasattr(self, "trajectory_cache"):
            self.trajectory_cache.close()
        if hasattr(self, "moveit_py"):
            self.moveit_py.shutdown()
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

    origin = commander.get_frame_pose_stamped("small_object_0")
    origin_position, origin_euler = arrays_from_pose_msg(
        origin.pose, euler=True
    )
    commander.log(
        f"Object origin position: {origin_position.round(4)}, euler: {origin_euler.round(4)}"
    )

    while True:
        await asyncio.sleep(1)
        eef_world = commander.eef_pose_stamped()
        position, _ = arrays_from_pose_msg(eef_world.pose, euler=True)
        position = position - origin_position
        commander.log(f"Eef relative position: {position.round(4).tolist()}")


async def asyncio_runner(
    coro: Coroutine, spin_future: concurrent.futures.Future, max_workers: int
):
    """Run a coroutine in an asyncio event loop.

    This function sets the default executor for the asyncio event loop to the
    thread pool executor provided. Used to run coroutines in a custom thread
    pool executor for performance reasons (e.g. more workers).

    Args:
        coro: The coroutine to run.
    """
    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as tpe:
        loop = asyncio.get_event_loop()
        loop.set_default_executor(tpe)
        spin_task = asyncio.wrap_future(spin_future)
        coro_task = asyncio.create_task(coro)
        done, _ = await asyncio.wait(
            [spin_task, coro_task], return_when=asyncio.FIRST_COMPLETED
        )
        for task in done:
            task.result()


def main(args=None):
    rclpy.init(args=args)

    # Parse non-ROS arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--coroutine-module", type=str, default=None)
    parser.add_argument("--coroutine-name", type=str, default=None)
    parser.add_argument("--coroutine-config", type=str, default=None)
    parser.add_argument("--max-workers", type=int, default=4)
    parser.add_argument("--debug", action="store_true", default=False)

    non_ros_args = rclpy.utilities.remove_ros_args(args)
    args, _ = parser.parse_known_args(non_ros_args)

    if args.coroutine_module is not None or args.coroutine_name is not None:
        if args.coroutine_name is None or args.coroutine_module is None:
            raise ValueError(
                "Both coroutine_module and coroutine_name must be provided "
                "when one is provided"
            )
        print(
            f"Loading coroutine {args.coroutine_name} "
            f"from module {args.coroutine_module} "
        )
        coro_fn: Callable[[Commander, Optional[str]], Coroutine] = getattr(
            importlib.import_module(args.coroutine_module), args.coroutine_name
        )
    else:
        print("No coroutine module or name provided, running in debug mode")
        coro_fn = debug_commander
        args.coroutine_config = None
        args.debug = True

    if args.coroutine_config is not None:
        print(f"Config file: {args.coroutine_config}")

    if args.debug:
        print("Debug mode enabled")
        debugpy.listen(1300)
        print("Waiting for debugger to attach")
        debugpy.wait_for_client()
        print("Debugger attached")

    try:
        commander = Commander()
        executor = MultiThreadedExecutor(num_threads=args.max_workers)
        executor.add_node(commander)

        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as tpe:
            try:
                spin_future = tpe.submit(executor.spin)
                coro = coro_fn(commander, args.coroutine_config)
                asyncio.run(
                    asyncio_runner(coro, spin_future, args.max_workers)
                )
            finally:
                try:
                    print("Shutting down commander")
                    commander.destroy_node()
                except Exception as e:
                    print(f"Error while shutting down commander: {e}")
                try:
                    print("Shutting down executor")
                    executor.shutdown()
                except Exception as e:
                    print(f"Error while shutting down executor: {e}")

    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down rclpy")
        rclpy.try_shutdown()  # type: ignore
