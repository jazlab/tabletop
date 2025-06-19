import argparse
import asyncio
import glob
import hashlib
import heapq
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
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from enum import IntEnum
from types import TracebackType
from typing import Any, ContextManager, Optional, Self, cast

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
from moveit.core.controller_manager import ExecutionStatus  # type: ignore
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
    Constraints,
    LinkPadding,
    ObjectColor,
)
from moveit_msgs.msg import (
    PlanningScene as PlanningSceneMsg,
)
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.logging_severity import LoggingSeverity
from std_srvs.srv import Trigger
from tabletop_msgs.srv import (
    GetArmDoor,
    GetFlic,
    GetHandFixation,
    GetReward,
    SetArmDoor,
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
    InvalidTrajectoryError,
    MaxAttemptsReachedError,
    MaxExecutionAttemptsReachedError,
    MaxPlanningAttemptsReachedError,
    ObjectManipulationError,
    PlanningGoalT,
    ServiceCallUnsuccessfulError,
    add_mesh_collision_object_msg,
    add_plane_collision_object_msg,
    add_primitive_collision_object_msg,
    add_primitive_collision_object_msg_from_geometry,
    all_close_poses_stamped,
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

from tabletop_server.nodes.base import DEFAULT_LOG_SEVERITY, BaseNode
from tabletop_server.nodes.mock_teensy import ArmDoorState


def asyncio_task_decorator(
    coro_fn: Callable[..., Coroutine],
) -> Callable[..., asyncio.Task]:
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


OBJECT_MANIPULATION_PHASES = [
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
        "dashboard.installation",
        "dashboard.program",
        "teensy.spin_period",
        "teensy.arm_door_timeout",
        "flic.spin_period",
        "planning.max_attempts",
        "planning.group_name",
        "planning.default_pipeline",
        "planning.linear_pipeline",
        "planning.planning_link",
        "planning.idle_state",
        "planning.pre_fetch_offset",
        "planning.pre_attach_offset",
        "planning.post_attach_offset",
        "execution.max_attempts",
        "execution.velocity_scaling_factor",
        "execution.acceleration_scaling_factor",
        "trajectory_cache.use_cached_trajectories",
        "trajectory_cache.freeze_cache",
        "trajectory_cache.kwargs",
        "object_manipulation.allowed_collisions",
        "object_manipulation.touch_links",
        "object_manipulation.mount_ids",
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

        self.moveit_py = MoveItPy("moveit_py", provide_planning_service=True)

        self.init_attributes()

        self.init_planning_scene()

        self.init_attached_object()

        self.init_link_padding()

        self.init_services()

        self.log("Commander initialized")

    def init_attributes(self):
        """Setup variables for the commander."""
        # Collision object kwargs
        self.collision_object_init_kwargs: dict[str, dict[str, Any]] = {}

        # Trajectory cache
        trajectory_cache_config = self.get_parameter_wrapper(
            "trajectory_cache.kwargs"
        )
        self.trajectory_cache = FuzzyTrajectoryCache(
            rig_hash=self.rig_hash,
            **trajectory_cache_config,
        )

        # Whether the robot has been reset
        self.initial_reset = False

        # Object manipulation lock
        self.object_manipulation_lock = asyncio.Lock()

    # TODO: Add services
    def init_services(self):
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
        pass
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
            group_name = self.planning_group_name
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
    def planning_group_name(self) -> str:
        """Get the planning group name from the parameter server."""
        return self.get_parameter_wrapper("planning.group_name")

    @property
    def planning_link(self) -> str:
        """Get the planning link from the parameter server."""
        return self.get_parameter_wrapper("planning.planning_link")

    @property
    def allowed_object_manipulation_collisions(self) -> list[tuple[str, str]]:
        """Get the allowed object manipulation collisions from the parameter server."""
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
        severity: str = DEFAULT_LOG_SEVERITY,
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

    def log_planning_scene(self, severity: str = DEFAULT_LOG_SEVERITY):
        """Log the planning scene."""
        return
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

    def log_collision_matrix(self, severity: str = DEFAULT_LOG_SEVERITY):
        """Log the collision matrix."""
        if self.log_level < LoggingSeverity[severity]:
            return

        self.log(
            f"Allowed collision matrix: \n{self.collision_matrix_df.to_string()}",
            severity=severity,
        )

    def log_collision_objects(self, severity: str = DEFAULT_LOG_SEVERITY):
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
        async with asyncio.timeout(timeout):
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
                "/dashboard_client/load_program",
                self.get_parameter_wrapper("dashboard.program"),
            )
            await self.dashboard_trigger("/dashboard_client/brake_release")
            while True:
                try:
                    await self.dashboard_trigger("/dashboard_client/play")
                    break
                except ServiceCallUnsuccessfulError:
                    self.log(
                        "Failed attempt to play dashboard program, "
                        "retrying after 3 seconds...",
                        severity="WARN",
                    )
                    await asyncio.sleep(3)

    ###########################################################################
    ########## Teensy Interface ###############################################
    ###########################################################################

    async def _get_arm_door(self) -> GetArmDoor.Response:
        """Get the arm door state asynchronously."""
        response = await self.service_call_async(
            srv_request=GetArmDoor.Request(),
            srv_type=GetArmDoor,
            srv_name="/teensy/get_arm_door",
        )
        return cast(GetArmDoor.Response, response)

    async def _get_reward(self) -> GetReward.Response:
        """Get the reward state asynchronously."""
        response = await self.service_call_async(
            srv_request=GetReward.Request(),
            srv_type=GetReward,
            srv_name="/teensy/get_reward",
        )
        return cast(GetReward.Response, response)

    async def _get_hand_fixation(self) -> GetHandFixation.Response:
        """Get the hand fixation state asynchronously."""
        response = await self.service_call_async(
            srv_request=GetHandFixation.Request(),
            srv_type=GetHandFixation,
            srv_name="/teensy/get_hand_fixation",
        )
        return cast(GetHandFixation.Response, response)

    async def _get_flic(self) -> GetFlic.Response:
        """Get the flic state asynchronously."""
        response = await self.service_call_async(
            srv_request=GetFlic.Request(),
            srv_type=GetFlic,
            srv_name="/flic/get_flic",
        )
        return cast(GetFlic.Response, response)

    @asyncio_task_decorator
    async def smartglass_reveal(self) -> SetSmartglass.Response:
        """Coroutine to call the smartglass service to reveal the smartglass asynchronously."""
        self.log("Smartglass Reveal")
        response = await self.service_call_async(
            srv_request=SetSmartglass.Request(is_revealed=True),
            srv_type=SetSmartglass,
            srv_name="/teensy/set_smartglass",
        )
        return cast(SetSmartglass.Response, response)

    @asyncio_task_decorator
    async def smartglass_occlude(self) -> SetSmartglass.Response:
        """Coroutine to call the smartglass service to occlude the smartglass asynchronously."""
        self.log("Smartglass Occlude")
        response = await self.service_call_async(
            srv_request=SetSmartglass.Request(is_revealed=False),
            srv_type=SetSmartglass,
            srv_name="/teensy/set_smartglass",
        )
        return cast(SetSmartglass.Response, response)

    async def _start_arm_door_open(self) -> SetArmDoor.Response:
        """Coroutine to call the arm door service to open the arm door asynchronously."""
        self.log("Arm Door Open")
        response = await self.service_call_async(
            srv_request=SetArmDoor.Request(open=True),
            srv_type=SetArmDoor,
            srv_name="/teensy/set_arm_door",
        )
        return cast(SetArmDoor.Response, response)

    async def _start_arm_door_close(self) -> SetArmDoor.Response:
        """
        Coroutine to call the arm door service to close the arm door
        asynchronously.
        """
        self.log("Arm Door Close")
        response = await self.service_call_async(
            srv_request=SetArmDoor.Request(open=False),
            srv_type=SetArmDoor,
            srv_name="/teensy/set_arm_door",
        )
        return cast(SetArmDoor.Response, response)

    async def _start_reward(self, duration: float) -> SetReward.Response:
        """
        Coroutine to call the reward service to deliver a reward for a given
        duration.
        """
        self.log(f"Delivering reward for {duration} s")
        if duration < 0:
            raise ValueError("Duration must be greater than 0!")
        response = await self.service_call_async(
            srv_request=SetReward.Request(duration_ms=int(duration * 1000)),
            srv_type=SetReward,
            srv_name="/teensy/set_reward",
        )
        return cast(SetReward.Response, response)

    async def _wait_for_arm_door_open(
        self, timeout: Optional[float] = None
    ) -> bool:
        """Wait for arm door to open, then return True."""
        spin_period = self.get_parameter_wrapper("teensy.spin_period")
        timeout = (
            timeout
            if timeout is not None
            else self.get_parameter_wrapper("teensy.arm_door_timeout")
        )
        try:
            async with asyncio.timeout(timeout):
                response = await self._get_arm_door()
                if response.state == ArmDoorState.OPEN:
                    return True

                while response.state != ArmDoorState.OPEN:
                    response = await self._get_arm_door()
                    await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    async def _wait_for_arm_door_close(
        self, timeout: Optional[float] = None
    ) -> bool:
        """Wait for arm door to close."""
        spin_period = self.get_parameter_wrapper("teensy.spin_period")
        timeout = (
            timeout
            if timeout is not None
            else self.get_parameter_wrapper("teensy.arm_door_timeout")
        )
        response = await self._get_arm_door()
        if response.is_closed:
            return True
        try:
            async with asyncio.timeout(timeout):
                while not response.is_closed:
                    response = await self._get_arm_door()
                    await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    async def _wait_for_reward(self, duration: float) -> bool:
        """Wait for reward to start, then return True."""
        spin_period = self.get_parameter_wrapper("teensy.spin_period")
        timeout = duration + 2 * spin_period  # TODO: 2 is arbitrary, check
        response = await self._get_reward()
        if response.is_active:
            return True
        try:
            async with asyncio.timeout(timeout):
                while not response.is_active:
                    response = await self._get_reward()
                    await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    @asyncio_task_decorator
    async def wait_for_hand_fixation_press(
        self, timeout: Optional[float] = None
    ) -> bool:
        """Wait for hand fixation to be pressed

        Args:
            timeout: Timeout in seconds. If None, the default timeout from
                parameters is used.

        Returns:
            True if hand fixation was pressed within the timeout,
            False otherwise.
        """
        spin_period = self.get_parameter_wrapper("teensy.spin_period")
        initial_fixation = await self._get_hand_fixation()
        if initial_fixation.is_pressed:
            return True
        fixation = initial_fixation
        try:
            async with asyncio.timeout(timeout):
                while (
                    fixation.last_time_pressed_ms
                    == initial_fixation.last_time_pressed_ms
                ):
                    fixation = await self._get_hand_fixation()
                    await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    # TODO: Consider reimplementing using action server model (unsure if this is possible with the teensy)
    @asyncio_task_decorator
    async def wait_for_hand_fixation_release(
        self, timeout: Optional[float] = None
    ) -> bool:
        """Wait for hand fixation to be released

        Args:
            timeout: Timeout in seconds. If None, the default timeout from
                parameters is used.

        Returns:
            True if hand fixation was released within the timeout,
            False otherwise.
        """
        spin_period = self.get_parameter_wrapper("teensy.spin_period")
        initial_fixation = await self._get_hand_fixation()
        if not initial_fixation.is_pressed:
            return True
        fixation = initial_fixation
        try:
            async with asyncio.timeout(timeout):
                while (
                    fixation.last_time_released_ms
                    == initial_fixation.last_time_released_ms
                ):
                    fixation = await self._get_hand_fixation()
                    await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    # TODO: Potential race condition (between monkey and get_flic lol)
    @asyncio_task_decorator
    async def wait_for_flic_press(
        self, timeout: Optional[float] = None
    ) -> bool:
        """Wait for flic button press, then return True."""
        spin_period = self.get_parameter_wrapper("flic.spin_period")

        initial_flic = await self._get_flic()
        flic = initial_flic
        try:
            async with asyncio.timeout(timeout):
                while (
                    flic.last_time_pressed_ms
                    == initial_flic.last_time_pressed_ms
                ):
                    flic = await self._get_flic()
                    await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    @asyncio_task_decorator
    async def arm_door_open_and_wait(
        self, timeout: Optional[float] = None
    ) -> bool:
        """Open arm door and wait for it to be open."""
        await self._start_arm_door_open()
        return await self._wait_for_arm_door_open(timeout)

    @asyncio_task_decorator
    async def arm_door_close_and_wait(
        self, timeout: Optional[float] = None
    ) -> bool:
        """Close arm door and wait for it to be closed."""
        await self._start_arm_door_close()
        return await self._wait_for_arm_door_close(timeout)

    @asyncio_task_decorator
    async def reward_and_wait(self, duration: float):
        """Start reward and wait for it to be active."""
        await self._start_reward(duration)
        # Default timeout is duration plus spin period if not specified
        timeout = duration + self.get_parameter_wrapper("teensy.spin_period")

        if not await self._wait_for_reward(timeout):
            raise RuntimeError("Reward took longer than expected timeout")

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
            group_name = self.planning_group_name

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
            group_name = self.planning_group_name

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
                    self.log_planning_scene(severity="DEBUG")
                    self.log_collision_matrix(severity="DEBUG")
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
        self.log_planning_scene(severity="DEBUG")
        # self.log_collision_objects(severity="DEBUG")
        self.log_collision_matrix(severity="DEBUG")

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
            object_id, self.planning_link, touch_links=self.touch_links
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
            target_name = self.get_parameter_wrapper("planning.idle_state")
        elif target_name == "pre_present":
            target_name = self.get_parameter_wrapper(
                "planning.pre_present_state"
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
            eef_pose = scene.current_state.get_pose(self.planning_link)

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

    def object_init_pose_stamped_with_offset(
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
        return self.object_init_pose_stamped_with_offset(
            object_id, self.get_parameter_wrapper("planning.pre_fetch_offset")
        )

    def pre_attach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the pre-attach pose of an object."""
        return self.object_init_pose_stamped_with_offset(
            object_id, self.get_parameter_wrapper("planning.pre_attach_offset")
        )

    def attach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the attach pose of an object."""
        return self.object_init_pose_stamped(object_id)

    def post_attach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the post-attach pose of an object."""
        return self.object_init_pose_stamped_with_offset(
            object_id,
            self.get_parameter_wrapper("planning.post_attach_offset"),
        )

    def post_fetch_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the post-fetch pose of an object."""
        return self.object_init_pose_stamped_with_offset(
            object_id, self.get_parameter_wrapper("planning.post_fetch_offset")
        )

    def pre_present_pose_stamped(self, _: str) -> PoseStamped:
        """Get the pre-present pose."""
        return self.create_pose_stamped(
            **self.get_parameter_wrapper("planning.pre_present_pose")
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

    ###########################################################################
    ########## Planning and execution #########################################
    ###########################################################################

    def get_empty_trajectory(self) -> RobotTrajectory:
        return RobotTrajectory(self.robot_model)

    def _parse_plan_args(
        self,
        goal: PlanningGoalT,
        *,
        start_state: Optional[RobotState] = None,
        group_name: Optional[str] = None,
        pose_link: Optional[str] = None,
        planning_pipeline: str | list[str] = "default",
        path_constraints: Optional[Constraints] = None,
        max_plan_attempts: Optional[int] = None,
        planning_scene: Optional[PlanningScene] = None,
        **unused_kwargs: Any,
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        """Parse the planning kwargs.

        Args:
            goal: The goal to plan for.
            start_state: The start state to plan for.
            group_name: The name of the planning group.
            pose_link: The link to use for the goal pose.
            planning_pipeline: The planning pipeline to use.
            path_constraints: The path constraints to use.
            max_plan_attempts: The maximum number of planning attempts.
            planning_scene: The planning scene to use.
            **unused_kwargs: Additional keyword arguments that are not used for
                planning.

        Returns:
            A tuple of the parsed kwargs and any unused kwargs.
        """
        self.log(
            f"Parsing plan args with goal {goal}, "
            f"start_state {start_state}, "
            f"group_name {group_name}, "
            f"pose_link {pose_link}, "
            f"planning_pipeline {planning_pipeline}, "
            f"path_constraints {path_constraints}, "
            f"max_plan_attempts {max_plan_attempts}, "
            f"planning_scene {planning_scene}, "
            f"unused_kwargs {unused_kwargs}",
            severity="DEBUG",
        )

        # TODO: Implement pose_link functionality
        if pose_link is not None and pose_link != self.planning_link:
            raise NotImplementedError(
                "pose_link functionality is not implemented"
            )

        # Set start state to current state if not provided
        if start_state is None:
            start_state = self.current_state

        # Set pose_link to planning link if not provided
        if pose_link is None:
            pose_link = self.planning_link

        # Set the group name to the planning group name if not provided
        if group_name is None:
            group_name = self.planning_group_name

        # Set the goal to the target state if the goal is a configuration name
        if isinstance(goal, str):
            goal = self.get_target_state(goal, group_name)
        elif isinstance(goal, PoseStamped):
            if goal.header.frame_id != self.planning_frame:
                goal = self.change_reference_frame(goal, self.planning_frame)

        # Set the planning pipeline(s) from the parameter server if the planning
        # pipeline(s) equals "default" or "linear"
        if isinstance(planning_pipeline, str):
            planning_pipeline = [planning_pipeline]

        for i, pipeline in enumerate(planning_pipeline):
            if pipeline == "default":
                planning_pipeline[i] = self.get_parameter_wrapper(
                    "planning.default_pipeline"
                )
            elif pipeline == "linear":
                if not isinstance(goal, PoseStamped):
                    raise ValueError(
                        "Linear pipeline requires a PoseStamped goal"
                    )
                planning_pipeline[i] = self.get_parameter_wrapper(
                    "planning.linear_pipeline"
                )
            else:
                raise ValueError(
                    f"Planning pipelines besides 'default' and 'linear' are not supported: {pipeline}"
                )

        if len(planning_pipeline) == 1:
            planning_pipeline = planning_pipeline[0]

        if max_plan_attempts is None:
            max_plan_attempts = cast(
                int, self.get_parameter_wrapper("planning.max_attempts")
            )

        return {
            "goal": goal,
            "start_state": start_state,
            "pose_link": pose_link,
            "group_name": group_name,
            "planning_pipeline": planning_pipeline,
            "path_constraints": path_constraints,
            "max_plan_attempts": max_plan_attempts,
            "planning_scene": planning_scene,
        }, unused_kwargs

    def _parse_plan_and_execute_args(
        self,
        goal: PlanningGoalT,
        **kwargs: Any,
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        """Parse the planning and execute kwargs."""
        plan_kwargs, execute_kwargs = self._parse_plan_args(goal, **kwargs)
        execute_kwargs["group_name"] = plan_kwargs["group_name"]
        return plan_kwargs, execute_kwargs

    def _get_planning_component_and_request_params(
        self,
        goal: PlanningGoalT,
        *,
        start_state: RobotState,
        pose_link: str,
        group_name: str,
        planning_pipeline: str | list[str],
        path_constraints: Constraints | None,
    ) -> tuple[
        PlanningComponent,
        PlanRequestParameters | MultiPipelinePlanRequestParameters,
    ]:
        """Get the planning component and request parameters for the given kwargs.

        Args:
            goal: The goal to plan for.
            start_state: The start state to plan for.
            pose_link: The link to use for the goal pose.
            group_name: The name of the planning group.
            planning_pipeline: The planning pipeline to use.
            path_constraints: The path constraints to use.

        Returns:
            A tuple of the planning component and request parameters.
        """
        self.log(
            f"Preparing planning component and request parameters with "
            f"goal {goal}, "
            f"start_state {start_state}, "
            f"pose_link {pose_link}, "
            f"group_name {group_name}, "
            f"planning_pipeline {planning_pipeline}, "
            f"path_constraints {path_constraints}",
            severity="DEBUG",
        )

        planning_component = self.get_planning_component(group_name)

        # Set start state
        if not planning_component.set_start_state(robot_state=start_state):
            raise ValueError(f"Invalid start state: {start_state}")

        # Check that pose_link is the planning link
        if pose_link != self.planning_link:
            raise NotImplementedError(
                "pose_link functionality is not implemented"
            )

        # Set goal state
        goal_kwargs = {}
        if isinstance(goal, PoseStamped):
            goal_kwargs["pose_stamped_msg"] = goal
            goal_kwargs["pose_link"] = pose_link
        elif isinstance(goal, RobotState):
            goal_kwargs["robot_state"] = goal
        elif isinstance(goal, str):
            goal_kwargs["configuration_name"] = goal

        if not planning_component.set_goal_state(**goal_kwargs):
            raise ValueError(f"Invalid goal: {goal}")

        # Set path constraints
        if path_constraints is not None:
            planning_component.set_path_constraints(path_constraints)

        # Plan
        if isinstance(planning_pipeline, str):
            request_params = PlanRequestParameters(
                self.moveit_py, planning_pipeline
            )
        else:
            assert isinstance(planning_pipeline, (list, tuple))
            request_params = MultiPipelinePlanRequestParameters(
                self.moveit_py, planning_pipeline
            )

        return planning_component, request_params

    def _plan_once_blocking(
        self,
        planning_component: PlanningComponent,
        request_params: PlanRequestParameters
        | MultiPipelinePlanRequestParameters,
        planning_scene: Optional[PlanningScene] = None,
    ) -> MotionPlanResponse:
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
            return planning_component.plan(
                self.moveit_py,
                multi_plan_parameters=request_params,
                planning_scene=planning_scene,
            )
        else:
            return planning_component.plan(
                self.moveit_py,
                single_plan_parameters=request_params,
                planning_scene=planning_scene,
            )

    def _plan_blocking(
        self,
        goal: PlanningGoalT,
        *,
        cancel_event: Optional[threading.Event] = None,
        parse_kwargs: bool = True,
        **kwargs: Any,
    ) -> MotionPlanResponse | None:
        """
        Plan a trajectory to the given waypoint, retrying up to max_plan_attempts
        times until successful.

        Args:
            goal: The goal to plan for.
            parse_kwargs: Whether to parse the kwargs.
            cancel_event: An event that can be used to cancel planning.
            **kwargs: Additional keyword arguments to pass to `_parse_plan_args()`.
        Returns:
            The planned trajectory.
        Raises:
            MaxAttemptsReachedError: If the maximum number of planning attempts
                is reached.
            asyncio.CancelledError: If the planning is cancelled by the cancel_event.

        See Also:
            `_parse_plan_args()`: For implementation and parameter details
            `_get_planning_component_and_request_params()`: For further
                implementation details
        """
        self.log(
            f"Planning trajectory with goal {goal}, kwargs {kwargs}",
            severity="DEBUG",
        )

        # Parse the planning args if requested
        if parse_kwargs:
            kwargs, unused_kwargs = self._parse_plan_args(goal, **kwargs)
            assert len(unused_kwargs) == 0, f"Unused kwargs: {unused_kwargs}"
        else:
            kwargs["goal"] = goal

        if isinstance(goal, PoseStamped) and self.all_close_pose_stamped(
            goal, self.eef_pose_stamped()
        ):
            self.log("Already at goal, skipping planning")
            return None

        max_plan_attempts = kwargs.pop("max_plan_attempts")
        planning_scene = kwargs.pop("planning_scene")

        # Get the planning component and request parameters
        planning_component, request_params = (
            self._get_planning_component_and_request_params(**kwargs)
        )

        # Plan until successful or max attempts reached
        plan_responses = []
        for i in range(max_plan_attempts):
            if cancel_event is not None and cancel_event.is_set():
                raise asyncio.CancelledError("Plan cancelled")

            plan_response = self._plan_once_blocking(
                planning_component, request_params, planning_scene
            )
            if plan_response:
                self.log_plan_response(
                    plan_response,
                    attempt=i,
                    max_attempts=max_plan_attempts,
                    severity="DEBUG",
                )
                break
            else:
                plan_responses.append(plan_response)
                self.log_plan_response(
                    plan_response,
                    attempt=i,
                    max_attempts=max_plan_attempts,
                    severity="DEBUG",
                )
        else:
            raise MaxPlanningAttemptsReachedError(
                plan_responses=plan_responses,
                max_attempts=max_plan_attempts,
            )

        return plan_response

    async def plan(
        self, *args: Any, **kwargs: Any
    ) -> MotionPlanResponse | None:
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
        path_tolerance: Optional[float] = None,
        resample_dt: Optional[float] = None,
        min_angle_change: Optional[float] = None,
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
        """
        trajectory = robot_trajectory_copy(trajectory)

        # Get the parameters from the parameter server if not provided
        kwargs = self.get_parameter_wrapper("execution.totg")
        if path_tolerance is not None:
            kwargs["path_tolerance"] = path_tolerance
        if resample_dt is not None:
            kwargs["resample_dt"] = resample_dt
        if min_angle_change is not None:
            kwargs["min_angle_change"] = min_angle_change

        old_num_waypoints = len(trajectory)
        old_duration = trajectory.duration
        old_path_length = trajectory.path_length

        # if "resample_dt" not in kwargs and old_duration != 0.0:
        #     kwargs["resample_dt"] = old_duration / old_num_waypoints

        self.log(
            "Applying time parameterization to trajectory with kwargs "
            f"{kwargs}",
            severity="DEBUG",
        )
        # Apply time parameterization
        if not trajectory.apply_totg_time_parameterization(
            velocity_scaling_factor=velocity_scaling_factor,
            acceleration_scaling_factor=acceleration_scaling_factor,
            **kwargs,
        ):
            raise RuntimeError("Failed to apply time parameterization")

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
        mitigate_overshoot: Optional[bool] = None,
        overshoot_threshold: Optional[float] = None,
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
        """
        trajectory = robot_trajectory_copy(trajectory)

        kwargs = self.get_parameter_wrapper("execution.smoothing")
        if mitigate_overshoot is not None:
            kwargs["mitigate_overshoot"] = mitigate_overshoot
        if overshoot_threshold is not None:
            kwargs["overshoot_threshold"] = overshoot_threshold

        old_num_waypoints = len(trajectory)
        old_duration = trajectory.duration
        old_path_length = trajectory.path_length

        self.log(
            f"Applying smoothing to trajectory with kwargs {kwargs}",
            severity="DEBUG",
        )
        # Apply smoothing
        if not trajectory.apply_ruckig_smoothing(
            velocity_scaling_factor=velocity_scaling_factor,
            acceleration_scaling_factor=acceleration_scaling_factor,
            **kwargs,
        ):
            raise RuntimeError("Failed to apply smoothing")

        self.log(
            "Time parameterization applied successfully with: "
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
                raise InvalidTrajectoryError(trajectory)

    def _execute_once_blocking(
        self, trajectory_msg: RobotTrajectoryMsg
    ) -> ExecutionStatus:
        """Execute the given robot trajectory message once.

        Args:
            trajectory_msg: The robot trajectory message to execute.
        Returns:
            The status of the execution.
        """
        self.log("Executing trajectory once", severity="DEBUG")
        self.trajectory_execution_manager.push(trajectory_msg)
        return self.trajectory_execution_manager.execute_and_wait()

    def _execute_blocking(
        self,
        trajectory: RobotTrajectory,
        *,
        validate_trajectory: bool = True,
        velocity_scaling_factor: Optional[float] = None,
        acceleration_scaling_factor: Optional[float] = None,
        path_tolerance: Optional[float] = None,
        resample_dt: Optional[float] = None,
        mitigate_overshoot: Optional[bool] = None,
        overshoot_threshold: Optional[float] = None,
        min_angle_change: Optional[float] = None,
        group_name: Optional[str] = None,
        max_execution_attempts: int = 1,
        cancel_event: Optional[threading.Event] = None,
    ) -> RobotTrajectory:
        """Execute the given robot trajectory, retrying up to max_execution_attempts times
        until successful.

        Args:
            trajectory: The robot trajectory to execute.
            parse_kwargs: Whether to parse the kwargs.
            **kwargs: Additional keyword arguments to pass to `_parse_execute_args()`.
        Returns:
            ExecutionStatus: The status of the execution.
        Raises:
            MaxAttemptsReachedError: If the maximum number of execution attempts
                is reached.
            asyncio.CancelledError: If the execution is cancelled by the cancel_event.
        See Also:
            `_parse_execute_args()`: For implementation and parameter details
        """
        self.log(
            f"Executing trajectory with validate_trajectory {validate_trajectory}, "
            f"velocity_scaling_factor {velocity_scaling_factor}, acceleration_scaling_factor {acceleration_scaling_factor}, "
            f"path_tolerance {path_tolerance}, resample_dt {resample_dt}, min_angle_change {min_angle_change}, "
            f"group_name {group_name}, max_execution_attempts {max_execution_attempts}, cancel_event {cancel_event}",
            severity="DEBUG",
        )

        if velocity_scaling_factor is None:
            velocity_scaling_factor = cast(
                float,
                self.get_parameter_wrapper(
                    "execution.velocity_scaling_factor"
                ),
            )
        if acceleration_scaling_factor is None:
            acceleration_scaling_factor = cast(
                float,
                self.get_parameter_wrapper(
                    "execution.acceleration_scaling_factor"
                ),
            )

        # Apply time parameterization
        totg_trajectory = self._apply_totg(
            trajectory,
            velocity_scaling_factor=velocity_scaling_factor,
            acceleration_scaling_factor=acceleration_scaling_factor,
            path_tolerance=path_tolerance,
            resample_dt=resample_dt,
            min_angle_change=min_angle_change,
        )

        # Apply smoothing
        smoothed_trajectory = self._apply_smoothing(
            totg_trajectory,
            velocity_scaling_factor=velocity_scaling_factor,
            acceleration_scaling_factor=acceleration_scaling_factor,
            mitigate_overshoot=mitigate_overshoot,
            overshoot_threshold=overshoot_threshold,
        )

        if validate_trajectory:
            try:
                self._validate_trajectory(smoothed_trajectory)
                trajectory = smoothed_trajectory
            except InvalidTrajectoryError:
                self.log(
                    "Invalid trajectory after time parameterization and smoothing, validating totg trajectory",
                    severity="WARN",
                )
                try:
                    self._validate_trajectory(totg_trajectory)
                    trajectory = totg_trajectory
                except InvalidTrajectoryError:
                    self.log(
                        "Invalid trajectory after time parameterization and smoothing, validating unparameterized trajectory",
                        severity="WARN",
                    )
                    self._validate_trajectory(trajectory)

        # Convert to trajectory message
        trajectory_msg = smoothed_trajectory.get_robot_trajectory_msg()

        # Execute the trajectory until successful or max attempts reached
        execution_statuses = []
        for i in range(max_execution_attempts):
            if cancel_event is not None and cancel_event.is_set():
                raise asyncio.CancelledError("Execution cancelled")

            execution_status = self._execute_once_blocking(trajectory_msg)
            if execution_status:
                self.log(
                    f"Execution attempt {i + 1}/{max_execution_attempts} succeeded",
                    severity="DEBUG",
                )
                return trajectory
            else:
                self.log(
                    f"Execution attempt {i + 1}/{max_execution_attempts} "
                    f"failed with status {execution_status.status}",
                    severity="DEBUG",
                )
                execution_statuses.append(execution_status)
        else:
            raise MaxExecutionAttemptsReachedError(
                execution_statuses=execution_statuses,
                max_attempts=max_execution_attempts,
            )

    async def execute(self, *args: Any, **kwargs: Any) -> RobotTrajectory:
        """Execute the given robot trajectory in a separate thread.

        Retries up to max_execution_attempts times until successful.

        See Also:
            `_execute_blocking()`: For parameter details
        """
        cancel_event = threading.Event()
        try:
            return await asyncio.to_thread(
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

    async def plan_and_execute(
        self,
        goal: PlanningGoalT,
        cache_trajectory: bool = True,
        **kwargs: Any,
    ) -> dict[str, Any] | None:
        """Plan and execute a trajectory, using the cached trajectory if available.

        Args:
            goal: The goal to plan for.
            cache_trajectory: Whether to cache the planned trajectory.
            **kwargs: Keyword arguments to pass to `_parse_plan_and_execute_args()`.

        Returns:
            A dictionary containing the kwargs to cache the trajectory, or None
            if the trajectory was found in the cache.
        """
        self.log(
            "Planning and executing trajectory (with cache)", severity="DEBUG"
        )

        # Parse the planning kwargs
        parsed_kwargs, execute_kwargs = self._parse_plan_and_execute_args(
            goal, **kwargs
        )

        # Attempt to get the cached trajectory, otherwise plan and execute normally
        if self.use_cached_trajectories:
            start_state = parsed_kwargs["start_state"]
            goal_key = parsed_kwargs["goal"]
            pose_link = parsed_kwargs["pose_link"]
            group_name = parsed_kwargs["group_name"]
            try:
                trajectories = self.trajectory_cache.get_trajectories(
                    start_state=start_state,
                    goal=goal_key,
                    pose_link=pose_link,
                    group_name=group_name,
                )
            except KeyError:
                self.log(
                    "No cached trajectory found, planning and executing normally"
                )
            else:
                self.log(
                    "Cached trajectories found, trying to execute in order of path length"
                )
                num_trajectories = len(trajectories)
                i = 0
                while len(trajectories) > 0:
                    trajectory = heapq.heappop(trajectories)
                    try:
                        await self.execute(
                            trajectory,
                            **execute_kwargs,
                        )
                        return
                    except (
                        MaxExecutionAttemptsReachedError,
                        InvalidTrajectoryError,
                    ) as e:
                        self.log(
                            f"Error while executing cached trajectory {i + 1}/{num_trajectories}: {e}",
                            severity="WARN",
                        )
                        i += 1
                self.log(
                    "All cached trajectories failed, planning and executing normally"
                )
        else:
            self.log(
                "Not using cached trajectories, planning and executing normally"
            )

        response = await self.plan(parse_kwargs=False, **parsed_kwargs)
        if response is None:
            return

        await self.execute(response.trajectory, **execute_kwargs)

        to_cache_kwargs = {
            "trajectory": response.trajectory,
            "pose_link": parsed_kwargs["pose_link"],
            "true_start_state": parsed_kwargs["start_state"],
            "true_end_state": self.current_state,
            "true_goal": parsed_kwargs["goal"],
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

        if phase in OBJECT_MANIPULATION_PHASES:
            self.allow_collision(
                *zip(*self.allowed_object_manipulation_collisions)
            )

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
                extra_kwargs["planning_pipeline"] = "default"
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
        except (InvalidTrajectoryError, MaxAttemptsReachedError) as e:
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
                    await self.plan_and_execute(
                        goal, planning_pipeline="default", **kwargs
                    )
                case _:
                    raise
        finally:
            if phase in OBJECT_MANIPULATION_PHASES:
                self.disallow_collision(
                    *zip(*self.allowed_object_manipulation_collisions)
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
                    self.planning_link,
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
        except (InvalidTrajectoryError, MaxAttemptsReachedError) as e:
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
            except (InvalidTrajectoryError, MaxAttemptsReachedError):
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
                    await self.reset_dashboard(timeout)
                    await self.reset_rig()
                    break
                except (TimeoutError, CommanderRecoverableError) as e:
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
        self.log("Destroying commander node", severity="DEBUG")
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


async def asyncio_runner(coro: Coroutine, max_workers: int):
    """Run a coroutine in an asyncio event loop.

    This function sets the default executor for the asyncio event loop to the
    thread pool executor provided. Used to run coroutines in a custom thread
    pool executor for performance reasons (e.g. more workers).

    Args:
        coro: The coroutine to run.
    """
    with ThreadPoolExecutor(max_workers=max_workers) as tpe:
        loop = asyncio.get_event_loop()
        loop.set_default_executor(tpe)
        await coro


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
        debugpy.listen(4567)
        print("Waiting for debugger to attach")
        debugpy.wait_for_client()
        print("Debugger attached")

    try:
        commander = Commander()
        executor = SingleThreadedExecutor()
        executor.add_node(commander)

        with ThreadPoolExecutor(max_workers=1) as tpe:
            try:
                tpe.submit(executor.spin)
                coro = coro_fn(commander, args.coroutine_config)
                asyncio.run(asyncio_runner(coro, args.max_workers))
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
        print("Keyboard interrupt")
    except SystemExit:
        print("System exit")
    finally:
        print("Shutting down rclpy")
        rclpy.try_shutdown()  # type: ignore
