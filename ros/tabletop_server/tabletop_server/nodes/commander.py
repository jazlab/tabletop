import asyncio
import concurrent.futures
import glob
import hashlib
import importlib
import os
import threading
import traceback
from collections.abc import AsyncGenerator, Iterable, Mapping
from concurrent.futures import ThreadPoolExecutor
from contextlib import asynccontextmanager
from copy import deepcopy
from typing import Any, Callable, Coroutine, Optional, cast

import debugpy
import numpy as np
import pandas as pd
import rclpy
import rclpy.utilities
import yaml
from geometry_msgs.msg import Pose, PoseStamped
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
from moveit_msgs.msg import (
    AllowedCollisionMatrix,
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.msg import (
    PlanningScene as PlanningSceneMsg,
)
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
from rclpy.duration import Duration
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.executors import SingleThreadedExecutor
from shape_msgs.msg import (
    SolidPrimitive,
)
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
    simplify_bounding_primitive,
    simplify_convex_hull,
    simplify_quadratic_decimation,
    transform_geometry,
)
from tabletop_utils.ros import (
    MaxAttemptsReachedError,
    PlanningGoalT,
    ServiceCallError,
    ServiceCallUnsuccessfulError,
    arrays_from_pose_msg,
    attached_collision_object_msg,
    change_reference_frame_pose_stamped,
    matrix_from_pose_msg,
    mesh_collision_object_msg,
    moveit_error_code_map,
    object_color_msg,
    plane_collision_object_msg,
    pose_msg,
    pose_msg_from_matrix,
    pose_stamped_msg,
    primitive_collision_object_msg,
    quaternion_msg_from_axis_angle,
)
from tabletop_utils.trajectory_cache import FuzzyTrajectoryCache
from tf_transformations import identity_matrix
from ur_dashboard_msgs.srv import Load

from tabletop_server.nodes.base import DEFAULT_LOG_SEVERITY, BaseNode
from tabletop_server.nodes.mock_teensy import ArmDoorState


def create_task_wrapper(coro: Coroutine) -> asyncio.Task:
    """Wraps a coroutine in an asyncio.Task.

    Args:
        self: The instance of the class.
        coro: The coroutine to wrap.

    Returns:
        The created asyncio.Task.
    """
    return asyncio.create_task(coro)


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
        return create_task_wrapper(coro)

    return wrapper


class Commander(BaseNode):
    default_params: dict[str, Any] = BaseNode.default_params | {}
    required_params: set[str] = BaseNode.required_params | {
        "simulate",
        "max_plan_attempts",
        "max_execution_attempts",
        "dashboard.installation",
        "dashboard.program",
        "dashboard.init_timeout",
        "rig.init_timeout",
        "teensy.spin_period",
        "teensy.arm_door_timeout",
        "flic.spin_period",
        "planning.group_name",
        "planning.default_pipeline",
        "planning.linear_pipeline",
        "planning.eef_link",
        "planning.object_touch_links",
        "planning.idle_state",
        "planning.pre_fetch_offset",
        "planning.pre_attach_offset",
        "planning.post_attach_offset",
        "planning_scene.rig_meshes",
        "planning_scene.object_meshes",
    }

    ############################################################
    ########## Init ############################################
    ############################################################

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

        self.init_services()

        self.log("Commander initialized")

    def init_attributes(self):
        """Setup variables for the commander."""
        # Dynamic and static object IDs
        self.static_object_ids: set[str] = set()

        # Dynamic object initial stamped poses
        self.dynamic_object_init_poses_stamped: dict[str, PoseStamped] = {}

        # Last post-attach state
        self.last_post_attach_state: RobotState | None = None

        # Trajectory cache
        trajectory_cache_config = self.get_parameter_wrapper(
            "planning.trajectory_cache"
        )
        metadata = self.get_trajectory_cache_metadata()
        self.trajectory_cache = FuzzyTrajectoryCache(
            **trajectory_cache_config, metadata=metadata
        )

        # Execution and planning locks
        self.execution_lock = threading.Lock()
        self.planning_lock = threading.Lock()

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

    def get_planning_component(
        self, group_name: Optional[str] = None
    ) -> PlanningComponent:
        """Get the planning component for a given planning group name.

        Args:
            group_name: The name of the planning group. If None, the default
                planning group name from parameters is used.

        Returns:
            The planning component for the specified group.
        """
        if group_name is None:
            group_name = self.get_parameter_wrapper("planning.group_name")
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

    ############################################################
    ########## Logging #########################################
    ############################################################

    def log_plan_response(
        self,
        plan_response: MotionPlanResponse,
        severity: str = DEFAULT_LOG_SEVERITY,
    ) -> str:
        """Log a motion plan response.

        Args:
            plan_response: The motion plan response to log.
            severity: The logging severity level.

        Returns:
            A string containing the formatted log message.
        """
        full_msg = []
        if plan_response.error_code.val == MoveItErrorCodes.SUCCESS:
            msg = "Plan succeeded"
            self.log(msg, severity=severity)
            full_msg.append(msg)
        else:
            msg = f"Plan failed with error code: {moveit_error_code_map[plan_response.error_code.val]}"
            self.log(msg, severity=severity)
            full_msg.append(msg)
        full_msg.append(f"Plan result planner id: {plan_response.planner_id}")
        full_msg.append(
            f"Plan result planning time: {plan_response.planning_time}"
        )
        full_msg.append(f"Plan result planner id: {plan_response.planner_id}")
        return "\n".join(full_msg)

    ############################################################
    ########## Dashboard interface #############################
    ############################################################

    def dashboard_trigger(self, srv_name: str) -> Trigger.Response:
        """Call a dashboard client Trigger service."""
        self.log(f"Triggering {srv_name} in UR Dashboard", severity="DEBUG")
        response = self.service_call(
            srv_request=Trigger.Request(), srv_type=Trigger, srv_name=srv_name
        )
        return cast(Trigger.Response, response)

    async def dashboard_trigger_async(self, srv_name: str) -> Trigger.Response:
        """Asynchronously call a dashboard client Trigger service."""
        self.log(
            f"Triggering {srv_name} in UR Dashboard asynchronously",
            severity="DEBUG",
        )
        response = await self.service_call_async(
            srv_request=Trigger.Request(), srv_type=Trigger, srv_name=srv_name
        )
        return cast(Trigger.Response, response)

    def dashboard_load(self, srv_name: str, filename: str) -> None:
        """Load a program or installation on the robot dashboard."""
        self.log(
            f"Loading {srv_name}: {filename} in UR Dashboard", severity="DEBUG"
        )
        self.service_call(
            srv_request=Load.Request(filename=filename),
            srv_type=Load,
            srv_name=srv_name,
        )

    async def dashboard_load_async(
        self,
        srv_name: str,
        filename: str,
    ) -> Load.Response:
        """Asynchronously load a program or installation on the robot dashboard."""
        self.log(
            f"Loading {srv_name}: {filename} in UR Dashboard asynchronously",
            severity="DEBUG",
        )
        response = await self.service_call_async(
            srv_request=Load.Request(filename=filename),
            srv_type=Load,
            srv_name=srv_name,
        )
        return cast(Load.Response, response)

    def reset_dashboard(self):
        """Call a sequence of dashboard client services to reset the dashboard."""
        self.log("Resetting dashboard")
        self.dashboard_trigger("/dashboard_client/close_popup")
        self.dashboard_trigger("/dashboard_client/close_safety_popup")
        self.dashboard_trigger("/dashboard_client/unlock_protective_stop")
        self.dashboard_load(
            "/dashboard_client/load_program",
            self.get_parameter_wrapper("dashboard.program"),
        )
        self.dashboard_trigger("/dashboard_client/brake_release")
        self.dashboard_trigger("/dashboard_client/play")

    async def reset_dashboard_async(self):
        """Asynchronously call a sequence of dashboard client services to reset the dashboard."""
        self.log("Resetting dashboard")
        await self.dashboard_trigger_async("/dashboard_client/close_popup")
        await self.dashboard_trigger_async(
            "/dashboard_client/close_safety_popup"
        )
        await self.dashboard_trigger_async(
            "/dashboard_client/unlock_protective_stop"
        )
        await self.dashboard_load_async(
            "/dashboard_client/load_program",
            self.get_parameter_wrapper("dashboard.program"),
        )
        await self.dashboard_trigger_async("/dashboard_client/brake_release")
        await self.dashboard_trigger_async("/dashboard_client/play")

    def init_dashboard(self, *, timeout: Optional[int | float] = None):
        """Initialize the robot dashboard.

        Args:
            timeout: The timeout in seconds for initialization. If None, the default
                timeout from parameters is used.

        Raises:
            TimeoutError: If dashboard initialization times out.
        """
        self.log("Initializing dashboard")
        if timeout is None:
            timeout = cast(
                int | float,
                self.get_parameter_wrapper("dashboard.init_timeout"),
            )

        start_time = self.get_clock().now()
        while True:
            try:
                self.wait_for_service(
                    srv_type=Trigger, srv_name="/dashboard_client/close_popup"
                )
                self.reset_dashboard()
                break
            except (
                TimeoutError,
                ServiceCallError,
                ServiceCallUnsuccessfulError,
            ) as e:
                self.log(
                    f"Error initializing dashboard: {type(e).__name__}: {e}",
                    severity="ERROR",
                )
                if (
                    self.get_clock().now() - start_time
                ).nanoseconds / 1e9 > timeout:
                    raise TimeoutError("Dashboard initialization timed out")
                self.get_clock().sleep_for(Duration(seconds=1))

    ############################################################
    ########## Teensy interface ################################
    ############################################################

    # TODO: Cast instead of type: ignore
    async def _get_arm_door_async(self) -> GetArmDoor.Response:
        """Get the arm door state asynchronously."""
        response = await self.service_call_async(
            srv_request=GetArmDoor.Request(),
            srv_type=GetArmDoor,
            srv_name="/teensy/get_arm_door",
        )
        return cast(GetArmDoor.Response, response)

    async def _get_reward_async(self) -> GetReward.Response:
        """Get the reward state asynchronously."""
        return await self.service_call_async(
            srv_request=GetReward.Request(),
            srv_type=GetReward,
            srv_name="/teensy/get_reward",
        )  # type: ignore

    async def _get_hand_fixation_async(self) -> GetHandFixation.Response:
        """Get the hand fixation state asynchronously."""
        return await self.service_call_async(
            srv_request=GetHandFixation.Request(),
            srv_type=GetHandFixation,
            srv_name="/teensy/get_hand_fixation",
        )  # type: ignore

    async def _get_flic_async(self) -> GetFlic.Response:
        """Get the flic state asynchronously."""
        return await self.service_call_async(
            srv_request=GetFlic.Request(),
            srv_type=GetFlic,
            srv_name="/flic/get_flic",
        )  # type: ignore

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

    async def _start_arm_door_open_async(self) -> SetArmDoor.Response:
        """Coroutine to call the arm door service to open the arm door asynchronously."""
        self.log("Arm Door Open")
        response = await self.service_call_async(
            srv_request=SetArmDoor.Request(open=True),
            srv_type=SetArmDoor,
            srv_name="/teensy/set_arm_door",
        )
        return cast(SetArmDoor.Response, response)

    async def _start_arm_door_close_async(self) -> SetArmDoor.Response:
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

    async def _start_reward_async(self, duration: float) -> SetReward.Response:
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

    async def _wait_for_arm_door_open_async(
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
                response = await self._get_arm_door_async()
                if response.state == ArmDoorState.OPEN:
                    return True

                while response.state != ArmDoorState.OPEN:
                    response = await self._get_arm_door_async()
                    await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    async def _wait_for_arm_door_close_async(
        self, timeout: Optional[float] = None
    ) -> bool:
        """Wait for arm door to close."""
        spin_period = self.get_parameter_wrapper("teensy.spin_period")
        timeout = (
            timeout
            if timeout is not None
            else self.get_parameter_wrapper("teensy.arm_door_timeout")
        )
        response = await self._get_arm_door_async()
        if response.is_closed:
            return True
        try:
            async with asyncio.timeout(timeout):
                while not response.is_closed:
                    response = await self._get_arm_door_async()
                    await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    async def _wait_for_reward_async(self, duration: float) -> bool:
        """Wait for reward to start, then return True."""
        spin_period = self.get_parameter_wrapper("teensy.spin_period")
        timeout = duration + 2 * spin_period  # TODO: 2 is arbitrary, check
        response = await self._get_reward_async()
        if response.is_active:
            return True
        try:
            async with asyncio.timeout(timeout):
                while not response.is_active:
                    response = await self._get_reward_async()
                    await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    @asyncio_task_decorator
    async def wait_for_hand_fixation_press_async(
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
        initial_fixation = await self._get_hand_fixation_async()
        if initial_fixation.is_pressed:
            return True
        fixation = initial_fixation
        try:
            async with asyncio.timeout(timeout):
                while (
                    fixation.last_time_pressed_ms
                    == initial_fixation.last_time_pressed_ms
                ):
                    fixation = await self._get_hand_fixation_async()
                    await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    # TODO: Consider reimplementing using action server model (unsure if this is possible with the teensy)
    @asyncio_task_decorator
    async def wait_for_hand_fixation_release_async(
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
        initial_fixation = await self._get_hand_fixation_async()
        if not initial_fixation.is_pressed:
            return True
        fixation = initial_fixation
        try:
            async with asyncio.timeout(timeout):
                while (
                    fixation.last_time_released_ms
                    == initial_fixation.last_time_released_ms
                ):
                    fixation = await self._get_hand_fixation_async()
                    await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    # TODO: Potential race condition (between monkey and get_flic_async lol)
    @asyncio_task_decorator
    async def wait_for_flic_press_async(
        self, timeout: Optional[float] = None
    ) -> bool:
        """Wait for flic button press, then return True."""
        spin_period = self.get_parameter_wrapper("flic.spin_period")

        initial_flic = await self._get_flic_async()
        flic = initial_flic
        try:
            async with asyncio.timeout(timeout):
                while (
                    flic.last_time_pressed_ms
                    == initial_flic.last_time_pressed_ms
                ):
                    flic = await self._get_flic_async()
                    await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    @asyncio_task_decorator
    async def arm_door_open_and_wait(
        self, timeout: Optional[float] = None
    ) -> bool:
        """Open arm door and wait for it to be open."""
        await self._start_arm_door_open_async()
        return await self._wait_for_arm_door_open_async(timeout)

    @asyncio_task_decorator
    async def arm_door_close_and_wait(
        self, timeout: Optional[float] = None
    ) -> bool:
        """Close arm door and wait for it to be closed."""
        await self._start_arm_door_close_async()
        return await self._wait_for_arm_door_close_async(timeout)

    @asyncio_task_decorator
    async def reward_and_wait(self, duration: float):
        """Start reward and wait for it to be active."""
        await self._start_reward_async(duration)
        # Default timeout is duration plus spin period if not specified
        timeout = duration + self.get_parameter_wrapper("teensy.spin_period")

        if not await self._wait_for_reward_async(timeout):
            raise RuntimeError("Reward took longer than expected timeout")

    ############################################################
    ########## Poses ###########################################
    ############################################################

    @property
    def eef_link(self) -> str:
        """Get the end-effector link from the parameter server."""
        return self.get_parameter_wrapper("planning.eef_link")

    @property
    def touch_links(self) -> list[str]:
        """Get the touch links from the parameter server."""
        return self.get_parameter_wrapper("planning.object_touch_links")

    @property
    def planning_frame(self) -> str:
        """Get the planning frame from the planning scene."""
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            planning_frame = scene.planning_frame
            assert planning_frame == "world"
            return planning_frame

    @property
    def planning_group_name(self) -> str:
        """Get the planning group name from the parameter server."""
        return self.get_parameter_wrapper("planning.group_name")

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
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            tf = scene.get_frame_transform(frame_id)
            if (
                tf == identity_matrix()
            ).all() and frame_id != self.planning_frame:
                raise ValueError(f"Frame transform to {frame_id} is undefined")
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
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            eef_pose = scene.current_state.get_pose(self.eef_link)

        if frame_id is None or frame_id == self.planning_frame:
            return self.create_pose_stamped(
                pose=eef_pose,
                frame_id=self.planning_frame,
            )
        else:
            pose_stamped = self.create_pose_stamped(
                pose=eef_pose,
                frame_id=self.planning_frame,
            )
            return self.change_reference_frame(
                pose_stamped=pose_stamped,
                new_frame_id=frame_id,
            )

    def null_pose_stamped(self, frame_id: Optional[str] = None) -> PoseStamped:
        """Get a null pose in the specified frame."""
        return self.create_pose_stamped(frame_id=frame_id)

    def object_init_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the initial pose of an object from the parameters."""
        return deepcopy(self.dynamic_object_init_poses_stamped[object_id])

    def pre_fetch_pose_stamped(
        self, object_id: str, subframe_name: str
    ) -> PoseStamped:
        """Get the pre-fetch pose of an object (relative to the object subframe)."""
        return self.create_pose_stamped(
            frame_id=object_id + f"/{subframe_name}",
            position=self.get_parameter_wrapper("planning.pre_fetch_offset"),
        )

    def pre_attach_pose_stamped(
        self, object_id: str, subframe_name: str
    ) -> PoseStamped:
        """Get the pre-attach pose of an object (relative to the object subframe)."""
        return self.create_pose_stamped(
            frame_id=object_id + f"/{subframe_name}",
            position=self.get_parameter_wrapper("planning.pre_attach_offset"),
        )

    def attach_pose_stamped(
        self, object_id: str, subframe_name: str
    ) -> PoseStamped:
        """Get the attach pose of an object (relative to the object subframe)."""
        return self.create_pose_stamped(
            frame_id=object_id + f"/{subframe_name}"
        )

    def post_attach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the post-attach pose of an object (relative to the planning frame)."""
        post_attach_pose = self.object_init_pose_stamped(object_id)
        post_attach_offset = self.get_parameter_wrapper(
            "planning.post_attach_offset"
        )
        post_attach_pose.pose.position.x += post_attach_offset[0]
        post_attach_pose.pose.position.y += post_attach_offset[1]
        post_attach_pose.pose.position.z += post_attach_offset[2]

        return post_attach_pose

    def pre_detach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the pre-detach (post-attach) pose of an object (relative to the planning frame)."""
        return self.post_attach_pose_stamped(object_id)

    def detach_pose_stamped(self, object_id: str) -> PoseStamped:
        """Get the detach pose of an object (relative to the planning frame)."""
        return self.object_init_pose_stamped(object_id)

    def post_detach_pose_stamped(
        self, object_id: str, subframe_name: str
    ) -> PoseStamped:
        """Get the post-detach (pre-attach) pose of an object (relative to the object subframe)."""
        return self.pre_attach_pose_stamped(object_id, subframe_name)

    def post_return_pose_stamped(
        self, object_id: str, subframe_name: str
    ) -> PoseStamped:
        """Get the post-return (pre-fetch) pose of an object (relative to the object subframe)."""
        return self.pre_fetch_pose_stamped(object_id, subframe_name)

    ############################################################
    ########## Planning scene #################################
    ############################################################

    def log_planning_scene(self, severity: str = DEFAULT_LOG_SEVERITY):
        """Log the planning scene."""
        self.log("Logging planning scene", severity=severity)
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
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
        # TODO: Make it so that I don't load the collision matrix into memory
        # unless I need to log it
        self.log(
            f"Allowed collision matrix: \n{self.collision_matrix_df.to_string()}",
            severity=severity,
        )

    @property
    def current_state(self) -> RobotState:
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            return deepcopy(scene.current_state)

    @property
    def object_grid(self) -> np.ndarray:
        """Get the object grid config from the parameters."""
        object_kwargs = self.get_parameter_wrapper(
            "planning_scene.object_meshes.object_kwargs"
        )

        object_grid = np.empty((3, 10), dtype=object)
        for object_id, kwargs in object_kwargs.items():
            y, x = kwargs["idx"]
            object_grid[y, x] = object_id

        return object_grid

    @property
    def collision_objects(self) -> list[CollisionObject]:
        """Get the collision objects from the planning scene."""
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            planning_scene_msg: PlanningSceneMsg = scene.planning_scene_message
            return planning_scene_msg.world.collision_objects  # type: ignore

    @property
    def collision_object_ids(self) -> list[str]:
        """Get the collision object ids from the planning scene."""
        return [
            collision_object.id for collision_object in self.collision_objects
        ]

    @property
    def attached_collision_objects(self) -> list[AttachedCollisionObject]:
        """Get the attached collision objects from the planning scene."""
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            planning_scene_msg: PlanningSceneMsg = scene.planning_scene_message
            return planning_scene_msg.robot_state.attached_collision_objects  # type: ignore

    @property
    def attached_collision_object_ids(self) -> list[str]:
        """Get the attached collision object ids from the planning scene."""
        return [
            attached_collision_object.object.id
            for attached_collision_object in self.attached_collision_objects
        ]

    @property
    def collision_matrix_df(self) -> pd.DataFrame:
        """Get the collision matrix as a pandas DataFrame."""
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            msg: AllowedCollisionMatrix = (
                scene.planning_scene_message.allowed_collision_matrix
            )
            object_ids = list(msg.entry_names)
            matrix = np.array([row.enabled for row in msg.entry_values])
            matrix_df = pd.DataFrame(
                matrix,
                columns=object_ids,
                index=object_ids,
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
                "sphere",
            ]
            collision_object_ids = set(object_ids) - set(robot_link_ids)
            columns = robot_link_ids + list(collision_object_ids)
            matrix_df = matrix_df.loc[columns, columns]

            return matrix_df

    def is_state_colliding(self, group_name: Optional[str] = None) -> bool:
        """Check if the current state of the planning scene is colliding."""
        if group_name is None:
            group_name = self.planning_group_name

        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            is_colliding = scene.is_state_colliding(group_name)
            if is_colliding:
                self.log("State is colliding", severity="WARN")
            else:
                self.log("State is not colliding", severity="DEBUG")
            return is_colliding

    def allow_collision(self, id_1: str, id_2: str):
        """Allow collision between two collision objects."""
        self.log(f"Allowing collision between {id_1} and {id_2}")
        with self.planning_scene_monitor.read_write() as scene:
            scene: PlanningScene = scene
            scene.allowed_collision_matrix.set_entry(id_1, id_2, True)
            scene.current_state.update()

    def disallow_collision(self, id_1: str, id_2: str):
        """Disallow collision between two collision objects."""
        self.log(f"Disallowing collision between {id_1} and {id_2}")
        with self.planning_scene_monitor.read_write() as scene:
            scene: PlanningScene = scene
            scene.allowed_collision_matrix.set_entry(id_1, id_2, False)
            scene.current_state.update()

    def process_collision_object(
        self,
        collision_object: CollisionObject,
        *,
        dynamic: bool,
        pose_stamped: Optional[PoseStamped] = None,
        color: Optional[str | Iterable[float] | Mapping[str, float]] = None,
        allowed_collision_ids: Optional[list[str]] = None,
    ):
        """Process a collision object.

        Adds the collision object to the planning scene and
        adds the object to the dynamic object poses stamped or
        static object ids.

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
        color_msg = (
            object_color_msg(collision_object.id, color)
            if color is not None
            else None
        )
        self.planning_scene_monitor.process_collision_object(
            collision_object, color_msg
        )

        if dynamic:
            if pose_stamped is None:
                raise ValueError(
                    "Pose stamped is required for dynamic collision objects"
                )
            self.dynamic_object_init_poses_stamped[collision_object.id] = (
                deepcopy(pose_stamped)
            )
        else:
            self.log(
                f"Ignoring pose stamped for static collision object: {collision_object.id}",
                severity="DEBUG",
            )
            self.static_object_ids.add(collision_object.id)

        if allowed_collision_ids is not None:
            for allowed_collision_id in allowed_collision_ids:
                self.allow_collision(collision_object.id, allowed_collision_id)

    def add_plane_collision_object(
        self,
        object_id: str,
        *,
        coef: list[float],
        header_frame_id: Optional[str] = None,
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
        if header_frame_id is None:
            header_frame_id = self.planning_frame

        collision_object = plane_collision_object_msg(
            object_id=object_id,
            coef=coef,
            header_frame_id=header_frame_id,
            operation="ADD",
        )

        self.process_collision_object(
            collision_object=collision_object,
            dynamic=dynamic,
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

        collision_object = primitive_collision_object_msg(
            object_id=object_id,
            type=type,
            dimensions=dimensions,
            pose_stamped=pose_stamped,
            operation="ADD",
        )

        self.process_collision_object(
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
            case "bounding_primitive":
                geometry = simplify_bounding_primitive(geometry)
            case "quadratic_decimation":
                geometry = simplify_quadratic_decimation(geometry)
            case None:
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
        collision_object = mesh_collision_object_msg(
            object_id=object_id,
            geometry=geometry,
            pose_stamped=pose_stamped,
            subframe_names=subframe_names,
            subframe_poses=subframe_poses,
            operation="ADD",
        )

        self.process_collision_object(
            collision_object=collision_object,
            dynamic=dynamic,
            pose_stamped=pose_stamped,
            color=color,
            allowed_collision_ids=allowed_collision_ids,
        )

    def add_dynamic_mesh_collision_objects(
        self,
        *,
        path: str,
        origin: list[float],
        delta: list[float],
        common_kwargs: dict[str, Any],
        object_kwargs: dict[str, Any],
    ):
        """Add dynamic (object) meshes as collision objects to the planning scene.

        Loads meshes from a directory and adds them in a grid
        pattern based on the their index and the origin and delta.

        Args:
            path: The directory path to the object meshes.
            origin: The origin of the object meshes.
            delta: The delta of the object meshes.
            common_kwargs: The common kwargs for the object meshes.
            object_kwargs: The object kwargs for the object meshes.
        """

        # Get object origin and delta to calculate object position from index
        origin_arr = np.array(origin)
        delta_arr = np.array(delta)

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
            self.log(f"Processing object mesh collision object: {object_id}")

            # Skip if object already exists in the planning scene
            if object_id in self.collision_object_ids:
                self.log(
                    f"Skipping object mesh {object_id} because it already exists in the planning scene"
                )
                continue

            # Get common and per-object configurations
            kwargs: dict[str, Any] = deepcopy(common_kwargs)

            # Get index from per-object configurations and calculate position
            y, x = overrides.pop("idx")
            idx_arr = np.array([x, y, 0], dtype=float)
            position = origin_arr + delta_arr * idx_arr

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
        self.log(f"Attaching object {object_id}")
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
        self.log(f"Detaching object {object_id}")
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
        self.log("Detaching all collision objects")
        for object_id in self.attached_collision_object_ids:
            self.detach_collision_object(object_id)

    def remove_collision_object(self, object_id: str):
        """Remove a collision object from the planning scene."""
        self.log(f"Removing collision object: {object_id}")

        if object_id in self.attached_collision_object_ids:
            self.log(
                f"Object {object_id} is attached to the robot! "
                "Detaching it first.",
                severity="WARNING",
            )
            self.detach_collision_object(object_id)

        collision_object = CollisionObject(
            id=object_id, operation=CollisionObject.REMOVE
        )
        self.planning_scene_monitor.process_collision_object(collision_object)

        # Remove object from dynamic object poses stamped or static object ids
        if object_id in self.dynamic_object_init_poses_stamped:
            del self.dynamic_object_init_poses_stamped[object_id]
        elif object_id in self.static_object_ids:
            self.static_object_ids.remove(object_id)
        else:
            assert (
                False
            ), f"Object {object_id} not found in dynamic or static object ids"

    def remove_all_collision_objects(self):
        """Remove all non-attached collision objects from the planning scene."""
        self.log("Removing all collision objects")
        with self.planning_scene_monitor.read_write() as scene:
            scene: PlanningScene = scene
            scene.remove_all_collision_objects()
            scene.current_state.update()

        # Check that there are no collision objects left
        assert len(self.collision_object_ids) == 0

        self.static_object_ids = set()
        self.dynamic_object_init_poses_stamped = {}

    def log_collision_objects(self, severity: str = DEFAULT_LOG_SEVERITY):
        """Log the collision objects."""
        self.log("Logging collision objects", severity="DEBUG")
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
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

        # Log static collision objects
        self.log("Static collision objects:", severity=severity)
        for collision_object_id in self.static_object_ids:
            self.log(
                f"Static collision object id: {collision_object_id}",
                severity=severity,
            )
        self.log("=" * 80, severity=severity)

        # Log dynamic collision objects
        self.log("Dynamic collision objects:", severity=severity)
        for (
            object_id,
            pose_stamped,
        ) in self.dynamic_object_init_poses_stamped.items():
            self.log(
                f"Dynamic collision object id: {object_id}",
                severity=severity,
            )
            self.log(
                f"Dynamic collision object pose: {pose_stamped}",
                severity=severity,
            )
            self.log("=" * 80, severity=severity)

    def init_planning_scene(self):
        """Setup the planning scene

        Adds plane, primitive, and mesh collision objects from the planning scene configuration."""
        self.log("Setting up planning scene")

        # Add plane collision objects
        try:
            planes_config: dict[str, Any] = self.get_parameter_wrapper(
                "planning_scene.planes"
            )
        except ParameterNotDeclaredException:
            pass
        else:
            for object_id, kwargs in planes_config.items():
                self.static_object_ids.add(object_id)
                self.add_plane_collision_object(
                    object_id=object_id, dynamic=False, **kwargs
                )

        # Add primitive collision objects
        try:
            primitives_config: dict[str, Any] = self.get_parameter_wrapper(
                "planning_scene.primitives"
            )
        except ParameterNotDeclaredException:
            pass
        else:
            for object_id, kwargs in primitives_config.items():
                self.add_primitive_collision_object(
                    object_id=object_id, dynamic=False, **kwargs
                )
                self.static_object_ids.add(object_id)

        # Add dynamic object meshes
        object_meshes_config: dict[str, Any] = self.get_parameter_wrapper(
            "planning_scene.object_meshes"
        )
        self.add_dynamic_mesh_collision_objects(**object_meshes_config)

        # Add rig mesh collision objects
        rig_meshes_config: dict[str, Any] = self.get_parameter_wrapper(
            "planning_scene.rig_meshes"
        )
        for object_id, kwargs in rig_meshes_config.items():
            self.add_mesh_collision_object(
                object_id=object_id, dynamic=False, **kwargs
            )
            self.static_object_ids.add(object_id)

        # Update planning scene
        for static_object_id in self.static_object_ids:
            self.allow_collision("base_link_inertia", static_object_id)
            self.allow_collision("sphere", static_object_id)

        # Log planning scene
        self.log_planning_scene(severity="DEBUG")
        self.log_collision_objects(severity="DEBUG")
        self.log_collision_matrix(severity="DEBUG")

    def get_planning_scene_copy(self) -> PlanningScene:
        """Get a copy of the planning scene."""
        with self.planning_scene_monitor.read_only() as scene:
            return deepcopy(scene)

    ############################################################
    ########## Planning and execution ##########################
    ############################################################

    def get_empty_trajectory(self) -> RobotTrajectory:
        return RobotTrajectory(self.robot_model)

    def cartesian_path_constraints(
        self,
        goal_pose_stamped: PoseStamped,
        start_pose_stamped: Optional[PoseStamped] = None,
        *,
        line_width: float = 0.0005,
        line_length_tolerance: float = 0.001,
        line_weight: float = 1.0,
        orientation_tolerance: float = 0.05,
        orientation_weight: float = 1.0,
    ) -> Constraints:
        """
        Construct a line constraint for the linear cartesian path.
        """

        # For line or plane constraints, we must make sure to set the
        # constraint name to "use_equality_constraints"
        constraints = Constraints()
        constraints.name = "use_equality_constraints"

        if start_pose_stamped is None:
            start_pose_stamped = self.eef_pose_stamped()

        if (
            start_pose_stamped.header.frame_id
            != goal_pose_stamped.header.frame_id
        ):
            start_pose_stamped = self.change_reference_frame(
                pose_stamped=start_pose_stamped,
                new_frame_id=goal_pose_stamped.header.frame_id,
            )
        assert (
            start_pose_stamped.header.frame_id
            == goal_pose_stamped.header.frame_id
        )

        start_position, start_orientation = arrays_from_pose_msg(
            start_pose_stamped.pose
        )
        goal_position, goal_orientation = arrays_from_pose_msg(
            goal_pose_stamped.pose
        )

        distance = np.linalg.norm(goal_position - start_position)
        direction = (goal_position - start_position) / distance

        # Calculate quaternion from direction vector
        angle = 0.0
        orientation = quaternion_msg_from_axis_angle(direction, angle)

        # Create line constraint
        line_constraint = PositionConstraint()
        line_constraint.header.frame_id = start_pose_stamped.header.frame_id
        line_constraint.link_name = self.eef_link
        line = SolidPrimitive()
        line.type = SolidPrimitive.BOX
        line.dimensions = [
            line_width,
            line_width,
            distance + line_length_tolerance * 2,
        ]
        line_constraint.constraint_region.primitives.append(line)  # type: ignore

        # Create line constraint pose
        line_pose = pose_msg(
            position=start_position + direction * distance / 2,
            orientation=orientation,
        )
        line_constraint.constraint_region.primitive_poses.append(line_pose)  # type: ignore

        # Set weight (relative importance) of line constraint
        line_constraint.weight = line_weight

        # Add line constraint to constraints
        constraints.position_constraints.append(line_constraint)  # type: ignore

        # Create orientation constraint if start and goal orientations are the same
        if orientation_tolerance is not None:
            if np.allclose(
                goal_orientation,
                start_orientation,
                atol=orientation_tolerance,
            ):
                orientation_constraint = OrientationConstraint()
                orientation_constraint.header.frame_id = (
                    start_pose_stamped.header.frame_id
                )
                orientation_constraint.link_name = self.eef_link
                orientation_constraint.orientation = (
                    goal_pose_stamped.pose.orientation
                )
                orientation_constraint.absolute_x_axis_tolerance = (
                    orientation_tolerance
                )
                orientation_constraint.absolute_y_axis_tolerance = (
                    orientation_tolerance
                )
                orientation_constraint.absolute_z_axis_tolerance = (
                    orientation_tolerance
                )
                orientation_constraint.weight = orientation_weight
                if orientation_constraint.weight is None:
                    raise ValueError(
                        "Orientation constraint weight is not set"
                    )
                constraints.orientation_constraints.append(  # type: ignore
                    orientation_constraint
                )
            else:
                assert False  # For debugging
                self.log(
                    "Start and goal orientations are different, skipping orientation constraint",
                    severity="WARNING",
                )

        return constraints

    def _plan_once(
        self,
        goal: PlanningGoalT,
        start_state: Optional[RobotState] = None,
        *,
        path_constraints: Optional[Constraints] = None,
        pose_link: Optional[str] = None,
        planning_pipeline: str | list[str] = "default",
        group_name: Optional[str] = None,
    ) -> MotionPlanResponse:
        """
        Plan a trajectory to the given waypoint once.

        Args:
            goal (PoseStamped | str): The goal pose in a stamped coordinate frame
                or a configuration name.
            path_constraints (Constraints, optional): The path constraints to use
                for the trajectory.
            pose_link (str, optional): The link name to use for the goal pose.
                Defaults to parameter "planning.eef_link" if not provided.
            planning_pipeline (str, optional): The planning pipeline to use.
                Defaults to parameter "planning.pipeline" if not provided.
        Returns:
            MotionPlanResponse: The planned trajectory.
        """
        self.log("Planning trajectory once:", severity="DEBUG")

        planning_component = self.get_planning_component(group_name)

        # Set start state to current state
        if start_state is not None:
            if not planning_component.set_start_state(robot_state=start_state):
                raise ValueError(f"Invalid start state: {start_state}")
        else:
            planning_component.set_start_state_to_current_state()

        # Set goal state from pose or configuration name
        original_pose_link = pose_link
        if pose_link is None:
            pose_link = self.eef_link

        if goal == "idle":
            goal = self.get_parameter_wrapper("planning.idle_state")
            if not isinstance(goal, str):
                goal = self.current_state
                goal.set_joint_group_positions(
                    group_name=group_name, positions=goal
                )

        # Set goal state
        goal_kwargs = {}
        if isinstance(goal, PoseStamped):
            goal_kwargs["pose_stamped_msg"] = goal
            goal_kwargs["pose_link"] = pose_link
        elif isinstance(goal, RobotState):
            goal_kwargs["robot_state"] = goal
        elif isinstance(goal, str):
            if original_pose_link is not None:
                raise ValueError(
                    "pose_link must not be provided if goal is a configuration name"
                )
            goal_kwargs["configuration_name"] = goal

        if not planning_component.set_goal_state(**goal_kwargs):
            raise ValueError(f"Invalid goal: {goal}")

        # Set planning pipeline
        if isinstance(planning_pipeline, str):
            if "default" in planning_pipeline:
                planning_pipeline = self.get_parameter_wrapper(
                    "planning.default_pipeline"
                )
            elif "linear" in planning_pipeline.lower():
                if not isinstance(goal, PoseStamped):
                    raise ValueError(
                        "Linear pipeline requires a PoseStamped goal"
                    )
                planning_pipeline = self.get_parameter_wrapper(
                    "planning.linear_pipeline"
                )
            else:
                try:
                    planning_pipeline = self.get_parameter_wrapper(
                        f"planning.{planning_pipeline}"
                    )
                except ParameterNotDeclaredException:
                    pass

        # Set path constraints
        if path_constraints is not None:
            planning_component.set_path_constraints(path_constraints)

        # Plan
        self.log(
            f"Planning to {goal} with path constraints {path_constraints} with pipeline {planning_pipeline}",
            severity="DEBUG",
        )
        if isinstance(planning_pipeline, str):
            request_params = PlanRequestParameters(
                self.moveit_py, planning_pipeline
            )
            return planning_component.plan(
                self.moveit_py, single_plan_parameters=request_params
            )
        else:
            assert isinstance(planning_pipeline, (list, tuple))
            request_params = MultiPipelinePlanRequestParameters(
                self.moveit_py, planning_pipeline
            )
            return planning_component.plan(
                self.moveit_py, multi_plan_parameters=request_params
            )

    def plan(
        self,
        *args: Any,
        max_attempts: Optional[int] = None,
        cancel_event: Optional[threading.Event] = None,
        **kwargs: Any,
    ) -> MotionPlanResponse:
        """
        Plan a trajectory to the given waypoint, retrying up to max_attempts
        times until successful.

        Args:
            max_attempts: The maximum number of planning attempts.
            cancel_event: An event that can be used to cancel the plan.
            *args: Additional positional arguments to pass to `plan_once()`.
            **kwargs: Additional keyword arguments to pass to `plan_once()`.
        Returns:
            The planned trajectory.
        """

        if max_attempts is None:
            max_attempts = cast(
                int, self.get_parameter_wrapper("max_plan_attempts")
            )

        failure_msgs = []
        for i in range(max_attempts):
            # Check if the plan has been cancelled
            if cancel_event is not None and cancel_event.is_set():
                raise asyncio.CancelledError("Plan cancelled")

            # Plan once and check if it was successful
            plan_response = self._plan_once(*args, **kwargs)
            if plan_response.error_code.val == MoveItErrorCodes.SUCCESS:
                self.log(f"Planning attempt {i + 1}/{max_attempts} succeeded")
                self.log_plan_response(plan_response)
                break
            else:
                error_msg = f"Planning attempt {i + 1}/{max_attempts} failed with error code {moveit_error_code_map[plan_response.error_code.val]}"
                failure_msgs.append(error_msg)
                self.log(
                    error_msg,
                    severity="WARN",
                )
                self.log_plan_response(plan_response, severity="DEBUG")
        else:
            error_msg = f"Max planning attempts ({max_attempts}) reached!: {failure_msgs}"
            self.log(error_msg, severity="ERROR")
            raise MaxAttemptsReachedError(error_msg)

        return plan_response

    async def _plan_async(
        self, *args: Any, **kwargs: Any
    ) -> MotionPlanResponse:
        """Asynchronously calls `plan()` method in a separate thread.

        See Also:
            `plan()`: For parameter details and synchronous implementation.
        """
        cancel_event = threading.Event()
        try:
            return await asyncio.to_thread(
                self.plan, *args, cancel_event=cancel_event, **kwargs
            )
        finally:
            cancel_event.set()

    @asyncio_task_decorator
    async def plan_async(
        self, *args: Any, **kwargs: Any
    ) -> MotionPlanResponse:
        """Asynchronously calls `plan()` method in a separate thread.

        See Also:
            `plan()`: For parameter details and synchronous implementation.
        """
        return await self._plan_async(*args, **kwargs)

    def _execute_once(
        self, robot_trajectory: RobotTrajectory
    ) -> ExecutionStatus:
        """Execute the given robot trajectory.

        Args:
            robot_trajectory: The robot trajectory to execute.

        Returns:
            ExecutionStatus: The status of the execution.
        """
        self.trajectory_execution_manager.push(
            robot_trajectory.get_robot_trajectory_msg()
        )
        return self.trajectory_execution_manager.execute_and_wait()

    async def _execute_once_async(
        self, trajectory: RobotTrajectory | RobotTrajectoryMsg
    ) -> ExecutionStatus:
        """Asynchronously execute the given robot trajectory.

        Wraps the trajectory_execution_manager.execute() method in an rclpy
        future to support awaiting the execution.

        Args:
            trajectory: The robot trajectory or trajectory message to execute.

        Returns:
            ExecutionStatus: The status of the execution.
        """
        future = concurrent.futures.Future()

        def done_callback(status: ExecutionStatus):
            future.set_result(status)

        if not isinstance(trajectory, RobotTrajectoryMsg):
            trajectory = trajectory.get_robot_trajectory_msg()

        self.trajectory_execution_manager.push(trajectory)
        self.trajectory_execution_manager.execute(done_callback)

        return await asyncio.wrap_future(future)

    def execute(
        self,
        *args: Any,
        max_attempts: Optional[int] = None,
        cancel_event: Optional[threading.Event] = None,
        **kwargs: Any,
    ):
        """Execute the given robot trajectory, retrying up to max_attempts times
        until successful.

        Args:
            max_attempts: The maximum number of execution attempts.
            cancel_event: An event that can be used to cancel the execution.
            *args: Additional positional arguments to pass to `execute_once()`.
            **kwargs: Additional keyword arguments to pass to `execute_once()`.
        Returns:
            ExecutionStatus: The status of the execution.
        """
        if max_attempts is None:
            max_attempts = self.get_parameter_wrapper("max_execution_attempts")

        failure_msgs = []
        for i in range(max_attempts):  # type: ignore
            if cancel_event is not None and cancel_event.is_set():
                raise asyncio.CancelledError("Execution cancelled")
            try:
                execution_status = self._execute_once(*args, **kwargs)
                if execution_status:
                    self.log(
                        f"Execution attempt {i + 1}/{max_attempts} succeeded"
                    )
                    break
                else:
                    error_msg = f"Execution attempt {i + 1}/{max_attempts} failed with status {execution_status.status}"
                    failure_msgs.append(error_msg)
                    self.log(
                        error_msg,
                        severity="WARN",
                    )
            except Exception as e:
                error_msg = f"Execution attempt {i + 1}/{max_attempts} raised exception {type(e).__name__}: {e}"
                failure_msgs.append(error_msg)
                self.log(
                    error_msg,
                    severity="WARN",
                )
        else:
            error_msg = f"Max execution attempts ({max_attempts}) reached!: {failure_msgs}"
            self.log(error_msg, severity="ERROR")
            raise MaxAttemptsReachedError(error_msg)

    async def _execute_async(
        self, *args: Any, max_attempts: Optional[int] = None, **kwargs: Any
    ):
        """Asynchronously execute the given robot trajectory, retrying up to
        max_attempts times until successful.

        Args:
            *args: Additional positional arguments to pass to `execute_once_async()`.
            max_attempts: The maximum number of execution attempts.
            **kwargs: Additional keyword arguments to pass to `execute_once_async()`.

        Raises:
            MaxAttemptsReachedError: If maximum execution attempts (param:
                max_execution_attempts) are reached

        See Also:
            `_execute_once_async()`: For parameter details.
        """
        if max_attempts is None:
            max_attempts = cast(
                int, self.get_parameter_wrapper("max_execution_attempts")
            )

        failure_msgs = []
        for i in range(max_attempts):
            try:
                execution_status = await self._execute_once_async(
                    *args, **kwargs
                )
                if execution_status:
                    self.log(
                        f"Execution attempt {i + 1}/{max_attempts} succeeded"
                    )
                    return
                else:
                    error_msg = f"Execution attempt {i + 1}/{max_attempts} failed with status {execution_status.status}"
                    failure_msgs.append(error_msg)
                    self.log(
                        error_msg,
                        severity="WARN",
                    )
            except Exception as e:
                error_msg = f"Execution attempt {i + 1}/{max_attempts} raised exception {type(e).__name__}: {e}"
                failure_msgs.append(error_msg)
                self.log(
                    error_msg,
                    severity="WARN",
                )
        else:
            error_msg = f"Max execution attempts ({max_attempts}) reached!: {failure_msgs}"
            self.log(error_msg, severity="ERROR")
            raise MaxAttemptsReachedError(error_msg)

    @asyncio_task_decorator
    async def execute_async(self, *args: Any, **kwargs: Any):
        """Asynchronously execute the given robot trajectory, retrying up to
        max_attempts times until successful.

        See Also:
            `_execute_async()`: For parameter details and synchronous implementation.
        """
        return await self._execute_async(*args, **kwargs)

    def plan_and_execute(
        self,
        *args: Any,
        cancel_event: Optional[threading.Event] = None,
        **kwargs: Any,
    ) -> MotionPlanResponse:
        """Plan and execute a trajectory.

        Performs max_plan_attempts planning attempts and max_execution_attempts
        execution attempts. If the method returns successfully, the trajectory
        has been executed (no status is returned).

        Args:
            *args: Additional positional arguments to pass to `plan()`.
            cancel_event: An event that can be used to cancel the plan and
                execute.
            **kwargs: Additional keyword arguments to pass to `plan()`.

        Returns:
            This method does not return anything but may raise exceptions

        Raises:
            MaxAttemptsReachedError: If maximum planning attempts (param:
                max_plan_attempts) or execution attempts (param:
                max_execution_attempts) are reached
        """
        plan_response = self.plan(*args, cancel_event=cancel_event, **kwargs)
        self.execute(plan_response.trajectory, cancel_event=cancel_event)
        return plan_response

    async def _plan_and_execute_async(
        self, *args: Any, **kwargs: Any
    ) -> MotionPlanResponse:
        """
        Asynchronous coroutine wrapper for `plan_and_execute()` method.

        Runs the `plan_and_execute()` method in a separate thread and awaits
        the result.

        See Also:
            `plan_and_execute()`: For parameter details and synchronous
                implementation.
        """
        cancel_event = threading.Event()
        try:
            return await asyncio.to_thread(
                self.plan_and_execute,
                *args,
                cancel_event=cancel_event,
                **kwargs,
            )
        finally:
            cancel_event.set()

    @asyncio_task_decorator
    async def plan_and_execute_async(
        self, *args: Any, **kwargs: Any
    ) -> MotionPlanResponse:
        """
        Asynchronous coroutine wrapper for `plan_and_execute()` method.

        Runs the `plan_and_execute()` method in a separate thread and awaits
        the result.
        See Also:
            `plan_and_execute()`: For parameter details and synchronous
                implementation.
        """
        return await self._plan_and_execute_async(*args, **kwargs)

    ############################################################
    ########## Fetch and return ################################
    ############################################################

    def get_trajectory_cache_metadata(self) -> dict[str, Any]:
        """Get the metadata for the trajectory cache.

        Returns:
            dict[str, Any]: The metadata for the trajectory cache.
        """
        metadata: dict[str, Any] = {}

        # Rig mesh hash
        rig_meshes_kwargs = self.get_parameter_wrapper(
            "planning_scene.rig_meshes"
        )
        hash_algorithm = hashlib.md5()
        for kwargs in rig_meshes_kwargs.values():
            with open(kwargs["path"], "rb") as f:
                while chunk := f.read(8192):
                    hash_algorithm.update(chunk)

        metadata["rig_mesh_hash"] = hash_algorithm.hexdigest()

        # Object grid
        metadata["object_grid"] = self.object_grid.tolist()

        return metadata

    async def plan_and_execute_async_cached(
        self,
        goal: PlanningGoalT,
        start_state: Optional[RobotState] = None,
        cache_trajectory: bool = True,
        *args: Any,
        **kwargs: Any,
    ) -> RobotTrajectoryMsg | None:
        """Plan and execute a trajectory, using the cached trajectory if available.

        Args:
            goal: The goal to query the cache or plan for.
            start_state: The start state to query the cache or plan for.
            cache_trajectory: Whether to cache the planned trajectory.
            *args: Additional positional arguments to pass to `plan_and_execute_async()`.
            **kwargs: Additional keyword arguments to pass to `plan_and_execute_async()`.

        Returns:
            RobotTrajectoryMsg | None: The newly planned trajectory, or the cached
                trajectory message if available.
        """
        if start_state is None:
            start_state = self.current_state

        # If the goal is a PoseStamped, change the reference frame to the
        # planning frame for the cache key
        if (
            isinstance(goal, PoseStamped)
            and goal.header.frame_id != self.planning_frame
        ):
            goal_key = self.change_reference_frame(goal, self.planning_frame)
        else:
            goal_key = goal

        # Attempt to get the cached trajectory, otherwise plan and execute normally
        try:
            try:
                trajectory_msg = self.trajectory_cache[(goal_key, start_state)]
                self.log(
                    f"Using cached trajectory for "
                    f"goal: {goal_key}, start state: {start_state}"
                )
            except KeyError:
                if not isinstance(goal_key, RobotState):
                    raise
                # If the goal is a RobotState, try to get the cached trajectory
                # with the start state and goal key swapped
                trajectory_msg = self.trajectory_cache[(start_state, goal_key)]
                self.log(
                    f"Using reversed cached trajectory for "
                    f"goal: {goal_key}, start state: {start_state}"
                )
                trajectory = RobotTrajectory(self.robot_model)
                trajectory.set_robot_trajectory_msg(trajectory_msg)
                trajectory = cast(RobotTrajectory, reversed(trajectory))
                trajectory_msg = trajectory.get_robot_trajectory_msg()

            # Execute the cached trajectory
            self.log(
                f"Executing cached trajectory: {trajectory_msg}",
                severity="DEBUG",
            )
            await self.execute_async(trajectory_msg)
            return None
        except KeyError:
            self.log(
                f"No cached trajectory for "
                f"goal: {goal_key}, start state: {start_state}"
            )
        except Exception as e:
            self.log(f"Error while executing cached trajectory: {e}")

        self.log(
            f"Planning normally and executing trajectory for "
            f"goal: {goal_key}, start state: {start_state}"
        )
        response = await self._plan_and_execute_async(
            goal=goal, start_state=start_state, *args, **kwargs
        )
        # Cache the trajectory if requested
        trajectory_msg = response.trajectory.get_robot_trajectory_msg()
        if cache_trajectory:
            self.trajectory_cache[(goal_key, start_state)] = trajectory_msg
            self.log(
                f"Cached trajectory for "
                f"goal: {goal_key}, start state: {start_state}"
            )
        return trajectory_msg

    @asyncio_task_decorator
    async def fetch_object_async(
        self,
        object_id: str,
        end_goal: PoseStamped | str,
        *,
        subframe_name: str = "default",
    ) -> None:
        """Fetches an object and moves it to the specified end goal.

        Args:
            object_id: The ID of the object to fetch
            end_goal: The pose to move the object to after fetching
            subframe_name: The subframe name of the object
        """
        self.log(f"Fetching object {object_id} from subframe {subframe_name}")

        trajectory_msgs = []
        goals = []
        start_states = []

        # Pre-fetch pose
        pre_fetch_pose_stamped = self.pre_fetch_pose_stamped(
            object_id, subframe_name
        )
        self.log(f"Moving to pre-fetch pose {pre_fetch_pose_stamped}")
        trajectory_msg = await self.plan_and_execute_async_cached(
            goal=pre_fetch_pose_stamped, cache_trajectory=False
        )
        if trajectory_msg is not None:
            trajectory_msgs.append(trajectory_msg)
            goals.append(pre_fetch_pose_stamped)
            start_states.append(self.current_state)

        # Allow collision between touch links and object
        for touch_link in self.touch_links:
            self.allow_collision(touch_link, object_id)

        # Pre-attach pose
        pre_attach_pose_stamped = self.pre_attach_pose_stamped(
            object_id, subframe_name
        )
        self.log(f"Moving to pre-attach pose {pre_attach_pose_stamped}")
        trajectory_msg = await self.plan_and_execute_async_cached(
            goal=pre_attach_pose_stamped,
            planning_pipeline="linear",
            cache_trajectory=False,
        )
        if trajectory_msg is not None:
            trajectory_msgs.append(trajectory_msg)
            goals.append(pre_attach_pose_stamped)
            start_states.append(self.current_state)

        # Attach pose (no offset with respect to object frame)
        attach_pose_stamped = self.attach_pose_stamped(
            object_id, subframe_name
        )
        self.log(f"Moving to attach pose {attach_pose_stamped}")
        trajectory_msg = await self.plan_and_execute_async_cached(
            goal=attach_pose_stamped,
            planning_pipeline="linear",
            cache_trajectory=False,
        )
        if trajectory_msg is not None:
            trajectory_msgs.append(trajectory_msg)
            goals.append(attach_pose_stamped)
            start_states.append(self.current_state)

        # Attach object
        self.attach_collision_object(
            object_id=object_id,
            link_name=self.eef_link,
            touch_links=self.touch_links,
        )

        try:
            # Allow collision between object and static objects
            # Needed to prevent errors when removing object from its tool pocket
            for static_object_id in self.static_object_ids:
                self.allow_collision(object_id, static_object_id)

            # Post-attach pose
            post_attach_pose_stamped = self.post_attach_pose_stamped(object_id)
            self.log(f"Moving to post-attach pose {post_attach_pose_stamped}")
            trajectory_msg = await self.plan_and_execute_async_cached(
                goal=post_attach_pose_stamped,
                planning_pipeline="linear",
                cache_trajectory=False,
            )
            if trajectory_msg is not None:
                trajectory_msgs.append(trajectory_msg)
                goals.append(post_attach_pose_stamped)
                start_states.append(self.current_state)

            # Save the current state as the last post-attach state
            # self.last_post_attach_state = self.current_state
        finally:
            # Disallow collision between object and static objects
            for static_object_id in self.static_object_ids:
                self.disallow_collision(object_id, static_object_id)

        # Move to target pose
        self.log(f"Moving to end goal {end_goal}")
        trajectory_msg = await self.plan_and_execute_async_cached(
            goal=end_goal,
            planning_pipeline="linear",
            cache_trajectory=False,
        )
        trajectory_msgs.append(trajectory_msg)
        goals.append(end_goal)
        start_states.append(self.current_state)

        # Cache all trajectories
        for trajectory_msg, goal, start_state in zip(
            trajectory_msgs, goals, start_states
        ):
            self.trajectory_cache[(goal, start_state)] = trajectory_msg

    @asyncio_task_decorator
    async def return_object_async(
        self,
        end_goal: Optional[PoseStamped | str] = None,
        *,
        subframe_name: str = "default",
    ):
        """Return an object to its original position.

        This method is the reverse of fetch_object_async.

        Args:
            subframe_name: The subframe name of the object (default: "default")
            end_goal: Optional pose to move to after returning the object
        """
        # Get object ID from planning scene and check that there is exactly one
        # attached collision object
        attached_collision_object_ids = self.attached_collision_object_ids
        if len(attached_collision_object_ids) != 1:
            raise RuntimeError(
                f"Expected exactly one attached collision object, "
                f"but got {len(attached_collision_object_ids)}"
            )

        object_id = attached_collision_object_ids[0]
        self.log(f"Returning object {object_id}")

        # Move to saved post-attach state
        if self.last_post_attach_state is not None:
            self.log(
                f"Moving to saved post-attach state {self.last_post_attach_state}"
            )
            await self._plan_and_execute_async(
                goal=self.last_post_attach_state,
            )
            self.last_post_attach_state = None
        else:
            self.log(
                "No saved post-attach state found, moving to pre-detach pose",
                severity="WARN",
            )
            await self._plan_and_execute_async(
                goal=self.pre_detach_pose_stamped(object_id),
            )

        # Allow collision between object and static objects
        # Needed to prevent errors when inserting object into its tool pocket
        for static_object_id in self.static_object_ids:
            self.allow_collision(object_id, static_object_id)

        # Move to the detach pose
        self.log(
            f"Moving to detach pose {self.detach_pose_stamped(object_id)}"
        )
        await self._plan_and_execute_async(
            goal=self.detach_pose_stamped(object_id),
            planning_pipeline="linear",
        )

        # Detach the object
        self.detach_collision_object(object_id=object_id)

        # Disallow collision between object and static objects
        for static_object_id in self.static_object_ids:
            self.disallow_collision(object_id, static_object_id)

        # Allow collision between robot and object
        for touch_link in self.touch_links:
            self.allow_collision(touch_link, object_id)

        # Move to the post-detach pose
        self.log(
            f"Moving to post-detach pose {self.post_detach_pose_stamped(object_id, subframe_name)}"
        )
        await self._plan_and_execute_async(
            goal=self.post_detach_pose_stamped(object_id, subframe_name),
            planning_pipeline="linear",
        )

        # Move to the post-return (pre-fetch) pose
        self.log(
            f"Moving to post-return pose {self.post_return_pose_stamped(object_id, subframe_name)}"
        )
        await self._plan_and_execute_async(
            goal=self.post_return_pose_stamped(object_id, subframe_name),
            planning_pipeline="linear",
        )

        # Disallow collision between touch links and object
        for touch_link in self.touch_links:
            self.disallow_collision(touch_link, object_id)

        # Move to end pose if specified
        if end_goal is not None:
            self.log(f"Moving to end goal {end_goal}")
            await self._plan_and_execute_async(goal=end_goal)

    ############################################################
    ########## Reset rig #######################################
    ############################################################

    async def move_simulation_out_of_collision_async(
        self, goal: PoseStamped | str, pose_link: Optional[str] = None
    ):
        """
        Move the robot out of collision with the scene asynchronously.

        Using this function will remove all collision objects from the
        planning scene and move the robot to the target pose, then add the
        collision objects back to the planning scene. To be used only with
        ursim. With the real robot, the user should manually (via the teach
        pendant) move the robot away from the collision objects.
        """
        self.log("Moving out of collision asynchronously")
        assert self.get_parameter_wrapper("simulate")
        self.detach_collision_object(
            object_id=self.attached_collision_object_ids[0]
        )
        self.remove_all_collision_objects()
        await self._plan_and_execute_async(goal=goal, pose_link=pose_link)
        self.init_planning_scene()

    async def reset_rig_async(
        self,
        end_goal: Optional[PoseStamped | str] = None,
        severity: str = "INFO",
    ):
        """Move the robot out of collision if necessary and return any attached
        objects to their original positions.
        """
        self.log("Resetting rig asynchronously", severity=severity)
        if self.is_state_colliding(self.planning_group_name):
            if self.get_parameter_wrapper("simulate"):
                await self.move_simulation_out_of_collision_async(goal="idle")
            else:
                raise RuntimeError(
                    "Robot is in collision with the scene! "
                    "Please move the robot away from the collision objects manually."
                )
        if len(self.attached_collision_object_ids) > 0:
            await self.return_object_async(end_goal=end_goal)
        else:
            await self.plan_and_execute_async(goal="idle")

    ############################################################
    ########## Context manager #################################
    ############################################################

    def schedule(self, *coros: Coroutine) -> asyncio.Task | list[asyncio.Task]:
        """Schedule coroutines to run.

        Args:
            *coros: Coroutines to schedule.

        Returns:
            List of scheduled tasks.
        """
        tasks = []
        for coro in coros:
            tasks.append(create_task_wrapper(coro))
        return tasks[0] if len(tasks) == 1 else tasks

    @asynccontextmanager
    async def context_manager(self) -> AsyncGenerator[None, None]:
        """
        Context manager for planning and executing actions.

        This context manager handles exceptions that occur during planning and
        executing actions, and automatically resets the robot and moves it out
        of collision if necessary.
        """
        self.log("Entering planning context manager", severity="DEBUG")

        await self.reset_rig_async(end_goal="idle", severity="DEBUG")

        try:
            yield None
        except (
            TimeoutError,
            MaxAttemptsReachedError,
            ServiceCallError,
            ServiceCallUnsuccessfulError,
        ) as e:
            self.log(
                "Caught exception while running commander:", severity="WARN"
            )
            self.log(f"{type(e).__name__}: {e}", severity="WARN")
            self.log(f"Traceback: {traceback.format_exc()}", severity="WARN")

            # TODO: This is a hack to ensure the robot is reset and moved out of
            # collision before the context manager is entered. Only needed for
            # ursim. Need to return object to original pose if object is attached.
            while True:
                try:
                    await self.reset_dashboard_async()
                    await self.reset_rig_async()
                    break
                except (
                    TimeoutError,
                    MaxAttemptsReachedError,
                    ServiceCallError,
                    ServiceCallUnsuccessfulError,
                ) as e:
                    self.log(
                        "Caught exception while resetting robot:",
                        severity="WARN",
                    )
                    self.log(f"{type(e).__name__}: {e}", severity="WARN")
                    self.log(
                        f"Traceback: {traceback.format_exc()}",
                        severity="DEBUG",
                    )
                    self.log(
                        "Trying again after 5 seconds...", severity="WARN"
                    )
                    await asyncio.sleep(5)

    ############################################################
    ########## Destroy #########################################
    ############################################################

    def destroy_node(self) -> None:
        self.trajectory_cache.close()
        self.moveit_py.shutdown()
        super().destroy_node()


# Example script using the commander node


async def run_commander_example(
    commander: Commander, config: Mapping[str, Any]
) -> None:
    try:
        commander.log(f"Run Config: \n{yaml.dump(config)}", severity="DEBUG")

        waypoints: dict[str, PoseStamped] = {}
        for waypoint_name, waypoint_config in config["waypoints"][
            "poses"
        ].items():
            waypoint_pose = commander.create_pose_stamped(**waypoint_config)
            waypoints[waypoint_name] = waypoint_pose

        if len(waypoints) < 1:
            raise ValueError(
                "No valid waypoints found in commander parameters!"
            )

        object_ids: list[str] = config["object_ids"]

        commander.init_dashboard()
        # await commander.reset_dashboard_async()
        i = 0
        while True:
            async with commander.context_manager():
                async with asyncio.timeout(config["plan_and_execute_timeout"]):
                    object_id = object_ids[i]
                    end_goal = waypoints[object_id]

                    arm_door_task = commander.arm_door_close_and_wait()
                    smartglass_task = commander.smartglass_occlude()

                    await arm_door_task
                    await smartglass_task

                    await commander.fetch_object_async(object_id, end_goal)

                    await commander.wait_for_hand_fixation_press_async(
                        timeout=2
                    )

                    arm_door_task = commander.arm_door_open_and_wait()
                    smartglass_task = commander.smartglass_reveal()

                    await arm_door_task
                    await smartglass_task

                    await commander.return_object_async()

                    i = (i + 1) % len(object_ids)
    except Exception as e:
        print("Re-raising exception from run():")
        print(f"{type(e).__name__}: {e}")
        traceback.print_exc()
        raise e


def main(args=None):
    rclpy.init(args=args)

    # Get config file from arguments
    non_ros_args = rclpy.utilities.remove_ros_args(args)  # type: ignore

    # Attach debugger if requested
    if non_ros_args[-1] == "--debug":
        debugpy.listen(4567)
        print("Waiting for debugger to attach")
        debugpy.wait_for_client()
        print("Debugger attached")

    # Load config file
    coro_module, coro_name, config = non_ros_args[1:4]
    print(
        f"Running coroutine {coro_name} from module {coro_module} with config {config}"
    )

    coro: Callable[[Commander, Mapping[str, Any]], Coroutine] = getattr(
        importlib.import_module(coro_module), coro_name
    )

    with open(config, "r") as f:
        config = yaml.safe_load(f)

    try:
        commander = Commander()
        executor = SingleThreadedExecutor()
        executor.add_node(commander)

        with ThreadPoolExecutor(max_workers=1) as tpe:
            try:
                tpe.submit(executor.spin)
                asyncio.run(coro(commander, config))
            finally:
                print("Shutting down commander")
                commander.destroy_node()
                print("Shutting down executor")
                executor.shutdown()
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    except SystemExit:
        print("System exit")
    finally:
        print("Shutting down rclpy")
        rclpy.try_shutdown()
