import asyncio
import glob
import os
import time
import traceback
from collections.abc import (
    AsyncGenerator,
    Iterable,
    Mapping,
)
from contextlib import asynccontextmanager
from copy import deepcopy
from typing import Any, Callable, Coroutine, Optional

import numpy as np
import pandas as pd
import rclpy
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
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future as RclpyFuture
from shape_msgs.msg import (
    Plane,
    SolidPrimitive,
)
from std_srvs.srv import Trigger
from tabletop_msgs.srv import (
    GetArmDoor,
    GetFlic,
    GetHandFixation,
    GetReward,
    GetSmartglass,
    SetArmDoor,
    SetReward,
    SetSmartglass,
)
from tabletop_utils.mesh import (
    load_geometry,
    simplify_bounding_primitive,
    simplify_convex_hull,
    simplify_quadratic_decimation,
)
from tabletop_utils.ros import (
    MaxAttemptsReachedError,
    ServiceCallError,
    arrays_from_pose_msg,
    attached_collision_object_msg,
    change_reference_frame_pose_stamped,
    euler_from_quaternion_msg,
    matrix_from_pose_msg,
    mesh_collision_object_msg,
    moveit_error_code_map,
    object_color_msg,
    pose_msg,
    pose_msg_from_matrix,
    pose_stamped_msg,
    quaternion_msg_from_axis_angle,
    quaternion_msg_from_euler,
)
from tf_transformations import identity_matrix
from ur_dashboard_msgs.srv import Load

from tabletop_server.nodes.base import DEFAULT_LOG_SEVERITY, BaseNode


def asyncio_task_decorator(coro_fn: Callable[..., Coroutine]):
    """
    Decorator for methods that should be run in the current asyncio.TaskGroup.

    This decorator is designed for BaseNode methods. It will only work for
    methods whose first argument is `self` and whose class has an
    `asyncio.TaskGroup` attribute named `tg`.
    """

    def wrapper(self, *args, **kwargs):
        if self.tg is None:
            return coro_fn(self, *args, **kwargs)
        else:
            return self.tg.create_task(coro_fn(self, *args, **kwargs))

    return wrapper


class Commander(BaseNode):
    default_params: dict[str, Any] = BaseNode.default_params | {}
    required_params: set[str] = BaseNode.required_params | {
        "max_plan_attempts",
        "max_execution_attempts",
        "dashboard.installation",
        "dashboard.program",
        "dashboard.init_timeout",
        "rig.init_timeout",
        "teensy.spin_period_s",
        "flic.spin_period_s",
        "planning.group_name",
        "planning.default_pipeline",
        "planning.linear_pipeline",
        "planning.eef_link",
        "planning.object_touch_links",
        "planning.idle_pose",
        "planning.pre_fetch_offset",
        "planning.pre_attach_offset",
        "planning.post_attach_offset",
        "planning_scene.floor_coef",
        "planning_scene.divider_coef",
        "planning_scene.static_meshes.path",
        "planning_scene.static_meshes.scale",
        "planning_scene.static_meshes.simplification",
        "planning_scene.static_meshes.color",
        "planning_scene.dynamic_meshes.path",
        "planning_scene.dynamic_meshes.scale",
        "planning_scene.dynamic_meshes.simplification",
        "planning_scene.dynamic_meshes.poses",
    }

    def __init__(self):
        super().__init__(
            "commander",
            automatically_declare_parameters_from_overrides=True,
        )

        # Initialize MoveItPy
        self.moveit_py = MoveItPy("moveit_py", provide_planning_service=True)

        # Initialize MoveItPy components
        # self.planning_component: PlanningComponent = (
        #     self.moveit_py.get_planning_component(
        #         self.get_parameter_wrapper("planning.group_name")
        #     )
        # )
        self.trajectory_execution_manager: TrajectoryExecutionManager = (
            self.moveit_py.get_trajectory_execution_manager()
        )
        self.robot_model: RobotModel = self.moveit_py.get_robot_model()

        self.log(
            f"Robot model: {self.robot_model.get_model_info()}",
            severity="DEBUG",
        )

        self.planning_scene_monitor: PlanningSceneMonitor = (
            self.moveit_py.get_planning_scene_monitor()
        )

        self.setup_planning_scene()

        self.log("Commander initialized")
        # self._change_state("RESET")

    # @property
    def get_planning_component(
        self, group_name: Optional[str] = None
    ) -> PlanningComponent:
        if group_name is None:
            group_name = self.get_parameter_wrapper("planning.group_name")
        return self.moveit_py.get_planning_component(group_name)

    # @property
    # def trajectory_execution_manager(self) -> TrajectoryExecutionManager:
    #     return self.moveit_py.get_trajectory_execution_manager()

    # @property
    # def planning_scene_monitor(self) -> PlanningSceneMonitor:
    #     return self.moveit_py.get_planning_scene_monitor()

    # @property
    # def robot_model(self) -> RobotModel:
    #     return self.moveit_py.get_robot_model()

    ############################################################
    ########## Logging #########################################
    ############################################################

    def log_plan_response(
        self,
        plan_response: MotionPlanResponse,
        severity: str = DEFAULT_LOG_SEVERITY,
    ) -> str:
        """
        Log the result of a plan.
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

    def dashboard_trigger(self, srv_name: str) -> None:
        """
        Call a dashboard client Trigger service.
        """
        self.log(f"Triggering {srv_name} in UR Dashboard", severity="DEBUG")
        self.service_call(
            srv_request=Trigger.Request(), srv_type=Trigger, srv_name=srv_name
        )

    @asyncio_task_decorator
    async def dashboard_trigger_async(self, srv_name: str) -> Trigger.Response:
        """
        Coroutine to call a dashboard client Trigger service asynchronously.
        """
        self.log(
            f"Triggering {srv_name} in UR Dashboard asynchronously",
            severity="DEBUG",
        )
        return await self.service_call_async(
            srv_request=Trigger.Request(), srv_type=Trigger, srv_name=srv_name
        )  # type: ignore

    def dashboard_load(self, srv_name: str, filename: str) -> None:
        """
        Load a program or installation on the robot dashboard by calling a
        dashboard client Load service.
        """
        self.log(
            f"Loading {srv_name}: {filename} in UR Dashboard", severity="DEBUG"
        )
        self.service_call(
            srv_request=Load.Request(filename=filename),
            srv_type=Load,
            srv_name=srv_name,
        )

    @asyncio_task_decorator
    async def dashboard_load_async(
        self,
        srv_name: str,
        filename: str,
    ):
        """
        Coroutine to load a program or installation on the robot dashboard
        by calling a dashboard client Load service asynchronously.
        """
        self.log(
            f"Loading {srv_name}: {filename} in UR Dashboard asynchronously",
            severity="DEBUG",
        )
        return await self.service_call_async(
            srv_request=Load.Request(filename=filename),
            srv_type=Load,
            srv_name=srv_name,
        )

    def reset_dashboard(self):
        """
        Call a sequence of dashboard client services to reset dashboard.
        """
        self.log("Resetting dashboard")
        self.dashboard_trigger("/dashboard_client/close_safety_popup")
        self.dashboard_trigger("/dashboard_client/close_popup")
        self.dashboard_trigger("/dashboard_client/unlock_protective_stop")
        self.dashboard_load(
            "/dashboard_client/load_program",
            self.get_parameter_wrapper("dashboard.program"),
        )
        self.dashboard_trigger("/dashboard_client/brake_release")
        self.dashboard_trigger("/dashboard_client/play")

    @asyncio_task_decorator
    async def reset_dashboard_async(self):
        """
        Coroutine to call a sequence of dashboard client services to reset the
        dashboard asynchronously.
        """
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
        await self.dashboard_trigger_async("/dashboard_client/close_popup")
        await self.dashboard_trigger_async(
            "/dashboard_client/close_safety_popup"
        )
        await self.dashboard_trigger_async("/dashboard_client/play")
        await self.dashboard_trigger_async("/dashboard_client/close_popup")
        await self.dashboard_trigger_async(
            "/dashboard_client/close_safety_popup"
        )

    def init_dashboard(self, timeout_s: Optional[float] = None):
        """
        Initialize the robot dashboard.
        """
        self.log("Initializing dashboard")
        if timeout_s is None:
            timeout_s = self.get_parameter_wrapper("dashboard.init_timeout")

        start_time = time.time()
        while True:
            try:
                self.wait_for_service(Trigger, "/dashboard_client/close_popup")
                self.reset_dashboard()
                break
            except (TimeoutError, ServiceCallError) as e:
                self.log(
                    f"Error initializing dashboard: {type(e).__name__}: {e}",
                    severity="ERROR",
                )
                if time.time() - start_time > timeout_s:  # type: ignore
                    raise TimeoutError("Dashboard initialization timed out")
                time.sleep(1)

    ############################################################
    ########## Teensy interface ################################
    ############################################################

    @asyncio_task_decorator
    async def get_smartglass_async(self) -> GetSmartglass.Response:
        """Get the smartglass state."""
        return await self.service_call_async(
            srv_request=GetSmartglass.Request(),
            srv_type=GetSmartglass,
            srv_name="/teensy/get_smartglass",
        )  # type: ignore

    @asyncio_task_decorator
    async def get_arm_door_async(self) -> GetArmDoor.Response:
        """Get the arm door state."""
        return await self.service_call_async(
            srv_request=GetArmDoor.Request(),
            srv_type=GetArmDoor,
            srv_name="/teensy/get_arm_door",
        )  # type: ignore

    @asyncio_task_decorator
    async def get_reward_async(self) -> GetReward.Response:
        """Get the reward state."""
        return await self.service_call_async(
            srv_request=GetReward.Request(),
            srv_type=GetReward,
            srv_name="/teensy/get_reward",
        )  # type: ignore

    @asyncio_task_decorator
    async def get_hand_fixation_async(self) -> GetHandFixation.Response:
        """Get the hand fixation state."""
        return await self.service_call_async(
            srv_request=GetHandFixation.Request(),
            srv_type=GetHandFixation,
            srv_name="/teensy/get_hand_fixation",
        )  # type: ignore

    @asyncio_task_decorator
    async def get_flic_async(self) -> GetFlic.Response:
        """Get the flic state."""
        return await self.service_call_async(
            srv_request=GetFlic.Request(),
            srv_type=GetFlic,
            srv_name="/flic/get_flic",
        )  # type: ignore

    @asyncio_task_decorator
    async def start_smartglass_reveal_async(self):
        """
        Coroutine to call the smartglass service to reveal the smartglass
        asynchronously.
        """
        self.log("Smartglass Reveal")
        return await self.service_call_async(
            srv_request=SetSmartglass.Request(is_revealed=True),
            srv_type=SetSmartglass,
            srv_name="/teensy/set_smartglass",
        )

    @asyncio_task_decorator
    async def start_smartglass_occlude_async(self):
        """
        Coroutine to call the smartglass service to occlude the smartglass
        asynchronously.
        """
        self.log("Smartglass Occlude")
        return await self.service_call_async(
            srv_request=SetSmartglass.Request(is_revealed=False),
            srv_type=SetSmartglass,
            srv_name="/teensy/set_smartglass",
        )

    @asyncio_task_decorator
    async def start_arm_door_open_async(self):
        """
        Coroutine to call the arm door service to open the arm door
        asynchronously.
        """
        self.log("Arm Door Open")
        return await self.service_call_async(
            srv_request=SetArmDoor.Request(is_open=True),
            srv_type=SetArmDoor,
            srv_name="/teensy/set_arm_door",
        )

    @asyncio_task_decorator
    async def start_arm_door_close_async(self):
        """
        Coroutine to call the arm door service to close the arm door
        asynchronously.
        """
        self.log("Arm Door Close")
        return await self.service_call_async(
            srv_request=SetArmDoor.Request(is_open=False),
            srv_type=SetArmDoor,
            srv_name="/teensy/set_arm_door",
        )

    @asyncio_task_decorator
    async def start_reward_async(self, duration_s: float):
        """
        Coroutine to call the reward service to deliver a reward for a given
        duration.
        """
        self.log(f"Delivering reward for {duration_s} s")
        if duration_s < 0:
            raise ValueError("Duration must be greater than 0!")
        return await self.service_call_async(
            srv_request=SetReward.Request(duration_ms=int(duration_s * 1000)),
            srv_type=SetReward,
            srv_name="/teensy/set_reward",
        )

    @asyncio_task_decorator
    async def wait_for_smartglass_reveal_async(
        self, timeout_s: Optional[float] = None
    ):
        """Wait for smartglass reveal, then return True."""
        smartglass_state = await self.get_smartglass_async()
        if smartglass_state.is_revealed:
            return True
        try:
            async with asyncio.timeout(timeout_s):
                while not smartglass_state.is_revealed:
                    smartglass_state = await self.get_smartglass_async()
                    await asyncio.sleep(
                        self.get_parameter_wrapper("teensy.spin_period_s")
                    )
            return True
        except TimeoutError:
            return False

    @asyncio_task_decorator
    async def wait_for_smartglass_occlude_async(
        self, timeout_s: Optional[float] = None
    ):
        """Wait for smartglass occlusion, then return True."""
        smartglass_state = await self.get_smartglass_async()
        if not smartglass_state.is_revealed:
            return True
        try:
            async with asyncio.timeout(timeout_s):
                while smartglass_state.is_revealed:
                    smartglass_state = await self.get_smartglass_async()
                    await asyncio.sleep(
                        self.get_parameter_wrapper("teensy.spin_period_s")
                    )
            return True
        except TimeoutError:
            return False

    @asyncio_task_decorator
    async def wait_for_arm_door_open_async(
        self, timeout_s: Optional[float] = None
    ):
        """Wait for arm door to open, then return True."""
        arm_door_state = await self.get_arm_door_async()
        if arm_door_state.is_open:
            return True
        try:
            async with asyncio.timeout(timeout_s):
                while not arm_door_state.is_open:
                    arm_door_state = await self.get_arm_door_async()
                    await asyncio.sleep(
                        self.get_parameter_wrapper("teensy.spin_period_s")
                    )
            return True
        except TimeoutError:
            return False

    @asyncio_task_decorator
    async def wait_for_arm_door_close_async(
        self, timeout_s: Optional[float] = None
    ):
        """Wait for arm door to close, then return True."""
        arm_door_state = await self.get_arm_door_async()
        if not arm_door_state.is_open:
            return True
        try:
            async with asyncio.timeout(timeout_s):
                while arm_door_state.is_open:
                    arm_door_state = await self.get_arm_door_async()
                    await asyncio.sleep(
                        self.get_parameter_wrapper("teensy.spin_period_s")
                    )
            return True
        except TimeoutError:
            return False

    @asyncio_task_decorator
    async def wait_for_reward_async(self, timeout_s: Optional[float] = None):
        """Wait for reward to start, then return True."""
        reward_state = await self.get_reward_async()
        if reward_state.is_active:
            return True
        try:
            async with asyncio.timeout(timeout_s):
                while not reward_state.is_active:
                    reward_state = await self.get_reward_async()
                    await asyncio.sleep(
                        self.get_parameter_wrapper("teensy.spin_period_s")
                    )
            return True
        except TimeoutError:
            return False

    @asyncio_task_decorator
    async def wait_for_hand_fixation_press_async(
        self, timeout_sec: Optional[float] = None
    ):
        """Wait for hand fixation state to turn on, then return True."""
        initial_fixation = await self.get_hand_fixation_async()
        if initial_fixation.is_pressed:
            return True
        fixation = initial_fixation
        try:
            async with asyncio.timeout(timeout_sec):
                while (
                    fixation.last_time_pressed_ms
                    == initial_fixation.last_time_pressed_ms
                ):
                    fixation = await self.get_hand_fixation_async()
                    await asyncio.sleep(
                        self.get_parameter_wrapper("teensy.spin_period_s")
                    )
            return True
        except TimeoutError:
            return False

    @asyncio_task_decorator
    async def wait_for_hand_fixation_release_async(
        self, timeout_sec: Optional[float] = None
    ):
        """Wait for hand fixation state to turn off, then return True."""
        initial_fixation = await self.get_hand_fixation_async()
        if not initial_fixation.is_pressed:
            return True
        fixation = initial_fixation
        try:
            async with asyncio.timeout(timeout_sec):
                while (
                    fixation.last_time_released_ms
                    == initial_fixation.last_time_released_ms
                ):
                    fixation = await self.get_hand_fixation_async()
                    await asyncio.sleep(
                        self.get_parameter_wrapper("teensy.spin_period_s")
                    )
            return True
        except TimeoutError:
            return False

    # TODO: Potential race condition (between monkey and get_flic_async lol)
    @asyncio_task_decorator
    async def wait_for_flic_press_async(
        self, timeout_s: Optional[float] = None
    ):
        """Wait for flic button press, then return True."""
        initial_flic = await self.get_flic_async()
        flic = initial_flic
        try:
            async with asyncio.timeout(timeout_s):
                while (
                    flic.last_time_pressed_ms
                    == initial_flic.last_time_pressed_ms
                ):
                    flic = await self.get_flic_async()
                    await asyncio.sleep(
                        self.get_parameter_wrapper("flic.spin_period_s")
                    )
            return True
        except TimeoutError:
            return False

    @asyncio_task_decorator
    async def smartglass_reveal_and_wait(
        self, timeout_s: Optional[float] = None
    ):
        """Reveal smartglass and wait for it to be revealed."""
        await self.start_smartglass_reveal_async()
        return await self.wait_for_smartglass_reveal_async(timeout_s)

    @asyncio_task_decorator
    async def smartglass_occlude_and_wait(
        self, timeout_s: Optional[float] = None
    ):
        """Occlude smartglass and wait for it to be occluded."""
        await self.start_smartglass_occlude_async()
        return await self.wait_for_smartglass_occlude_async(timeout_s)

    @asyncio_task_decorator
    async def arm_door_open_and_wait(self, timeout_s: Optional[float] = None):
        """Open arm door and wait for it to be open."""
        await self.start_arm_door_open_async()
        return await self.wait_for_arm_door_open_async(timeout_s)

    @asyncio_task_decorator
    async def arm_door_close_and_wait(self, timeout_s: Optional[float] = None):
        """Close arm door and wait for it to be closed."""
        await self.start_arm_door_close_async()
        return await self.wait_for_arm_door_close_async(timeout_s)

    @asyncio_task_decorator
    async def reward_and_wait(
        self, duration_s: float, timeout_s: Optional[float] = None
    ):
        """Start reward and wait for it to be active."""
        await self.start_reward_async(duration_s)
        # Default timeout is duration plus spin period if not specified
        if timeout_s is None:
            timeout_s = duration_s + self.get_parameter_wrapper(
                "teensy.spin_period_s"
            )
        if await self.wait_for_reward_async(timeout_s):
            return True
        else:
            raise RuntimeError("Reward took longer than expected timeout")

    ############################################################
    ########## Poses ###########################################
    ############################################################

    @property
    def eef_link(self) -> str:
        return self.get_parameter_wrapper("planning.eef_link")

    @property
    def touch_links(self) -> list[str]:
        return self.get_parameter_wrapper("planning.object_touch_links")

    @property
    def planning_frame(self) -> str:
        """
        Get the planning frame from the planning scene.
        """
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            return scene.planning_frame

    @property
    def planning_group_name(self) -> str:
        """
        Get the planning group name from the parameter server.
        """
        return self.get_parameter_wrapper("planning.group_name")

    def create_pose(self, **kwargs) -> Pose:
        """
        Create a Pose message from keyword arguments.
        """
        return pose_msg(**kwargs)

    def create_pose_stamped(self, **kwargs) -> PoseStamped:
        """
        Create a PoseStamped message from keyword arguments.

        If the `frame_id` is not specified, the planning frame will be used.

        Args:
            **kwargs: Keyword arguments to pass to `pose_stamped_msg()`.

        Returns:
            PoseStamped: The PoseStamped message.
        """
        pose_stamped = pose_stamped_msg(**kwargs)
        if not pose_stamped.header.frame_id:
            pose_stamped.header.frame_id = self.planning_frame

        return pose_stamped

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

    def get_frame_pose_stamped(self, frame_id: str, **kwargs) -> PoseStamped:
        """
        Get the frame pose for a given frame id from the planning scene.
        """
        return self.create_pose_stamped(
            pose=pose_msg_from_matrix(self.get_frame_transform(frame_id)),
            **kwargs,
        )

    def change_reference_frame(
        self, pose_stamped: PoseStamped, new_frame_id: str
    ) -> PoseStamped:
        """
        Change the reference frame of a pose stamped message.
        """
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

    def get_idle_pose_stamped(self) -> PoseStamped:
        """
        Get the idle pose from the planning scene.
        """
        return self.create_pose_stamped(
            **self.get_parameter_wrapper("planning.idle_pose")
        )

    def eef_pose_stamped(self, frame_id: Optional[str] = None) -> PoseStamped:
        """
        Get the current end-effector pose.
        """
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
        """
        Get a null pose in the planning frame.
        """
        return self.create_pose_stamped(
            frame_id=frame_id if frame_id is not None else self.planning_frame,
        )

    def object_init_pose_stamped(self, object_id: str) -> PoseStamped:
        """
        Get the initial pose of an object from the parameters.
        """
        return self.create_pose_stamped(
            **self.get_parameter_wrapper(
                f"planning_scene.dynamic_meshes.poses.{object_id}"
            )
        )

    def pre_fetch_pose_stamped(
        self, object_id: str, subframe_name: str
    ) -> PoseStamped:
        return self.create_pose_stamped(
            frame_id=object_id + f"/{subframe_name}",
            position=self.get_parameter_wrapper("planning.pre_fetch_offset"),
        )

    def pre_attach_pose_stamped(
        self, object_id: str, subframe_name: str
    ) -> PoseStamped:
        return self.create_pose_stamped(
            frame_id=object_id + f"/{subframe_name}",
            position=self.get_parameter_wrapper("planning.pre_attach_offset"),
        )

    def attach_pose_stamped(
        self, object_id: str, subframe_name: str
    ) -> PoseStamped:
        attach_pose = PoseStamped()
        attach_pose.header.frame_id = object_id + f"/{subframe_name}"
        return attach_pose

    def post_attach_pose_stamped(self, object_id: str) -> PoseStamped:
        post_attach_pose = self.object_init_pose_stamped(object_id)
        post_attach_offset = self.get_parameter_wrapper(
            "planning.post_attach_offset"
        )
        post_attach_pose.pose.position.x += post_attach_offset[0]
        post_attach_pose.pose.position.y += post_attach_offset[1]
        post_attach_pose.pose.position.z += post_attach_offset[2]

        return post_attach_pose

    def pre_detach_pose_stamped(self, object_id: str) -> PoseStamped:
        return self.post_attach_pose_stamped(object_id)

    def pre_detach_pose_stamped_wrist_2(self, object_id: str) -> PoseStamped:
        pose_stamped = self.pre_detach_pose_stamped(object_id)
        pose_stamped.pose.position.z -= 0.13

        roll, pitch, yaw = euler_from_quaternion_msg(
            pose_stamped.pose.orientation
        )
        roll -= 1.57
        yaw -= 1.57
        pose_stamped.pose.orientation = quaternion_msg_from_euler(
            roll, pitch, yaw
        )

        return pose_stamped

    def detach_pose_stamped(self, object_id: str) -> PoseStamped:
        return self.object_init_pose_stamped(object_id)

    def post_detach_pose_stamped(
        self, object_id: str, subframe_name: str
    ) -> PoseStamped:
        return self.pre_attach_pose_stamped(object_id, subframe_name)

    def post_return_pose_stamped(
        self, object_id: str, subframe_name: str
    ) -> PoseStamped:
        return self.pre_fetch_pose_stamped(object_id, subframe_name)

    ############################################################
    ########## Planning scene #################################
    ############################################################

    @property
    def current_state(self) -> RobotState:
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            return deepcopy(scene.current_state)

    def is_state_colliding(self, group_name: Optional[str] = None) -> bool:
        """
        Check if the current state of the planning scene is colliding.
        """
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

    @property
    def collision_objects(self) -> list[CollisionObject]:
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            planning_scene_msg: PlanningSceneMsg = scene.planning_scene_message
            return planning_scene_msg.world.collision_objects  # type: ignore

    @property
    def collision_object_ids(self) -> list[str]:
        return [
            collision_object.id for collision_object in self.collision_objects
        ]

    @property
    def attached_collision_objects(self) -> list[AttachedCollisionObject]:
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            planning_scene_msg: PlanningSceneMsg = scene.planning_scene_message
            return planning_scene_msg.robot_state.attached_collision_objects  # type: ignore

    @property
    def attached_collision_object_ids(self) -> list[str]:
        return [
            attached_collision_object.object.id
            for attached_collision_object in self.attached_collision_objects
        ]

    @property
    def collision_matrix_df(self) -> pd.DataFrame:
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

    def add_floor_collision_object(
        self,
        *,
        header_frame_id: Optional[str] = None,
    ):
        """
        Add the floor collision object to the planning scene.
        """

        self.log("Processing floor collision object")

        if header_frame_id is None:
            header_frame_id = self.planning_frame

        collision_object = CollisionObject()
        collision_object.header.frame_id = header_frame_id
        collision_object.id = "floor"

        plane = Plane()
        plane.coef = self.get_parameter_wrapper("planning_scene.floor_coef")

        collision_object.planes.append(plane)  # type: ignore
        collision_object.operation = CollisionObject.ADD

        self.planning_scene_monitor.process_collision_object(collision_object)

    def add_divider_collision_object(
        self,
        *,
        header_frame_id: Optional[str] = None,
    ):
        """
        Add the divider collision object to the planning scene.
        """
        self.log("Processing divider collision object")

        if header_frame_id is None:
            header_frame_id = self.planning_frame

        collision_object = CollisionObject()
        collision_object.header.frame_id = header_frame_id
        collision_object.id = "divider"

        plane = Plane()
        plane.coef = self.get_parameter_wrapper("planning_scene.divider_coef")
        collision_object.planes.append(plane)  # type: ignore

        collision_object.operation = CollisionObject.ADD

        self.planning_scene_monitor.process_collision_object(collision_object)

    def add_mesh_collision_object(
        self,
        *,
        path: str,
        object_id: str,
        pose_stamped: PoseStamped,
        scale: float = 1.0,
        simplification: Optional[str] = None,
        add_default_subframe: bool = False,
        correction_tf: Optional[np.ndarray] = None,
        color: Optional[str | Iterable[float] | Mapping[str, float]] = None,
    ):
        """
        Add a mesh collision object at a given path to the planning scene.

        Args:
            path (str): The path to the mesh file.
            object_id (str): The id for the collision object.
            scale (float, optional): The scale of the mesh.
            pose (dict, optional): The pose of the collision object.
            simplification (str, optional): The simplification method to use.
        """
        # Get base frame id from argument or planning frame
        if not pose_stamped.header.frame_id:
            pose_stamped.header.frame_id = self.planning_frame

        # Load geometry
        geometry = load_geometry(path, scale=scale)

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
        if correction_tf is not None:
            geometry = geometry.apply_transform(correction_tf)

        # Create collision object
        collision_object = mesh_collision_object_msg(
            geometry=geometry,
            object_id=object_id,
            pose_stamped=pose_stamped,
            subframe_names=["default"] if add_default_subframe else [],
            subframe_poses=[Pose()] if add_default_subframe else [],
            operation="add",
        )

        if color is not None:
            color_msg = object_color_msg(object_id, color)
        else:
            color_msg = None

        # Add collision object to planning scene
        self.planning_scene_monitor.process_collision_object(
            collision_object, color_msg
        )

    def remove_collision_object(self, object_id: str):
        collision_object = CollisionObject(
            id=object_id, operation=CollisionObject.REMOVE
        )
        self.planning_scene_monitor.process_collision_object(collision_object)

    def remove_all_collision_objects(self):
        self.log("Removing all collision objects", severity="DEBUG")
        with self.planning_scene_monitor.read_write() as scene:
            scene: PlanningScene = scene
            scene.remove_all_collision_objects()
            scene.current_state.update()

    def attach_collision_object(
        self,
        *,
        object_id: str,
        link_name: str,
        touch_links: Optional[list[str]] = None,
    ):
        """
        Attach an object to the robot.
        """
        self.log(f"Attaching object {object_id}")
        attached_collision_object = attached_collision_object_msg(
            object_id=object_id,
            link_name=link_name,
            operation="add",
            touch_links=touch_links,
        )
        self.planning_scene_monitor.process_attached_collision_object(
            attached_collision_object
        )

    def detach_collision_object(
        self,
        object_id: str,
        link_name: Optional[str] = None,
    ):
        self.log(f"Detaching object {object_id}")
        attached_collision_object = attached_collision_object_msg(
            object_id=object_id,
            operation="remove",
            link_name=link_name,
        )
        self.planning_scene_monitor.process_attached_collision_object(
            attached_collision_object
        )

    def log_collision_objects(self, severity: str = DEFAULT_LOG_SEVERITY):
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

    def allow_collision(self, id_1: str, id_2: str):
        self.log(f"Allowing collision between {id_1} and {id_2}")
        with self.planning_scene_monitor.read_write() as scene:
            scene: PlanningScene = scene
            scene.allowed_collision_matrix.set_entry(id_1, id_2, True)
            scene.current_state.update()

    def disallow_collision(self, id_1: str, id_2: str):
        self.log(f"Disallowing collision between {id_1} and {id_2}")
        with self.planning_scene_monitor.read_write() as scene:
            scene: PlanningScene = scene
            scene.allowed_collision_matrix.set_entry(id_1, id_2, False)
            scene.current_state.update()

    def log_collision_matrix(self, severity: str = DEFAULT_LOG_SEVERITY):
        self.log(
            f"Allowed collision matrix: \n{self.collision_matrix_df.to_string()}",
            severity=severity,
        )

    def log_planning_scene(self, severity: str = DEFAULT_LOG_SEVERITY):
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

    def setup_planning_scene(self):
        """
        Setup the planning scene by adding a floor collision object and
        collision objects from the planning scene configuration.
        """
        self.log("Setting up planning scene")

        # Add floor collision object
        self.add_floor_collision_object()

        # Add divider collision object
        self.add_divider_collision_object()

        # Add collision objects
        self.static_object_ids = []
        self.dynamic_object_ids = []
        for mesh_type in ["static_meshes", "dynamic_meshes"]:
            # Get parameters
            prefix = f"planning_scene.{mesh_type}"
            meshes_config = self.get_parameter_wrapper(prefix)
            meshes_path: str = meshes_config["path"]
            if not os.path.exists(meshes_path):
                raise FileNotFoundError(
                    f"Mesh path {meshes_path} does not exist"
                )
            meshes_scale: float = meshes_config["scale"]
            meshes_simplification: str = meshes_config["simplification"]
            meshes_color = meshes_config["color"]
            try:
                meshes_correction = meshes_config["correction"]
                meshes_correction = self.create_pose(**meshes_correction)
                meshes_correction_tf = matrix_from_pose_msg(meshes_correction)
            except KeyError:
                meshes_correction_tf = None

            # Process individual mesh files from directory
            if os.path.isdir(meshes_path):
                paths = glob.glob(
                    os.path.join(meshes_path, "*.stl")
                ) + glob.glob(os.path.join(meshes_path, "*.dae"))
            else:
                paths = [meshes_path]

            for path in paths:
                self.log(f"Processing mesh collision object from path: {path}")
                object_id = os.path.splitext(os.path.basename(path))[0]
                if mesh_type == "static_meshes":
                    self.static_object_ids.append(object_id)
                else:
                    if object_id in self.collision_object_ids:
                        self.log(
                            f"Skipping dynamic mesh {object_id} because it already exists in the planning scene",
                            severity="INFO",
                        )
                        continue
                    self.dynamic_object_ids.append(object_id)
                try:
                    pose_config = meshes_config["poses"][object_id]
                    pose_stamped = self.create_pose_stamped(**pose_config)
                except KeyError:
                    pose_stamped = self.create_pose_stamped(
                        frame_id=self.planning_frame
                    )

                self.add_mesh_collision_object(
                    path=path,
                    object_id=object_id,
                    pose_stamped=pose_stamped,
                    scale=meshes_scale,
                    simplification=meshes_simplification,
                    add_default_subframe=True,
                    correction_tf=meshes_correction_tf,
                    color=meshes_color,
                )

        # Update planning scene
        for static_object_id in self.static_object_ids:
            self.allow_collision("base_link_inertia", static_object_id)
            self.allow_collision("sphere", static_object_id)

        for dynamic_object_id in self.dynamic_object_ids:
            self.allow_collision("sphere", dynamic_object_id)

        # Log planning scene
        self.log_planning_scene(severity="DEBUG")
        self.log_collision_objects(severity="DEBUG")
        self.log_collision_matrix(severity="DEBUG")

    def get_planning_scene_copy(self) -> PlanningScene:
        with self.planning_scene_monitor.read_only() as scene:
            return deepcopy(scene)

    ############################################################
    ########## Planning and execution ##########################
    ############################################################

    def cartesian_path_constraints(
        self,
        goal_pose_stamped: PoseStamped,
        start_pose_stamped: Optional[PoseStamped] = None,
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
        line_pose = self.create_pose(
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

    def plan_once(
        self,
        goal: PoseStamped | str | RobotState,
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

        # Set goal state from pose or configuration name
        original_pose_link = pose_link
        if pose_link is None:
            pose_link = self.eef_link

        if goal == "idle":
            goal = self.get_idle_pose_stamped()

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

                try:
                    linear_path_constraints = self.get_parameter_wrapper(
                        "planning.default_linear_path_constraints"
                    )
                    if path_constraints is None:
                        path_constraints = self.cartesian_path_constraints(
                            goal_pose_stamped=goal,
                            **linear_path_constraints,
                        )
                except ParameterNotDeclaredException:
                    pass

                if path_constraints is not None:
                    path_constraints = self.cartesian_path_constraints(
                        goal_pose_stamped=goal
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

        # Set start state to current state
        planning_component.set_start_state_to_current_state()

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
        self, *args, max_attempts: Optional[int] = None, **kwargs
    ) -> MotionPlanResponse:
        """
        Plan a trajectory to the given waypoint, retrying up to max_attempts
        times until successful.

        Args:
            max_attempts (int, optional): The maximum number of planning attempts.
            *args: Additional positional arguments to pass to `plan_once()`.
            **kwargs: Additional keyword arguments to pass to `plan_once()`.
        Returns:
            MotionPlanResponse: The planned trajectory.
        """

        if max_attempts is None:
            max_attempts = self.get_parameter_wrapper("max_plan_attempts")

        failure_msgs = []
        for i in range(max_attempts):  # type: ignore
            try:
                plan_response = self.plan_once(*args, **kwargs)
                if plan_response.error_code.val == MoveItErrorCodes.SUCCESS:
                    self.log(
                        f"Planning attempt {i + 1}/{max_attempts} succeeded"
                    )
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
            except Exception as e:
                error_msg = f"Planning attempt {i + 1}/{max_attempts} raised exception {type(e).__name__}: {e}"
                failure_msgs.append(error_msg)
                self.log(
                    error_msg,
                    severity="WARN",
                )
                self.log(
                    traceback.format_exc(),
                    severity="DEBUG",
                )
        else:
            error_msg = f"Max planning attempts ({max_attempts}) reached!: {failure_msgs}"
            self.log(error_msg, severity="ERROR")
            raise MaxAttemptsReachedError(error_msg)

        return plan_response

    @asyncio_task_decorator
    async def plan_async(self, *args, **kwargs) -> MotionPlanResponse:
        """
        Asynchronous coroutine wrapper for `plan()` method.

        Creates an rclpy task to compute the trajectory in a separate thread.

        See Also:
            `plan()`: For parameter details and synchronous implementation.
        """
        return await self.create_rclpy_task(
            self.plan,
            *args,
            **kwargs,
        )

    async def plan_generator_async(
        self, *args, goals: list[PoseStamped | str], **kwargs
    ) -> AsyncGenerator[MotionPlanResponse, None]:
        """
        Asynchronous coroutine generator that plans a trajectory for each goal
        in the list.

        Args:
            goals (list[PoseStamped | str]): The list of goals to plan
                trajectories for.
            *args: Additional positional arguments to pass to `plan()`.
            **kwargs: Additional keyword arguments to pass to `plan()`.

        Returns:
            AsyncGenerator[MotionPlanResponse, None]: An asynchronous generator
                that yields the planned trajectory for each goal.
        """
        while goals:
            goal = goals.pop(0)
            new_goals = yield await self.plan_async(
                goal=goal,
                *args,
                **kwargs,
            )
            if new_goals:
                goals.extend(new_goals)

    def execute_once(
        self, robot_trajectory: RobotTrajectory
    ) -> ExecutionStatus:
        """
        Execute the given robot trajectory.

        Args:
            robot_trajectory (RobotTrajectory): The robot trajectory to execute.

        Returns:
            ExecutionStatus: The status of the execution.
        """
        self.trajectory_execution_manager.push(
            robot_trajectory.get_robot_trajectory_msg()
        )
        return self.trajectory_execution_manager.execute_and_wait()

    @asyncio_task_decorator
    async def execute_once_async(
        self, robot_trajectory: RobotTrajectory
    ) -> ExecutionStatus:
        """
        Coroutine to execute the given robot trajectory asynchronously.

        Wraps the trajectory_execution_manager.execute() method in an rclpy
        future to support awaiting the execution.

        Args:
            robot_trajectory (RobotTrajectory): The robot trajectory to execute.

        Returns:
            ExecutionStatus: The status of the execution.
        """
        future = RclpyFuture()

        def done_callback():
            future.set_result(
                self.trajectory_execution_manager.get_last_execution_status()
            )

        self.trajectory_execution_manager.push(robot_trajectory)
        self.trajectory_execution_manager.execute(done_callback)

        return await future  # type: ignore

    def execute(
        self, *args, max_attempts: Optional[int] = None, **kwargs
    ) -> None:
        """
        Execute the given robot trajectory, retrying up to max_attempts times
        until successful.

        Args:
            max_attempts (int, optional): The maximum number of execution attempts.
            *args: Additional positional arguments to pass to `execute_once()`.
            **kwargs: Additional keyword arguments to pass to `execute_once()`.
        Returns:
            ExecutionStatus: The status of the execution.
        """
        if max_attempts is None:
            max_attempts = self.get_parameter_wrapper("max_execution_attempts")

        failure_msgs = []
        for i in range(max_attempts):  # type: ignore
            try:
                execution_status = self.execute_once(*args, **kwargs)
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

    @asyncio_task_decorator
    async def execute_async(
        self, *args, max_attempts: Optional[int] = None, **kwargs
    ) -> None:
        """
        Execute the given robot trajectory, retrying up to max_attempts times
        until successful.

        Args:
            robot_trajectory (RobotTrajectory): The robot trajectory to execute.
            max_attempts (int, optional): The maximum number of execution attempts.
        Returns:
            ExecutionStatus: The status of the execution.
        """
        if max_attempts is None:
            max_attempts = self.get_parameter_wrapper("max_execution_attempts")

        failure_msgs = []
        for i in range(max_attempts):  # type: ignore
            try:
                execution_status = await self.execute_once_async(
                    *args, **kwargs
                )
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

    def plan_and_execute(self, *args, **kwargs) -> None:
        """
        Plan and execute a trajectory.

        Performs max_plan_attempts planning attempts and max_execution_attempts
        execution attempts. If the method returns successfully, the trajectory
        has been executed (no status is returned).

        Args:
            *args: Additional positional arguments to pass to `plan()`.
            **kwargs: Additional keyword arguments to pass to `plan()`.

        Returns:
            None: This method does not return anything but may raise exceptions

        Raises:
            MaxAttemptsReachedError: If maximum planning attempts (param:
                max_plan_attempts) or execution attempts (param:
                max_execution_attempts) are reached
        """
        # Plan the trajectory
        plan_response = self.plan(*args, **kwargs)
        # Execute the plan
        self.execute(plan_response.trajectory)

    @asyncio_task_decorator
    async def plan_and_execute_async(self, *args, **kwargs) -> None:
        """
        Asynchronous coroutine wrapper for `plan_and_execute()` method.

        Creates an rclpy task to compute the plan and execute the planned
        trajectory in a separate thread.

        See Also:
            `plan_and_execute()`: For parameter details and synchronous
                implementation.
        """
        await self.create_rclpy_task(self.plan_and_execute, *args, **kwargs)

    ############################################################
    ########## Fetch and return ################################
    ############################################################

    def fetch_object(
        self,
        object_id: str,
        end_goal: PoseStamped | str,
        subframe_name: str = "default",
    ):
        """Synchronous version of fetch_object_async.

        Fetches an object and moves it to the specified end goal.

        Args:
            object_id: The ID of the object to fetch
            end_goal: The pose to move the object to after fetching
            subframe_name: The subframe name of the object (default: "default")
        """
        self.log(f"Fetching object {object_id} from subframe {subframe_name}")

        # Pre-fetch pose
        self.log(
            f"Moving to pre-fetch pose {self.pre_fetch_pose_stamped(object_id, subframe_name)}"
        )
        self.plan_and_execute(
            goal=self.pre_fetch_pose_stamped(object_id, subframe_name)
        )

        # Allow collision between touch links and object
        for touch_link in self.touch_links:
            self.allow_collision(touch_link, object_id)

        self.log_collision_matrix(severity="DEBUG")

        # Pre-attach pose
        self.log(
            f"Moving to pre-attach pose {self.pre_attach_pose_stamped(object_id, subframe_name)}"
        )
        self.plan_and_execute(
            goal=self.pre_attach_pose_stamped(object_id, subframe_name),
            planning_pipeline="linear",
        )

        # Attach pose (no offset with respect to object frame)
        self.log(
            f"Moving to attach pose {self.attach_pose_stamped(object_id, subframe_name)}"
        )
        self.plan_and_execute(
            goal=self.attach_pose_stamped(object_id, subframe_name),
            planning_pipeline="linear",
        )

        # Attach object
        self.attach_collision_object(
            object_id=object_id,
            link_name=self.eef_link,
            touch_links=self.touch_links,
        )

        # Allow collision between object and static objects
        # Needed to prevent errors when removing object from its tool pocket
        for static_object_id in self.static_object_ids:
            self.allow_collision(object_id, static_object_id)

        self.log_collision_matrix(severity="DEBUG")

        # Post-attach pose
        self.log(
            f"Moving to post-attach pose {self.post_attach_pose_stamped(object_id)}"
        )
        self.plan_and_execute(
            goal=self.post_attach_pose_stamped(object_id),
            planning_pipeline="linear",
        )

        self.last_post_attach_state = self.current_state

        # Disallow collision between object and static objects
        for static_object_id in self.static_object_ids:
            self.disallow_collision(object_id, static_object_id)

        # Move to target pose
        self.log(f"Moving to end goal {end_goal}")
        self.plan_and_execute(goal=end_goal)

    @asyncio_task_decorator
    async def fetch_object_async(
        self,
        object_id: str,
        end_goal: PoseStamped | str,
        subframe_name: str = "default",
    ) -> None:
        """
        Asynchronous coroutine wrapper for `fetch_object()` method.

        See Also:
            `fetch_object()`: For parameter details and synchronous
                implementation.
        """
        self.log(f"Fetching object {object_id} from subframe {subframe_name}")

        # Pre-fetch pose
        pre_fetch_pose_stamped = self.pre_fetch_pose_stamped(
            object_id, subframe_name
        )
        self.log(f"Moving to pre-fetch pose {pre_fetch_pose_stamped}")
        await self.plan_and_execute_async(goal=pre_fetch_pose_stamped)

        # Allow collision between touch links and object
        for touch_link in self.touch_links:
            self.allow_collision(touch_link, object_id)

        self.log_collision_matrix(severity="DEBUG")

        # Pre-attach pose
        pre_attach_pose_stamped = self.pre_attach_pose_stamped(
            object_id, subframe_name
        )
        self.log(f"Moving to pre-attach pose {pre_attach_pose_stamped}")
        await self.plan_and_execute_async(
            goal=pre_attach_pose_stamped,
            planning_pipeline="linear",
        )

        # Attach pose (no offset with respect to object frame)
        attach_pose_stamped = self.attach_pose_stamped(
            object_id, subframe_name
        )
        self.log(f"Moving to attach pose {attach_pose_stamped}")
        await self.plan_and_execute_async(
            goal=attach_pose_stamped,
            planning_pipeline="linear",
        )

        # Attach object
        self.attach_collision_object(
            object_id=object_id,
            link_name=self.eef_link,
            touch_links=self.touch_links,
        )

        # Allow collision between object and static objects
        # Needed to prevent errors when removing object from its tool pocket
        for static_object_id in self.static_object_ids:
            self.allow_collision(object_id, static_object_id)

        self.log_collision_matrix(severity="DEBUG")

        # Post-attach pose
        post_attach_pose_stamped = self.post_attach_pose_stamped(object_id)
        self.log(f"Moving to post-attach pose {post_attach_pose_stamped}")
        await self.plan_and_execute_async(
            goal=post_attach_pose_stamped,
            planning_pipeline="linear",
        )

        self.last_post_attach_state = self.current_state

        # Disallow collision between object and static objects
        for static_object_id in self.static_object_ids:
            self.disallow_collision(object_id, static_object_id)

        # Move to target pose
        self.log(f"Moving to end goal {end_goal}")
        await self.plan_and_execute_async(goal=end_goal)

    def return_object(
        self,
        subframe_name: str = "default",
        end_goal: Optional[PoseStamped | str] = None,
    ) -> None:
        """Synchronous version of return_object_async.

        Returns an object to its original position.

        Args:
            subframe_name: The subframe name of the object (default: "default")
            end_goal: Optional pose to move to after returning the object
        """
        self.log("Returning object")

        # Get object ID from planning scene and check that there is exactly one
        # attached collision object
        attached_collision_object_ids = self.attached_collision_object_ids
        if len(attached_collision_object_ids) != 1:
            raise RuntimeError(
                f"Expected exactly one attached collision object, "
                f"but got {len(attached_collision_object_ids)}"
            )

        object_id = attached_collision_object_ids[0]

        # Move to saved post-attach state
        self.log(
            f"Moving to saved post-attach state {self.last_post_attach_state}"
        )
        self.plan_and_execute(
            goal=self.last_post_attach_state,
        )

        # Allow collision between object and static objects
        # Needed to prevent errors when inserting object into its tool pocket
        for static_object_id in self.static_object_ids:
            self.allow_collision(object_id, static_object_id)

        # Move to the detach pose
        self.log(
            f"Moving to detach pose {self.detach_pose_stamped(object_id)}"
        )
        self.plan_and_execute(
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
        self.plan_and_execute(
            goal=self.post_detach_pose_stamped(object_id, subframe_name),
            planning_pipeline="linear",
        )

        # Move to the post-return (pre-fetch) pose
        self.log(
            f"Moving to post-return pose {self.post_return_pose_stamped(object_id, subframe_name)}"
        )
        self.plan_and_execute(
            goal=self.post_return_pose_stamped(object_id, subframe_name),
            planning_pipeline="linear",
        )

        # Disallow collision between touch links and object
        for touch_link in self.touch_links:
            self.disallow_collision(touch_link, object_id)

        # Move to end pose if specified
        if end_goal is not None:
            self.log(f"Moving to end goal {end_goal}")
            self.plan_and_execute(goal=end_goal)

    @asyncio_task_decorator
    async def return_object_async(
        self,
        subframe_name: str = "default",
        end_goal: Optional[PoseStamped | str] = None,
    ) -> None:
        """Return an object to its original position.

        This method is the reverse of fetch_object_async.

        Args:
            subframe_name: The subframe name of the object (default: "default")
            end_goal: Optional pose to move to after returning the object
        """
        self.log("Returning object")

        # Get object ID from planning scene and check that there is exactly one
        # attached collision object
        attached_collision_object_ids = self.attached_collision_object_ids
        if len(attached_collision_object_ids) != 1:
            raise RuntimeError(
                f"Expected exactly one attached collision object, "
                f"but got {len(attached_collision_object_ids)}"
            )

        object_id = attached_collision_object_ids[0]

        # Move to the pre-detach pose for wrist_2
        # This is needed to solve inverse kinematics issues with pilz
        # pre_detach_pose_stamped_wrist2 = self.pre_detach_pose_stamped_wrist_2(
        #     object_id
        # )
        # self.log(
        #     f"Moving to pre-detach pose for wrist_2 {pre_detach_pose_stamped_wrist2}"
        # )
        # await self.plan_and_execute_async(
        #     goal=pre_detach_pose_stamped_wrist2,
        #     group_name="ur_wrist_2",
        #     pose_link="wrist_2_link",
        # )

        # # Move to the pre-detach pose (for eef)
        # self.log(
        #     f"Moving to pre-detach pose {self.pre_detach_pose_stamped(object_id)}"
        # )
        # await self.plan_and_execute_async(
        #     goal=self.pre_detach_pose_stamped(object_id),
        #     planning_pipeline="linear",
        # )

        # Move to saved post-attach state
        self.log(
            f"Moving to saved post-attach state {self.last_post_attach_state}"
        )
        await self.plan_and_execute_async(
            goal=self.last_post_attach_state,
        )

        # Allow collision between object and static objects
        # Needed to prevent errors when inserting object into its tool pocket
        for static_object_id in self.static_object_ids:
            self.allow_collision(object_id, static_object_id)

        # Move to the detach pose
        self.log(
            f"Moving to detach pose {self.detach_pose_stamped(object_id)}"
        )
        await self.plan_and_execute_async(
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
        await self.plan_and_execute_async(
            goal=self.post_detach_pose_stamped(object_id, subframe_name),
            planning_pipeline="linear",
        )

        # Move to the post-return (pre-fetch) pose
        self.log(
            f"Moving to post-return pose {self.post_return_pose_stamped(object_id, subframe_name)}"
        )
        await self.plan_and_execute_async(
            goal=self.post_return_pose_stamped(object_id, subframe_name),
            planning_pipeline="linear",
        )

        # Disallow collision between touch links and object
        for touch_link in self.touch_links:
            self.disallow_collision(touch_link, object_id)

        # Move to end pose if specified
        if end_goal is not None:
            self.log(f"Moving to end goal {end_goal}")
            await self.plan_and_execute_async(goal=end_goal)

    ############################################################
    ########## Reset rig #######################################
    ############################################################

    def move_out_of_collision(
        self, goal: PoseStamped | str, pose_link: Optional[str] = None
    ):
        """Move the robot out of collision with the scene."""
        self.log("Moving out of collision")
        self.remove_all_collision_objects()
        self.plan_and_execute(goal=goal, pose_link=pose_link)
        self.setup_planning_scene()

    @asyncio_task_decorator
    async def move_out_of_collision_async(
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
        self.remove_all_collision_objects()
        await self.plan_and_execute_async(goal=goal, pose_link=pose_link)
        self.setup_planning_scene()

    def reset_rig(self, end_goal: Optional[PoseStamped | str] = None):
        """Reset the robot to the idle pose."""
        self.log("Resetting rig")
        if self.is_state_colliding(self.planning_group_name):
            self.log(
                "Resetting rig: Moving out of collision",
                severity="DEBUG",
            )
            self.move_out_of_collision(goal="idle")
        if len(self.attached_collision_object_ids) > 0:
            self.log(
                "Resetting rig: Returning object to original pose",
                severity="DEBUG",
            )
            self.return_object(end_goal=end_goal)

    @asyncio_task_decorator
    async def reset_rig_async(
        self, end_goal: Optional[PoseStamped | str] = None
    ):
        """
        Move the robot out of collision if necessary and return any attached
        objects to their original positions.
        """
        self.log("Resetting rig asynchronously")
        if self.is_state_colliding(self.planning_group_name):
            self.log(
                "Resetting rig asynchronously: Moving out of collision",
                severity="DEBUG",
            )
            await self.move_out_of_collision_async(goal="idle")
        if len(self.attached_collision_object_ids) > 0:
            self.log(
                "Resetting rig asynchronously: Returning object to original pose",
                severity="DEBUG",
            )
            await self.return_object_async(end_goal=end_goal)

    ############################################################
    ########## Initialize ######################################
    ############################################################

    def init_rig(self, timeout_s: Optional[float] = None):
        """Initialize the robot to the idle pose."""
        self.log("Initializing rig")
        if timeout_s is None:
            timeout_s = self.get_parameter_wrapper("rig.init_timeout")

        start_time = time.time()
        while True:
            try:
                self.reset_rig()
                break
            except (TimeoutError, MaxAttemptsReachedError) as e:
                self.log(
                    f"Error resetting rig: {type(e).__name__}: {e}",
                    severity="WARN",
                )
                if time.time() - start_time > timeout_s:  # type: ignore
                    raise e
                time.sleep(1)

    def init_commander(
        self,
        dashboard_timeout_s: Optional[float] = None,
        rig_timeout_s: Optional[float] = None,
    ):
        """Initialize the commander."""
        self.log("Initializing commander")
        self.init_dashboard(dashboard_timeout_s)
        self.init_rig(rig_timeout_s)

    ############################################################
    ########## Planning context manager #######################
    ############################################################

    @asynccontextmanager
    async def planning_context_manager_async(self):
        """
        Context manager for planning and executing actions.

        This context manager handles exceptions that occur during planning and
        executing actions, and automatically resets the robot and moves it out
        of collision if necessary.
        """
        self.log("Entering planning context manager", severity="DEBUG")

        try:
            try:
                async with asyncio.TaskGroup() as tg:
                    self.tg = tg
                    yield tg
            except ExceptionGroup as e:
                self.log(
                    "Caught TaskGroup exceptions in TaskGroup context manager:",
                    severity="WARN",
                )
                for exception in e.exceptions:
                    self.log(
                        f"Task exception: {type(exception).__name__}",
                        severity="WARN",
                    )
                    self.log(f"{exception}", severity="WARN")
                raise e.exceptions[0]  # TODO: fix this
            finally:
                self.tg = None
        except (TimeoutError, MaxAttemptsReachedError, ServiceCallError) as e:
            self.log(
                "Caught exception while running commander:",
                severity="WARN",
            )
            self.log(f"{type(e).__name__}: {e}")

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

    def destroy_node(self):
        self.moveit_py.shutdown()
        super().destroy_node()


# Example script using the commander node


async def run(commander: Commander, config: Mapping[str, Any]):
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
        i = 0
        while True:
            async with commander.planning_context_manager_async():
                async with asyncio.timeout(config["plan_and_execute_timeout"]):
                    object_id = object_ids[i]
                    end_goal = waypoints[object_id]

                    arm_door_future = commander.arm_door_close_and_wait()
                    smartglass_future = commander.smartglass_occlude_and_wait()

                    await arm_door_future
                    await smartglass_future

                    await commander.fetch_object_async(object_id, end_goal)

                    await commander.wait_for_hand_fixation_press_async(
                        timeout_sec=2
                    )

                    arm_door_future = commander.arm_door_open_and_wait()
                    smartglass_future = commander.smartglass_reveal_and_wait()

                    await arm_door_future
                    await smartglass_future

                    await commander.return_object_async()

                    i = (i + 1) % len(object_ids)
    except Exception as e:
        print("Re-raising exception from run():")
        print(f"{type(e).__name__}: {e}")
        traceback.print_exc()
        raise e


def main(args=None):
    rclpy.init(args=args)

    non_ros_args = rclpy.utilities.remove_ros_args(args)  # type: ignore
    config_file = non_ros_args[1]

    with open(config_file, "r") as f:
        run_config = yaml.safe_load(f)

    try:
        executor = MultiThreadedExecutor()
        commander = Commander()
        executor.add_node(commander)

        future = executor.create_task(asyncio.run, run(commander, run_config))

        try:
            executor.spin_until_future_complete(future)
        finally:
            print("Shutting down executor")
            executor.shutdown()
            print("Shutting down commander")
            commander.destroy_node()
    finally:
        print("Shutting down rclpy")
        rclpy.shutdown()
