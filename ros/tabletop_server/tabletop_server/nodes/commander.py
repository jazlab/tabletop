import asyncio
import glob
import logging
import os
import time
import traceback
from collections.abc import AsyncGenerator, Iterable, Mapping
from contextlib import asynccontextmanager
from typing import Any, Optional

import numpy as np
import pandas as pd
import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.controller_manager import ExecutionStatus  # type: ignore
from moveit.core.planning_interface import MotionPlanResponse  # type: ignore
from moveit.core.planning_scene import PlanningScene  # type: ignore
from moveit.core.robot_model import RobotModel  # type: ignore
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
    MoveItErrorCodes,
)
from moveit_msgs.msg import (
    PlanningScene as PlanningSceneMsg,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future as RclpyFuture
from shape_msgs.msg import Plane
from std_msgs.msg import Header
from std_srvs.srv import SetBool, Trigger
from tabletop_msgs.srv import (
    GetArmDoor,
    GetFlic,
    GetHandFixation,
    GetReward,
    GetSmartglass,
    SetReward,
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
    attached_collision_object_msg,
    matrix_from_pose_msg,
    mesh_collision_object_msg,
    moveit_error_code_map,
    object_color_msg,
    pose_msg,
    pose_msg_from_matrix,
    pose_stamped_msg,
)
from tf_transformations import identity_matrix
from ur_dashboard_msgs.srv import Load

from tabletop_server.nodes.base import DEFAULT_LOG_SEVERITY, BaseNode


class Commander(BaseNode):
    default_params: dict[str, Any] = BaseNode.default_params | {}
    required_params: set[str] = BaseNode.required_params | {
        "max_plan_attempts",
        "max_execution_attempts",
        "dashboard.installation",
        "dashboard.program",
        "dashboard.init_timeout",
        "teensy.hand_fixation_spin_period_s",
        "teensy.flic_spin_period_s",
        "planning.group_name",
        "planning.pipeline",
        "planning.pose_link",
        "planning.eef_link",
        "planning.object_touch_links",
        "planning.idle_pose",
        "planning.pre_fetch_offset",
        "planning.pre_attach_offset",
        "planning.post_attach_offset",
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
        self.planning_component: PlanningComponent = (
            self.moveit_py.get_planning_component(
                self.get_parameter_wrapper("planning.group_name")
            )
        )
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
    # def planning_component(self) -> PlanningComponent:
    #     return self.moveit_py.get_planning_component(
    #         self.get_parameter_wrapper("planning.group_name")
    #     )

    # @property
    # def trajectory_execution_manager(self) -> TrajectoryExecutionManager:
    #     return self.moveit_py.get_trajectory_execution_manager()

    # @property
    # def planning_scene_monitor(self) -> PlanningSceneMonitor:
    #     return self.moveit_py.get_planning_scene_monitor()

    # @property
    # def robot_model(self) -> RobotModel:
    #     return self.moveit_py.get_robot_model()

    # Logging
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

    # Dashboard interface

    def dashboard_trigger(self, srv_name: str) -> None:
        """
        Call a dashboard client Trigger service.
        """
        self.log(f"Triggering {srv_name} in UR Dashboard")
        self.service_call(
            srv_request=Trigger.Request(), srv_type=Trigger, srv_name=srv_name
        )

    async def dashboard_trigger_async(self, srv_name: str) -> Trigger.Response:
        """
        Coroutine to call a dashboard client Trigger service asynchronously.
        """
        self.log(f"Triggering {srv_name} in UR Dashboard asynchronously")
        return await self.service_call_async(
            srv_request=Trigger.Request(), srv_type=Trigger, srv_name=srv_name
        )  # type: ignore

    def dashboard_load(self, srv_name: str, filename: str) -> None:
        """
        Load a program or installation on the robot dashboard by calling a
        dashboard client Load service.
        """
        self.log(f"Loading {srv_name}: {filename} in UR Dashboard")
        self.service_call(
            srv_request=Load.Request(filename=filename),
            srv_type=Load,
            srv_name=srv_name,
        )

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
            f"Loading {srv_name}: {filename} in UR Dashboard asynchronously"
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
        await self.dashboard_trigger_async("/dashboard_client/play")

    async def init_dashboard(self, timeout_s: Optional[float] = None):
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
            except (TimeoutError, ServiceCallError):
                self.log("Error initializing dashboard", severity="ERROR")

                if time.time() - start_time > timeout_s:  # type: ignore
                    raise TimeoutError("Dashboard initialization timed out")
                await asyncio.sleep(1)

    # Teensy interface
    async def get_smartglass_async(self) -> GetSmartglass.Response:
        """Get the smartglass state."""
        return await self.service_call_async(
            srv_request=GetSmartglass.Request(),
            srv_type=GetSmartglass,
            srv_name="/teensy/smartglass",
        )  # type: ignore

    async def get_arm_door_async(self) -> GetArmDoor.Response:
        """Get the arm door state."""
        return await self.service_call_async(
            srv_request=GetArmDoor.Request(),
            srv_type=GetArmDoor,
            srv_name="/teensy/arm_door",
        )  # type: ignore

    async def get_reward_async(self) -> GetReward.Response:
        """Get the reward state."""
        return await self.service_call_async(
            srv_request=GetReward.Request(),
            srv_type=GetReward,
            srv_name="/teensy/reward",
        )  # type: ignore

    async def get_hand_fixation_async(self) -> GetHandFixation.Response:
        """Get the hand fixation state."""
        return await self.service_call_async(
            srv_request=GetHandFixation.Request(),
            srv_type=GetHandFixation,
            srv_name="/teensy/hand_fixation",
        )  # type: ignore

    async def get_flic_async(self) -> GetFlic.Response:
        """Get the flic state."""
        return await self.service_call_async(
            srv_request=GetFlic.Request(),
            srv_type=GetFlic,
            srv_name="/teensy/flic",
        )  # type: ignore

    async def start_smartglass_reveal_async(self):
        """
        Coroutine to call the smartglass service to reveal the smartglass
        asynchronously.
        """
        self.log("Smartglass Reveal")
        return await self.service_call_async(
            srv_request=SetBool.Request(data=True),
            srv_type=SetBool,
            srv_name="/teensy/smartglass",
        )

    async def start_smartglass_occlude_async(self):
        """
        Coroutine to call the smartglass service to occlude the smartglass
        asynchronously.
        """
        self.log("Smartglass Occlude")
        return await self.service_call_async(
            srv_request=SetBool.Request(data=False),
            srv_type=SetBool,
            srv_name="/teensy/smartglass",
        )

    async def start_arm_door_open_async(self):
        """
        Coroutine to call the arm door service to open the arm door
        asynchronously.
        """
        self.log("Arm Door Open")
        return await self.service_call_async(
            srv_request=SetBool.Request(data=True),
            srv_type=SetBool,
            srv_name="/teensy/arm_door",
        )

    async def start_arm_door_close_async(self):
        """
        Coroutine to call the arm door service to close the arm door
        asynchronously.
        """
        self.log("Arm Door Close")
        return await self.service_call_async(
            srv_request=SetBool.Request(data=False),
            srv_type=SetBool,
            srv_name="/teensy/arm_door",
        )

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
            srv_name="/teensy/reward",
        )

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
                    fixation.last_time_pressed
                    == initial_fixation.last_time_pressed
                ):
                    fixation = await self.get_hand_fixation_async()
                    await asyncio.sleep(
                        self.get_parameter_wrapper("teensy.spin_period_s")
                    )
            return True
        except TimeoutError:
            return False

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
                    fixation.last_time_released
                    == initial_fixation.last_time_released
                ):
                    fixation = await self.get_hand_fixation_async()
                    await asyncio.sleep(
                        self.get_parameter_wrapper("teensy.spin_period_s")
                    )
            return True
        except TimeoutError:
            return False

    # TODO: Potential race condition (between monkey and get_flic_async lol)
    async def wait_for_flic_press_async(
        self, timeout_s: Optional[float] = None
    ):
        """Wait for flic button press, then return True."""
        initial_flic = await self.get_flic_async()
        flic = initial_flic
        try:
            async with asyncio.timeout(timeout_s):
                while flic.last_time_pressed == initial_flic.last_time_pressed:
                    flic = await self.get_flic_async()
                    await asyncio.sleep(
                        self.get_parameter_wrapper("teensy.spin_period_s")
                    )
            return True
        except TimeoutError:
            return False

    async def smartglass_reveal_and_wait(
        self, timeout_s: Optional[float] = None
    ):
        """Reveal smartglass and wait for it to be revealed."""
        await self.start_smartglass_reveal_async()
        return await self.wait_for_smartglass_reveal_async(timeout_s)

    async def smartglass_occlude_and_wait(
        self, timeout_s: Optional[float] = None
    ):
        """Occlude smartglass and wait for it to be occluded."""
        await self.start_smartglass_occlude_async()
        return await self.wait_for_smartglass_occlude_async(timeout_s)

    async def arm_door_open_and_wait(self, timeout_s: Optional[float] = None):
        """Open arm door and wait for it to be open."""
        await self.start_arm_door_open_async()
        return await self.wait_for_arm_door_open_async(timeout_s)

    async def arm_door_close_and_wait(self, timeout_s: Optional[float] = None):
        """Close arm door and wait for it to be closed."""
        await self.start_arm_door_close_async()
        return await self.wait_for_arm_door_close_async(timeout_s)

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

    def plan_once(
        self,
        goal: PoseStamped | str,
        pose_link: Optional[str] = None,
    ) -> MotionPlanResponse:
        """
        Plan a trajectory to the given waypoint once.

        Args:
            goal (PoseStamped): The goal pose in a stamped coordinate frame.
            pose_link (str, optional): The link name to use for the goal pose.
                Defaults to parameter "planning.pose_link" if not provided.
            max_attempts (int, optional): The maximum number of planning attempts.
        Returns:
            MotionPlanResponse: The planned trajectory.
        """
        self.log("Planning trajectory once:", severity="DEBUG")

        # Set goal state from pose or configuration name
        if pose_link is None:
            pose_link = self.get_parameter_wrapper("planning.pose_link")

        if goal == "idle":
            goal = self.get_idle_pose_stamped()

        plan_kwargs = {}
        if isinstance(goal, PoseStamped):
            plan_kwargs["pose_stamped_msg"] = goal
            plan_kwargs["pose_link"] = pose_link
        elif isinstance(goal, str):
            plan_kwargs["configuration_name"] = goal

        if not self.planning_component.set_goal_state(**plan_kwargs):
            raise ValueError(f"Invalid goal: {goal}")

        # Set start state to current state
        self.planning_component.set_start_state_to_current_state()

        # Plan
        if self.get_parameter_wrapper("planning.pipeline") == "default":
            return self.planning_component.plan()
        else:
            try:
                request_params = PlanRequestParameters(
                    self.moveit_py,
                    self.get_parameter_wrapper("planning.pipeline"),
                )
                return self.planning_component.plan(
                    single_plan_parameters=request_params
                )
            except TypeError:
                request_params = MultiPipelinePlanRequestParameters(
                    self.moveit_py,
                    self.get_parameter_wrapper("planning.pipeline"),
                )
                return self.planning_component.plan(
                    multi_plan_parameters=request_params
                )
            except Exception as e:
                self.log(f"Error planning: {e}", severity="ERROR")
                raise e

    def plan(
        self,
        goal: PoseStamped | str,
        pose_link: Optional[str] = None,
        max_attempts: Optional[int] = None,
    ) -> MotionPlanResponse:
        """
        Plan a trajectory to the given waypoint, retrying up to max_attempts
        times until successful.

        Args:
            goal (PoseStamped | str): The goal pose in a stamped coordinate frame.
            pose_link (str, optional): The link name to use for the goal pose.
                Defaults to parameter "planning.pose_link" if not provided.
            max_attempts (int, optional): The maximum number of planning attempts.
        Returns:
            MotionPlanResponse: The planned trajectory.
        """

        if max_attempts is None:
            max_attempts = self.get_parameter_wrapper("max_plan_attempts")

        failure_msgs = []
        for i in range(max_attempts):  # type: ignore
            try:
                plan_response = self.plan_once(goal, pose_link)
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
        else:
            error_msg = f"Max planning attempts ({max_attempts}) reached!: {failure_msgs}"
            self.log(error_msg, severity="ERROR")
            raise MaxAttemptsReachedError(error_msg)

        return plan_response

    async def plan_async(
        self,
        goal: PoseStamped | str,
        pose_link: Optional[str] = None,
        max_attempts: Optional[int] = None,
    ) -> MotionPlanResponse:
        """
        Asynchronous coroutine wrapper for `plan()` method.

        Creates an rclpy task to compute the trajectory in a separate thread.

        See Also:
            `plan()`: For parameter details and synchronous implementation.
        """
        return await self.create_rclpy_task(
            self.plan,
            goal=goal,
            pose_link=pose_link,
            max_attempts=max_attempts,
        )

    async def plan_generator_async(
        self,
        goals: list[PoseStamped | str],
        pose_link: Optional[str] = None,
        max_attempts_per_goal: Optional[int] = None,
    ) -> AsyncGenerator[MotionPlanResponse, None]:
        """
        Asynchronous coroutine generator that plans a trajectory for each goal
        in the list.

        Args:
            goals (list[PoseStamped | str]): The list of goals to plan
                trajectories for.

        Returns:
            AsyncGenerator[MotionPlanResponse, None]: An asynchronous generator
                that yields the planned trajectory for each goal.
        """
        while goals:
            goal = goals.pop(0)
            new_goals = yield await self.plan_async(
                goal, pose_link, max_attempts=max_attempts_per_goal
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

    def execute(
        self,
        robot_trajectory: RobotTrajectory,
        max_attempts: Optional[int] = None,
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
                execution_status = self.execute_once(robot_trajectory)
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

    async def execute_async(
        self,
        robot_trajectory: RobotTrajectory,
        max_attempts: Optional[int] = None,
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
                    robot_trajectory
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

    def plan_and_execute(
        self,
        goal: PoseStamped | str,
        pose_link: Optional[str] = None,
    ) -> None:
        """
        Plan and execute a trajectory.

        Performs max_plan_attempts planning attempts and max_execution_attempts
        execution attempts. If the method returns successfully, the trajectory
        has been executed (no status is returned).

        Args:
            pose_stamped (PoseStamped): The goal pose in a stamped coordinate
                frame.
            pose_link (str, optional): The link name to use for the goal pose.
                Defaults to parameter "planning.pose_link" if not provided.

        Returns:
            None: This method does not return anything but may raise exceptions

        Raises:
            MaxAttemptsReachedError: If maximum planning attempts (param:
                max_plan_attempts) or execution attempts (param:
                max_execution_attempts) are reached
        """
        # Plan the trajectory
        plan_response = self.plan(goal, pose_link)
        # Execute the plan
        self.execute(plan_response.trajectory)

    async def plan_and_execute_async(
        self, goal: PoseStamped | str, pose_link: Optional[str] = None
    ) -> None:
        """
        Asynchronous coroutine wrapper for `plan_and_execute()` method.

        Creates an rclpy task to compute the plan and execute the planned
        trajectory in a separate thread.

        See Also:
            `plan_and_execute()`: For parameter details and synchronous
                implementation.
        """
        await self.create_rclpy_task(
            self.plan_and_execute,
            goal=goal,
            pose_link=pose_link,
        )

    def get_frame_transform(self, frame_id: str) -> np.ndarray:
        """
        Get the frame transform for a given frame id from the planning scene.
        """
        with self.planning_scene_monitor.read_only() as scene:
            tf = scene.get_frame_transform(frame_id)
            if (tf == identity_matrix()).all():
                raise ValueError(f"Frame transform to {frame_id} is undefined")
            return tf

    def get_frame_pose(self, frame_id: str) -> Pose:
        """
        Get the frame pose for a given frame id from the planning scene.
        """
        tf = self.get_frame_transform(frame_id)
        return pose_msg_from_matrix(tf)

    def get_frame_pose_stamped(self, frame_id: str) -> PoseStamped:
        """
        Get the frame pose for a given frame id from the planning scene.
        """
        pose = self.get_frame_pose(frame_id)
        pose_stamped = PoseStamped(
            header=Header(frame_id=self.get_planning_frame()),
            pose=pose,
        )
        return pose_stamped

    def get_planning_frame(self) -> str:
        """
        Get the planning frame from the planning scene.
        """
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            return scene.planning_frame

    def get_planning_group_name(self) -> str:
        """
        Get the planning group name from the planning scene.
        """
        return self.planning_component.planning_group_name

    def get_idle_pose_stamped(self) -> PoseStamped:
        """
        Get the idle pose from the planning scene.
        """
        idle_pose = self.get_parameter_wrapper("planning.idle_pose")
        idle_pose = pose_msg(**idle_pose)

        idle_pose_stamped = PoseStamped()
        idle_pose_stamped.header.frame_id = self.get_planning_frame()
        idle_pose_stamped.pose = idle_pose
        return idle_pose_stamped

    def get_collision_objects(self) -> list[CollisionObject]:
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            planning_scene_msg: PlanningSceneMsg = scene.planning_scene_message
            return planning_scene_msg.world.collision_objects  # type: ignore

    def get_collision_object_ids(self) -> list[str]:
        return [
            collision_object.id
            for collision_object in self.get_collision_objects()
        ]

    def get_attached_collision_objects(self) -> list[AttachedCollisionObject]:
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            planning_scene_msg: PlanningSceneMsg = scene.planning_scene_message
            return planning_scene_msg.robot_state.attached_collision_objects  # type: ignore

    def get_attached_collision_object_ids(self) -> list[str]:
        return [
            attached_collision_object.object.id
            for attached_collision_object in self.get_attached_collision_objects()
        ]

    def get_collision_matrix_df(self) -> pd.DataFrame:
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
        """
        Check if the current state of the planning scene is colliding.
        """
        if group_name is None:
            group_name = self.get_planning_group_name()

        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            return scene.is_state_colliding(group_name)

    def process_floor_collision_object(
        self,
        *,
        header_frame_id: Optional[str] = None,
    ):
        """
        Add the floor collision object to the planning scene.
        """

        self.log("Processing floor collision object")

        if header_frame_id is None:
            header_frame_id = self.get_planning_frame()

        collision_object = CollisionObject()
        collision_object.header.frame_id = header_frame_id
        collision_object.id = "floor"

        plane = Plane()
        plane.coef = [0, 0, 1, 0]

        collision_object.planes.append(plane)  # type: ignore
        collision_object.operation = CollisionObject.ADD

        self.planning_scene_monitor.process_collision_object(collision_object)

    def add_mesh_collision_object(
        self,
        *,
        path: str,
        object_id: str,
        pose: Pose,
        scale: float = 1.0,
        simplification: Optional[str] = None,
        add_default_subframe: bool = False,
        correction_tf: Optional[np.ndarray] = None,
        header_frame_id: Optional[str] = None,
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
        if header_frame_id is None:
            header_frame_id = self.get_planning_frame()

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
            pose=pose,
            header_frame_id=header_frame_id,
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
            f"Allowed collision matrix: \n{self.get_collision_matrix_df().to_string()}",
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
        self.process_floor_collision_object()

        # Add collision objects
        static_object_ids = []
        dynamic_object_ids = []
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
                meshes_correction = pose_msg(**meshes_correction)
                meshes_correction_tf = matrix_from_pose_msg(meshes_correction)
            except KeyError:
                meshes_correction_tf = None

            # Process individual .stl files from directory
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
                    static_object_ids.append(object_id)
                else:
                    if object_id in self.get_collision_object_ids():
                        self.log(
                            f"Skipping dynamic mesh {object_id} because it already exists in the planning scene",
                            severity="INFO",
                        )
                        continue
                    dynamic_object_ids.append(object_id)
                try:
                    pose = meshes_config["poses"][object_id]
                    pose = pose_msg(**pose)
                except KeyError:
                    pose = Pose()

                self.add_mesh_collision_object(
                    path=path,
                    object_id=object_id,
                    scale=meshes_scale,
                    simplification=meshes_simplification,
                    pose=pose,
                    add_default_subframe=mesh_type == "dynamic_meshes",
                    correction_tf=meshes_correction_tf,
                    color=meshes_color,
                )

        # Update planning scene
        for static_object_id in static_object_ids:
            self.allow_collision("base_link_inertia", static_object_id)
            self.allow_collision("sphere", static_object_id)

        for dynamic_object_id in dynamic_object_ids:
            self.allow_collision("sphere", dynamic_object_id)

        # Log planning scene
        self.log_planning_scene(severity="DEBUG")
        self.log_collision_objects(severity="DEBUG")
        self.log_collision_matrix(severity="DEBUG")

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
        self.log("Moving out of collision with the sphere")

        self.remove_all_collision_objects()

        await self.plan_and_execute_async(goal, pose_link)

        self.setup_planning_scene()

    def object_init_pose(self, object_id: str) -> PoseStamped:
        """
        Get the initial pose of an object from the parameters.
        """
        return pose_stamped_msg(
            header={"frame_id": self.get_planning_frame()},
            pose=self.get_parameter_wrapper(
                f"planning_scene.dynamic_meshes.poses.{object_id}"
            ),
        )

    # Fetch poses

    def pre_fetch_pose(
        self, object_id: str, subframe_name: str
    ) -> PoseStamped:
        return pose_stamped_msg(
            header={"frame_id": object_id + f"/{subframe_name}"},
            pose={
                "position": self.get_parameter_wrapper(
                    "planning.pre_fetch_offset"
                )
            },
        )

    def pre_attach_pose(
        self, object_id: str, subframe_name: str
    ) -> PoseStamped:
        return pose_stamped_msg(
            header={"frame_id": object_id + f"/{subframe_name}"},
            pose={
                "position": self.get_parameter_wrapper(
                    "planning.pre_attach_offset"
                )
            },
        )

    def attach_pose(self, object_id: str, subframe_name: str) -> PoseStamped:
        attach_pose = PoseStamped()
        attach_pose.header.frame_id = object_id + f"/{subframe_name}"
        return attach_pose

    def post_attach_pose(
        self, object_id: str, subframe_name: str
    ) -> PoseStamped:
        post_attach_pose = self.object_init_pose(object_id)
        post_attach_offset = self.get_parameter_wrapper(
            "planning.post_attach_offset"
        )
        post_attach_pose.pose.position.x += post_attach_offset[0]
        post_attach_pose.pose.position.y += post_attach_offset[1]
        post_attach_pose.pose.position.z += post_attach_offset[2]

        return post_attach_pose

    # Return poses
    def pre_detach_pose(
        self, object_id: str, subframe_name: str
    ) -> PoseStamped:
        return self.post_attach_pose(object_id, subframe_name)

    def detach_pose(self, object_id: str) -> PoseStamped:
        return self.object_init_pose(object_id)

    def post_detach_pose(
        self, object_id: str, subframe_name: str
    ) -> PoseStamped:
        return self.pre_attach_pose(object_id, subframe_name)

    def post_return_pose(
        self, object_id: str, subframe_name: str
    ) -> PoseStamped:
        return self.pre_fetch_pose(object_id, subframe_name)

    @property
    def touch_links(self) -> list[str]:
        return self.get_parameter_wrapper("planning.object_touch_links")

    async def fetch_object_async(
        self,
        object_id: str,
        end_goal: PoseStamped | str,
        subframe_name: str = "default",
    ):
        self.log(f"Fetching object {object_id} from subframe {subframe_name}")

        # Pre-fetch pose
        self.log(
            f"Moving to pre-fetch pose {self.pre_fetch_pose(object_id, subframe_name)}"
        )
        await self.plan_and_execute_async(
            goal=self.pre_fetch_pose(object_id, subframe_name)
        )

        self.log(
            f"Allowed collision matrix before fetching: \n"
            f"{self.get_collision_matrix_df().to_string()}",
            severity="DEBUG",
        )

        # Allow collision between touch links and object
        for touch_link in self.touch_links:
            self.allow_collision(touch_link, object_id)

        # Pre-attach pose
        self.log(
            f"Moving to pre-attach pose {self.pre_attach_pose(object_id, subframe_name)}"
        )
        await self.plan_and_execute_async(
            goal=self.pre_attach_pose(object_id, subframe_name)
        )

        # Attach pose (no offset with respect to object frame)
        self.log(
            f"Moving to attach pose {self.attach_pose(object_id, subframe_name)}"
        )
        await self.plan_and_execute_async(
            goal=self.attach_pose(object_id, subframe_name)
        )

        # Attach object
        self.attach_collision_object(
            object_id=object_id,
            link_name=self.get_parameter_wrapper("planning.pose_link"),
            touch_links=self.touch_links,
        )

        self.log(
            f"Allowed collision matrix after attaching: \n"
            f"{self.get_collision_matrix_df().to_string()}",
            severity="DEBUG",
        )

        # Post-attach pose
        self.log(
            f"Moving to post-attach pose {self.post_attach_pose(object_id, subframe_name)}"
        )
        await self.plan_and_execute_async(
            goal=self.post_attach_pose(object_id, subframe_name)
        )

        # Move to target pose
        self.log(f"Moving to end goal {end_goal}")
        await self.plan_and_execute_async(goal=end_goal)

        # TODO: add line constraint for linear cartesian path
        # line_constraint = PositionConstraint()
        # line_constraint.header.frame_id = reference_frame_id
        # line_constraint.link_name = self.get_parameter_wrapper(
        #     "planning.pose_link"
        # )
        # line = SolidPrimitive()
        # line.type = SolidPrimitive.BOX
        # line.dimensions = {0.0005, 0.0005, 1.0}
        # line_constraint.constraint_region.primitives.append(line)

    # TODO: Test this
    async def return_object_async(
        self,
        subframe_name: str = "default",
        end_goal: Optional[PoseStamped | str] = None,
    ) -> None:
        """Return an object to its original position.

        This method is the reverse of fetch_object_async.

        Args:
            object_id: The ID of the object to return
            return_pose: The pose to return the object to
            subframe_name: The subframe name of the object (default: "default")
            end_pose: Optional pose to move to after returning the object
        """
        self.log("Returning object")

        # Get object ID from planning scene and check that there is exactly one
        # attached collision object
        attached_collision_object_ids = (
            self.get_attached_collision_object_ids()
        )
        if len(attached_collision_object_ids) != 1:
            raise RuntimeError(
                f"Expected exactly one attached collision object, "
                f"but got {len(attached_collision_object_ids)}"
            )

        object_id = attached_collision_object_ids[0]

        # Move to the pre-detach pose
        self.log(
            f"Moving to pre-detach pose {self.pre_detach_pose(object_id, subframe_name)}"
        )
        await self.plan_and_execute_async(
            goal=self.pre_detach_pose(object_id, subframe_name)
        )

        # Move to the detach pose
        self.log(f"Moving to detach pose {self.detach_pose(object_id)}")
        await self.plan_and_execute_async(goal=self.detach_pose(object_id))

        # Detach the object
        self.detach_collision_object(object_id=object_id)

        # Allow collision between robot and object
        for touch_link in self.touch_links:
            self.allow_collision(touch_link, object_id)

        # Move to the post-detach pose
        self.log(
            f"Moving to post-detach pose {self.post_detach_pose(object_id, subframe_name)}"
        )
        await self.plan_and_execute_async(
            goal=self.post_detach_pose(object_id, subframe_name)
        )

        # Move to the post-return (pre-fetch) pose
        self.log(
            f"Moving to post-return pose {self.post_return_pose(object_id, subframe_name)}"
        )
        await self.plan_and_execute_async(
            goal=self.post_return_pose(object_id, subframe_name)
        )

        # Disallow collision between touch links and object
        for touch_link in self.touch_links:
            self.disallow_collision(touch_link, object_id)

        # Move to end pose if specified
        if end_goal is not None:
            self.log(f"Moving to end goal {end_goal}")
            await self.plan_and_execute_async(goal=end_goal)

    async def reset_rig_async(
        self, end_goal: Optional[PoseStamped | str] = None
    ):
        """
        Move the robot out of collision if necessary and return any attached
        objects to their original positions.
        """
        self.log("Resetting robot")
        if self.is_state_colliding(self.get_planning_group_name()):
            await self.move_out_of_collision_async(goal="idle")
        if len(self.get_attached_collision_object_ids()) > 0:
            await self.return_object_async(end_goal=end_goal)

    @asynccontextmanager
    async def planning_context_manager_async(self):
        """
        Context manager for planning and executing actions.

        This context manager handles exceptions that occur during planning and
        executing actions, and automatically resets the robot and moves it out
        of collision if necessary.
        """
        await self.reset_rig_async()

        try:
            yield
        except (TimeoutError, MaxAttemptsReachedError, ServiceCallError) as e:
            self.log(
                "Caught exception while running commander:",
                severity="WARN",
            )
            self.log(f"{type(e).__name__}: {e}")

            # TODO: This is a hack to ensure the robot is reset and moved out of
            # collision before the context manager is entered. Only works for
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

    def destroy_node(self):
        self.moveit_py.shutdown()
        super().destroy_node()


# Example script using the commander node


async def fetch(commander: Commander, config: Mapping[str, Any]):
    try:
        waypoints: dict[str, PoseStamped] = {}
        for waypoint_name, waypoint_config in config["waypoints"][
            "poses_stamped"
        ].items():
            waypoint_pose = pose_stamped_msg(**waypoint_config)
            waypoints[waypoint_name] = waypoint_pose

        if len(waypoints) < 1:
            raise ValueError(
                "No valid waypoints found in commander parameters!"
            )

        object_ids: list[str] = config["object_ids"]

        commander.reset_dashboard()
        i = 0
        while True:
            async with commander.planning_context_manager_async():
                async with asyncio.timeout(config["plan_and_execute_timeout"]):
                    object_id = object_ids[i]
                    end_goal = waypoints[object_id]

                    await commander.fetch_object_async(object_id, end_goal)

                    await asyncio.sleep(1)

                    await commander.return_object_async(end_goal="idle")

                    await asyncio.sleep(1)

                    i += 1
                    if i >= len(object_ids):
                        i = 0

    except Exception as e:
        print("Re-raising exception from run():")
        print(f"{type(e).__name__}: {e}")
        traceback.print_exc()
        raise e


async def move(commander: Commander, config: Mapping[str, Any]):
    i = 0

    try:
        waypoints_path: list[int] = config["waypoints"]["path"]
        waypoints = {}

        for waypoint_name, waypoint_config in config["waypoints"][
            "poses_stamped"
        ].items():
            waypoint_pose = pose_stamped_msg(**waypoint_config)
            waypoints[waypoint_name] = waypoint_pose

        if len(waypoints) < 1:
            raise ValueError(
                "No valid waypoints found in commander parameters!"
            )

        commander.reset_dashboard()

        while True:
            try:
                print("Robot reset")

                while True:
                    async with asyncio.timeout(
                        config["plan_and_execute_timeout"]
                    ):
                        name = waypoints_path[i]

                        plan_exec_future = commander.plan_and_execute_async(
                            waypoints[name]
                        )
                        if i % 2 == 0:
                            arm_door_future = (
                                commander.arm_door_open_and_wait()
                            )
                            smartglass_future = (
                                commander.smartglass_reveal_and_wait()
                            )
                        else:
                            arm_door_future = (
                                commander.arm_door_close_and_wait()
                            )
                            smartglass_future = (
                                commander.smartglass_occlude_and_wait()
                            )

                        await asyncio.gather(
                            plan_exec_future,
                            arm_door_future,
                            smartglass_future,
                        )

                        i += 1
                        if i >= len(waypoints_path):
                            i = 0
            except (
                TimeoutError,
                MaxAttemptsReachedError,
                ServiceCallError,
            ) as e:
                print(
                    f"Caught exception: \n'{type(e).__name__}: {e}' \nwhile running commander"
                )
                if (
                    commander.get_logger().get_effective_level()
                    == logging.DEBUG
                ):
                    traceback.print_exc()
                await asyncio.sleep(10)
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
        config = yaml.safe_load(f)

    print(f"Config: {config}")

    try:
        executor = MultiThreadedExecutor()
        commander = Commander()
        executor.add_node(commander)

        future = executor.create_task(asyncio.run, fetch(commander, config))

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
