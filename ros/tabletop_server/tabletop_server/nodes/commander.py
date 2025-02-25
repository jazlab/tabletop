import asyncio
import glob
import logging
import os
import traceback
from collections.abc import Awaitable, Iterable, Mapping
from copy import deepcopy
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
    CollisionObject,
    MoveItErrorCodes,
    RobotTrajectory,
)
from moveit_msgs.msg import (
    PlanningScene as PlanningSceneMsg,
)
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.task import Future as RclpyFuture
from shape_msgs.msg import Plane
from std_msgs.msg import Header
from std_srvs.srv import SetBool, Trigger
from tabletop_msgs.srv import SetUint32
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
        "dashboard.connect_timeout",
        "idle_pose",
        "planning.group_name",
        "planning.pipeline",
        "planning.pose_link",
        "planning.eef_link",
        "planning.pre_object_offset",
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

    def dashboard_trigger(self, srv_name: str) -> None:
        """
        Call a dashboard client Trigger service.
        """
        self.log(f"Triggering {srv_name} in UR Dashboard")
        self.service_call(
            srv_request=Trigger.Request(), srv_type=Trigger, srv_name=srv_name
        )

    def dashboard_trigger_async(self, srv_name: str) -> Awaitable:
        """
        Coroutine to call a dashboard client Trigger service asynchronously.
        """
        self.log(f"Triggering {srv_name} in UR Dashboard asynchronously")
        return self.service_call_async(
            srv_request=Trigger.Request(), srv_type=Trigger, srv_name=srv_name
        )

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

    def reset_robot(self):
        """
        Call a sequence of dashboard client services to reset the robot.
        """
        self.log("Resetting robot")
        self.wait_for_service(Trigger, "/dashboard_client/close_popup")
        self.dashboard_trigger("/dashboard_client/close_popup")
        self.dashboard_trigger("/dashboard_client/close_safety_popup")
        self.dashboard_trigger("/dashboard_client/unlock_protective_stop")
        self.dashboard_load(
            "/dashboard_client/load_program",
            self.get_parameter_wrapper("dashboard.program"),
        )
        self.dashboard_trigger("/dashboard_client/brake_release")
        self.dashboard_trigger("/dashboard_client/play")

    async def reset_robot_async(self):
        """
        Coroutine to call a sequence of dashboard client services to reset the
        robot asynchronously.
        """
        self.log("Resetting robot")
        self.wait_for_service(Trigger, "/dashboard_client/close_popup")
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

    async def smartglass_reveal_async(self):
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

    async def smartglass_occlude_async(self):
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

    async def arm_door_open_async(self):
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

    async def arm_door_close_async(self):
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

    async def reward_async(self, duration_s: float):
        """
        Coroutine to call the reward service to deliver a reward for a given
        duration.
        """
        self.log(f"Delivering reward for {duration_s} s")
        if duration_s < 0:
            raise ValueError("Duration must be greater than 0!")
        return await self.service_call_async(
            srv_request=SetUint32.Request(data=int(duration_s * 1000)),
            srv_type=SetUint32,
            srv_name="/teensy/reward",
        )

    async def wait_for_hand_fixation_async(self):
        """
        Coroutine to call the hand fixation service to wait for the hand
        fixation asynchronously.
        """
        return await self.service_call_async(
            srv_request=Trigger.Request(),
            srv_type=Trigger,
            srv_name="/teensy/hand_fixation",
        )

    def hand_fixation_state(self):
        """Get current hand fixation state."""
        return self.service_call(
            srv_request=Trigger.Request(),
            srv_type=Trigger,
            srv_name="/sensors/hand_fixation",
        )

    async def wait_for_hand_fixation_on_async(self, timeout_sec: float):
        """Wait for hand fixation state to turn on, then return True.

        If already on, return True immediately. If timeout_sec is reached,
        return False.
        """
        try:
            async with asyncio.timeout(timeout_sec):
                while not self.hand_fixation_state():
                    await asyncio.sleep(0.001)
            return True
        except asyncio.TimeoutError:
            return False

    async def wait_for_hand_fixation_off_async(self, timeout_sec: float):
        """Wait for hand fixation state to turn off, then return True.

        If already off, return True immediately. If timeout_sec is reached,
        return False.
        """
        try:
            async with asyncio.timeout(timeout_sec):
                while self.hand_fixation_state():
                    await asyncio.sleep(0.001)
            return True
        except asyncio.TimeoutError:
            return False

    async def start_flic_button_async(self):
        """
        Coroutine to call the Flic button service to start the Flic button
        asynchronously.
        """
        return await self.service_call_async(
            srv_request=Trigger.Request(),
            srv_type=Trigger,
            srv_name="/sensor/flic",
        )

    # TODO: Remove this
    async def wait_for_flic_button_async(self):
        """
        Coroutine to simulate a flic button press asynchronously.
        """
        self.log("Waiting for flic button press")
        reaction_time = np.random.exponential(10.0)
        await asyncio.sleep(reaction_time)
        self.log("Flic button pressed")
        return True

    def plan(
        self, goal: PoseStamped, pose_link: Optional[str] = None
    ) -> MotionPlanResponse:
        """
        Plan a trajectory to the given waypoint.

        Args:
            goal (PoseStamped): The goal pose in a stamped coordinate frame.
            pose_link (str, optional): The link name to use for the goal pose.
                Defaults to parameter "planning.pose_link" if not provided.

        Returns:
            MotionPlanResponse: The planned trajectory.
        """
        self.log(f"Planning trajectory to waypoint: {goal}")

        self.planning_component.set_start_state_to_current_state()

        if pose_link is None:
            pose_link = self.get_parameter_wrapper("planning.pose_link")
        self.planning_component.set_goal_state(
            pose_stamped_msg=goal, pose_link=pose_link
        )

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

    async def plan_async(
        self, goal: PoseStamped, pose_link: Optional[str] = None
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
        )

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

    def execute(self, robot_trajectory: RobotTrajectory) -> ExecutionStatus:
        """
        Execute the given robot trajectory.

        Args:
            robot_trajectory (RobotTrajectory): The robot trajectory to execute.

        Returns:
            ExecutionStatus: The status of the execution.
        """
        self.trajectory_execution_manager.push(robot_trajectory)
        return self.trajectory_execution_manager.execute_and_wait()

    async def execute_async(
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

    def plan_and_execute(
        self, pose_stamped: PoseStamped, pose_link: Optional[str] = None
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
        failure_msgs = []
        max_plan_attempts: int = self.get_parameter_wrapper(
            "max_plan_attempts"
        )
        for i in range(max_plan_attempts):
            try:
                plan_response = self.plan(pose_stamped, pose_link)
                self.log_plan_response(plan_response, severity="DEBUG")
                if plan_response.error_code.val == MoveItErrorCodes.SUCCESS:
                    self.log(
                        f"Planning attempt {i + 1}/{max_plan_attempts} succeeded"
                    )
                    break
                else:
                    error_msg = f"Planning attempt {i + 1}/{max_plan_attempts} failed with error code {moveit_error_code_map[plan_response.error_code.val]}"
                    failure_msgs.append(error_msg)
                    self.log(
                        error_msg,
                        severity="WARN",
                    )
            except Exception as e:
                error_msg = f"Planning attempt {i + 1}/{max_plan_attempts} raised exception {type(e).__name__}: {e}"
                failure_msgs.append(error_msg)
                self.log(
                    error_msg,
                    severity="WARN",
                )
        else:
            error_msg = f"Max planning attempts ({max_plan_attempts}) reached!: {failure_msgs}"
            self.log(error_msg, severity="ERROR")
            raise MaxAttemptsReachedError(error_msg)

        # Execute the plan
        failure_msgs = []
        max_execution_attempts: int = self.get_parameter_wrapper(
            "max_execution_attempts"
        )
        for i in range(max_execution_attempts):
            try:
                execution_status = self.execute(
                    plan_response.trajectory.get_robot_trajectory_msg()
                )
                if execution_status:
                    self.log(
                        f"Execution attempt {i + 1}/{max_execution_attempts} succeeded"
                    )
                    break
                else:
                    error_msg = f"Execution attempt {i + 1}/{max_execution_attempts} failed with status {execution_status.status}"
                    failure_msgs.append(error_msg)
                    self.log(
                        error_msg,
                        severity="WARN",
                    )
            except Exception as e:
                error_msg = f"Execution attempt {i + 1}/{max_execution_attempts} raised exception {type(e).__name__}: {e}"
                failure_msgs.append(error_msg)
                self.log(
                    error_msg,
                    severity="WARN",
                )
        else:
            error_msg = f"Max execution attempts ({max_execution_attempts}) reached!: {failure_msgs}"
            self.log(error_msg, severity="ERROR")
            raise MaxAttemptsReachedError(error_msg)

    async def plan_and_execute_async(
        self, pose_stamped: PoseStamped, pose_link: Optional[str] = None
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
            pose_stamped=pose_stamped,
            pose_link=pose_link,
        )

    async def plan_and_execute_to_idle_async(self):
        raise NotImplementedError("Not implemented")

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
            return scene.planning_frame

    def get_planning_group_name(self) -> str:
        """
        Get the planning group name from the planning scene.
        """
        return self.planning_component.planning_group_name

    def get_collision_object_ids(self) -> list[str]:
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            planning_scene_msg: PlanningSceneMsg = scene.planning_scene_message
            return [
                collision_object.id
                for collision_object in planning_scene_msg.world.collision_objects
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

    def is_state_colliding(self, group_name: str) -> bool:
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

    def update_collision_matrix(self, id_1: str, id_2: str, allow: bool):
        self.log(f"Updating collision matrix for {id_1} and {id_2} to {allow}")
        with self.planning_scene_monitor.read_write() as scene:
            scene: PlanningScene = scene
            scene.allowed_collision_matrix.set_entry(id_1, id_2, allow)
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
            planning_scene_msg, title="Planning Scene Msg:", severity=severity
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
            # TODO: Change to dictionary lookup
            meshes_path: str = self.get_parameter_wrapper(f"{prefix}.path")
            meshes_scale: float = self.get_parameter_wrapper(f"{prefix}.scale")
            meshes_simplification: str = self.get_parameter_wrapper(
                f"{prefix}.simplification"
            )
            try:
                meshes_correction = self.get_parameter_wrapper(
                    f"{prefix}.correction"
                )
                meshes_correction = pose_msg(**meshes_correction)
                meshes_correction_tf = matrix_from_pose_msg(meshes_correction)
            except ParameterNotDeclaredException:
                meshes_correction_tf = None

            meshes_color: list[float] = self.get_parameter_wrapper(
                f"{prefix}.color"
            )

            if not os.path.exists(meshes_path):
                raise FileNotFoundError(
                    f"Mesh path {meshes_path} does not exist"
                )

            # Process individual .stl files from directory
            if os.path.isdir(meshes_path):
                for path in glob.glob(
                    os.path.join(meshes_path, "*.stl")
                ) + glob.glob(os.path.join(meshes_path, "*.dae")):
                    self.log(
                        f"Processing mesh collision object from path: {path}"
                    )
                    object_id = os.path.splitext(os.path.basename(path))[0]
                    if mesh_type == "static_meshes":
                        static_object_ids.append(object_id)
                    else:
                        dynamic_object_ids.append(object_id)

                    try:
                        pose = self.get_parameter_wrapper(
                            f"{prefix}.poses.{object_id}"
                        )
                        pose = pose_msg(**pose)
                    except ParameterNotDeclaredException:
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
            # Process single .stl or .dae file
            else:
                self.log(
                    f"Processing mesh collision object from path: {meshes_path}"
                )
                object_id = os.path.splitext(os.path.basename(meshes_path))[0]
                if mesh_type == "static_meshes":
                    static_object_ids.append(object_id)
                else:
                    dynamic_object_ids.append(object_id)

                try:
                    pose = self.get_parameter_wrapper(
                        f"{prefix}.poses.{object_id}"
                    )
                    pose = pose_msg(**pose)
                except ParameterNotDeclaredException:
                    pose = Pose()

                self.add_mesh_collision_object(
                    path=meshes_path,
                    object_id=object_id,
                    pose=pose,
                    scale=meshes_scale,
                    simplification=meshes_simplification,
                    add_default_subframe=mesh_type == "dynamic_meshes",
                    correction_tf=meshes_correction_tf,
                    color=meshes_color,
                )

        # Update planning scene
        for static_object_id in static_object_ids:
            self.update_collision_matrix(
                "base_link_inertia", static_object_id, True
            )
            self.update_collision_matrix("sphere", static_object_id, True)

        for dynamic_object_id in dynamic_object_ids:
            self.update_collision_matrix("sphere", dynamic_object_id, True)

        # Log planning scene
        self.log_planning_scene(severity="DEBUG")
        self.log_collision_objects(severity="DEBUG")
        self.log_collision_matrix(severity="DEBUG")

    async def fetch_object_async(
        self,
        object_id: str,
        target_pose: PoseStamped,
        subframe_name: str = "default",
    ) -> PoseStamped:
        self.log(f"Fetching object {object_id}")

        object_frame_id: str = object_id + f"/{subframe_name}"
        return_pose = self.get_frame_pose_stamped(object_frame_id)

        self.log(
            f"Fetching object {object_id} from frame {object_frame_id}",
            severity="DEBUG",
        )
        self.log(
            f"{object_frame_id}: {return_pose.pose}",
            severity="DEBUG",
        )

        pre_fetch_pose = PoseStamped()
        pre_fetch_pose.header.frame_id = object_frame_id
        pre_fetch_offset = self.get_parameter_wrapper(
            "planning.pre_object_offset"
        )
        pre_fetch_pose.pose.position.x += pre_fetch_offset[0]
        pre_fetch_pose.pose.position.y += pre_fetch_offset[1]
        pre_fetch_pose.pose.position.z += pre_fetch_offset[2]

        await self.plan_and_execute_async(pre_fetch_pose)

        touch_links: list[str] = self.get_parameter_wrapper(
            "planning.object_touch_links"
        )
        for touch_link in touch_links:
            self.update_collision_matrix(touch_link, object_id, True)

        fetch_pose = PoseStamped()
        fetch_pose.header.frame_id = object_frame_id

        await self.plan_and_execute_async(fetch_pose)

        pose_link: str = self.get_parameter_wrapper("planning.pose_link")
        self.attach_collision_object(
            object_id=object_id,
            link_name=pose_link,
            touch_links=touch_links,
        )

        await self.plan_and_execute_async(target_pose)

        return return_pose

        # line_constraint = PositionConstraint()
        # line_constraint.header.frame_id = reference_frame_id
        # line_constraint.link_name = self.get_parameter_wrapper(
        #     "planning.pose_link"
        # )
        # line = SolidPrimitive()
        # line.type = SolidPrimitive.BOX
        # line.dimensions = {0.0005, 0.0005, 1.0}
        # line_constraint.constraint_region.primitives.append(line)

        # with self.planning_scene_monitor.read_write() as scene:
        #     scene: PlanningScene = scene
        #     scene.apply_collision_object(object_id)
        #     scene.current_state.update()

    async def return_object_async(
        self,
        object_id: str,
        return_pose: PoseStamped,
        end_pose: Optional[PoseStamped] = None,
    ) -> None:
        pose_link: str = self.get_parameter_wrapper("planning.pose_link")

        pre_return_pose = deepcopy(return_pose)
        pre_return_offset = self.get_parameter_wrapper(
            "planning.pre_object_offset"
        )
        pre_return_pose.pose.position.x -= pre_return_offset[0]
        pre_return_pose.pose.position.y -= pre_return_offset[1]
        pre_return_pose.pose.position.z -= pre_return_offset[2]

        await self.plan_and_execute_async(pre_return_pose, pose_link)

        await self.plan_and_execute_async(return_pose, pose_link)

        self.detach_collision_object(object_id=object_id)

        self.update_collision_matrix(pose_link, object_id, False)

        if end_pose is None:
            idle_pose_config = self.get_parameter_wrapper("idle_pose")
            idle_pose = PoseStamped()
            idle_pose.header.frame_id = self.get_planning_frame()
            idle_pose.pose = pose_msg(**idle_pose_config)
            await self.plan_and_execute_async(idle_pose, pose_link)
        else:
            await self.plan_and_execute_async(end_pose, pose_link)

    def destroy_node(self):
        self.moveit_py.shutdown()
        super().destroy_node()


# Example script using the commander node


async def fetch(commander: Commander, run_config: str):
    try:
        with open(run_config, "r") as f:
            config = yaml.safe_load(f)

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

        reset = False
        removed_collisions = False
        while not reset:
            try:
                commander.reset_robot()

                if commander.is_state_colliding(
                    commander.get_planning_group_name()
                ):
                    commander.remove_all_collision_objects()
                    removed_collisions = True

                await commander.plan_and_execute_async(
                    waypoints["object_area"]
                )

                if removed_collisions:
                    commander.setup_planning_scene()
                    removed_collisions = False

                reset = True
            except (
                TimeoutError,
                MaxAttemptsReachedError,
                ServiceCallError,
            ) as e:
                print(
                    f"Caught exception: \n'{type(e).__name__}: {e}' \nwhile resetting robot"
                )
                await asyncio.sleep(1)

        i = 0
        while True:
            try:
                commander.reset_robot()
                print("Robot reset")

                while True:
                    async with asyncio.timeout(
                        config["plan_and_execute_timeout"]
                    ):
                        object_id = object_ids[i]

                        current_pose = commander.get_frame_pose_stamped(
                            f"{object_id}/default"
                        )
                        print(f"Current pose: {current_pose}")

                        target_pose = waypoints[object_id]
                        return_pose = await commander.fetch_object_async(
                            object_id, target_pose
                        )

                        await asyncio.sleep(1)

                        await commander.return_object_async(
                            object_id, return_pose
                        )

                        await asyncio.sleep(1)

                        i += 1
                        if i >= len(object_ids):
                            i = 0
            except (
                TimeoutError,
                MaxAttemptsReachedError,
                ServiceCallError,
            ) as e:
                print(
                    f"Caught exception: \n'{type(e).__name__}: {e}' \nwhile running commander"
                )
                await asyncio.sleep(1)
    except Exception as e:
        print(
            f"Re-raising exception: \n'{type(e).__name__}: {e}' \nfrom run()"
        )
        traceback.print_exc()
        raise e


async def run(commander: Commander, run_config: str):
    i = 0

    try:
        with open(run_config, "r") as f:
            config = yaml.safe_load(f)

        print(config)

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

        commander.setup_planning_scene()

        while True:
            try:
                commander.reset_robot()
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
                            arm_door_future = commander.arm_door_open_async()
                            smartglass_future = (
                                commander.smartglass_reveal_async()
                            )
                        else:
                            arm_door_future = commander.arm_door_close_async()
                            smartglass_future = (
                                commander.smartglass_occlude_async()
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
        print(
            f"Re-raising exception: \n'{type(e).__name__}: {e}' \nfrom run()"
        )
        traceback.print_exc()
        raise e


def main(args=None):
    rclpy.init(args=args)

    non_ros_args = rclpy.utilities.remove_ros_args(args)  # type: ignore
    run_config = non_ros_args[1]

    try:
        executor: rclpy.Executor = rclpy.executors.MultiThreadedExecutor()  # type: ignore
        commander = Commander()
        executor.add_node(commander)

        future = executor.create_task(
            asyncio.run, fetch(commander, run_config)
        )

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
