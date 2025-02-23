import asyncio
import glob
import logging
import os
import traceback
from collections.abc import Awaitable, Iterable, Mapping
from typing import Any, Optional

import numpy as np
import rclpy
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
    AttachedCollisionObject,
    CollisionObject,
    MoveItErrorCodes,
    RobotTrajectory,
)
from moveit_msgs.msg import (
    PlanningScene as PlanningSceneMsg,
)
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.task import Future as RclpyFuture
from shape_msgs.msg import Plane
from std_srvs.srv import SetBool, Trigger
from tabletop_msgs.srv import SetUint32
from ur_dashboard_msgs.srv import Load

from tabletop_server.nodes.base import DEFAULT_LOG_SEVERITY, BaseNode
from tabletop_server.utils import (
    MaxAttemptsReachedError,
    ServiceCallError,
    collision_object_from_geometry,
    create_object_color_msg,
    load_geometry,
    matrix_from_pose_msg,
    moveit_error_code_to_str,
    pose_msg_from_dict,
    pose_msg_from_matrix,
    pose_stamped_msg_from_dict,
    simplify_bounding_primitive,
    simplify_convex_hull,
    simplify_quadratic_decimation,
)

type PathType = str | os.PathLike


class Commander(BaseNode):
    default_params: dict[str, Any] = BaseNode.default_params | {}
    required_params: set[str] = BaseNode.required_params | {
        "max_plan_attempts",
        "max_execution_attempts",
        "max_reset_attempts",
        "plan_and_execute_timeout",
        "planning.group_name",
        "planning.pipeline",
        "dashboard.installation",
        "dashboard.program",
        "dashboard.connect_timeout",
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
        # Initialize callback groups
        self.flic_cg = MutuallyExclusiveCallbackGroup()
        self.state_machine_cg = MutuallyExclusiveCallbackGroup()
        self.dashboard_cg = MutuallyExclusiveCallbackGroup()
        self.reentrant_cg = ReentrantCallbackGroup()

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

        # self.sensors_sub = self.create_subscription(
        #     TeensySensors,
        #     "/teensy_sensors",
        #     self.sensors_callback,
        #     callback_group=self.flic_cg,
        # )

        self.log("Commander initialized")

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
        return await self.service_call_async(
            srv_request=SetBool.Request(data=False),
            srv_type=SetBool,
            srv_name="/teensy/arm_door",
        )

    async def reward_start_async(self, duration_ms: int):
        """
        Coroutine to call the reward service to deliver a reward for a given
        duration.
        """
        if duration_ms < 0:
            raise ValueError("Duration must be greater than 0!")
        return await self.service_call_async(
            srv_request=SetUint32.Request(data=duration_ms),
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
            msg = f"Plan failed with error code: {moveit_error_code_to_str[plan_response.error_code.val]}"
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
                    error_msg = f"Planning attempt {i + 1}/{max_plan_attempts} failed with error code {moveit_error_code_to_str[plan_response.error_code.val]}"
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

    def get_frame_transform(self, frame_id: str) -> np.ndarray:
        """
        Get the frame transform for a given frame id from the planning scene.
        """
        with self.planning_scene_monitor.read_only() as scene:
            return scene.get_frame_transform(frame_id)

    def get_frame_pose(self, frame_id: str) -> Pose:
        """
        Get the frame pose for a given frame id from the planning scene.
        """
        tf = self.get_frame_transform(frame_id)
        return pose_msg_from_matrix(tf)

    def get_planning_frame(self) -> str:
        """
        Get the planning frame from the planning scene.
        """
        with self.planning_scene_monitor.read_only() as scene:
            return scene.planning_frame

    def process_floor_collision_object(self):
        """
        Add the floor collision object to the planning scene.
        """

        self.log("Processing floor collision object")

        collision_object = CollisionObject()
        collision_object.header.frame_id = self.get_planning_frame()
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
        scale: float = 1.0,
        simplification: Optional[str] = None,
        pose: Optional[Pose] = None,
        add_bottom_subframe: bool = False,
        correction_tf: Optional[np.ndarray] = None,
        reference_frame_id: Optional[str] = None,
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
        if reference_frame_id is None:
            reference_frame_id = self.get_planning_frame()

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
        collision_object = collision_object_from_geometry(
            geometry=geometry,
            object_id=object_id,
            pose=pose,
            reference_frame_id=reference_frame_id,
            subframe_names=["bottom"] if add_bottom_subframe else [],
            subframe_poses=[pose] if add_bottom_subframe else [],
            operation="add",
        )

        if color is not None:
            color_msg = create_object_color_msg(object_id, color)
        else:
            color_msg = None

        # Add collision object to planning scene
        self.planning_scene_monitor.process_collision_object(
            collision_object, color_msg
        )

    def attach_collision_object(
        self,
        *,
        object_id: Optional[str] = None,
        link_name: str,
    ):
        """
        Attach an object to the robot.
        """
        self.log(f"Attaching object {object_id}")
        attached_collision_object = AttachedCollisionObject()
        attached_collision_object.object.id = object_id
        attached_collision_object.link_name = link_name
        attached_collision_object.object.operation = CollisionObject.ADD
        self.planning_scene_monitor.process_attached_collision_object(
            attached_collision_object
        )

    def detach_collision_object(self, object_id: str):
        self.log(f"Detaching object {object_id}")
        attached_collision_object = AttachedCollisionObject()
        attached_collision_object.object.id = object_id
        attached_collision_object.object.operation = CollisionObject.REMOVE
        self.planning_scene_monitor.process_attached_collision_object(
            attached_collision_object
        )

    def allow_collision(self, id_1: str, id_2: str):
        with self.planning_scene_monitor.read_write() as scene:
            scene: PlanningScene = scene
            scene.allowed_collision_matrix.set_entry(id_1, id_2, True)
            scene.current_state.update()

    def log_planning_scene(self, severity: str = "INFO"):
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene
            planning_scene_msg: PlanningSceneMsg = scene.planning_scene_message
            planning_scene_msg.robot_state.attached_collision_objects = []
            planning_scene_msg.world.collision_objects = []
            self.log(
                f"Planning scene: {planning_scene_msg}", severity=severity
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
        for mesh_type in ["static_meshes", "dynamic_meshes"]:
            # Get parameters
            prefix = f"planning_scene.{mesh_type}"
            meshes_path: str = self.get_parameter_wrapper(f"{prefix}.path")
            meshes_scale: float = self.get_parameter_wrapper(f"{prefix}.scale")
            meshes_simplification: str = self.get_parameter_wrapper(
                f"{prefix}.simplification"
            )
            try:
                meshes_correction = self.get_parameter_wrapper(
                    f"{prefix}.correction"
                )
                meshes_correction = pose_msg_from_dict(meshes_correction)
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

                    try:
                        pose = self.get_parameter_wrapper(
                            f"{prefix}.poses.{object_id}"
                        )
                        pose = pose_msg_from_dict(pose)
                    except ParameterNotDeclaredException:
                        pose = None

                    self.add_mesh_collision_object(
                        path=path,
                        object_id=object_id,
                        scale=meshes_scale,
                        simplification=meshes_simplification,
                        pose=pose,
                        add_bottom_subframe=mesh_type == "dynamic_meshes",
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

                try:
                    pose = self.get_parameter_wrapper(
                        f"{prefix}.poses.{object_id}"
                    )
                    pose = pose_msg_from_dict(pose)
                except ParameterNotDeclaredException:
                    pose = None

                self.add_mesh_collision_object(
                    path=meshes_path,
                    object_id=object_id,
                    scale=meshes_scale,
                    simplification=meshes_simplification,
                    pose=pose,
                    add_bottom_subframe=mesh_type == "dynamic_meshes",
                    correction_tf=meshes_correction_tf,
                    color=meshes_color,
                )

        # Update planning scene
        for static_object_id in static_object_ids:
            self.allow_collision("base_link_inertial", static_object_id)

        # Log planning scene
        self.log_planning_scene(severity="DEBUG")

    def remove_collision_object(self, object_id: str):
        with self.planning_scene_monitor.read_write() as scene:
            scene.remove_collision_object(object_id)
            scene.current_state.update()

    async def fetch_object_async(
        self,
        object_id: str,
        target_pose: PoseStamped,
    ) -> Pose:
        self.log(f"Fetching object {object_id}")

        pose_link: str = self.get_parameter_wrapper("planning.pose_link")
        object_frame_id: str = object_id + "/bottom"

        fetch_pose = PoseStamped()
        fetch_pose.header.frame_id = object_frame_id
        fetch_pose.pose.position.x -= 0.1  # TODO: Make this a parameter
        fetch_pose.pose.position.z -= 0.1  # TODO: Make this a parameter

        await self.plan_and_execute_async(fetch_pose, pose_link)

        self.allow_collision(pose_link, object_id)

        fetch_pose = PoseStamped(header=fetch_pose.header)

        await self.plan_and_execute_async(fetch_pose, pose_link)

        self.attach_collision_object(object_id=object_id, link_name=pose_link)

        await self.plan_and_execute_async(target_pose, pose_link)

        return self.get_frame_pose(object_frame_id)

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

    def return_object(self, object_id):
        raise NotImplementedError("Return object not implemented")
        self.log(f"Returning object {object_id}")
        with self.planning_scene_monitor.read_only() as scene:
            object_pose = scene.get_object_pose(object_id)
            if object_pose is None:
                self.log(
                    f"Object {object_id} not found in planning scene",
                    severity="ERROR",
                )
                return

            target_pose = PoseStamped()
            target_pose.header = object_pose.header
            target_pose.pose = object_pose.pose
            target_pose.pose.position.z -= 0.1  # TODO: Make this a parameter

            self.plan_and_execute(target_pose)

    def destroy_node(self):
        self.moveit_py.shutdown()
        super().destroy_node()


async def run(commander: Commander):
    i = 0

    try:
        waypoints_path: list[int] = commander.get_parameter_wrapper(
            "waypoints.path"
        )
        waypoints = {}

        for name in waypoints_path:
            prefix = f"waypoints.poses_stamped.{name}"
            pose_stamped_dict = commander.get_parameter_wrapper(prefix)
            waypoints[name] = pose_stamped_msg_from_dict(pose_stamped_dict)

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
                        commander.get_parameter_wrapper(
                            "plan_and_execute_timeout"
                        )
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
                await asyncio.sleep(1)
    except Exception as e:
        print(
            f"Re-raising exception: \n'{type(e).__name__}: {e}' \nfrom run()"
        )
        if commander.get_logger().get_effective_level() == logging.DEBUG:
            traceback.print_exc()
        raise e


def main(args=None):
    rclpy.init(args=args)
    try:
        executor: rclpy.Executor = rclpy.executors.MultiThreadedExecutor()  # type: ignore
        commander = Commander()
        executor.add_node(commander)

        future = executor.create_task(asyncio.run, run(commander))

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
