import asyncio
import glob
import os
from collections.abc import Awaitable
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit.core.controller_manager import ExecutionStatus  # type: ignore
from moveit.core.planning_interface import MotionPlanResponse  # type: ignore
from moveit.core.planning_scene import PlanningScene  # type: ignore
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
    PlanningComponent,
    PlanningSceneMonitor,
    PlanRequestParameters,
    TrajectoryExecutionManager,
)
from moveit_msgs.msg import (
    CollisionObject,
    PositionConstraint,
    RobotTrajectory,
)
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.task import Future as RclpyFuture
from shape_msgs.msg import Plane, SolidPrimitive
from std_msgs.msg import Header
from std_srvs.srv import SetBool, Trigger
from tabletop_msgs.srv import SetUint32
from ur_dashboard_msgs.srv import Load

from tabletop_server.nodes import BaseNode
from tabletop_server.utils import (
    MaxAttemptsReachedError,
    ServiceCallError,
    collision_object_from_mesh,
    load_mesh,
    pose_stamped_from_params,
)


class Commander(BaseNode):
    default_params: dict[str, Any] = BaseNode.default_params | {}
    required_params: set[str] = BaseNode.required_params | {
        "state_machine_period",
        "max_plan_attempts",
        "max_execution_attempts",
        "max_reset_attempts",
        "planning.group_name",
        "planning.pose_link",
        "planning.pipeline",
        "dashboard.installation",
        "dashboard.program",
        "dashboard.connect_timeout",
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
                self.get_parameter("planning.group_name").value
            )
        )
        self.trajectory_execution_manager: TrajectoryExecutionManager = (
            self.moveit_py.get_trajectory_execution_manager()
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
        Trigger a service via the dashboard client.
        """
        self.service_call(
            srv_request=Trigger.Request(), srv_type=Trigger, srv_name=srv_name
        )

    def dashboard_trigger_async(self, srv_name: str) -> Awaitable:
        """
        Trigger a service via the dashboard client asynchronously.
        """
        return self.service_call_async(
            srv_request=Trigger.Request(), srv_type=Trigger, srv_name=srv_name
        )

    def dashboard_load(self, srv_name: str, filename: str) -> None:
        """
        Load a program or installation via the dashboard client.
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
        Load a program or installation via the dashboard client.
        """
        self.log(f"Loading {srv_name}: {filename} in UR Dashboard")
        return await self.service_call_async(
            srv_request=Load.Request(filename=filename),
            srv_type=Load,
            srv_name=srv_name,
        )

    def reset_robot(self):
        self.log("Resetting robot")
        self.wait_for_service(Trigger, "/dashboard_client/close_popup")
        self.dashboard_trigger("/dashboard_client/close_popup")
        self.dashboard_trigger("/dashboard_client/close_safety_popup")
        self.dashboard_trigger("/dashboard_client/unlock_protective_stop")
        self.dashboard_load(
            "/dashboard_client/load_program",
            self.get_parameter("dashboard.program").value,  # type: ignore
        )
        self.dashboard_trigger("/dashboard_client/brake_release")
        self.dashboard_trigger("/dashboard_client/play")

    async def reset_robot_async(self):
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
            self.get_parameter("dashboard.program").value,  # type: ignore
        )
        await self.dashboard_trigger_async("/dashboard_client/brake_release")
        await self.dashboard_trigger_async("/dashboard_client/play")

    async def smartglass_reveal_async(self):
        return await self.service_call_async(
            srv_request=SetBool.Request(data=True),
            srv_type=SetBool,
            srv_name="/teensy/smartglass",
        )

    async def smartglass_occlude_async(self):
        """
        Occlude the smartglass.
        """
        return await self.service_call_async(
            srv_request=SetBool.Request(data=False),
            srv_type=SetBool,
            srv_name="/teensy/smartglass",
        )

    async def arm_door_open_async(self):
        return await self.service_call_async(
            srv_request=SetBool.Request(data=True),
            srv_type=SetBool,
            srv_name="/teensy/arm_door",
        )

    async def arm_door_close_async(self):
        return await self.service_call_async(
            srv_request=SetBool.Request(data=False),
            srv_type=SetBool,
            srv_name="/teensy/arm_door",
        )

    async def reward_start_async(self, duration_ms: int):
        """
        Deliver a reward for a given duration.
        """
        if duration_ms < 0:
            raise ValueError("Duration must be greater than 0!")
        return await self.service_call_async(
            srv_request=SetUint32.Request(data=duration_ms),
            srv_type=SetUint32,
            srv_name="/teensy/reward",
        )

    async def wait_for_hand_fixation_async(self):
        return await self.service_call_async(
            srv_request=Trigger.Request(),
            srv_type=Trigger,
            srv_name="/teensy/hand_fixation",
        )

    async def start_flic_button_async(self):
        return await self.service_call_async(
            srv_request=Trigger.Request(),
            srv_type=Trigger,
            srv_name="/sensor/flic",
        )

    def plan(
        self, goal: PoseStamped, pose_link: Optional[str] = None
    ) -> MotionPlanResponse:
        """
        Coroutine to plan the trajectory from the current state to the current
        waypoint.
        """
        self.log(f"Planning trajectory to waypoint: {goal}")

        self.planning_component.set_start_state_to_current_state()
        self.planning_component.set_goal_state(
            pose_stamped_msg=goal,
            pose_link=pose_link
            or self.get_parameter("planning.pose_link").value,
        )

        if self.get_parameter("planning.pipeline").value == "default":
            return self.planning_component.plan()
        else:
            try:
                request_params = PlanRequestParameters(
                    self.moveit_py,
                    self.get_parameter("planning.pipeline").value,
                )
                return self.planning_component.plan(
                    single_plan_parameters=request_params
                )
            except TypeError:
                request_params = MultiPipelinePlanRequestParameters(
                    self.moveit_py,
                    self.get_parameter("planning.pipeline").value,
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
        Plan the trajectory to the current waypoint asynchronously.
        """

        return await self.create_rclpy_task(
            self.plan,
            goal=goal,
            pose_link=pose_link,
        )

    def execute(self, robot_trajectory: RobotTrajectory) -> ExecutionStatus:
        """
        Start the execution of the plan asynchronously and add a callback to
        handle the execution result (non-blocking).
        """
        self.trajectory_execution_manager.push(robot_trajectory)
        return self.trajectory_execution_manager.execute_and_wait()

    async def execute_async(
        self, robot_trajectory: RobotTrajectory
    ) -> ExecutionStatus:
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
        # Plan the trajectory
        failure_msgs = []
        max_plan_attempts: int = self.get_parameter("max_plan_attempts").value  # type: ignore
        for i in range(max_plan_attempts):
            try:
                plan_result = self.plan(pose_stamped, pose_link)
                if plan_result:
                    break
                else:
                    error_msg = f"Planning attempt {i + 1}/{max_plan_attempts} failed with error code {plan_result.error_code}"
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
        max_execution_attempts: int = self.get_parameter(
            "max_execution_attempts"
        ).value  # type: ignore
        for i in range(max_execution_attempts):
            try:
                execution_status = self.execute(
                    plan_result.trajectory.get_robot_trajectory_msg()
                )
                if execution_status:
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
        await self.create_rclpy_task(
            self.plan_and_execute,
            pose_stamped=pose_stamped,
            pose_link=pose_link,
        )

    def setup_planning_scene(self):
        """
        Setup the planning scene by adding a floor collision object.
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = "floor"

        plane = Plane()
        plane.coef = [0, 0, 1, 0]

        collision_object.planes.append(plane)  # type: ignore
        collision_object.operation = CollisionObject.ADD

        self.planning_scene_monitor.process_collision_object(collision_object)

        rig_dir = "/root/ws/src/tabletop/ros/tabletop_description/meshes"

        correction = [-0.889, -0.845439, 0.884936]
        # base_width = 0.047625
        # correction[2] -= base_width / 2

        for path in sorted(glob.glob(os.path.join(rig_dir, "*.stl"))):
            id = os.path.splitext(os.path.basename(path))[0]
            self.log(f"Loading collision object: {id}")
            mesh = load_mesh(path, scale=0.001, max_faces=1000)
            mesh = mesh.apply_translation(correction)
            collision_object = collision_object_from_mesh(
                mesh=mesh, id=id, base_frame_id="world"
            )
            self.planning_scene_monitor.process_collision_object(
                collision_object
            )

        with self.planning_scene_monitor.read_write() as scene:
            # scene.apply_collision_object(collision_object)
            # scene.current_state.update()
            # TODO: Make this a parameter
            scene.save_geometry_to_file(
                "/root/ws/src/tabletop/ros/tabletop_description/meshes/planning_scene.pcd"
            )

        # object_transform_matrix = self.get_frame_transform("scene")
        # object_pose = PoseStamped()
        # object_pose.header.frame_id = "world"
        # object_pose.pose = pose_from_matrix(object_transform_matrix)
        # print(f"Object pose: type {type(object_pose)}, {object_pose}")
        # planning_frame = self.get_planning_frame()
        # print(f"Planning frame: {planning_frame}")

    def attach_object(self, object_id):
        with self.planning_scene_monitor.read_write() as scene:
            scene.remove_collision_object(object_id)
            scene.current_state.update()

    def log_planning_scene(self):
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene  # type: ignore
            self.log(f"Planning scene: {scene.planning_scene_message}")

    def get_frame_transform(self, object_id):
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene  # type: ignore
            return scene.get_frame_transform(object_id)

    def get_planning_frame(self):
        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene  # type: ignore
            return scene.planning_frame

    async def fetch_object_async(
        self,
        object_id: str,
        target_pose: PoseStamped,
        reference_frame_id: str = "world",
    ):
        raise NotImplementedError("Fetch object not implemented")
        self.log(f"Fetching object {object_id}")
        header = Header(frame_id=reference_frame_id)

        with self.planning_scene_monitor.read_only() as scene:
            scene: PlanningScene = scene  # type: ignore
            object_pose = scene.get_frame_transform(object_id)

        self.log(f"Object pose type: {type(object_pose)}, {object_pose}")

        fetch_pose = PoseStamped(header=header, pose=object_pose)
        fetch_pose.pose.position.z -= 0.1  # TODO: Make this a parameter

        await self.plan_and_execute_async(fetch_pose)

        line_constraint = PositionConstraint()
        line_constraint.header.frame_id = reference_frame_id
        line_constraint.link_name = self.get_parameter(
            "planning.pose_link"
        ).value  # type: ignore
        line = SolidPrimitive()
        line.type = SolidPrimitive.BOX
        line.dimensions = {0.0005, 0.0005, 1.0}
        line_constraint.constraint_region.primitives.append(line)  # type: ignore

        with self.planning_scene_monitor.read_write() as scene:
            scene: PlanningScene = scene  # type: ignore
            scene.apply_collision_object(object_id)
            scene.current_state.update()

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
    # Initialize waypoints
    waypoints_path: list[int] = commander.get_parameter("waypoints.path").value  # type: ignore
    waypoints = {}

    for name in waypoints_path:
        prefix = f"waypoints.poses_stamped.{name}"
        waypoints[name] = pose_stamped_from_params(commander, prefix)

    if len(waypoints) < 1:
        raise ValueError("No valid waypoints found in commander parameters!")

    while True:
        try:
            commander.reset_robot()
            print("Robot reset")

            for name in waypoints_path:
                async with asyncio.timeout(10):
                    await commander.plan_and_execute_async(waypoints[name])

            for i, name in enumerate(waypoints_path):
                plan_exec_future = commander.plan_and_execute_async(
                    waypoints[name]
                )
                if i % 2 == 0:
                    arm_door_future = commander.arm_door_open_async()
                    smartglass_future = commander.smartglass_reveal_async()
                else:
                    arm_door_future = commander.arm_door_close_async()
                    smartglass_future = commander.smartglass_occlude_async()

                await asyncio.gather(
                    plan_exec_future, arm_door_future, smartglass_future
                )

        except (TimeoutError, MaxAttemptsReachedError, ServiceCallError) as e:
            print(
                f"Caught exception: \n'{type(e).__name__}: {e}' \nwhile running commander"
            )
        except Exception as e:
            print(
                f"Re-raising exception: \n'{type(e).__name__}: {e}' \nfrom run()"
            )
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
