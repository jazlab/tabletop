import time
from threading import Lock
from typing import Any

import rclpy
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
    PlanRequestParameters,
)
from moveit_msgs.msg import CollisionObject
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from shape_msgs.msg import Plane
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import Load

from tabletop_server.base_node import BaseNode
from tabletop_server.utils import (
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

    def __init__(self, executor):
        super().__init__(
            "commander",
            automatically_declare_parameters_from_overrides=True,
        )
        # Initialize callback groups
        self.state_machine_mutex_group = MutuallyExclusiveCallbackGroup()
        self.reentrant_group = ReentrantCallbackGroup()
        self._executor = executor
        self._mutex = Lock()

        # Initialize MoveItPy
        self.moveit_py = MoveItPy("moveit_py")

        # Initialize MoveItPy components
        self.planning_scene_monitor = (
            self.moveit_py.get_planning_scene_monitor()
        )
        self.setup_planning_scene()

        self.planning_component = self.moveit_py.get_planning_component(
            self.get_parameter("planning.group_name").value
        )
        self.trajectory_execution_manager = (
            self.moveit_py.get_trajectory_execution_manager()
        )

        # Initialize waypoints
        self.waypoints_path = self.get_parameter("waypoints.path").value
        self.waypoints = {}

        for name in set(self.waypoints_path):
            prefix = f"waypoints.poses_stamped.{name}"
            self.waypoints[name] = pose_stamped_from_params(self, prefix)

        if len(self.waypoints) < 1:
            self.log("No valid waypoints found. Exiting...", severity="ERROR")
            exit(1)

        # Initialize state variables
        self.i = 0
        self.plan_attempts = 0
        self.execution_attempts = 0
        self.reset_attempts = 0

        # Start the state machine timer
        self.timer = self.create_timer(
            self.get_parameter("state_machine_period").value,
            self.state_machine,
            callback_group=self.state_machine_mutex_group,
        )

        self.log("Commander initialized")
        self.change_state("INITIALIZED")

    def setup_planning_scene(self):
        """
        Setup the planning scene by adding a floor collision object.
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = "floor"

        plane = Plane()
        plane.coef = [0, 0, 1, 0]

        collision_object.planes.append(plane)

        collision_object.operation = CollisionObject.ADD

        with self.planning_scene_monitor.read_write() as scene:
            scene.apply_collision_object(collision_object)
            scene.current_state.update()

    def log(self, message, severity="INFO"):
        if severity == "DEBUG":
            self.get_logger().debug(message)
        elif severity == "INFO":
            self.get_logger().info(message)
        elif severity == "WARN":
            self.get_logger().warning(message)
        elif severity == "ERROR":
            self.get_logger().error(message)
        elif severity == "FATAL":
            self.get_logger().fatal(message)

    def reset_robot(self):
        self.log("Resetting robot")
        try:
            # self.dashboard_trigger(
            #     "/dashboard_client/connect",
            #     wait_timeout=self.get_parameter(
            #         "dashboard.connect_timeout"
            #     ).value,
            # )
            # self.dashboard_load(
            #     "/dashboard_client/load_installation",
            #     self.get_parameter("dashboard.installation").value,
            # )
            self.dashboard_trigger("/dashboard_client/close_popup")
            self.dashboard_trigger("/dashboard_client/close_safety_popup")
            self.dashboard_trigger("/dashboard_client/unlock_protective_stop")
            self.dashboard_load(
                "/dashboard_client/load_program",
                self.get_parameter("dashboard.program").value,
            )
            self.dashboard_trigger("/dashboard_client/brake_release")
            self.dashboard_trigger("/dashboard_client/play")

            self.reset_attempts = 0
            self.change_state("READY")
        except Exception as e:
            self.log(
                f"Error resetting robot: {type(e).__name__}: {e}",
                severity="ERROR",
            )
            self.reset_attempts += 1
            max_reset_attempts = self.get_parameter("max_reset_attempts").value
            if self.reset_attempts >= max_reset_attempts:
                self.log(
                    "Max reset attempts reached, entering ERROR state",
                    severity="ERROR",
                )
                raise TimeoutError("Max reset attempts reached")
            else:
                self.log(
                    f"Resetting robot failed, trying again ({self.reset_attempts}/{max_reset_attempts} attempts)...",
                    severity="WARN",
                )

    def dashboard_trigger(
        self, srv_name, wait_timeout=None, call_timeout=None
    ):
        """
        Trigger a service via the dashboard client.
        """
        self.service_wait_and_call(
            srv_type=Trigger,
            srv_name=srv_name,
            srv_request=Trigger.Request(),
            wait_timeout=wait_timeout,
            call_timeout=call_timeout,
        )

    def dashboard_load(
        self, service_name, filename, wait_timeout=None, call_timeout=None
    ):
        """
        Load a program or installation via the dashboard client.
        """
        request = Load.Request()
        request.filename = filename
        self.service_wait_and_call(
            srv_type=Load,
            srv_name=service_name,
            srv_request=request,
            wait_timeout=wait_timeout,
            call_timeout=call_timeout,
        )

    async def plan_async(self):
        """
        Coroutine to plan the trajectory from the current state to the current
        waypoint.
        """
        self.log(
            f"Planning trajectory to waypoint {self.i}: {self.waypoints_path[self.i]}"
        )
        self.log(f"Waypoint: {self.waypoints[self.waypoints_path[self.i]]}")

        self.planning_component.set_start_state_to_current_state()
        goal = self.waypoints[self.waypoints_path[self.i]]
        self.planning_component.set_goal_state(
            pose_stamped_msg=goal,
            pose_link=self.get_parameter("planning.pose_link").value,
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

    # Callbacks
    def plan_callback(self, task):
        """
        Done callback for the planning of the trajectory to the current waypoint.
        """
        assert self.state == "PLANNING"
        # Check if the planning succeeded
        try:
            self.plan_result = task.result()
            if self.plan_result is None:
                self.log(
                    "Planning failed and returned None!", severity="ERROR"
                )
            elif self.plan_result.error_code.val != 1:
                self.log(
                    f"Planning failed with error code {self.plan_result.error_code.val}",
                    severity="WARN",
                )
            else:
                self.plan_attempts = 0
                self.log("Planning finished!")
                self.change_state("PLANNED")
                return
        except Exception as e:
            self.log(
                f"Planning failed with exception {type(e).__name__}: {e}",
                severity="ERROR",
            )

        # Check if we have reached the maximum number of planning attempts
        self.plan_attempts += 1
        max_plan_attempts = self.get_parameter("max_plan_attempts").value
        if self.plan_attempts >= max_plan_attempts:
            self.log(
                f"Max planning attempts ({max_plan_attempts}) reached, entering ERROR state",
                severity="ERROR",
            )
            self.change_state("ERROR")
        else:
            self.log(
                f"Planning failed, trying again ({self.plan_attempts}/{max_plan_attempts} attempts)...",
                severity="WARN",
            )
            self.change_state("READY")

    def execution_callback(self, response):
        """
        Done callback for the execution of the current plan.
        """
        assert self.state == "EXECUTING"
        if response.status == "SUCCEEDED":
            self.log("Execution succeeded!")
            self.execution_attempts = 0
            self.i = (self.i + 1) % len(self.waypoints_path)
            self.log(
                f"Moving on to waypoint {self.i}: {self.waypoints_path[self.i]}"
            )
            self.change_state("READY")
        else:
            self.log(
                f"Execution failed with status {response.status}",
                severity="WARN",
            )

            # Check if we have reached the maximum number of execution attempts
            self.execution_attempts += 1
            max_execution_attempts = self.get_parameter(
                "max_execution_attempts"
            ).value
            if self.execution_attempts >= max_execution_attempts:
                self.log(
                    f"Max execution attempts ({max_execution_attempts}) reached, entering ERROR state",
                    severity="ERROR",
                )
                self.change_state("ERROR")
            else:
                self.log(
                    f"Execution failed, trying again ({self.execution_attempts}/{max_execution_attempts} attempts)...",
                    severity="WARN",
                )
                self.change_state("READY")

    # State machine functions
    def plan(self):
        """
        Plan the trajectory to the current waypoint asynchronously and add a
        callback to handle the plan result (non-blocking).
        """
        self.change_state("PLANNING")
        self._plan_task = self._executor.create_task(self.plan_async())
        self._plan_task.add_done_callback(self.plan_callback)

    def execute(self):
        """
        Start the execution of the plan asynchronously and add a callback to
        handle the execution result (non-blocking).
        """
        self.change_state("EXECUTING")
        self.trajectory_execution_manager.push(
            self.plan_result.trajectory.get_robot_trajectory_msg()
        )
        self.trajectory_execution_manager.execute(self.execution_callback)
        
    def execute_sync(self):
        self.trajectory_execution_manager.push(
            self.plan_result.trajectory.get_robot_trajectory_msg()
        )
        return self.trajectory_execution_manager.execute()

    def change_state(self, state):
        """
        Change the state of the commander node.
        """
        self.log(f"Changing state to {state}")
        self.state = state

        # State machine functions

    def task(self):
        """
        Plan the trajectory to the current waypoint asynchronously and add a
        callback to handle the plan result (non-blocking).
        """
        

        self.change_state("TASK")
        task_generator_fn = self.get_parameter("tasks").value[self.i]
        task_function = task_generators["task_generator_fn"]
        
        

        try:
            self._task = self._executor.create_task(self.task_async())
            self._task.add_done_callback(self.task_callback)
        except Exception as e:
            self.log(
                f"Task failed with exception {type(e).__name__}: {e}",
                severity="ERROR",
            )
            self.change_state("ERROR")
            
    def plan_and_execute_sync(self, pose):
        for i in range(self.get_parameter("max_plan_attempts").value):  
            plan_result = await self.plan_async()
            self.validate_plan(plan_result)
            break
        else:
            raise TimeoutError("Max planning attempts reached")
        
        for i in range(self.get_parameter("max_execution_attempts").value):
            response = self.execute_sync()
            if response.status == "SUCCEEDED":
                break
        else:
            raise TimeoutError("Max execution attempts reached")
        
    async def _plan_and_execute_async(self, pose):
        self._plan_and_execute_task()
    
    def plan_and_execute(self, pose, done_callback=None):
        self._plan_and_execute_task = self._executor.create_task(self._plan_and_execute_async(pose))
        if done_callback:
            self._plan_and_execute_task.add_done_callback(done_callback)
        
    def arm_door_open(self):
        

    def state_machine(self):
        """
        State machine for the commander node.
        """
        match self.state:
            case "INITIALIZED":
                self.reset_robot()
            case "READY":
            #     self.trial_generator()
            # case "TASK":
                self.task()
            case "PLAN":
                self.plan()
            case "PLANNING":
                pass
            case "PLANNED":
                self.execute()
            case "EXECUTING":
                pass
            case "ERROR":
                self.log(
                    "Commander entered ERROR state, resetting...",
                    severity="ERROR",
                )
                # TODO: Remove sleep
                time.sleep(1)
                self.reset_robot()
            case _:
                raise ValueError(f"Invalid state: {self.state}")


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()

    commander = Commander(executor)

    executor.add_node(commander)
    executor.spin()

    commander.destroy_node()
    rclpy.shutdown()
