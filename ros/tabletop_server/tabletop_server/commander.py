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
from rclpy.exceptions import (
    ParameterAlreadyDeclaredException,
    ParameterNotDeclaredException,
)
from rclpy.node import Node
from shape_msgs.msg import Plane
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import Load

from .utils import (
    ServiceCallTimeoutError,
    ServiceWaitTimeoutError,
    pose_stamped_from_params,
)


class Commander(Node):
    default_params: dict[str, Any] = {}
    required_params: set[str] = {
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
        "dashboard.default_wait_timeout",
        "dashboard.default_call_timeout",
    }

    def __init__(self, executor):
        super().__init__(
            "commander",
            automatically_declare_parameters_from_overrides=True,
        )
        # Initialize parameters
        assert not self.required_params & self.default_params.keys()
        self._check_required_parameters()
        self._declare_default_parameters()
        self.log_params()

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

        self.change_state("INITIALIZED")
        self.log("Commander initialized")

        # Start the state machine timer
        self.timer = self.create_timer(
            self.get_parameter("state_machine_period").value,
            self.state_machine,
            callback_group=self.state_machine_mutex_group,
        )

    def _check_required_parameters(self):
        """
        Check if all required parameters are declared and exit if not.
        """
        for name in self.required_params:
            try:
                self.get_parameter(name)
            except ParameterNotDeclaredException:
                self.log(
                    f"Required parameter {name} not declared", severity="ERROR"
                )
                exit(1)

    def _declare_default_parameters(self):
        """
        Declare the default parameters, which are used if no overrides are
        provided.
        """
        for name, value in self.default_params.items():
            try:
                self.declare_parameter(name, value)
            except ParameterAlreadyDeclaredException:
                self.log(
                    f"Parameter {name} already declared, using override",
                    severity="WARN",
                )

    def log_params(self, prefix: str = "", severity: str = "INFO"):
        params = self.get_parameters_by_prefix(prefix)
        for name, param in params.items():
            self.log(f"{prefix}.{name}: {param.value}", severity=severity)

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
        self.change_state("RESETTING")
        self.log("Resetting robot")
        try:
            # self.dashboard_trigger(
            #     "/dashboard_client/connect",
            #     wait_timeout=self.get_parameter(
            #         "dashboard.connect_timeout"
            #     ).value,
            # )
            self.dashboard_trigger(
                "/dashboard_client/close_popup",
            )
            self.dashboard_trigger(
                "/dashboard_client/close_safety_popup",
            )
            self.dashboard_trigger("/dashboard_client/unlock_protective_stop")
            self.dashboard_load(
                "/dashboard_client/load_program",
                self.get_parameter("dashboard.program").value,
            )
            # self.dashboard_load(
            #     "/dashboard_client/load_installation",
            #     self.get_parameter("dashboard.installation").value,
            # )
            self.dashboard_trigger("/dashboard_client/brake_release")
            self.dashboard_trigger("/dashboard_client/play")
        except (ServiceWaitTimeoutError, ServiceCallTimeoutError) as e:
            self.log(f"Timeout while resetting robot: {e}", severity="ERROR")
            self.log("Resetting robot failed", severity="ERROR")
            self.change_state("ERROR")
        except Exception as e:
            self.log(f"Error resetting robot: {e}", severity="ERROR")
            self.change_state("ERROR")
        else:
            self.change_state("READY")

    def dashboard_trigger(
        self, service_name, wait_timeout=None, call_timeout=None
    ):
        """
        Trigger a service via the dashboard client.
        """
        self.log(f"Triggering {service_name} service")
        service_client = self.create_client(Trigger, service_name)
        try:
            # Wait for the service to be available
            self.log(
                f"Waiting for {service_name} service to be available...",
                severity="INFO",
            )
            wait_timeout = (
                wait_timeout
                if wait_timeout is not None
                else self.get_parameter("dashboard.default_wait_timeout").value
            )
            if not service_client.wait_for_service(timeout_sec=wait_timeout):
                error_msg = f"{service_name} not available!"
                self.log(error_msg, severity="ERROR")
                raise ServiceWaitTimeoutError(error_msg)

            self.log(f"{service_name} service is available", severity="INFO")

            # Call the service
            self.log(f"Calling {service_name} service...", severity="INFO")
            call_timeout = (
                call_timeout
                if call_timeout is not None
                else self.get_parameter("dashboard.default_call_timeout").value
            )
            request = Trigger.Request()
            response = service_client.call(request, timeout_sec=call_timeout)

            # Check if the service call succeeded
            if response is None:
                error_msg = f"{service_name} service call timed out!"
                self.log(error_msg, severity="ERROR")
                raise ServiceCallTimeoutError(error_msg)
            elif not response.success:
                error_msg = f"{service_name} service call failed: '{response.message}'!"
                self.log(error_msg, severity="ERROR")
                raise RuntimeError(error_msg)
            else:
                self.log(
                    f"{service_name} service call succeeded: '{response.message}'"
                )
        finally:
            service_client.destroy()

    def dashboard_load(
        self, service_name, filename, wait_timeout=None, call_timeout=None
    ):
        """
        Load a program or installation via the dashboard client.
        """
        self.log(f"Loading {filename} via the {service_name} service")
        service_client = self.create_client(Load, service_name)
        try:
            # Wait for the service to be available
            self.log(
                f"Waiting for {service_name} service to be available...",
                severity="INFO",
            )
            wait_timeout = (
                wait_timeout
                if wait_timeout is not None
                else self.get_parameter("dashboard.default_wait_timeout").value
            )
            if not service_client.wait_for_service(timeout_sec=wait_timeout):
                error_msg = f"{service_name} not available!"
                self.log(error_msg, severity="ERROR")
                raise ServiceWaitTimeoutError(error_msg)

            self.log(f"{service_name} service is available", severity="INFO")

            # Call the service
            self.log(f"Calling {service_name} service...", severity="INFO")
            call_timeout = (
                call_timeout
                if call_timeout is not None
                else self.get_parameter("dashboard.default_call_timeout").value
            )
            request = Load.Request()
            request.filename = filename
            response = service_client.call(request, timeout_sec=call_timeout)

            # Check if the service call succeeded
            if response is None:
                error_msg = f"{service_name} service call timed out!"
                self.log(error_msg, severity="ERROR")
                raise ServiceCallTimeoutError(error_msg)
            elif not response.success:
                error_msg = (
                    f"{service_name} service call failed: '{response.answer}'!"
                )
                self.log(error_msg, severity="ERROR")
                raise RuntimeError(error_msg)
            else:
                self.log(
                    f"{service_name} service call succeeded: '{response.answer}'"
                )
        finally:
            service_client.destroy()

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
                    severity="ERROR",
                )
            else:
                self.plan_attempts = 0
                self.log("Planning finished!")
                self.change_state("PLANNED")
                return
        except Exception as e:
            self.log(f"Planning failed with exception: {e}", severity="ERROR")

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

    def change_state(self, state):
        """
        Change the state of the commander node.
        """
        self.log(f"Changing state to {state}")
        self.state = state

    def state_machine(self):
        """
        State machine for the commander node.
        """
        match self.state:
            case "INITIALIZED":
                self.reset_robot()
            case "READY":
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
