import rclpy
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from moveit_msgs.msg import CollisionObject
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from shape_msgs.msg import Plane
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import Load

from .utils import quaternion_from_euler


class Commander(Node):
    default_params: dict = {
        "group_name": "ur_manipulator",
        "ur_program": "external_control.urp",
        "ur_installation": "default.installation",
        "timer_sec": 1.0,
        "waypoint_names": [],
        "pose_link": "eef",
    }

    def __init__(self):
        super().__init__(
            "commander",
            automatically_declare_parameters_from_overrides=True,
        )
        self._declare_parameters()

        self.state_machine_mutex_group = MutuallyExclusiveCallbackGroup()
        self.reentrant_group = ReentrantCallbackGroup()

        # Initialize MoveItPy
        self.moveit_py = MoveItPy("moveit_py")

        # Get the planning component and trajectory execution manager
        self.planning_component = self.moveit_py.get_planning_component(
            self.get_parameter("group_name").value
        )
        self.trajectory_execution_manager = (
            self.moveit_py.get_trajectory_execution_manager()
        )
        self.planning_scene_monitor = (
            self.moveit_py.get_planning_scene_monitor()
        )

        self.setup_planning_scene()

        self.waypoint_path = self.get_parameter("waypoint_path").value
        self.waypoints = {}

        for name in set(self.waypoint_path):
            waypoint = PoseStamped()
            waypoint.header.frame_id = self.get_parameter(
                f"waypoints.{name}.header.frame_id"
            ).value
            waypoint.pose.position.x = self.get_parameter(
                f"waypoints.{name}.pose.position.x"
            ).value
            waypoint.pose.position.y = self.get_parameter(
                f"waypoints.{name}.pose.position.y"
            ).value
            waypoint.pose.position.z = self.get_parameter(
                f"waypoints.{name}.pose.position.z"
            ).value
            waypoint.pose.orientation = quaternion_from_euler(
                self.get_parameter(
                    f"waypoints.{name}.pose.orientation.roll"
                ).value,
                self.get_parameter(
                    f"waypoints.{name}.pose.orientation.pitch"
                ).value,
                self.get_parameter(
                    f"waypoints.{name}.pose.orientation.yaw"
                ).value,
            )
            self.waypoints[name] = waypoint

        if len(self.waypoints) < 1:
            self.log("No valid waypoints found. Exiting...", level="error")
            exit(1)

        self.i = 0

        self.log("Commander initialized")
        self.change_state("INITIALIZED")

        self.timer = self.create_timer(
            self.get_parameter("timer_sec").value,
            self.state_machine,
            callback_group=self.state_machine_mutex_group,
        )

    def setup_planning_scene(self):
        with self.planning_scene_monitor.read_write() as scene:
            collision_object = CollisionObject()
            collision_object.header.frame_id = "world"
            collision_object.id = "floor"

            plane = Plane()
            plane.coef = [0, 0, 1, 0]

            collision_object.planes.append(plane)

            collision_object.operation = CollisionObject.ADD

            scene.apply_collision_object(collision_object)
            scene.current_state.update()  # Important to ensure the scene is updated

    def _declare_parameters(self):
        for param, value in self.default_params.items():
            try:
                self.declare_parameter(param, value)
            except ParameterAlreadyDeclaredException:
                self.log(
                    f"Parameter {param} already declared, using override",
                    level="warn",
                )

    def log(self, msg, level="info"):
        if level == "info":
            self.get_logger().info(msg)
        elif level == "error":
            self.get_logger().error(msg)
        elif level == "warn":
            self.get_logger().warn(msg)
        elif level == "debug":
            self.get_logger().debug(msg)
        elif level == "trace":
            self.get_logger().trace(msg)

    def start_robot(self):
        self.change_state("STARTING")
        self.log("Starting robot")
        self.dashboard_trigger("/dashboard_client/brake_release")
        self.dashboard_load_program(self.get_parameter("ur_program").value)
        self.dashboard_trigger("/dashboard_client/play")
        self.change_state("READY")

    def dashboard_trigger(self, service_name):
        self.log(f"Triggering {service_name}")
        service_client = self.create_client(Trigger, service_name)
        while not service_client.wait_for_service(timeout_sec=2.0):
            self.log(f"{service_name} not available, waiting again...")

        request = Trigger.Request()
        response = service_client.call(request)
        if response is None:
            self.log(f"{service_name} service call timed out", level="error")
        elif response.success:
            self.log(
                f"{service_name} service call succeeded: '{response.message}'"
            )
        else:
            self.log(
                f"{service_name} service call failed: '{response.message}'",
                level="error",
            )
        service_client.destroy()

    def dashboard_load_program(self, program_name):
        self.log(f"Loading program: {program_name}")
        service_client = self.create_client(
            Load, "/dashboard_client/load_program"
        )
        while not service_client.wait_for_service(timeout_sec=2.0):
            self.log("load_program service not available, waiting again...")

        request = Load.Request()
        request.filename = program_name
        response = service_client.call(request)
        if response is None:
            self.log("load_program service call timed out", level="error")
        elif not response.success:
            self.log(
                "/dashboard_client/load_program service call failed: '"
                f"{response.answer}'",
                level="error",
            )
        else:
            self.log(
                "/dashboard_client/load_program service call succeeded: '"
                f"{response.answer}'"
            )
        service_client.destroy()

    def plan(self, single_plan_parameters=None, multi_plan_parameters=None):
        self.change_state("PLANNING")
        self.log("Planning trajectory to waypoint %d" % self.i)
        self.log(f"Waypoint: {self.waypoints[self.waypoint_path[self.i]]}")

        self.planning_component.set_start_state_to_current_state()
        goal = self.waypoints[self.waypoint_path[self.i]]
        self.planning_component.set_goal_state(
            pose_stamped_msg=goal,
            pose_link=self.get_parameter("pose_link").value,
        )

        if multi_plan_parameters is not None:
            self.plan_result = self.planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            self.plan_result = self.planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            self.plan_result = self.planning_component.plan()

        self.log("Planning finished!")
        self.change_state("PLANNED")

    def execute(self):
        if self.plan_result:
            self.change_state("EXECUTING")
            self.log("Executing plan")

            robot_trajectory_msg = (
                self.plan_result.trajectory.get_robot_trajectory_msg()
            )
            self.trajectory_execution_manager.push(robot_trajectory_msg)
            self.trajectory_execution_manager.execute(self.execution_callback)
        else:
            self.log("Planning failed! Trying again...", level="error")
            self.change_state("READY")

    def execution_callback(self, response):
        if response.status == "SUCCEEDED":
            self.change_state("EXECUTED")
            self.log("Execution succeeded!")
            self.i = (self.i + 1) % len(self.waypoint_path)
            self.log("Moving on to waypoint %d" % self.i)
            self.change_state("READY")
        else:
            self.log(
                f"Execution failed with status {response.status}",
                level="warn",
            )
            self.log("Trying execution again...", level="warn")
            self.change_state("PLANNED")

    def change_state(self, state):
        self.log(f"Changing state to {state}")
        self.state = state

    def state_machine(self):
        match self.state:
            case "INITIALIZED":
                self.start_robot()
            case "READY":
                self.plan()
            case "PLANNED":
                self.execute()
            case "EXECUTING":
                pass
            case "ERROR":
                self.log(
                    "Commander entered ERROR state, restarting...",
                    level="error",
                )
                self.change_state("INITIALIZED")
            case _:
                raise ValueError(f"Invalid state: {self.state}")


def main(args=None):
    rclpy.init(args=args)
    # node = Node(
    #     "commander",
    #     automatically_declare_parameters_from_overrides=True,
    # )
    # params = node.get_parameters_by_prefix("")
    # node.get_logger().info("type(params): %s" % type(params))
    # node.get_logger().info("Commander started")

    # for name, param in params.items():
    #     node.get_logger().info("type(param): %s" % type(param))
    #     node.get_logger().info(f"{name}: {param.name}: {param.value}")

    commander = Commander()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.spin()
    # commander = Commander(moveit_py)
    commander.destroy_node()
    rclpy.shutdown()
