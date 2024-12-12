import rclpy
from geometry_msgs.msg import PoseStamped
from moveit import MoveItPy
from rclpy.node import Node


class Commander(Node):
    def __init__(self, moveit_py: MoveItPy):
        super().__init__("commander")
        self._declare_parameters()
        # Read parameters
        self.timer_sec = self.get_parameter("timer_sec").value
        # goal_names = self.get_parameter("goal_names").value

        # Initialize MoveItPy
        self.moveit_py = moveit_py
        # Get the planning component for your robot
        self.planning_component = self.moveit_py.get_planning_component(
            "ur_manipulator"
        )

        self.trajectory_execution_manager = (
            self.moveit_py.get_trajectory_execution_manager()
        )

        self.goals = []

        goal = PoseStamped()
        goal.header.frame_id = "base_link"
        goal.pose.position.x = 0.5
        goal.pose.position.y = 0.5
        goal.pose.position.z = 0.5
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goals.append(goal)

        goal = PoseStamped()
        goal.header.frame_id = "base_link"
        goal.pose.position.x = -0.5
        goal.pose.position.y = -0.5
        goal.pose.position.z = -0.5
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = -1.0

        self.goals.append(goal)

        goal = self.planning_component.get_current_pose().pose

        self.goals.append(goal)

        if len(self.goals) < 1:
            self.get_logger().error("No valid goal found. Exiting...")
            exit(1)

        self.timer = self.create_timer(self.timer_sec, self.timer_callback)
        self.i = 0

    def _declare_parameters(self):
        self.declare_parameter("timer_sec", 1)
        self.declare_parameter("goals", [])

    def timer_callback(self):
        goal = self.goals[self.i % len(self.goals)]

        self.get_logger().info(
            f"Last execution status: "
            f"{self.moveit_py.trajectory_execution_manager.get_last_execution_status()}"
        )
        if (
            self.moveit_py.trajectory_execution_manager.get_last_execution_status()
            == "SUCCEEDED"
        ):
            self.get_logger().info("Last execution finished.")
            plan = self.moveit_pyplanning_component.plan(goal)
            if plan.success:
                self.get_logger().info("Plan succeeded, executing...")
                self.execute(plan)
                self.i += 1
            else:
                self.get_logger().error("Plan failed. Exiting...")
        else:
            self.get_logger().info("Last execution not finished.")


def main(args=None):
    rclpy.init(args=args)
    node = Node(
        "commander",
        automatically_declare_parameters_from_overrides=True,
    )
    node.get_logger().info("Commander started")
    params = node.get_parameters_by_prefix("")
    node.get_logger().info("type(params): %s" % type(params))

    for name, param in params.items():
        node.get_logger().info("type(param): %s" % type(param))
        node.get_logger().info(f"{name}: {param.name}: {param.value}")

    moveit_py = MoveItPy("moveit_py")
    rclpy.spin(node)
    # commander = Commander(moveit_py)
    # rclpy.spin(commander)
    # commander.destroy_node()
    # rclpy.shutdown()
