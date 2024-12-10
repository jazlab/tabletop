import rclpy
from geometry_msgs.msg import PoseStamped
from moveit import MoveItPy
from rclpy.logging import get_logger
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
            f"{self.trajectory_execution_manager.get_last_execution_status()}"
        )
        if (
            self.trajectory_execution_manager.get_last_execution_status()
            == "SUCCEEDED"
        ):
            self.get_logger().info("Last execution finished.")
            plan = self.planning_component.plan(goal)
            if plan.success:
                self.get_logger().info("Plan succeeded, executing...")
                self.moveit_py.execute(plan)
                self.i += 1
            else:
                self.get_logger().error("Plan failed. Exiting...")
        else:
            self.get_logger().info("Last execution not finished.")


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("commander")
    logger.info(args)
    moveit_py = MoveItPy("moveit_py")
    node = Commander(moveit_py)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
