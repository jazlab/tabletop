import rclpy
from geometry_msgs.msg import PoseStamped
from moveit import MoveItPy
from rclpy.node import Node


class TabletopServer(Node):
    def __init__(self):
        super().__init__("tabletop_server")
        self._declare_parameters()
        # Read parameters
        self.delay_sec = self.get_parameter("delay_sec").value
        # goal_names = self.get_parameter("goal_names").value

        # Initialize MoveItPy
        self.moveit_py = MoveItPy(self)

        # Get the planning component for your robot
        self.planning_component = self.moveit_py.get_planning_component(
            "ur_manipulator"
        )

        self.goals = []

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "base_link"
        goal.pose.position.x = 0.5
        goal.pose.position.y = 0.5
        goal.pose.position.z = 0.5
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goals.append(goal)

        goal = self.planning_component.get_current_pose().pose

        self.goals.append(goal)

        if len(self.goals) < 1:
            self.get_logger().error("No valid goal found. Exiting...")
            exit(1)

        self.timer = self.create_timer(self.delay_sec, self.timer_callback)
        self.i = 0

    def _declare_parameters(self):
        self.declare_parameter("delay_s", 6)
        self.declare_parameter("goal_names", ["pos1", "pos2"])
        self.declare_parameter("check_starting_point", False)

    def timer_callback(self):
        goal = self.goals[self.i % len(self.goals)]

        plan = self.planning_component.plan(goal)
        if plan.success:
            self.get_logger().info("Plan succeeded, executing...")
            self.planning_component.execute(plan)
            self.i += 1
        else:
            self.get_logger().error("Plan failed. Exiting...")


def main(args=None):
    rclpy.init(args=args)
    node = TabletopServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
