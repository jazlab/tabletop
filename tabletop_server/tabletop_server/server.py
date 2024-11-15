import rclpy
from geometry_msgs.msg import Pose
from rclpy import Future
from rclpy.node import Node
from tabletop_msgs.srv import PlanRequest


class TabletopServer(Node):
    def __init__(self):
        super().__init__("tabletop_server")
        # Get Parameters
        self._declare_parameters()
        self.freq = self.get_parameter("freq").value
        self.goals = self.get_parameter("goals").value

        # Create Service Client
        self.moveit_client = self.create_client(
            PlanRequest, "tabletop_moveit/target_pose"
        )
        while not self.moveit_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "MoveIt interface service not available, waiting again..."
            )
        self.req = PlanRequest.Request()
        self.moveit_future = Future().set_result(None)

        # Create Goals
        goal = Pose()
        goal.position.x = 0.5
        goal.position.y = 0.5
        goal.position.z = 0.5
        goal.orientation.x = 0.0
        goal.orientation.y = 0.0
        goal.orientation.z = 0.0
        goal.orientation.w = 1.0

        self.goals.append(goal)

        goal = Pose()
        goal.position.x = -0.5
        goal.position.y = -0.5
        goal.position.z = -0.5
        goal.orientation.x = 0.0
        goal.orientation.y = 0.0
        goal.orientation.z = 0.0
        goal.orientation.w = -1.0

        self.goals.append(goal)

        if len(self.goals) < 1:
            self.get_logger().error("No valid goal found. Exiting...")
            exit(1)

        self.timer = self.create_timer(self.delay_sec, self.timer_callback)
        self.i = 0

    def _declare_parameters(self):
        self.declare_parameter("freq", 1.0)
        self.declare_parameter("goals", [])

    def timer_callback(self):
        if self.moveit_future.done():
            try:
                response = self.moveit_future.result()
            except Exception as e:
                self.get_logger().info("MoveIt service call failed %r" % (e,))
            else:
                if response is not None:
                    if response.success:
                        self.get_logger().info(
                            "Plan and execution succeeded! Moving on..."
                        )
                        self.i += 1
                    else:
                        self.get_logger().info(
                            "Plan or execution failed! Trying again..."
                        )
            goal = self.goals[self.i % len(self.goals)]
            self.req.goal = goal
            self.moveit_future = self.moveit_client.call_async(self.req)
        else:
            self.get_logger().info("Waiting for previous request to finish")


def tabletop_server(args=None):
    rclpy.init(args=args)
    node = TabletopServer()
    rclpy.spin(node)
    rclpy.shutdown()
