import random

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String


class TabletopCore(Node):
    def __init__(self):
        super().__init__("ur5e_controller")
        self.declare_parameter("waypoints_file", "waypoints.yaml")
        waypoints_file = self.get_parameter("waypoints_file").value

        with open(waypoints_file, "r") as file:
            self.waypoints = yaml.safe_load(file)

        self.action_client = ActionClient(self, MoveGroup, "move_group")
        self.current_waypoint_index = 0

        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints have been processed.")
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        pose = Pose()
        pose.position.x = waypoint["position"]["x"]
        pose.position.y = waypoint["position"]["y"]
        pose.position.z = waypoint["position"]["z"]
        pose.orientation.x = waypoint["orientation"]["x"]
        pose.orientation.y = waypoint["orientation"]["y"]
        pose.orientation.z = waypoint["orientation"]["z"]
        pose.orientation.w = waypoint["orientation"]["w"]

        goal_msg = None
        goal_msg.request.workspace_parameters.header.frame_id = "world"
        goal_msg.request.workspace_parameters.header.stamp = (
            self.get_clock().now().to_msg()
        )
        goal_msg.request.goal_constraints.append(pose)

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == GoalStatus.SUCCEEDED:
            self.get_logger().info("Waypoint reached")
            self.current_waypoint_index += 1
            self.send_next_waypoint()
        else:
            self.get_logger().info("Failed to reach waypoint")


class Teensy(Node):
    def __init__(self, timer_period=0.5):
        super().__init__("server")
        # Callback Groups
        self.reentrant_group = ReentrantCallbackGroup()

        # Publishers
        self.sensor_pub = self.create_publisher(
            String, "serial_data", 1000, callback_group=self.reentrant_group
        )

        # Subscribers
        self.control_sub = self.create_subscription(
            String,
            "control",
            self.control_callback,
            10,
            callback_group=self.reentrant_group,
        )

        # Timers
        self.create_timer(timer_period, self.read_sensor_callback)

    def read_sensor_callback(self):
        # Simulate reading from serial buffers
        serial_data = "Serial Data: %d" % random.randint(0, 100)
        msg = String()
        msg.data = serial_data

        # Publish the data
        self.sensor_pub.publish(msg)

        # Log the data
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def control_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)


def tabletop_server(args=None):
    rclpy.init(args=args)

    server = TabletopCore()

    rclpy.spin(server)

    rclpy.shutdown()


def teensy(args=None):
    rclpy.init(args=args)

    server = Teensy()

    rclpy.spin(server)

    rclpy.shutdown()
