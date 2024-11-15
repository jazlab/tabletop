import random

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String


class TeensyController(Node):
    def __init__(self):
        super().__init__("server")
        # Callback Groups
        self.reentrant_group = ReentrantCallbackGroup()

        # Subscribers
        self.control_sub = self.create_subscription(
            String,
            "teensy_control",
            self.control_callback,
            10,
            callback_group=self.reentrant_group,
        )

    def control_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        self.get_logger().info("Sending control to motors...")


class TeensySensor(Node):
    def __init__(self):
        super().__init__("server")
        # Callback Groups
        self.reentrant_group = ReentrantCallbackGroup()

        # Publishers
        self.sensor_pub = self.create_publisher(
            String, "serial_data", 1000, callback_group=self.reentrant_group
        )

        # Timers
        self.create_timer(200, self.read_sensor_callback)

    def read_sensor_callback(self):
        # Simulate reading from serial buffers
        serial_data = "Serial Data: %d" % random.randint(0, 100)
        msg = String()
        msg.data = serial_data

        # Publish the data
        self.sensor_pub.publish(msg)


def teensy_controller(args=None):
    rclpy.init(args=args)
    teensy_controller = TeensyController()
    rclpy.spin(teensy_controller)
    rclpy.shutdown()


def teensy_sensor(args=None):
    rclpy.init(args=args)
    teensy_sensors = TeensySensor()
    rclpy.spin(teensy_sensors)
    rclpy.shutdown()
