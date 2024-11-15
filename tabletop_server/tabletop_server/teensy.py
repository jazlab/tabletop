import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TeensyController(Node):
    def __init__(self):
        super().__init__("server")
        # Subscribers
        self.control_sub = self.create_subscription(
            String,
            "teensy_control",
            self.control_callback,
            10,
        )

    def control_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        self.get_logger().info("Sending control to motors...")


class TeensySensor(Node):
    def __init__(self):
        super().__init__("server")
        # Publishers
        self.sensor_pub = self.create_publisher(String, "serial_data", 1000)

        # Timers
        self.create_timer(10, self.read_sensor_callback)

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
