import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TeensySensor(Node):
    def __init__(self):
        super().__init__("server")
        # Publishers
        self.sensor_pub = self.create_publisher(String, "sensors", 1000)

        # Timers
        self.create_timer(10, self.read_sensor_callback)

    def read_sensor_callback(self):
        # Simulate reading from serial buffers
        serial_data = "Serial Data: %d" % random.randint(0, 100)
        msg = String()
        msg.data = serial_data

        # Publish the data
        self.sensor_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    teensy_sensor = TeensySensor()
    rclpy.spin(teensy_sensor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
