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


def main(args=None):
    rclpy.init(args=args)
    teensy_controller = TeensyController()
    rclpy.spin(teensy_controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
