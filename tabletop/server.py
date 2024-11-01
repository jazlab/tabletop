import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


class Server(Node):
    def __init__(self, timer_period=0.5):
        super().__init__("server")
        self.reentrant_group = self.ReentrantCallbackGroup()
        self.teensy_sub = self.create_subscription(
            String, "teensy_sensors", self.teensy_callback, 1000
        )
        self.robot_pub = self.create_publisher(String, "robot_commands", 1000)
        self.create_timer(timer_period, self.robot_commands_callback)

    def teensy_sensors_callback(self, msg):
        pass

    def robot_commands_callback(self):
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    server = Server()
    executor = rclpy.executors.MultiThreadedExecutor()

    executor.add_node(server)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
