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
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            String, "teensy", self.listener_callback, 10
        )

    def teensy_callback(self, msg):
        self.


def main(args=None):
    rclpy.init(args=args)

    server = Server()

    executor = rclpy.get_global_executor()
    executor.add_node(minimal_publisher)
    executor.add_node(minimal_subscriber)
    executor.spin()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
