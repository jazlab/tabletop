import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Commander(Node):
    def __init__(self, timer_period=0.5):
        super().__init__("server")
        # Callback Groups
        # self.reentrant_group = ReentrantCallbackGroup()
        # self.mutex_group = MutuallyExclusiveCallbackGroup()
        # Publishers
        self.robot_pub = self.create_publisher(
            String, "robot_commands", 1000, callback_group=self.reentrant_group
        )

        # Timers
        self.create_timer(timer_period, self.robot_commands_callback)

        # Variables
        self.i = 0
        self.j = 0

    def robot_commands_callback(self):
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.robot_pub.publish(msg)

        if self.i % 10 == 0:
            self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


class TeensySub(Node):
    def __init__(self, timer_period=0.5):
        super().__init__("server")
        # Subscribers
        self.teensy_sub = self.create_subscription(
            String,
            "teensy_sensors",
            self.teensy_sensors_callback,
            1000,
            callback_group=self.reentrant_group,
        )

    def teensy_sensors_callback(self, msg):
        self.get_logger().info('Received message: "%s"' % msg.data)
        # Process the received message here
        # For example, you can parse the message and update some internal state
        self.j += int(msg.data)


def main(args=None):
    rclpy.init(args=args)

    server = Commander()

    rclpy.spin(server)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
