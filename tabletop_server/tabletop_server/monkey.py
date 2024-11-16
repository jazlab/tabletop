import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class MonkeyNode(Node):
    def __init__(self):
        super().__init__("monkey_node")
        self.subscription = self.create_subscription(
            Bool, "arm_door_state", self.arm_door_state_callback, 10
        )
        self.monkey_reward_publisher = self.create_publisher(
            Bool, "monkey_reward", 10
        )
        self.monkey_fixation_publisher = self.create_publisher(
            Bool, "monkey_fixation", 10
        )
        self.arm_door_state = False

    def arm_door_state_callback(self, msg):
        self.arm_door_state = msg.data
        if self.arm_door_state:
            self.get_logger().info(
                "Arm door state is True, starting reward sequence."
            )
            self.publish_monkey_reward()

    def publish_monkey_reward(self):
        time.sleep(random.uniform(1, 5))  # Random delay between 1 to 5 seconds
        self.monkey_reward_publisher.publish(Bool(data=True))
        self.get_logger().info("Published True to monkey_reward.")
        self.publish_monkey_fixation()

    def publish_monkey_fixation(self):
        time.sleep(random.uniform(1, 5))  # Random delay between 1 to 5 seconds
        self.monkey_fixation_publisher.publish(Bool(data=False))
        self.get_logger().info("Published False to monkey_fixation.")


def main(args=None):
    rclpy.init(args=args)
    monkey_node = MonkeyNode()
    rclpy.spin(monkey_node)
    monkey_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
