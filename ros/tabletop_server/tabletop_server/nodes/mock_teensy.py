import random

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import SetBool
from tabletop_msgs.msg import TeensySensor
from tabletop_msgs.srv import SetFloat


class MockTeensy(Node):
    def __init__(self):
        super().__init__("server")

        self.reentrant_cg = ReentrantCallbackGroup()

        self.arm_door_service = self.create_service(
            SetBool,
            "arm_door",
            self.arm_door_callback,
            callback_group=self.reentrant_cg,
        )

        self.smartglass_service = self.create_service(
            SetBool,
            "smartglass",
            self.smartglass_callback,
            callback_group=self.reentrant_cg,
        )

        self.reward_service = self.create_service(
            SetFloat,
            "reward",
            self.reward_callback,
            callback_group=self.reentrant_cg,
        )

        self.hand_fixation_service = self.create_service(
            SetBool,
            "hand_fixation",
            self.hand_fixation_callback,
            callback_group=self.reentrant_cg,
        )

        self.sensor_pub = self.create_publisher(TeensySensor, "sensors", 1000)

    def arm_door_callback(self, request, response):
        self.get_logger().info("Received arm door request")

        response.success = random.random() < 0.9
        if response.success:
            self.get_logger().info("Arm door opened")
            response.message = "Arm door opened"
        else:
            self.get_logger().info("Arm door failed to open")
            response.message = "Arm door failed to open"

        return response

    def smartglass_callback(self, request, response):
        self.get_logger().info("Received smartglass request")

        response.success = random.random() < 0.9
        if response.success:
            self.get_logger().info("Smartglass opened")
            response.message = "Smartglass opened"
        else:
            self.get_logger().info("Smartglass failed to open")
            response.message = "Smartglass failed to open"
        return response


def main(args=None):
    rclpy.init(args=args)
    mock_teensy = MockTeensy()
    rclpy.spin(mock_teensy)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
