import random

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import SetBool
from tabletop_msgs.msg import TeensySensor
from tabletop_msgs.srv import SetUint32

pin_states = [bool(random.getrandbits(1)) for _ in range(55)]


def digitalWrite(pin: int, value: bool):
    pin_states[pin] = value


def digitalRead(pin: int) -> bool:
    return pin_states[pin]


class MockTeensy(Node):
    def __init__(self):
        super().__init__("server")

        self.reentrant_cg = ReentrantCallbackGroup()

        self.pin = {
            "arm_door_control": 2,
            "smartglass_control": 3,
            "reward_control": 4,
            "arm_door_state": 5,
            "smartglass_state": 6,
            "reward_state": 7,
            "hand_fixation_state": 8,
        }

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
            SetUint32,
            "reward",
            self.reward_callback,
            callback_group=self.reentrant_cg,
        )

        self.sensor_pub = self.create_publisher(TeensySensor, "sensors", 1000)

    def arm_door_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ):
        pin = self.pin["arm_door_control"]
        digitalWrite(pin, request.data)

        if response.success:
            self.get_logger().info("Arm door opened")
            response.message = "Arm door opened"
        else:
            self.get_logger().info("Arm door failed to open")
            response.message = "Arm door failed to open"
        return response

    def smartglass_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ):
        return self.random_response_success(request, response)

    def reward_callback(
        self, request: SetUint32.Request, response: SetUint32.Response
    ):
        return self.random_response_success(request, response)


def main(args=None):
    rclpy.init(args=args)
    mock_teensy = MockTeensy()
    rclpy.spin(mock_teensy)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
