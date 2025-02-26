import time

import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from tabletop_msgs.msg import TeensySensor
from tabletop_msgs.srv import GetFloat

from tabletop_server.nodes import BaseNode


class SensorServer(BaseNode):
    def __init__(self):
        super().__init__("sensor_server")

        self.mutex_group = MutuallyExclusiveCallbackGroup()

        self.sensor_sub = self.create_subscription(
            msg_type=TeensySensor,
            topic="/teensy/sensors",
            callback=self.sensor_callback,
            qos_profile=1000,
            callback_group=self.mutex_group,
        )

        self.time_since_fixation_on_service = self.create_service(
            srv_type=GetFloat,
            srv_name="/sensor_server/time_since_fixation_on",
            callback=self.time_since_fixation_on_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.time_since_fixation_off_service = self.create_service(
            srv_type=GetFloat,
            srv_name="/sensor_server/time_since_fixation_off",
            callback=self.time_since_fixation_off_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.last_sensor_data = TeensySensor()
        self.last_fixation_on_time = time.time()
        self.last_fixation_off_time = time.time()

    def sensor_callback(self, msg: TeensySensor):
        self.last_sensor_data = msg
        if msg.fixation_button_state:
            self.last_fixation_on_time = time.time()
        else:
            self.last_fixation_off_time = time.time()

    def time_since_fixation_on_callback(
        self, request: GetFloat.Request, response: GetFloat.Response
    ):
        """
        Check if the fixation button is pressed.
        """
        response.success = True
        response.data = time.time() - self.last_fixation_on_time
        response.message = (
            f"Time since fixation on: {response.data:.2f} seconds"
        )
        return response

    def time_since_fixation_off_callback(
        self, request: GetFloat.Request, response: GetFloat.Response
    ):
        response.success = True
        response.data = time.time() - self.last_fixation_off_time
        response.message = (
            f"Time since fixation off: {response.data:.2f} seconds"
        )
        return response


def main(args=None):
    rclpy.init(args=args)

    try:
        executor = MultiThreadedExecutor()
        sensor_server = SensorServer()
        executor.add_node(sensor_server)

        try:
            executor.spin()
        finally:
            print("Shutting down executor")
            executor.shutdown()
            print("Shutting down sensor server")
            sensor_server.destroy_node()
    finally:
        print("Shutting down rclpy")
        rclpy.shutdown()
