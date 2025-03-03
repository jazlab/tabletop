import random
import time

import rclpy
from std_msgs.msg import String
from tabletop_msgs.srv import GetFlic

from tabletop_server.nodes.base import BaseNode

DEFAULT_LOG_SEVERITY = "INFO"


class Flic(BaseNode):
    default_params = BaseNode.default_params | {
        "simulate": True,
        "simulate_delay_sec": 6.0,
    }

    def __init__(self):
        # Initialize base node

        self.log_pub = None
        super().__init__("flic")

        # Log publisher

        self.log_pub = self.create_publisher(String, "flic/log", 10)

        # Simulation parameters

        self.simulate: bool = self.get_parameter_wrapper("simulate")
        self.simulate_delay_sec: float = self.get_parameter_wrapper(
            "simulate_delay_sec"
        )

        # State variables

        self.flic_last_time_pressed_ms = int(time.time() * 1000)

        # Services

        self.get_flic_service = self.create_service(
            GetFlic,
            "flic/get_flic",
            self.get_flic_callback,
        )

        # One-shot timers for delayed state updates

        self.simulate_delay_timer = None

    def log(self, message: str, severity: str = DEFAULT_LOG_SEVERITY):
        super().log(message, severity)
        if self.log_pub is not None:
            self.log_pub.publish(String(data=message))

    def get_flic_callback(
        self, request: GetFlic.Request, response: GetFlic.Response
    ):
        # Schedule a timer to update the state pin after a random delay
        if self.simulate_delay_timer is None:
            delay = random.uniform(0.0, self.simulate_delay_sec)
            self.simulate_delay_timer = self.create_timer(
                delay, lambda: self.delay_timer_callback(delay=delay)
            )

        # Set the response message
        response.success = True
        response.last_time_pressed_ms = self.flic_last_time_pressed_ms
        response.message = (
            f"Flic last pressed at {response.last_time_pressed_ms} ms"
        )
        self.log(response.message)

        return response

    def delay_timer_callback(self, delay: float):
        # Update the pin state
        self.flic_last_time_pressed_ms = int(time.time() * 1000)

        assert self.simulate_delay_timer is not None
        self.simulate_delay_timer.cancel()
        self.simulate_delay_timer = None

        self.log(f"Flic simulated delay for {delay} seconds")


def main(args=None):
    rclpy.init(args=args)

    try:
        executor: rclpy.Executor = rclpy.executors.SingleThreadedExecutor()  # type: ignore
        flic = Flic()
        executor.add_node(flic)

        try:
            executor.spin()
        finally:
            print("Shutting down executor")
            executor.shutdown()
            print("Shutting down flic")
            flic.destroy_node()
    finally:
        print("Shutting down rclpy")
        rclpy.shutdown()
