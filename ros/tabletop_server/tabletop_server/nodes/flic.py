import asyncio
import logging
import random
import time

import rclpy
from tabletop_msgs.srv import GetFlic

from tabletop_server.executor import AIOExecutor
from tabletop_server.flic_client import AIOFlicClient
from tabletop_server.nodes.base import BaseNode

logger = logging.getLogger(__name__)

DEFAULT_LOG_SEVERITY = "INFO"


class Flic(BaseNode):
    default_params = BaseNode.default_params | {
        "simulate": False,
        "simulate_delay": 6.0,
        "num_buttons": 1,
    }

    def __init__(self, flic_client: AIOFlicClient):
        # Initialize base node
        super().__init__("flic")

        self.flic_client = flic_client

        # Simulation parameters
        self.simulate: bool = self.get_parameter_wrapper("simulate")
        self.simulate_delay: float = self.get_parameter_wrapper(
            "simulate_delay"
        )

        # State variables
        self.flic_last_time_pressed_ms = int(time.time() * 1000)

        # Services
        self.get_flic_service = self.create_service(
            GetFlic,
            "flic/get_flic",
            self.simulated_get_flic_callback
            if self.simulate
            else self.get_flic_callback,  # type: ignore
        )

        # One-shot timers for delayed state updates
        self.simulate_delay_timer = self.create_timer(
            0.0, self.simulated_delay_timer_callback, autostart=False
        )

        self.log(f"Flic initialized, simulate: {self.simulate}")

    def simulated_get_flic_callback(
        self, request: GetFlic.Request, response: GetFlic.Response
    ) -> GetFlic.Response:
        # Schedule a timer to update the state pin after a random delay
        if self.simulate_delay_timer.is_canceled():
            delay = random.uniform(0.0, self.simulate_delay)
            self.simulate_delay_timer.timer_period_ns = delay * 1e9
            self.simulate_delay_timer.reset()

        # Set the response message
        response.success = True
        response.last_time_pressed_ms = self.flic_last_time_pressed_ms
        response.message = (
            f"Flic last pressed at {response.last_time_pressed_ms} ms"
        )
        self.log(response.message)

        return response

    def simulated_delay_timer_callback(self, delay: float):
        # Update the pin state
        self.flic_last_time_pressed_ms = int(time.time() * 1000)

        assert self.simulate_delay_timer is not None
        self.simulate_delay_timer.cancel()

        self.log(f"Flic simulated delay for {delay} seconds")

    def get_flic_callback(
        self, request: GetFlic.Request, response: GetFlic.Response
    ) -> GetFlic.Response:
        # Set the response message
        response.success = True
        response.last_time_pressed_ms = int(
            self.flic_client.last_time_button_down_sec * 1000
        )
        response.message = (
            f"Flic last pressed at {response.last_time_pressed_ms} ms"
        )
        self.log(response.message)

        return response


async def wait_for_buttons(flic_client: AIOFlicClient, num_buttons: int):
    await flic_client.connect_existing_buttons()
    while flic_client.num_buttons < num_buttons:
        logger.info(f"Waiting for {flic_client.num_buttons} buttons")
        await asyncio.sleep(0.25)
    logger.info(f"Connected to {flic_client.num_buttons} buttons")


async def main_async(args=None):
    logging.basicConfig(level=logging.DEBUG)
    rclpy.init(args=args)
    loop = asyncio.get_event_loop()
    try:
        executor = AIOExecutor()
        _, flic_client = await loop.create_connection(
            lambda: AIOFlicClient(loop=loop), "172.17.0.1", 5551
        )

        # Create the flic node
        flic = Flic(flic_client)
        executor.add_node(flic)

        # Wait for num_buttons to be connected
        await wait_for_buttons(
            flic_client, flic.get_parameter_wrapper("num_buttons")
        )

        # Spin indefinitely
        try:
            await executor.spin()
        finally:
            print("Shutting down executor")
            executor.shutdown()
            print("Shutting down flic")
            flic.destroy_node()
    finally:
        print("Shutting down rclpy")
        rclpy.shutdown()


def main():
    asyncio.run(main_async())
