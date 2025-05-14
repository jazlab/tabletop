import asyncio
import random

import rclpy
from tabletop_msgs.srv import GetFlic

from tabletop_server.executor import AIOExecutor
from tabletop_server.flic_client import AIOFlicClient
from tabletop_server.nodes.base import BaseNode


class Flic(BaseNode):
    default_params = BaseNode.default_params | {
        "simulate": False,
        "simulate_max_delay": 6.0,
        "num_buttons": 1,
    }

    def __init__(self):
        # Initialize base node
        super().__init__("flic")

        # Simulation parameters
        self.simulate: bool = self.get_parameter_wrapper("simulate")
        self.simulate_max_delay: float = self.get_parameter_wrapper(
            "simulate_max_delay"
        )

        # State variables
        self.flic_last_time_pressed_ms = int(self.time() * 1000)

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

    def get_flic_callback(
        self, request: GetFlic.Request, response: GetFlic.Response
    ) -> GetFlic.Response:
        # Set the response message
        response.success = True
        response.last_time_pressed_ms = int(
            self.flic_client.last_time_button_down_sec * 1000  # type: ignore
        )
        response.message = (
            f"Flic last pressed at {response.last_time_pressed_ms} ms"
        )
        self.log(response.message)

        return response

    def simulated_get_flic_callback(
        self, request: GetFlic.Request, response: GetFlic.Response
    ) -> GetFlic.Response:
        # Schedule a timer to update the state pin after a random delay
        if self.simulate_delay_timer.is_canceled():
            self.simulate_cur_delay = random.uniform(
                0.0, self.simulate_max_delay
            )
            self.simulate_delay_timer.timer_period_ns = (
                self.simulate_cur_delay * 1e9
            )
            self.simulate_delay_timer.reset()

        # Set the response message
        response.success = True
        response.last_time_pressed_ms = self.flic_last_time_pressed_ms
        response.message = (
            f"Flic last pressed at {response.last_time_pressed_ms} ms"
        )
        self.log(response.message)

        return response

    def simulated_delay_timer_callback(self):
        self.flic_last_time_pressed_ms = int(self.time() * 1000)
        self.simulate_delay_timer.cancel()
        self.log(f"Flic simulated delay for {self.simulate_cur_delay} seconds")

    async def start_flic_client(self):
        """Start the flic client."""
        if self.simulate:
            self.log("Simulating flic client, no client started")
            self.flic_client = None
        else:
            self.log("Starting flic client")
            loop = asyncio.get_event_loop()
            _, self.flic_client = await loop.create_connection(
                lambda: AIOFlicClient(loop=loop), "172.17.0.1", 5551
            )
            await self.flic_client.connect_existing_buttons()

    async def wait_for_buttons(self):
        """Wait for the specified number of buttons to be connected."""
        if self.simulate:
            self.log("Simulating flic client, no buttons to wait for")
        else:
            assert self.flic_client is not None
            await self.flic_client.connect_existing_buttons()
            num_buttons = self.get_parameter_wrapper("num_buttons")
            while self.flic_client.num_buttons < num_buttons:
                self.log(f"Waiting for {self.flic_client.num_buttons} buttons")
                await asyncio.sleep(0.25)
            self.log(f"Connected to {self.flic_client.num_buttons} buttons")


async def main_async(args=None):
    rclpy.init(args=args)
    try:
        executor = AIOExecutor()
        flic = Flic()
        executor.add_node(flic)

        try:
            await flic.start_flic_client()
            await flic.wait_for_buttons()
            await executor.spin()
        finally:
            print("Shutting down flic")
            flic.destroy_node()
            print("Shutting down executor")
            executor.shutdown()
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    except SystemExit:
        print("System exit")
    finally:
        print("Shutting down rclpy")
        rclpy.try_shutdown()


def main():
    asyncio.run(main_async())
