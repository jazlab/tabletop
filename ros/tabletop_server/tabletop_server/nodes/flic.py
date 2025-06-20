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
    }

    def __init__(self):
        # Initialize base node
        super().__init__("flic")

        # Simulation parameters
        self.simulate: bool = self.get_parameter_wrapper("simulate")
        self.simulate_max_delay: float = self.get_parameter_wrapper(
            "simulate_max_delay"
        )

        # Services
        self.get_flic_service = self.create_service(
            GetFlic,
            "flic/get_flic",
            self.simulated_get_flic_callback
            if self.simulate
            else self.get_flic_callback,  # type: ignore
        )

        # Simulated button press timer
        if self.simulate:
            self.simulated_last_time_pressed_ms = int(self.time() * 1000)
            self.simulate_delay_timer = self.create_timer(
                0.0, self.simulated_delay_timer_callback, autostart=False
            )

        self.log(f"Flic initialized, simulate: {self.simulate}")

    async def get_flic_callback(
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
        response.last_time_pressed_ms = self.simulated_last_time_pressed_ms
        response.message = (
            f"Flic last pressed at {response.last_time_pressed_ms} ms"
        )
        self.log(response.message)

        return response

    def simulated_delay_timer_callback(self):
        self.simulated_last_time_pressed_ms = int(self.time() * 1000)
        self.simulate_delay_timer.cancel()
        self.log(f"Flic simulated delay for {self.simulate_cur_delay} seconds")

    async def init_flic_client(self):
        """Start the flic client."""
        if self.simulate:
            self.log("Simulating flic client, no client started")
        else:
            self.log("Initializing flic client")
            loop = asyncio.get_event_loop()
            _, self.flic_client = await loop.create_connection(
                lambda: AIOFlicClient(loop=loop), "172.17.0.1", 5551
            )

            self.log("Connecting to existing buttons")
            await self.flic_client.connect_existing_buttons()

            self.log(
                f"Flic client initialized with {self.flic_client.num_buttons} buttons"
            )

    async def wait_for_closed(self):
        """Wait for the flic client to close."""
        await self.flic_client.wait_for_closed()

    def destroy_node(self):
        """Destroy the node."""
        if hasattr(self, "flic_client"):
            self.log("Closing flic client")
            self.flic_client.close()
        super().destroy_node()


async def main_async(args=None):
    rclpy.init(args=args)
    try:
        executor = AIOExecutor()
        flic = Flic()
        executor.add_node(flic)

        try:
            await flic.init_flic_client()
            await asyncio.gather(executor.spin(), flic.wait_for_closed())
        finally:
            print("Shutting down flic")
            flic.destroy_node()
            print("Shutting down executor")
            executor.shutdown()
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        print("Shutting down rclpy")
        rclpy.try_shutdown()  # type: ignore


def main():
    asyncio.run(main_async())
