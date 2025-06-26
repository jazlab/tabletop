import asyncio
import random

import debugpy
import rclpy
from rclpy.action.server import (
    ActionServer,
    CancelResponse,
    GoalResponse,
    ServerGoalHandle,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tabletop_interfaces.action import FlicResponseTime

from tabletop_server import aio_executor
from tabletop_server.flic_client import AIOFlicClient
from tabletop_server.nodes.base import BaseNode
from tabletop_server.nodes.commander import argparse


class Flic(BaseNode):
    default_params = BaseNode.default_params | {
        "simulate": False,
        "simulate_min_delay": 3.0,
        "simulate_max_delay": 6.0,
        "server_ip": "172.17.0.1",
        "server_port": 5551,
        "spin_period": 0.05,
    }

    def __init__(self):
        # Initialize base node
        super().__init__("flic")

        self.simulate = self.get_parameter_wrapper("simulate")

        # Services
        # qos = copy(QoSPresetProfiles.SERVICES_DEFAULT.value)
        # qos.liveliness = QoSLivelinessPolicy.AUTOMATIC
        # qos.liveliness_lease_duration = Duration(seconds=100)
        self.flic_response_time_server = ActionServer(
            self,
            FlicResponseTime,
            "flic/response_time",
            self.flic_response_time_callback,
            cancel_callback=self.flic_response_time_cancel_callback,
            goal_callback=self.flic_response_time_goal_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.goal_lock = asyncio.Lock()

        self.log(f"Flic initialized, simulate: {self.simulate}")

    async def flic_response_time_goal_callback(self, _) -> GoalResponse:
        async with self.goal_lock:
            if hasattr(self, "cancel_event"):
                self.log(
                    "Cannot accept new goal, previous goal not finished",
                    severity="WARN",
                )
                return GoalResponse.REJECT
            else:
                self.cancel_event = asyncio.Event()
                return GoalResponse.ACCEPT

    async def flic_response_time_cancel_callback(self, _) -> CancelResponse:
        async with self.goal_lock:
            if hasattr(self, "cancel_event"):
                self.cancel_event.set()
                return CancelResponse.ACCEPT
            else:
                self.log(
                    "Cannot cancel goal, no goal in progress",
                    severity="WARN",
                )
                return CancelResponse.REJECT

    async def flic_response_time_callback(
        self, goal_handle: ServerGoalHandle
    ) -> FlicResponseTime.Result:
        """Flic response time action callback."""
        try:
            self.log("Flic response time action started")
            result = FlicResponseTime.Result()

            # Button task to wait for the (potentially simulated) button to be pressed
            start_time = self.get_clock().now()
            if self.simulate:
                min_delay = self.get_parameter_wrapper("simulate_min_delay")
                max_delay = self.get_parameter_wrapper("simulate_max_delay")
                button_task = asyncio.create_task(
                    asyncio.sleep(random.uniform(min_delay, max_delay))
                )
            else:
                button_task = asyncio.create_task(
                    self.flic_client.wait_for_button_down()
                )

            # Cancel event task to terminate early
            cancel_task = asyncio.create_task(self.cancel_event.wait())

            # Wait for the button to be pressed or the cancel event to be set
            try:
                await asyncio.wait(
                    [button_task, cancel_task],
                    return_when=asyncio.FIRST_COMPLETED,
                )
                if cancel_task.done():
                    self.log("Flic response time action cancelled")
                    goal_handle.canceled()
                    return result
                else:
                    response_time = self.get_clock().now() - start_time
                    result.response_time = response_time.to_msg()
                    self.log(
                        f"Flic response time: {response_time.nanoseconds / 1e9:.2f} s"
                    )
                    goal_handle.succeed()
                    return result
            finally:
                cancel_task.cancel()
                button_task.cancel()
        finally:
            async with self.goal_lock:
                del self.cancel_event

    async def init_flic_client(self):
        """Start the flic client."""
        if self.simulate:
            self.log("Simulating flic client, no client started")
        else:
            self.log("Initializing flic client")
            loop = asyncio.get_event_loop()
            _, self.flic_client = await loop.create_connection(
                lambda: AIOFlicClient(loop=loop),
                self.get_parameter_wrapper("server_ip"),
                self.get_parameter_wrapper("server_port"),
            )

            self.log("Connecting to existing buttons")
            await self.flic_client.connect_existing_buttons()

            self.log(
                f"Flic client initialized with {self.flic_client.num_buttons} buttons"
            )

    async def wait_for_closed(self):
        """Wait for the flic client to close."""
        if self.simulate:
            await asyncio.sleep(float("inf"))
        else:
            await self.flic_client.wait_for_closed()

    def destroy_node(self):
        """Destroy the node."""
        if hasattr(self, "flic_client"):
            self.log("Closing flic client")
            self.flic_client.close()
        super().destroy_node()


async def main_async(args=None):
    rclpy.init(args=args)

    # Parse non-ROS arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug", action="store_true", default=False)

    non_ros_args = rclpy.utilities.remove_ros_args(args)  # type: ignore
    args, _ = parser.parse_known_args(non_ros_args)

    if args.debug:
        print("Debug mode enabled")
        debugpy.listen(1301)
        print("Waiting for debugger to attach")
        debugpy.wait_for_client()
        print("Debugger attached")

    try:
        executor = aio_executor.AIOExecutor()
        flic = Flic()
        executor.add_node(flic)

        try:
            await flic.init_flic_client()
            spin_task = asyncio.create_task(executor.spin())
            closed_task = asyncio.create_task(flic.wait_for_closed())
            try:
                await asyncio.wait(
                    [spin_task, closed_task],
                    return_when=asyncio.FIRST_COMPLETED,
                )
            finally:
                spin_task.cancel()
                closed_task.cancel()
        finally:
            print("Shutting down flic")
            flic.destroy_node()
            print("Shutting down executor")
            executor.shutdown()
    finally:
        print("Shutting down rclpy")
        rclpy.try_shutdown()  # type: ignore


def main(args=None):
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("Keyboard interrupt")
