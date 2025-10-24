import argparse
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
from rclpy.time import Time
from tabletop_interfaces.action import FlicResponseTime

from tabletop_py.flic.client import (
    BluetoothControllerState,
    ClickType,
    FlicClient,
)
from tabletop_server.executors import AIOExecutor
from tabletop_server.nodes.base import BaseNode


class Flic(BaseNode):
    default_params = BaseNode.default_params | {
        "simulate": False,
        "simulate_min_delay": 3.0,
        "simulate_max_delay": 6.0,
        "server_ip": "localhost",
        # "server_ip": "172.17.0.1",
        "server_port": 5551,
        "max_connections": 3,
        "auto_disconnect_time": 30,
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
            "/flic/response_time",
            self.flic_response_time_callback,
            cancel_callback=self.flic_response_time_cancel_callback,
            goal_callback=self.flic_response_time_goal_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.goal_lock = asyncio.Lock()

        self.log(f"Flic initialized, simulate: {self.simulate}")

    async def flic_response_time_goal_callback(
        self, goal_request: FlicResponseTime.Goal
    ) -> GoalResponse:
        async with self.goal_lock:
            if hasattr(self, "cancel_event"):
                self.log(
                    "Cannot accept new goal, previous goal not finished",
                    severity="WARN",
                )
                return GoalResponse.REJECT

            if self.simulate:
                self.log("Accepting goal for simulated button")
                self.cancel_event = asyncio.Event()
                return GoalResponse.ACCEPT

            info = await self.flic_client.get_info()
            if goal_request.bd_addr not in info.bd_addr_of_verified_buttons:
                self.log(
                    f"Button {goal_request.bd_addr} not found",
                    severity="WARN",
                )
                return GoalResponse.REJECT
            else:
                self.log(f"Accepting goal for button {goal_request.bd_addr}")
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
            start_time = self.get_clock().now()

            # Button task to wait for the (potentially simulated) button to be pressed
            if self.simulate:
                min_delay = self.get_parameter_wrapper("simulate_min_delay")
                max_delay = self.get_parameter_wrapper("simulate_max_delay")
                button_task = asyncio.create_task(
                    asyncio.sleep(random.uniform(min_delay, max_delay))
                )
            else:
                # Connect to the button
                auto_disconnect_time = self.get_parameter_wrapper(
                    "auto_disconnect_time"
                )
                async with asyncio.timeout(1):
                    cc = await self.flic_client.connect(
                        goal_handle.request.bd_addr,
                        auto_disconnect_time=auto_disconnect_time,
                    )
                button_task = asyncio.create_task(
                    cc.wait_for_button_event(ClickType.ButtonDown)
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
                    return FlicResponseTime.Result()
                else:
                    assert button_task.done()
                    if self.simulate:
                        button_event_time = self.get_clock().now()
                    else:
                        button_event_time = button_task.result()
                    assert isinstance(button_event_time, Time)
                    response_time = button_event_time - start_time
                    result = FlicResponseTime.Result()
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
            max_connections = self.get_parameter_wrapper("max_connections")
            host = self.get_parameter_wrapper("server_ip")
            port = self.get_parameter_wrapper("server_port")
            loop = asyncio.get_event_loop()
            _, self.flic_client = await loop.create_connection(
                lambda: FlicClient(
                    loop=loop,
                    max_connection_channels=max_connections,
                    time_fn=lambda: self.get_clock().now(),
                ),
                host,
                port,
            )
            info = await self.flic_client.get_info()
            if (
                info.bluetooth_controller_state
                != BluetoothControllerState.Attached
            ):
                raise RuntimeError("Bluetooth controller not attached")
            if len(info.bd_addr_of_verified_buttons) == 0:
                raise RuntimeError("No buttons found")

            await self.flic_client.disconnect_all()

    async def wait_for_closed(self):
        """Wait for the flic client to close."""
        if self.simulate:
            await asyncio.sleep(float("inf"))
        else:
            await self.flic_client.wait_for_closed()

    def destroy_node(self):
        """Destroy the node."""
        if hasattr(self, "flic_client") and not self.flic_client.closed:
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
        executor = AIOExecutor()
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
