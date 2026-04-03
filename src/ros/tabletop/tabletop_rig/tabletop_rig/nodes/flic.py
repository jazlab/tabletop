"""ROS2 node for Flic Bluetooth button integration.

This module provides a ROS2 node that interfaces with Flic Bluetooth
buttons for measuring response times in behavioral experiments. The node
connects to a Flic server daemon and exposes button press events as a
ROS2 action.

The node can operate in two modes:
- Real mode: Connects to actual Flic buttons via the Flic server
- Simulation mode: Generates random delays for testing

Actions provided:
    flic/response_time: Wait for button press and return response time

Parameters:
    simulate: Run in simulation mode without hardware (bool).
    simulate_min_delay: Minimum simulated delay in seconds.
    simulate_max_delay: Maximum simulated delay in seconds.
    server_ip: Flic server IP address.
    server_port: Flic server port.
    max_connections: Maximum concurrent button connections.
    auto_disconnect_time: Auto-disconnect timeout in seconds.

Example:
    ros2 run tabletop_rig flic --ros-args -p simulate:=true
"""

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
from std_msgs.msg import Header
from tabletop_interfaces.action import FlicResponseTime

import tabletop_py.flic.client
from tabletop_py.flic.client import (
    BluetoothControllerState,
    ButtonConnectionChannel,
    ClickType,
    FlicClient,
    LatencyMode,
)
from tabletop_rig.executors import AIOExecutor
from tabletop_rig.nodes.base import BaseNode


class Flic(BaseNode):
    """ROS2 node for Flic Bluetooth button response time measurement.

    Provides an action server that waits for a Flic button press and
    returns the response time. Used in behavioral experiments to measure
    subject reaction times.

    Attributes:
        simulate: Whether running in simulation mode.
        flic_client: Connection to the Flic server (real mode only).
        goal_lock: Asyncio lock for managing concurrent goals.
    """

    default_params = BaseNode.default_params | {
        "simulate": False,
        "simulate_min_delay": 1.0,
        "simulate_max_delay": 3.0,
        "server_ip": "0.0.0.0",
        # "server_ip": "172.17.0.1",
        "server_port": 5551,
        "max_connection_channels": 64,
        "latency_mode": "LowLatency",
        "auto_disconnect_time": 0,
        "ignore_queued": False,
        "connect_timeout": 1,
    }

    def __init__(self):
        """Initialize the Flic node and action server."""
        super().__init__("flic")

        tabletop_py.flic.client.logger = self.get_logger().get_child("client")

        self.simulate = self.param("simulate")
        self.simulate_button_event = asyncio.Event()
        self.last_simulated_button_time: Time | None = None
        self.last_simulated_button_addr: str | None = None

        self.response_time_server = ActionServer(
            self,
            FlicResponseTime,
            "~/response_time",
            self.flic_response_time_callback,
            cancel_callback=self.flic_response_time_cancel_callback,
            goal_callback=self.flic_response_time_goal_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.button_pressed_publisher = self.create_publisher(
            Header, "~/button_pressed_time", 10
        )

        self.goal_lock = asyncio.Lock()

        self.log(f"Flic initialized, simulate: {self.simulate}")

    async def init_flic_client(self):
        """Initialize the connection to the Flic server.

        In real mode, establishes a TCP connection to the Flic server
        daemon, verifies the Bluetooth controller is attached, and
        checks that at least one button is registered.

        In simulation mode, this is a no-op.

        Raises:
            RuntimeError: If Bluetooth controller is not attached or
                no buttons are found.
        """
        if self.simulate:
            self.log("Simulating flic client, no client started")
        else:
            self.log("Initializing flic client")
            max_connection_channels = self.param("max_connection_channels")
            host = self.param("server_ip")
            port = self.param("server_port")
            loop = asyncio.get_event_loop()
            _, self.flic_client = await loop.create_connection(
                lambda: FlicClient(
                    loop=loop,
                    max_connection_channels=max_connection_channels,
                    time_fn=lambda: self.get_clock().now(),
                ),
                host,
                port,
            )

            await self.flic_client.disconnect_all()

            info = await self.flic_client.get_info()

            if (
                info.bluetooth_controller_state
                != BluetoothControllerState.Attached
            ):
                raise RuntimeError("Bluetooth controller not attached")

            if len(info.bd_addr_of_verified_buttons) == 0:
                raise RuntimeError("No buttons found")

            if len(info.bd_addr_of_verified_buttons) > max_connection_channels:
                bd_addrs = info.bd_addr_of_verified_buttons[
                    :max_connection_channels
                ]
            else:
                bd_addrs = info.bd_addr_of_verified_buttons

            self.log(f"Connecting to {len(bd_addrs)} buttons")

            for bd_addr in bd_addrs:
                cc = ButtonConnectionChannel(
                    bd_addr,
                    latency_mode=LatencyMode[self.param("latency_mode")],
                    auto_disconnect_time=self.param("auto_disconnect_time"),
                    ignore_queued=self.param("ignore_queued"),
                )

                await self.flic_client.connect(cc)

            self.log("Flic client initialized!")

    async def flic_response_time_goal_callback(
        self, goal_request: FlicResponseTime.Goal
    ) -> GoalResponse:
        """Handle incoming action goal requests.

        Validates that no other goal is in progress and that the requested
        button exists (in real mode).

        Args:
            goal_request: The goal containing the button's Bluetooth address.

        Returns:
            GoalResponse.ACCEPT if the goal can be processed,
            GoalResponse.REJECT otherwise.
        """
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
        """Handle action cancellation requests.

        Sets the cancel event if a goal is in progress.

        Args:
            _: Unused cancel request.

        Returns:
            CancelResponse.ACCEPT if cancellation is possible,
            CancelResponse.REJECT if no goal is in progress.
        """
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
        """Execute the response time measurement action.

        Connects to the specified Flic button (or simulates a delay),
        waits for a button press or cancellation, and returns the
        elapsed time.

        Args:
            goal_handle: The active action goal handle.

        Returns:
            FlicResponseTime.Result containing the measured response time,
            or empty result if cancelled.
        """
        try:
            self.log("Flic response time action started")

            # Button task to wait for the (potentially simulated) button to be pressed
            if self.simulate:
                min_delay = self.param("simulate_min_delay")
                max_delay = self.param("simulate_max_delay")
                button_task = asyncio.create_task(
                    asyncio.sleep(random.uniform(min_delay, max_delay))
                )
            else:
                # Connect to the button
                cc = await self.flic_client.get_cc_existing(
                    goal_handle.request.bd_addr
                )
                if cc is None:
                    cc = ButtonConnectionChannel(
                        goal_handle.request.bd_addr,
                        latency_mode=LatencyMode[self.param("latency_mode")],
                        auto_disconnect_time=self.param(
                            "auto_disconnect_time"
                        ),
                        ignore_queued=self.param("ignore_queued"),
                    )

                    connect_timeout = self.param("connect_timeout")
                    async with asyncio.timeout(connect_timeout):
                        await self.flic_client.connect(cc)

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
                        response_time = self.get_clock().now()
                        self.last_simulated_button_time = response_time
                        self.last_simulated_button_addr = (
                            goal_handle.request.bd_addr
                        )
                        self.simulate_button_event.set()
                    else:
                        response_time = button_task.result()
                    assert isinstance(response_time, Time)
                    result = FlicResponseTime.Result(
                        response_time=response_time.to_msg()
                    )
                    # result.response_time = response_time.to_msg() TODO
                    self.log(f"Flic response time: {response_time}")
                    goal_handle.succeed()
                    return result
            finally:
                cancel_task.cancel()
                button_task.cancel()
                # self.flic_client.force_disconnect(goal_handle.request.bd_addr)
        finally:
            async with self.goal_lock:
                del self.cancel_event

    async def wait_for_closed(self):
        """Wait for the Flic client connection to close.

        In real mode, waits until the Flic server connection is closed.
        In simulation mode, waits indefinitely.
        """
        if self.simulate:
            await asyncio.sleep(float("inf"))
        else:
            await self.flic_client.wait_for_closed()

    async def spin_button_publisher(self):
        while True:
            if self.simulate:
                await self.simulate_button_event.wait()
                assert (
                    self.last_simulated_button_addr is not None
                    and self.last_simulated_button_time is not None
                )
                time = self.last_simulated_button_time
                frame_id = self.last_simulated_button_addr
                self.last_simulated_button_time = None
                self.last_simulated_button_addr = None
                self.simulate_button_event.clear()
            else:
                cc, time = await self.flic_client.wait_for_button_event(
                    ClickType.ButtonDown
                )
                frame_id = cc.bd_addr
            assert isinstance(time, Time)
            self.button_pressed_publisher.publish(
                Header(
                    stamp=time.to_msg(), frame_id=frame_id
                )  # TODO: Change to custom message
            )

    async def spin(self):
        await self.init_flic_client()
        async with asyncio.TaskGroup() as tg:
            publisher_task = tg.create_task(self.spin_button_publisher())
            await self.wait_for_closed()
            publisher_task.cancel()

    def destroy_node(self):
        """Clean up resources and destroy the node.

        Closes the Flic client connection if active before calling
        the parent destroy_node method.
        """
        if hasattr(self, "flic_client") and not self.flic_client.closed:
            self.log("Closing flic client")
            self.flic_client.close()
        if hasattr(self, "response_time_server"):
            self.response_time_server.destroy()
        super().destroy_node()


async def main_async(args=None):
    """Async entry point for the Flic node.

    Initializes ROS2, creates the node, connects to the Flic server,
    and spins until shutdown or connection loss.

    Args:
        args: Command line arguments (passed to rclpy.init).
    """
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
            # await flic.init_flic_client()
            # async with asyncio.TaskGroup() as tg:
            #     spin_task = tg.create_task(executor.spin())
            #     closed_task = tg.create_task(flic.wait_for_closed())
            #     spin_task.add_done_callback(lambda _: closed_task.cancel)
            #     closed_task.add_done_callback(lambda _: spin_task.cancel)
            future = executor.create_task(flic.spin())
            await executor.spin_until_future_complete(future)
        finally:
            print("Shutting down flic")
            flic.destroy_node()
            print("Shutting down executor")
            executor.shutdown()
    finally:
        print("Shutting down rclpy")
        rclpy.try_shutdown()  # type: ignore


def main(args=None):
    """Entry point for the flic node."""
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("Keyboard interrupt")
