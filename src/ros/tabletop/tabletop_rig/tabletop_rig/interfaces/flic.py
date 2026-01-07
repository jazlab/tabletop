"""Interface for Flic button response time measurement.

This module provides an interface to the Flic smart button system for measuring
reaction times in experiments. It communicates with the Flic node via ROS2
action to wait for button presses and measure the response latency.

Flic buttons are Bluetooth-enabled buttons commonly used in behavioral
experiments to capture precise response timing.
"""

import asyncio
from typing import Optional, cast

from action_msgs.msg import GoalStatus
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.duration import Duration
from tabletop_interfaces.action import FlicResponseTime

from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.nodes.base import BaseNode


class FlicInterface(BaseInterface):
    """Interface for measuring response times via Flic button presses.

    This interface provides async methods to wait for Flic button events
    and measure the response time from a trigger to the button press.

    Attributes:
        response_time_client: Action client for the FlicResponseTime action.
    """

    def __init__(self, node: BaseNode) -> None:
        """Initialize the Flic interface.

        Sets up the action client for communicating with the Flic node
        and waits for the action server to become available.

        Args:
            node: Parent ROS2 node to create the action client on.
        """
        super().__init__(node, "flic_interface")

        # Flic response time action client
        self.response_time_client = ActionClient(
            self.node,
            FlicResponseTime,
            "/flic/response_time",
        )

        # Wait for action server
        self.log("Waiting for response time server")
        self.response_time_client.wait_for_server()

        self.log("Flic interface initialized")

    async def response_time(
        self, bd_addr: str, timeout: Optional[float] = None
    ) -> float | None:
        """Wait for a Flic button press and measure response time.

        Sends a goal to the Flic action server to start timing, then waits
        for the subject to press the button. Returns the measured response
        time or None if the timeout is reached.

        Args:
            bd_addr: Bluetooth device address of the Flic button to monitor.
            timeout: Maximum time to wait for a button press in seconds.
                If None, waits indefinitely.

        Returns:
            Response time in seconds from action goal acceptance to button
            press, or None if the timeout was reached before a press.

        Raises:
            RuntimeError: If the goal is rejected by the action server or
                the action fails.
        """

        try:
            async with asyncio.timeout(timeout):
                goal_handle = cast(
                    ClientGoalHandle,
                    await self.response_time_client.send_goal_async(
                        FlicResponseTime.Goal(bd_addr=bd_addr)
                    ),
                )
                if not goal_handle.accepted:
                    raise RuntimeError("Flic goal not accepted")

                try:
                    response = await goal_handle.get_result_async()
                except asyncio.CancelledError:
                    goal_handle.cancel_goal_async()
                    raise
        except TimeoutError:
            return None

        if response.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError("Flic goal failed")

        response_time = Duration.from_msg(response.result.response_time)
        return response_time.nanoseconds / 1e9
