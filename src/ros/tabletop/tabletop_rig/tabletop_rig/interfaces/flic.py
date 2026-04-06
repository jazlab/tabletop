"""Interface for Flic button response time measurement.

This module provides an interface to the Flic smart button system for measuring
reaction times in experiments. It communicates with the Flic node via ROS2
action to wait for button presses and measure the response latency.

Flic buttons are Bluetooth-enabled buttons commonly used in behavioral
experiments to capture precise response timing.
"""

import asyncio
from typing import Optional

from tabletop_interfaces.action import FlicResponseTime

from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.nodes.base import (
    AIOActionClient,
    BaseNode,
)
from tabletop_rig.utils.ros import seconds_from_ros_time


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
        super().__init__("flic_interface", node)

        self.log("Waiting for flic node")
        if not self.node.wait_for_node(
            "flic",
            timeout=self.node.param("wait_for_node_timeout"),
        ):
            raise RuntimeError("flic node not available")

        self._response_time_client = AIOActionClient(
            self.node,
            FlicResponseTime,
            "flic/response_time",
        )

        self.log("Flic interface initialized")

    async def response_time(
        self, bd_addr: str, timeout: Optional[float] = None
    ) -> float | None:
        """Wait for a Flic button press and return the response time.

        Sends a goal to the Flic action server to connect to the button,
        then waits for the subject to press the button. Returns the ROS
        timestamp that the button was pressed (converted to seconds) or
        None if the timeout is reached.

        Args:
            bd_addr: Bluetooth device address of the Flic button to monitor.
            timeout: Maximum time to wait for a button press in seconds.
                If None, waits indefinitely.

        Returns:
            ROS timestamp (converted to seconds) that button was pressed
            or None if the timeout was reached before a press.

        Raises:
            RuntimeError: If the goal is rejected by the action server or
                the action fails.
        """
        try:
            async with asyncio.timeout(timeout):
                goal_handle = await self._response_time_client.send_goal_async(
                    FlicResponseTime.Goal(bd_addr=bd_addr)
                )

                result: FlicResponseTime.Result = (
                    await self._response_time_client.get_result_async(
                        goal_handle
                    )
                )
        except TimeoutError:
            return None

        return seconds_from_ros_time(result.response_time)

    def destroy_interface(self):
        """Clean up FlicResponseTime action client"""
        self.log("Destroying FlicInterface")
        if hasattr(self, "_response_time_client"):
            self._response_time_client.destroy()
        super().destroy_interface()
