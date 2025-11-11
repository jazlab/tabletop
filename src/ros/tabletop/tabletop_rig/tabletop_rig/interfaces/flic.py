import asyncio
from typing import Optional, cast

from action_msgs.msg import GoalStatus
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.duration import Duration
from tabletop_interfaces.action import FlicResponseTime

from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.nodes.base import BaseNode


class FlicInterface(BaseInterface):
    ###########################################################################
    ########## Initialization #################################################
    ###########################################################################

    def __init__(self, node: BaseNode):
        """Initializes the Eyelink Interface

        Sets up MoveItPy, trajectory execution manager, robot model, and planning scene monitor.
        """
        super().__init__(node, "flic_interface")

        # Flic response time action client
        self.flic_response_time_client = ActionClient(
            self,
            FlicResponseTime,
            "/flic/response_time",
        )

        # Wait for action server
        self.log("Waiting for flic response time server")
        self.flic_response_time_client.wait_for_server()

        self.log("Flic interface initialized")

    async def flic_response_time(
        self, bd_addr: str, timeout: Optional[float] = None
    ) -> float | None:
        """Wait for flic button press, then return response time, or None if timeout is reached."""

        try:
            async with asyncio.timeout(timeout):
                goal_handle = cast(
                    ClientGoalHandle,
                    await self.flic_response_time_client.send_goal_async(
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
