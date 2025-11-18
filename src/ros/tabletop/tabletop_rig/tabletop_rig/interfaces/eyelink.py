import asyncio
from collections.abc import (
    Callable,
)
from typing import Any, Coroutine, cast

from rclpy.action.client import ActionClient, ClientGoalHandle
from tabletop_interfaces.action import EyelinkSmoothPursuit

from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.nodes.base import BaseNode


class EyelinkInterface(BaseInterface):
    def __init__(self, node: BaseNode):
        """Initializes the Eyelink Interface

        Sets up MoveItPy, trajectory execution manager, robot model, and planning scene monitor.
        """
        super().__init__(node, "eyelink_interface")

        # Smooth pursuit action client
        self.eyelink_smooth_pursuit_client = ActionClient(
            self.node, EyelinkSmoothPursuit, "/eyelink/smooth_pursuit"
        )

        # Wait for action server
        self.log("Waiting for eyelink smooth pursuit server")
        self.eyelink_smooth_pursuit_client.wait_for_server()

        self.log("Eyelink interface initialized")

    def _smooth_pursuit_producer(
        self,
        feedback_msg: EyelinkSmoothPursuit.Impl.FeedbackMessage,
        queue: asyncio.Queue[bool],
        loop: asyncio.AbstractEventLoop,
    ):
        """Callback for the eyelink smooth pursuit feedback."""
        feedback = feedback_msg.feedback

        loop.call_soon_threadsafe(
            queue.put_nowait, feedback.is_smoothly_pursuing
        )
        self.log(
            f"Monkey {'' if feedback.is_smoothly_pursuing else 'not '}smoothly pursuing",
            severity="INFO",
        )

    async def _smooth_pursuit_consumer(
        self,
        queue: asyncio.Queue[bool],
        callback: Callable[[bool], None]
        | Callable[[bool], Coroutine[Any, Any, None]],
    ):
        """Consumer for the eyelink smooth pursuit queue."""
        while True:
            if asyncio.iscoroutinefunction(callback):
                await callback(await queue.get())
            else:
                callback(await queue.get())

    async def smooth_pursuit(
        self,
        callback: Callable[[bool], None]
        | Callable[[bool], Coroutine[Any, Any, None]],
    ):
        """Start the smooth pursuit action

        Receives smooth pursuit feedback and passes it to user-defined callback

        Args:
            callback: Function or coroutine function that is called when smooth
                pursuit feedback is received
        """
        queue: asyncio.Queue[bool] = asyncio.Queue()
        loop = asyncio.get_event_loop()
        goal_handle = cast(
            ClientGoalHandle,
            await self.eyelink_smooth_pursuit_client.send_goal_async(
                EyelinkSmoothPursuit.Goal(),
                feedback_callback=lambda feedback: self._smooth_pursuit_producer(
                    feedback, queue, loop
                ),
            ),
        )
        if not goal_handle.accepted:
            raise RuntimeError("goal not accepted")

        try:
            result_future = goal_handle.get_result_async()

            consumer_task = asyncio.create_task(
                self._smooth_pursuit_consumer(queue, callback)
            )
            result_future.add_done_callback(
                lambda _: loop.call_soon_threadsafe(consumer_task.cancel)
            )
            await consumer_task
        finally:
            goal_handle.cancel_goal_async()
