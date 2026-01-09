"""Interface for Eyelink eye tracking system.

This module provides an interface to communicate with the Eyelink eye tracker
for monitoring smooth pursuit eye movements. It uses ROS2 actions to receive
real-time feedback about whether the subject is smoothly tracking a target.

Smooth pursuit is a type of eye movement where the eyes follow a moving target.
This is commonly used in neuroscience experiments to ensure subject engagement.
"""

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
    """Interface for Eyelink eye tracker smooth pursuit monitoring.

    Provides async methods to start smooth pursuit tracking and receive
    callbacks when the pursuit status changes. Uses a producer-consumer
    pattern to safely bridge ROS callbacks with asyncio.

    Attributes:
        eyelink_smooth_pursuit_client: Action client for smooth pursuit monitoring.
    """

    def __init__(self, node: BaseNode) -> None:
        """Initialize the Eyelink interface.

        Sets up the action client for smooth pursuit monitoring and waits
        for the Eyelink action server to become available.

        Args:
            node: Parent ROS2 node to create the action client on.
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
    ) -> None:
        """Handle incoming smooth pursuit feedback from the action server.

        This callback runs in the ROS executor thread and safely passes
        feedback to the asyncio event loop via a thread-safe queue.

        Args:
            feedback_msg: The feedback message from the action server.
            queue: Asyncio queue to pass pursuit status to the consumer.
            loop: The asyncio event loop for thread-safe queue operations.
        """
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
    ) -> None:
        """Process pursuit status updates and invoke the user callback.

        Runs continuously, consuming items from the queue and passing them
        to the user-provided callback. Supports both sync and async callbacks.

        Args:
            queue: Queue of pursuit status updates from the producer.
            callback: User callback to invoke with each status update.
        """
        while True:
            if asyncio.iscoroutinefunction(callback):
                await callback(await queue.get())
            else:
                callback(await queue.get())

    async def smooth_pursuit(
        self,
        callback: Callable[[bool], None]
        | Callable[[bool], Coroutine[Any, Any, None]],
    ) -> None:
        """Monitor smooth pursuit and invoke callback on status changes.

        Starts the smooth pursuit action and sets up a producer-consumer
        pipeline to deliver pursuit status updates to the provided callback.
        Runs until cancelled or the action completes.

        Args:
            callback: Function or coroutine called with True when the subject
                is smoothly pursuing and False when not. Called each time
                the pursuit status changes.

        Raises:
            RuntimeError: If the action goal is rejected by the server.
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
