"""Interface for Eyelink eye tracking system.

This module provides an interface to communicate with the Eyelink eye tracker
for monitoring smooth pursuit eye movements. It uses ROS2 actions to receive
real-time feedback about whether the subject is smoothly tracking a target.

Smooth pursuit is a type of eye movement where the eyes follow a moving target.
This is commonly used in neuroscience experiments to ensure subject engagement.
"""

import asyncio
import inspect
import time
from collections.abc import (
    Callable,
)
from typing import Any, Coroutine, Optional

from rclpy.callback_groups import ReentrantCallbackGroup
from tabletop_interfaces.action import EyelinkSmoothPursuit

from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.nodes.base import AIOActionClient, BaseNode

_SAMPLE_ARRAY_TOPIC = "/eyelink/sample_array"
_ROSBAG_RECORDER_NODE_NAME = "rosbag2_recorder"


def _is_rosbag_recorder_endpoint(endpoint: Any) -> bool:
    """Distinguish rosbag from dashboards and other EyeLink subscribers."""
    node_name = str(getattr(endpoint, "node_name", "")).strip("/")
    return node_name == _ROSBAG_RECORDER_NODE_NAME or node_name.startswith(
        f"{_ROSBAG_RECORDER_NODE_NAME}_"
    )


class EyelinkInterface(BaseInterface):
    """Interface for Eyelink eye tracker smooth pursuit monitoring.

    Provides async methods to start smooth pursuit tracking and receive
    callbacks when the pursuit status changes. Uses a producer-consumer
    pattern to safely bridge ROS callbacks with asyncio.

    Attributes:
        eyelink_smooth_pursuit_client: Action client for smooth pursuit monitoring.
    """

    def __init__(
        self,
        node: BaseNode,
        name: str,
        *,
        parameter_fallback_prefix: Optional[str] = None,
    ) -> None:
        """Initialize the Eyelink interface.

        Sets up an AIOActionClient for the eyelink/smooth_pursuit action and
        waits for the eyelink node to be available.

        Args:
            node: Parent ROS2 node to create the action client on.
            name: Interface name (used for parameter lookup and logging).
            parameter_fallback_prefix: Optional fallback prefix for parameter
                lookup.

        Raises:
            RuntimeError: If the eyelink node is not available.
        """
        super().__init__(
            node, name, parameter_fallback_prefix=parameter_fallback_prefix
        )

        self.log("Waiting for eyelink node")
        if not self.node.wait_for_node_blocking("eyelink"):
            raise RuntimeError("eyelink node not available")

        # Smooth pursuit action client
        self._smooth_pursuit_client = AIOActionClient(
            self.node,
            EyelinkSmoothPursuit,
            "eyelink/smooth_pursuit",
            callback_group=ReentrantCallbackGroup(),
        )

        self.log("Eyelink interface initialized")

    async def wait_for_recorder_subscription(
        self, timeout: float = 10.0, poll_interval: float = 0.1
    ) -> None:
        """Wait until the actual rosbag recorder subscribes to sample batches.

        A raw subscription count is insufficient because the session dashboard
        also listens to this topic. Checking graph endpoint identity prevents a
        task from starting with apparently healthy EyeLink publication while
        the bag is not receiving it.
        """
        if timeout < 0:
            raise ValueError("EyeLink recorder timeout must be non-negative")
        if poll_interval <= 0:
            raise ValueError("EyeLink recorder poll interval must be positive")

        deadline = time.monotonic() + timeout
        while True:
            subscriptions = self.node.get_subscriptions_info_by_topic(
                _SAMPLE_ARRAY_TOPIC
            )
            recorder_endpoints = [
                endpoint
                for endpoint in subscriptions
                if _is_rosbag_recorder_endpoint(endpoint)
            ]
            if recorder_endpoints:
                self.log(
                    "EyeLink rosbag subscription confirmed on "
                    f"{_SAMPLE_ARRAY_TOPIC}"
                )
                return
            if time.monotonic() >= deadline:
                subscriber_names = sorted(
                    {
                        str(getattr(endpoint, "node_name", "unknown"))
                        for endpoint in subscriptions
                    }
                )
                observed = ", ".join(subscriber_names) or "none"
                raise RuntimeError(
                    "EyeLink recording preflight failed: no rosbag2_recorder "
                    f"subscription on {_SAMPLE_ARRAY_TOPIC} within "
                    f"{timeout:.1f}s (observed subscribers: {observed}). "
                    "Sync pulses and task motion were not started."
                )
            await asyncio.sleep(poll_interval)

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
            f"Monkey {'is' if feedback.is_smoothly_pursuing else 'is not'} "
            f"smoothly pursuing",
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
        if inspect.iscoroutinefunction(callback):
            while True:
                await callback(await queue.get())
        else:
            while True:
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
        loop = asyncio.get_running_loop()
        goal_handle = await self._smooth_pursuit_client.send_goal_async(
            EyelinkSmoothPursuit.Goal(),
            feedback_callback=lambda feedback: self._smooth_pursuit_producer(
                feedback, queue, loop
            ),
        )

        async with asyncio.TaskGroup() as tg:
            consumer_task = tg.create_task(
                self._smooth_pursuit_consumer(queue, callback)
            )
            await self._smooth_pursuit_client.get_result_async(goal_handle)
            consumer_task.cancel()

    def destroy_interface(self):
        """Clean up EyelinkSmoothPursuit action client and other ROS resources."""
        self.log("Destroying EyelinkInterface")

        if hasattr(self, "_smooth_pursuit_client"):
            self._smooth_pursuit_client.destroy()

        super().destroy_interface()
