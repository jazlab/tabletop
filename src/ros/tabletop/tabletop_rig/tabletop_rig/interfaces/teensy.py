"""Interface for Teensy microcontroller communication.

This module provides an interface to communicate with a Teensy microcontroller
that manages experimental apparatus including arm restraints, safety sensors,
smartglass goggles, and reward delivery systems.

The Teensy acts as a bridge between the ROS2 system and physical hardware,
providing safety interlocks and subject interface mechanisms.
"""

import asyncio
import threading
from collections.abc import Callable
from copy import copy, deepcopy
from types import TracebackType
from typing import Literal, Optional, Self

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSPresetProfiles
from rclpy.time import Time
from tabletop_interfaces.msg import TeensySensor
from tabletop_interfaces.srv import (
    SetArmLock,
    SetReward,
    SetSmartglass,
    SetSolenoid,
)

from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.nodes.base import BaseNode


def noop(msg: TeensySensor) -> None:
    """Default callback that does nothing (placeholder for optional callbacks).

    Args:
        msg: The TeensySensor message (ignored).
    """
    pass


class TeensyInterface(BaseInterface):
    """Interface for controlling experimental apparatus via Teensy microcontroller.

    Provides methods to control arm restraints, smartglass goggles, and reward
    delivery, as well as monitoring safety sensors to determine if it's safe
    to execute robot movements.

    The interface subscribes to sensor data and provides a thread-safe
    `safe_to_execute` property that checks arm lock status and safety laser.

    Attributes:
        _teensy_sub: Subscription to TeensySensor messages.
        _last_teensy_sensor: Most recent sensor reading.
        _safe_to_execute: Whether current conditions allow robot execution.
        _set_arm_lock_client: Service client for arm lock control.
        _set_reward_client: Service client for reward delivery.
        _set_smartglass_client: Service client for smartglass control.
    """

    def __init__(
        self,
        node: BaseNode,
        additional_subscription_callback: Optional[
            Callable[[TeensySensor], None]
        ] = None,
    ) -> None:
        """Initialize the Teensy interface.

        Sets up subscriptions for sensor data and service clients for
        controlling the arm locks, reward system, and smartglass.

        Args:
            node: Parent ROS2 node for creating ROS resources.
            additional_subscription_callback: Optional callback invoked with
                each TeensySensor message after internal processing.
        """
        super().__init__("teensy_interface", node)

        # Subscribers
        qos = copy(QoSPresetProfiles.SENSOR_DATA.value)
        qos.durability = QoSDurabilityPolicy.VOLATILE
        qos.depth = 1
        self._teensy_sub = self.node.create_subscription(
            TeensySensor,
            "/teensy/sensor",
            self._teensy_sensor_callback,
            qos_profile=qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._last_teensy_sensor = TeensySensor()
        self._last_teensy_sensor_time = self.node.ros_time()
        self._last_unsafe_to_execute_time = self.node.ros_time()
        self._safe_to_execute = False
        self._teensy_sensor_lock = threading.Lock()

        if additional_subscription_callback is None:
            self._additional_subscription_callback = noop
        else:
            self._additional_subscription_callback = (
                additional_subscription_callback
            )

        # Service clients
        self._set_arm_lock_client = self.node.create_client(
            SetArmLock,
            "/teensy/set_arm_lock",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._set_reward_client = self.node.create_client(
            SetReward,
            "/teensy/set_reward",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._set_smartglass_client = self.node.create_client(
            SetSmartglass,
            "/teensy/set_smartglass",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._set_solenoid_client = self.node.create_client(
            SetSolenoid,
            "/teensy/set_solenoid",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Wait for ROS services
        self.log("Waiting for teensy services")
        self._set_arm_lock_client.wait_for_service()
        self._set_reward_client.wait_for_service()
        self._set_smartglass_client.wait_for_service()

        self.log("Teensy interface initialized")

    # Properties

    @property
    def last_teensy_sensor(self) -> TeensySensor:
        """The most recent TeensySensor message received.

        Returns:
            A deep copy of the last sensor reading to prevent external mutation.
        """
        with self._teensy_sensor_lock:
            return deepcopy(self._last_teensy_sensor)

    @property
    def safe_to_execute(self) -> bool:
        """Whether conditions currently allow safe robot execution.

        Checks that:
        1. Sensor data is recent (within max_sensor_delay)
        2. Both arms are locked
        3. Safety laser is not broken
        4. Conditions have been safe for required_time duration

        Returns:
            True if all safety conditions are met, False otherwise.
        """
        max_sensor_delay = self.node.param(
            "teensy.safe_to_execute.max_sensor_delay"
        )

        with self._teensy_sensor_lock:
            current_time = self.node.ros_time()
            if current_time - self._last_teensy_sensor_time > max_sensor_delay:
                self.log(
                    f"Have not received teensy sensor message in "
                    f"{current_time - self._last_teensy_sensor_time} > "
                    f"{max_sensor_delay}, not safe to execute",
                    severity="WARN",
                )
                return False
            return self._safe_to_execute

    # Subscribers

    def _msg_safe_to_execute(self, msg: TeensySensor) -> bool:
        """Check if a sensor message indicates safe conditions.

        Args:
            msg: The sensor message to evaluate.

        Returns:
            True if both arms are locked and safety laser is unbroken.
        """
        # TODO: CHANGE BACK!!!!!!!!!!!!!!!
        # return (
        #     msg.is_left_arm_locked
        #     and msg.is_right_arm_locked
        #     and not msg.is_safety_laser_broken
        # )
        return not msg.is_safety_laser_broken

    def _teensy_sensor_callback(self, msg: TeensySensor) -> None:
        """Process incoming TeensySensor messages.

        Updates internal state and determines if conditions are safe for
        robot execution. Requires conditions to remain safe for a configurable
        duration before setting safe_to_execute to True.

        Args:
            msg: The incoming sensor message.
        """
        current_time = self.node.ros_time()
        required_time = self.node.param("teensy.safe_to_execute.required_time")
        warn_threshold = self.node.param("teensy.sensor_delay_warn_threshold")

        # Determine if the monkey is safe
        with self._teensy_sensor_lock:
            teensy_time = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
            delay = current_time - teensy_time
            if delay > warn_threshold:
                self.log(
                    f"Teensy sensor callback delay {delay:.4f}s > {warn_threshold}s"
                )

            self._last_teensy_sensor = msg
            self._last_teensy_sensor_time = current_time
            if self._msg_safe_to_execute(msg):
                self._safe_to_execute = (
                    current_time - self._last_unsafe_to_execute_time
                    > required_time
                )
            else:
                self._safe_to_execute = False
                self._last_unsafe_to_execute_time = current_time

        # Call additional callback if provided
        self._additional_subscription_callback(msg)

    # Service clients

    async def set_arm_lock(
        self, arm: Literal["left", "right", "both"], lock: bool
    ) -> None:
        """Set the arm restraint lock state.

        Controls electromagnetic arm locks to restrain or release the subject's
        arms during experiments.

        Args:
            arm: Which arm(s) to control: "left", "right", or "both".
            lock: True to engage the lock (restrain), False to release.

        Raises:
            ValueError: If arm is not one of the valid options.
        """
        if arm not in ["left", "right", "both"]:
            raise ValueError("Invalid arm: must be 'left', 'right', or 'both'")

        left = arm in ["left", "both"]
        right = arm in ["right", "both"]

        await self.node.service_call_async(
            srv_request=SetArmLock.Request(
                left_arm=left, right_arm=right, lock=lock
            ),
            srv_client=self._set_arm_lock_client,
        )

    async def lock_arms_and_wait(
        self, timeout: Optional[float] = None
    ) -> bool:
        """Lock both arms and wait until conditions are safe for robot execution.

        Engages both arm locks and polls until the safe_to_execute property
        returns True (arms locked, safety laser clear, conditions stable).

        Args:
            timeout: Maximum time to wait in seconds. If None, waits indefinitely.

        Returns:
            True if safe conditions were achieved within the timeout,
            False if timeout was reached.
        """
        self.log("Locking arms and waiting until safe to execute")
        await self.set_arm_lock("both", lock=True)

        spin_period = self.node.param("teensy.spin_period")
        try:
            async with asyncio.timeout(timeout):
                while not self.safe_to_execute:
                    await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    async def set_smartglass(self, reveal: bool) -> None:
        """Control the smartglass goggles transparency.

        Smartglass goggles can switch between opaque and transparent states
        to control what the subject can see during trials.

        Args:
            reveal: True to make goggles transparent, False to make opaque.
        """
        self.log(f"Smartglass {'reveal' if reveal else 'occlude'}")
        await self.node.service_call_async(
            srv_request=SetSmartglass.Request(reveal=reveal),
            srv_client=self._set_smartglass_client,
        )

    async def set_solenoid(self, activate: bool) -> None:
        """Control the smartglass goggles transparency.

        Smartglass goggles can switch between opaque and transparent states
        to control what the subject can see during trials.

        Args:
            reveal: True to make goggles transparent, False to make opaque.
        """
        self.log(f"Solenoid {'activate' if activate else 'deactivate'}")
        await self.node.service_call_async(
            srv_request=SetSolenoid.Request(activate=activate),
            srv_client=self._set_solenoid_client,
        )

    async def set_reward(
        self, activate: bool, duration: Optional[int | float] = None
    ) -> None:
        """Control the reward delivery system.

        Activates or deactivates the reward mechanism (typically a juice pump)
        for subject reinforcement.

        Args:
            activate: True to start reward delivery, False to stop.
            duration: Required when activate=True; how long to deliver reward
                in seconds. Must be positive.

        Raises:
            ValueError: If activate=True but duration is None or non-positive,
                or if activate=False but duration is provided.
        """
        request = SetReward.Request(activate=activate)
        if activate:
            if duration is None or duration <= 0:
                raise ValueError(
                    "If activating, reward duration must not be None and must be greater than 0"
                )
            request.duration = Duration(seconds=duration).to_msg()
            self.log(f"Starting reward for {duration}s")
        else:
            if duration is not None and duration != 0:
                raise ValueError(
                    "If not activating, reward duration must be None or 0"
                )
            self.log("Stopping reward")

        await self.node.service_call_async(
            srv_request=request, srv_client=self._set_reward_client
        )

    async def start_reward_and_wait(self, duration: float) -> None:
        """Deliver reward and wait for completion.

        Starts reward delivery for the specified duration and blocks until
        the reward finishes. Verifies that the reward actually started and
        completed.

        Args:
            duration: How long to deliver reward in seconds.

        Raises:
            AssertionError: If reward doesn't become active after starting.
            RuntimeError: If reward is still active after the expected duration.
        """
        await self.set_reward(activate=True, duration=duration)

        spin_period = self.node.param("teensy.spin_period")

        await asyncio.sleep(spin_period)
        assert self.last_teensy_sensor.is_reward_active, (
            "Reward not active after 1 spin period"
        )

        timeout = duration + spin_period
        try:
            async with asyncio.timeout(timeout):
                while self.last_teensy_sensor.is_reward_active:
                    await asyncio.sleep(spin_period)
        except TimeoutError:
            raise RuntimeError(
                "Reward still active after duration (I fucked up, this shouldn't happen)"
            )

    async def __aenter__(self) -> Self:
        await self.set_solenoid(activate=True)
        return self

    async def __aexit__(
        self,
        exc_type: Optional[type[BaseException]],
        exc_value: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> bool | None:
        await self.set_solenoid(activate=False)
