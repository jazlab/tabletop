"""Interface for Teensy microcontroller communication.

This module provides an interface to communicate with a Teensy microcontroller
that manages experimental apparatus including arm restraints, safety sensors,
the smartglass (a switchable glass pane in front of the subject), and reward
delivery systems.

The Teensy acts as a bridge between the ROS2 system and physical hardware,
providing safety interlocks and subject interface mechanisms.
"""

import asyncio
import threading
from collections.abc import Callable
from copy import copy, deepcopy
from typing import Literal, Optional

from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.duration import Duration
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.qos import QoSPresetProfiles
from tabletop_interfaces.msg import TeensySensor
from tabletop_interfaces.srv import (
    SetArmLock,
    SetReward,
    SetSmartglass,
    SetSolenoid,
)

from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.nodes.base import BaseNode
from tabletop_rig.utils.ros import seconds_from_ros_time


def noop(msg: TeensySensor) -> None:
    """Default callback that does nothing (placeholder for optional callbacks).

    Args:
        msg: The TeensySensor message (ignored).
    """
    pass


class TeensyInterface(BaseInterface):
    """Interface for controlling experimental apparatus via Teensy microcontroller.

    Provides methods to control arm restraints, the smartglass pane, and reward
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
        name: str,
        *,
        additional_subscription_callback: Optional[
            Callable[[TeensySensor], None]
        ] = None,
        parameter_fallback_prefix: Optional[str] = None,
    ) -> None:
        """Initialize the Teensy interface.

        Sets up subscriptions to teensy/sensor topic and service clients for
        teensy/set_arm_lock, teensy/set_reward, teensy/set_smartglass, and
        teensy/set_solenoid services. Waits for the teensy node to be available.

        Args:
            node: Parent ROS2 node for creating ROS resources.
            name: Interface name (used for parameter lookup and logging).
            additional_subscription_callback: Optional callback invoked with
                each TeensySensor message after internal processing.
            parameter_fallback_prefix: Optional fallback prefix for parameter
                lookup (e.g., 'common_teensy_interface').

        Raises:
            RuntimeError: If the teensy node is not available.
        """
        super().__init__(
            node, name, parameter_fallback_prefix=parameter_fallback_prefix
        )

        self.log("Waiting for teensy node")
        if not self.node.wait_for_node_blocking("teensy"):
            raise RuntimeError("teensy node not available")

        # Subscribers
        qos = copy(QoSPresetProfiles.SENSOR_DATA.value)
        self._teensy_sub = self.node.create_subscription(
            TeensySensor,
            "teensy/sensor",
            self._teensy_sensor_callback,
            qos_profile=qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._last_teensy_sensor = TeensySensor()
        self._last_unsafe_to_execute_time = self.node.ros_time()
        # True while the teensy clock tracks the host clock (sensor delay
        # >= 0). A negative delay sets this False so safe_to_execute latches
        # off until the clocks resync (see _teensy_sensor_callback).
        self._teensy_clock_in_sync = True
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
            "teensy/set_arm_lock",
            callback_group=ReentrantCallbackGroup(),
        )
        self._set_reward_client = self.node.create_client(
            SetReward,
            "teensy/set_reward",
            callback_group=ReentrantCallbackGroup(),
        )
        self._set_smartglass_client = self.node.create_client(
            SetSmartglass,
            "teensy/set_smartglass",
            callback_group=ReentrantCallbackGroup(),
        )
        self._set_solenoid_client = self.node.create_client(
            SetSolenoid,
            "teensy/set_solenoid",
            callback_group=ReentrantCallbackGroup(),
        )

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
        1. The teensy and host clocks are in sync (non-negative sensor
           delay); a negative delay forces this False until resynced
        2. Sensor data is recent (within max_sensor_delay)
        3. The message-level safety gate passes (see _msg_safe_to_execute:
           safety laser unbroken, plus both arms locked when
           safe_to_execute.require_arm_locks is enabled)
        4. Conditions have been safe for required_time duration

        Returns:
            True if all safety conditions are met, False otherwise.
        """
        max_sensor_delay = self.param("safe_to_execute.max_sensor_delay")
        required_safe_time = self.param("safe_to_execute.required_time")

        with self._teensy_sensor_lock:
            msg = self._last_teensy_sensor
            last_unsafe_time = self._last_unsafe_to_execute_time
            clock_in_sync = self._teensy_clock_in_sync

        if not clock_in_sync:
            # Host/teensy clocks are out of sync (negative sensor delay);
            # every time comparison below is unreliable, so refuse to run.
            return False

        last_sensor_time = seconds_from_ros_time(msg.header.stamp)
        current_time = self.node.ros_time()

        if not self._msg_safe_to_execute(msg):
            return False

        if current_time - last_sensor_time > max_sensor_delay:
            self.log(
                f"Have not received teensy sensor message in "
                f"{current_time - last_sensor_time} > "
                f"{max_sensor_delay}, not safe to execute",
                severity="WARN",
            )
            return False

        if current_time - last_unsafe_time < required_safe_time:
            return False

        return True

    # Subscribers

    def _msg_safe_to_execute(self, msg: TeensySensor) -> bool:
        """Check if a sensor message indicates safe conditions.

        The safety laser must always be unbroken. Whether the arm-lock
        state (both arms seated/locked) is additionally required is
        controlled by the optional boolean parameter
        ``safe_to_execute.require_arm_locks``:

        - ``True``: motion is gated on BOTH arm locks being engaged AND the
          safety laser being unbroken.
        - ``False`` (default): motion is gated solely on the safety laser
          being unbroken; the published arm-lock state is ignored.

        The parameter is read defensively: if it is not declared (e.g. the
        commander.yaml entry has not been deployed yet), it defaults to
        ``False`` so behaviour is unchanged from the laser-only gate.

        Args:
            msg: The sensor message to evaluate.

        Returns:
            True if the safety conditions are met for the configured gate.
        """
        try:
            require_arm_locks = bool(
                self.param("safe_to_execute.require_arm_locks")
            )
        except ParameterNotDeclaredException:
            require_arm_locks = False

        if require_arm_locks:
            return (
                msg.is_left_arm_locked
                and msg.is_right_arm_locked
                and not msg.is_safety_laser_broken
            )
        return not msg.is_safety_laser_broken

    def _teensy_sensor_callback(self, msg: TeensySensor) -> None:
        """Process incoming TeensySensor messages.

        Updates internal last_teensy_sensor and tracks the last time conditions
        were unsafe. Reads parameter 'sensor_delay_warn_threshold' and logs
        warning if latency exceeds threshold. Invokes the additional_subscription
        callback if provided.

        Args:
            msg: The incoming sensor message.
        """
        warn_threshold: float = self.param("sensor_delay_warn_threshold")
        ahead_threshold: float = self.param("sensor_ahead_threshold")

        received_time = self.node.ros_time()
        teensy_time = seconds_from_ros_time(msg.header.stamp)
        delay = received_time - teensy_time
        clock_in_sync = True
        if delay < -ahead_threshold:
            clock_in_sync = False
            # Teensy timestamp is ahead of host time: the clocks are out of
            # sync, so the host-vs-teensy comparisons in safe_to_execute are
            # unreliable. Warn; the flag below latches safe_to_execute off
            # until the delay is non-negative again.
            self.log(
                f"Teensy sensor timestamp {-delay:.4f}s ahead of host "
                f"time; clocks out of sync, not safe to execute",
                severity="WARN",
                throttle_duration_sec=2,
            )
        elif delay > warn_threshold:
            self.log(
                f"Teensy sensor callback delay {delay:.4f}s > {warn_threshold}s",
                severity="WARN",
                throttle_duration_sec=2,
            )

        # Determine if the monkey is safe
        with self._teensy_sensor_lock:
            self._last_teensy_sensor = msg
            self._teensy_clock_in_sync = clock_in_sync
            if not self._msg_safe_to_execute(msg):
                # Stamp on the host clock so the settling comparison in
                # safe_to_execute (also host time) is skew-free.
                self._last_unsafe_to_execute_time = received_time

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
        if arm not in ("left", "right", "both"):
            raise ValueError("Invalid arm: must be 'left', 'right', or 'both'")

        left_arm = arm in ("left", "both")
        right_arm = arm in ("right", "both")

        await self.node.service_call_async(
            srv_request=SetArmLock.Request(
                left_arm=left_arm, right_arm=right_arm, lock=lock
            ),
            srv_client=self._set_arm_lock_client,
        )

    async def lock_arms_and_wait(
        self,
        timeout: Optional[float] = None,
        *,
        condition: Optional[Callable[[], bool]] = None,
    ) -> bool:
        """Lock both arms and wait until conditions are safe for robot execution.

        Engages both arm locks and polls until the safe_to_execute property
        returns True (arms locked, safety laser clear, conditions stable).

        Args:
            timeout: Maximum time to wait in seconds. If None, waits indefinitely.
            condition: Alternative condition to wait for instead of 'safe_to_execute'

        Returns:
            True if safe conditions were achieved within the timeout,
            False if timeout was reached.
        """
        self.log("Locking arms and waiting until safe to execute")
        await self.set_arm_lock("both", lock=True)

        spin_period = self.param("spin_period")
        try:
            async with asyncio.timeout(timeout):
                if condition is None:
                    while not self.safe_to_execute:
                        await asyncio.sleep(spin_period)
                else:
                    while not condition():
                        await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    async def set_smartglass(self, reveal: bool) -> None:
        """Control the smartglass transparency.

        The smartglass is a switchable glass pane in front of the subject;
        it toggles between transparent and translucent/opaque states to
        control what the subject can see during trials.

        Args:
            reveal: True makes the pane transparent (subject can see),
                False makes it translucent/opaque (occludes the view).
        """
        self.log(f"Smartglass {'reveal' if reveal else 'occlude'}")
        await self.node.service_call_async(
            srv_request=SetSmartglass.Request(reveal=reveal),
            srv_client=self._set_smartglass_client,
        )

    async def set_sync_pulse_solenoid(self, activate: bool) -> None:
        """Control the sync pulse solenoid.

        The sync pulse solenoid can be activated to fire when the Teensy
        sync pulse fires

        Args:
            activate: True to start the sync pulse solenoid, False to stop
        """
        self.log(f"Solenoid {'activate' if activate else 'deactivate'}")
        await self.node.service_call_async(
            srv_request=SetSolenoid.Request(activate=activate),
            srv_client=self._set_solenoid_client,
        )

    def set_sync_pulse_solenoid_blocking(self, activate: bool) -> None:
        """Control the sync pulse solenoid.

        The sync pulse solenoid can be activated to fire when the Teensy
        sync pulse fires

        Args:
            activate: True to start the sync pulse solenoid, False to stop
        """
        self.log(f"Solenoid {'activate' if activate else 'deactivate'}")
        self.node.service_call_blocking(
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

        Starts reward delivery via set_reward(activate=True, duration) and
        polls the last_teensy_sensor property (reading from spin_period
        parameter) until is_reward_active becomes False.

        Args:
            duration: How long to deliver reward in seconds.

        Raises:
            AssertionError: If reward not active after one spin period.
            RuntimeError: If reward still active after timeout (duration +
                spin_period).
        """
        await self.set_reward(activate=True, duration=duration)

        spin_period = self.param("spin_period")

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
