"""Mock Teensy microcontroller node for testing.

This module provides a ROS2 node that simulates the Teensy microcontroller
used in the tabletop experimental rig. It emulates:

- Arm lock solenoid control and state sensing
- Safety laser broken state
- Smartglass (LCD shutter) control
- Reward solenoid control
- Sync pulse generation
- Tactile glove analog inputs

The simulation includes a "monkey loop" that models the subject's behavior,
such as placing arms in locks with realistic delays and occasionally
breaking the safety laser.

Topics published:
    teensy/sensor: TeensySensor messages at 100Hz
    teensy/log: Log messages from the mock Teensy

Services provided:
    teensy/ping: Ping service for latency measurement
    teensy/set_arm_lock: Control arm lock solenoids
    teensy/set_smartglass: Control smartglass visibility
    teensy/set_reward: Control reward solenoid
    teensy/set_solenoid: Control solenoid activation

Example:
    ros2 run tabletop_rig mock_teensy
"""

import argparse
import asyncio
import random
import time
from copy import copy
from typing import Any

import debugpy
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.qos import QoSDurabilityPolicy, QoSPresetProfiles
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from tabletop_interfaces.msg import TeensySensor
from tabletop_interfaces.srv import (
    Ping,
    SetArmLock,
    SetReward,
    SetSmartglass,
    SetSolenoid,
)

from tabletop_rig.executors import AIOExecutor
from tabletop_rig.nodes.base import BaseNode
from tabletop_rig.utils.logging import SeverityString

# Digital pin state constants
HIGH = True
LOW = False

# Analog pin aliases (matching Teensy 4.0 pinout)
A0 = 14
A1 = 15
A2 = 16
A3 = 17
A4 = 18
A5 = 19
A6 = 20
A7 = 21
A8 = 22
A9 = 23

# Pin assignments matching the actual Teensy firmware (main.cpp)
# Note: Setting the lock control pin to HIGH engages the solenoid,
# but the arm is only actually locked when the locked state pin reads HIGH
# (indicating the subject has placed their arm in the lock mechanism)
LEFT_ARM_LOCK_CONTROL_PIN = 1
RIGHT_ARM_LOCK_CONTROL_PIN = 2
SMARTGLASS_CONTROL_PIN = 3
REWARD_CONTROL_PIN = 4
SYNC_PULSE_CONTROL_PIN = 9
SOLENOID_CONTROL_PIN = 12
LEFT_ARM_LOCKED_STATE_PIN = 34
RIGHT_ARM_LOCKED_STATE_PIN = 35
SAFETY_LASER_BROKEN_STATE_PIN = 36
LEFT_GLOVE_STATE_PINS = [A0, A1, A2, A3, A4]
RIGHT_GLOVE_STATE_PINS = [A5, A6, A7, A8, A9]

# Simulated pin states
digital_output_pin_states = {
    LEFT_ARM_LOCK_CONTROL_PIN: LOW,
    RIGHT_ARM_LOCK_CONTROL_PIN: LOW,
    SMARTGLASS_CONTROL_PIN: LOW,
    REWARD_CONTROL_PIN: LOW,
    SYNC_PULSE_CONTROL_PIN: LOW,
    SOLENOID_CONTROL_PIN: LOW,
}

digital_input_pin_states = {
    LEFT_ARM_LOCKED_STATE_PIN: random.choice([HIGH, LOW]),
    RIGHT_ARM_LOCKED_STATE_PIN: random.choice([HIGH, LOW]),
    SAFETY_LASER_BROKEN_STATE_PIN: random.choice([HIGH, LOW]),
}

analog_input_pin_states = {
    p: random.randint(0, 1023)
    for p in LEFT_GLOVE_STATE_PINS + RIGHT_GLOVE_STATE_PINS
}


SENSOR_PERIOD_MS = 10
SYNC_PULSE_BASE_PERIOD_MS = 1000
SYNC_PULSE_DELAY_RANGE_MS = 200
SYNC_PULSE_DURATION_MS = 100


def digital_write(pin: int, value: bool) -> None:
    """Set a simulated digital output pin state.

    Args:
        pin: The output pin number.
        value: HIGH (True) or LOW (False).

    Raises:
        AssertionError: If pin is not a valid output pin.
    """
    assert pin in digital_output_pin_states, (
        f"Pin {pin} not found in digital_output_pin_states"
    )
    digital_output_pin_states[pin] = value


def digital_read(pin: int) -> bool:
    """Read a simulated digital input pin state.

    Args:
        pin: The input pin number.

    Returns:
        Current pin state (HIGH or LOW).

    Raises:
        AssertionError: If pin is not a valid input pin.
    """
    assert pin in digital_input_pin_states, (
        f"Pin {pin} not found in digital_input_pin_states"
    )
    return digital_input_pin_states[pin]


def analog_read(pin: int) -> int:
    """Read a simulated analog input pin value.

    Args:
        pin: The analog input pin number.

    Returns:
        Current 10-bit ADC value (0-1023).

    Raises:
        AssertionError: If pin is not a valid analog input pin.
    """
    assert pin in analog_input_pin_states, (
        f"Pin {pin} not found in analog_input_pin_states"
    )
    return analog_input_pin_states[pin]


def _change_input_pin_state(pin: int, value: bool) -> None:
    """Internal: Change a simulated input pin state.

    Used by the monkey simulation loop to update sensor states.

    Args:
        pin: The input pin number to modify.
        value: New pin state.
    """
    assert pin in digital_input_pin_states, (
        f"Pin {pin} not found in digital_input_pin_states"
    )
    digital_input_pin_states[pin] = value


def _read_output_pin_state(pin: int) -> bool:
    """Internal: Read an output pin state.

    Used by the monkey simulation to check control signals.

    Args:
        pin: The output pin number.

    Returns:
        Current output pin state.
    """
    assert pin in digital_output_pin_states, (
        f"Pin {pin} not found in digital_output_pin_states"
    )
    return digital_output_pin_states[pin]


async def _sleep_and_change_input_pin_state(
    pin: int, value: bool, delay_sec: float
) -> None:
    """Internal: Change an input pin state after a delay.

    Used to simulate the subject placing their arm in the lock
    after some reaction time.

    Args:
        pin: The input pin to modify.
        value: New pin state.
        delay_sec: Delay before changing state.
    """
    await asyncio.sleep(delay_sec)
    _change_input_pin_state(pin, value)


async def monkey_loop(
    min_arm_locked_delay_sec: float,
    max_arm_locked_delay_sec: float,
    safety_laser_broken_when_locked_prob: float,
    safety_laser_broken_when_unlocked_prob: float,
    loop_period_sec: float,
    logger: RcutilsLogger,
) -> None:
    """Simulate realistic subject behavior for testing.

    Models the subject's actions including:
    - Placing arms in locks with realistic delays after control signal
    - Breaking the safety laser probabilistically based on arm lock state
    - Handling the rare case of safety laser break while arms are locked

    The probabilities allow testing of:
    - Normal operation (arms locked, laser unbroken)
    - Safety interrupts (laser broken while arms unlocked)
    - Edge cases (laser broken while arms supposedly locked)

    Args:
        min_arm_locked_delay_sec: Minimum delay before arm registers as locked.
        max_arm_locked_delay_sec: Maximum delay before arm registers as locked.
        safety_laser_broken_when_locked_prob: Probability of laser break when
            both arms are locked (should be low, tests failure handling).
        safety_laser_broken_when_unlocked_prob: Probability of laser break when
            arms are unlocked (normal subject movement).
        loop_period_sec: Main loop period in seconds.

    Raises:
        ValueError: If delay parameters are invalid.
    """

    if (
        min_arm_locked_delay_sec < 0
        or max_arm_locked_delay_sec < min_arm_locked_delay_sec
        or loop_period_sec < max_arm_locked_delay_sec
    ):
        raise ValueError(
            f"Invalid delays: 0 <= min_arm_locked_delay_sec <= max_arm_locked_delay_sec <= loop_period_sec, "
            f"got {min_arm_locked_delay_sec}, {max_arm_locked_delay_sec}, {loop_period_sec}"
        )

    while True:
        start_time = time.time()
        left_arm_lock_task = None
        right_arm_lock_task = None

        # Update the left arm locked state based on the left arm lock pin
        if _read_output_pin_state(
            LEFT_ARM_LOCK_CONTROL_PIN
        ) and not digital_read(LEFT_ARM_LOCKED_STATE_PIN):
            delay = random.uniform(
                min_arm_locked_delay_sec, max_arm_locked_delay_sec
            )
            left_arm_lock_task = asyncio.create_task(
                _sleep_and_change_input_pin_state(
                    LEFT_ARM_LOCKED_STATE_PIN, HIGH, delay
                )
            )

        # Update the right arm locked state based on the right arm lock pin
        if _read_output_pin_state(
            RIGHT_ARM_LOCK_CONTROL_PIN
        ) and not digital_read(RIGHT_ARM_LOCKED_STATE_PIN):
            delay = random.uniform(
                min_arm_locked_delay_sec, max_arm_locked_delay_sec
            )
            right_arm_lock_task = asyncio.create_task(
                _sleep_and_change_input_pin_state(
                    RIGHT_ARM_LOCKED_STATE_PIN, HIGH, delay
                )
            )

        if left_arm_lock_task is not None:
            await left_arm_lock_task
        if right_arm_lock_task is not None:
            await right_arm_lock_task

        # Update the safety laser broken state based on the arm lock states
        if digital_read(LEFT_ARM_LOCKED_STATE_PIN) and digital_read(
            RIGHT_ARM_LOCKED_STATE_PIN
        ):
            # If both arms are locked, break the safety laser with a small probability
            _change_input_pin_state(
                SAFETY_LASER_BROKEN_STATE_PIN,
                (random.uniform(0, 1) < safety_laser_broken_when_locked_prob),
            )
            if digital_read(SAFETY_LASER_BROKEN_STATE_PIN):
                logger.info("Wiggins has broken confinement!!!!!!!!")
        else:
            # Otherwise, break the safety laser with a higher probability
            _change_input_pin_state(
                SAFETY_LASER_BROKEN_STATE_PIN,
                random.uniform(0, 1) < safety_laser_broken_when_unlocked_prob,
            )

        # Sleep for the remaining time in the loop period
        delay = loop_period_sec - (time.time() - start_time)
        await asyncio.sleep(delay)


class MockTeensy(BaseNode):
    """Mock Teensy microcontroller node for simulation and testing.

    Simulates the Teensy 4.0 microcontroller that controls the experimental
    rig hardware. Provides the same ROS2 interface as the real firmware,
    allowing testing of Commander node logic without hardware.

    The simulation includes:
    - Realistic arm lock behavior with subject response delays
    - Probabilistic safety laser state changes
    - Sync pulse generation with timing jitter
    - Reward solenoid timing

    Attributes:
        monkey_loop: Asyncio task running the subject behavior simulation.
        reward_active: Whether the reward solenoid is currently active.
        smartglass_revealed: Current smartglass visibility state.
    """

    default_params = BaseNode.default_params | {
        "monkey_loop.min_arm_locked_delay_sec": 0.5,
        "monkey_loop.max_arm_locked_delay_sec": 0.8,
        "monkey_loop.safety_laser_broken_when_locked_prob": 0.0,
        "monkey_loop.safety_laser_broken_when_unlocked_prob": 0.5,
        "monkey_loop.loop_period_sec": 1.0,
    }

    def __init__(self):
        """Initialize the mock Teensy node with all publishers and services."""
        super().__init__("teensy")
        self.log_pub = self.create_publisher(String, "~/log", 10)

        # State variables
        self.sync_pulse_control_pin_state = LOW
        self.sync_pulse_last_time_on = self.get_clock().now()
        self.sync_pulse_last_time_off = self.get_clock().now()
        self.reward_active = False
        self.smartglass_revealed = False
        self.solenoid_active = False

        # Write the initial pin states
        digital_write(LEFT_ARM_LOCK_CONTROL_PIN, HIGH)
        digital_write(RIGHT_ARM_LOCK_CONTROL_PIN, HIGH)
        digital_write(SMARTGLASS_CONTROL_PIN, self.smartglass_revealed)
        digital_write(REWARD_CONTROL_PIN, self.reward_active)
        digital_write(SOLENOID_CONTROL_PIN, self.solenoid_active)
        digital_write(
            SYNC_PULSE_CONTROL_PIN, self.sync_pulse_control_pin_state
        )

        # Publishers
        qos = copy(QoSPresetProfiles.SENSOR_DATA.value)
        qos.durability = QoSDurabilityPolicy.VOLATILE
        qos.depth = 1
        self.sensor_pub = self.create_publisher(
            TeensySensor,
            "~/sensor",
            qos_profile=qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Services
        # qos = copy(QoSPresetProfiles.SERVICES_DEFAULT.value)
        # qos.liveliness = QoSLivelinessPolicy.AUTOMATIC
        self.ping_service = self.create_service(
            Ping,
            "~/ping",
            self.ping_callback,  # pyright: ignore[reportArgumentType]
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.set_arm_lock_service = self.create_service(
            SetArmLock,
            "~/set_arm_lock",
            self.set_arm_lock_callback,  # pyright: ignore[reportArgumentType]
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.set_smartglass_service = self.create_service(
            SetSmartglass,
            "~/set_smartglass",
            self.set_smartglass_callback,  # pyright: ignore[reportArgumentType]
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.set_reward_service = self.create_service(
            SetReward,
            "~/set_reward",
            self.set_reward_callback,  # pyright: ignore[reportArgumentType]
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.set_solenoid_service = self.create_service(
            SetSolenoid,
            "~/set_solenoid",
            self.set_solenoid_callback,  # pyright: ignore[reportArgumentType]
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Timers
        self.sensor_timer = self.create_timer(
            SENSOR_PERIOD_MS / 1000.0, self.sensor_timer_callback
        )
        self.sync_pulse_end_timer = self.create_timer(
            SYNC_PULSE_DURATION_MS / 1000.0,
            self.sync_pulse_end_timer_callback,
            autostart=False,
        )
        self.sync_pulse_start_timer = self.create_timer(
            0.0, self.sync_pulse_start_timer_callback, autostart=False
        )
        self.sync_pulse_base_timer = self.create_timer(
            SYNC_PULSE_BASE_PERIOD_MS / 1000.0,
            self.sync_pulse_base_timer_callback,
        )
        self.reward_timer = self.create_timer(
            0.0, self.reward_timer_callback, autostart=False
        )

        self.log("MockTeensy initialized")

    def log(
        self,
        message: Any,
        severity: SeverityString | LoggingSeverity = "INFO",
        **kwargs,
    ) -> bool:
        """Log a message and publish it to teensy/log.

        Overrides BaseNode.log to also publish log messages to the
        teensy/log topic, mimicking the real Teensy firmware behavior.

        Args:
            message: Message to log.
            severity: Log severity level.
            **kwargs: Additional arguments passed to parent log method.

        Returns:
            True if message was logged successfully.
        """
        success = super().log(message, severity, **kwargs)

        if isinstance(severity, LoggingSeverity):
            severity = severity.name
        message = f"{severity.capitalize()}: {message}"

        if success and hasattr(self, "log_pub"):
            self.log_pub.publish(String(data=str(message)))

        return success

    def sync_pulse_end_timer_callback(self) -> None:
        """Handle end of sync pulse period.

        Brings the sync pulse pin LOW and logs the pulse duration.
        """
        assert self.sync_pulse_control_pin_state
        digital_write(SYNC_PULSE_CONTROL_PIN, LOW)
        self.sync_pulse_control_pin_state = LOW
        self.sync_pulse_last_time_off = self.get_clock().now()

        self.sync_pulse_end_timer.cancel()

        duration_ms = (
            self.sync_pulse_last_time_off - self.sync_pulse_last_time_on
        ).nanoseconds / 1e6
        self.log(
            f"Sync pulse ended after {duration_ms:.1f} ms",
            severity="DEBUG",
        )

    def sync_pulse_start_timer_callback(self) -> None:
        """Start a sync pulse after the random delay.

        Brings the sync pulse pin HIGH and starts the end timer.
        """
        assert not self.sync_pulse_control_pin_state
        digital_write(SYNC_PULSE_CONTROL_PIN, HIGH)
        self.sync_pulse_control_pin_state = HIGH
        self.sync_pulse_last_time_on = self.get_clock().now()

        self.sync_pulse_start_timer.cancel()

        self.sync_pulse_end_timer.reset()

        self.log(
            f"Sync pulse started for {SYNC_PULSE_DURATION_MS} ms",
            severity="DEBUG",
        )

    def sync_pulse_base_timer_callback(self) -> None:
        """Handle the base sync pulse timer.

        Schedules the next sync pulse with a random delay to simulate
        timing jitter in the actual hardware.
        """
        assert not self.sync_pulse_control_pin_state
        digital_write(SYNC_PULSE_CONTROL_PIN, LOW)
        self.sync_pulse_control_pin_state = LOW

        # Generate a random delay between 0 and 200 ms
        delay_ms = random.uniform(0, SYNC_PULSE_DELAY_RANGE_MS)
        self.sync_pulse_start_timer.timer_period_ns = delay_ms * 1e6
        self.sync_pulse_start_timer.reset()

        self.log(
            f"Sync pulse start timer scheduled for {delay_ms:.1f} ms",
            severity="DEBUG",
        )

    def ping_callback(
        self, request: Ping.Request, response: Ping.Response
    ) -> Ping.Response:
        """Handle ping service requests.

        Returns the current time to allow round-trip latency measurement.

        Args:
            request: Request containing sent_time timestamp.
            response: Response to populate.

        Returns:
            Response with received_time set to current clock time.
        """
        _ = request  # sent_time is not used, just for client reference
        response.received_time = self.get_clock().now().to_msg()
        response.success = True
        return response

    def set_arm_lock_callback(
        self, request: SetArmLock.Request, response: SetArmLock.Response
    ) -> SetArmLock.Response:
        """Handle arm lock control service requests.

        Sets the arm lock control pins and, for unlock requests,
        immediately updates the locked state (simulating immediate release).

        Args:
            request: Request specifying which arm(s) and lock/unlock.
            response: Response to populate.

        Returns:
            Response with success status and message.
        """
        if not request.left_arm and not request.right_arm:
            response.success = False
            response.message = "No arm specified"
            self.log(response.message, severity="WARN")
            return response

        pin_state = HIGH if request.lock else LOW
        message_arm = None
        if request.left_arm:
            digital_write(LEFT_ARM_LOCK_CONTROL_PIN, pin_state)

            # For simulation purposes, we need to update the left arm locked state
            # to reflect the arm lock control pin state
            if not request.lock:
                _change_input_pin_state(LEFT_ARM_LOCKED_STATE_PIN, LOW)

            if not request.right_arm:
                message_arm = "Left arm"
        if request.right_arm:
            digital_write(RIGHT_ARM_LOCK_CONTROL_PIN, pin_state)

            # For simulation purposes, we need to update the right arm locked state
            # to reflect the arm lock control pin state
            if not request.lock:
                _change_input_pin_state(RIGHT_ARM_LOCKED_STATE_PIN, LOW)

            if not request.left_arm:
                message_arm = "Right arm"

        if request.left_arm and request.right_arm:
            message_arm = "Both arms"

        assert message_arm is not None

        response.success = True
        response.message = (
            f"{message_arm} {'locked' if request.lock else 'released'}"
        )
        self.log(response.message)

        return response

    def set_smartglass_callback(
        self, request: SetSmartglass.Request, response: SetSmartglass.Response
    ) -> SetSmartglass.Response:
        """Handle smartglass control service requests.

        Args:
            request: Request specifying reveal or occlude.
            response: Response to populate.

        Returns:
            Response with success status and message.
        """
        pin_state = HIGH if request.reveal else LOW
        digital_write(SMARTGLASS_CONTROL_PIN, pin_state)
        response.success = True
        response.message = (
            f"Smartglass {'revealed' if request.reveal else 'occluded'}"
        )
        self.log(response.message)

        return response

    def set_solenoid_callback(
        self, request: SetSolenoid.Request, response: SetSolenoid.Response
    ) -> SetSolenoid.Response:
        """Handle solenoid control service requests.

        Args:
            request: Request specifying activate or deactivate.
            response: Response to populate.

        Returns:
            Response with success status and message.
        """
        pin_state = HIGH if request.activate else LOW
        digital_write(SOLENOID_CONTROL_PIN, pin_state)
        self.solenoid_active = request.activate
        response.success = True
        response.message = (
            f"Solenoid {'activated' if request.activate else 'deactivated'}"
        )
        self.log(response.message)

        return response

    def reward_timer_callback(self) -> None:
        """Handle reward duration expiration.

        Deactivates the reward solenoid when the timer fires.
        """
        digital_write(REWARD_CONTROL_PIN, LOW)
        self.reward_active = False
        self.reward_timer.cancel()
        self.log("Reward finished")

    def set_reward_callback(
        self, request: SetReward.Request, response: SetReward.Response
    ) -> SetReward.Response:
        """Handle reward control service requests.

        Activates or deactivates the reward solenoid. When activating,
        starts a timer to automatically deactivate after the specified
        duration.

        Args:
            request: Request specifying activate/deactivate and duration.
            response: Response to populate.

        Returns:
            Response with success status and message.
        """
        timer_is_canceled = self.reward_timer.is_canceled()
        assert self.reward_active != timer_is_canceled, (
            "Reward timer state and reward_active state are inconsistent"
        )

        if request.activate:
            # Activate reward for the duration specified in the request
            if request.duration.sec == 0 and request.duration.nanosec == 0:
                response.success = False
                response.message = (
                    "Reward duration should be provided when activating"
                )
                self.log(response.message, severity="WARN")
                return response

            digital_write(REWARD_CONTROL_PIN, HIGH)
            self.reward_active = True

            duration_ns = Duration.from_msg(request.duration).nanoseconds
            self.reward_timer.timer_period_ns = duration_ns
            self.reward_timer.reset()

            duration_s = duration_ns / 1e9
            response.message = f"Reward {'started' if timer_is_canceled else 'extended'} for {duration_s:.2f} s"
        else:
            # If the reward is being deactivated, the duration should be 0
            if request.duration.sec != 0 or request.duration.nanosec != 0:
                response.success = False
                response.message = (
                    "Reward duration should not be provided when deactivating"
                )
                self.log(response.message, severity="WARN")
                return response

            if not timer_is_canceled:
                digital_write(REWARD_CONTROL_PIN, LOW)
                self.reward_active = False

                old_period_ns = self.reward_timer.timer_period_ns
                time_until_ns = self.reward_timer.time_until_next_call()
                self.reward_timer.cancel()

                time_since_s = (old_period_ns - time_until_ns) / 1e9
                response.message = f"Reward stopped after {time_since_s:.2f} s"
            else:
                response.message = "Reward already stopped"

        response.success = True
        self.log(response.message)

        return response

    def sensor_timer_callback(self) -> None:
        """Publish current sensor state at 100Hz.

        Reads all simulated sensor inputs and publishes a TeensySensor
        message with arm lock states, safety laser state, reward state,
        smartglass state, tactile glove values, and sync pulse timing.
        """
        sensor_msg = TeensySensor()
        sensor_msg.header.stamp = self.get_clock().now().to_msg()
        sensor_msg.is_safety_laser_broken = digital_read(
            SAFETY_LASER_BROKEN_STATE_PIN
        )
        sensor_msg.is_left_arm_locked = digital_read(LEFT_ARM_LOCKED_STATE_PIN)
        sensor_msg.is_right_arm_locked = digital_read(
            RIGHT_ARM_LOCKED_STATE_PIN
        )

        sensor_msg.is_reward_active = self.reward_active
        sensor_msg.is_smartglass_revealed = self.smartglass_revealed

        # Update tactile glove states
        for i, p in enumerate(LEFT_GLOVE_STATE_PINS):
            sensor_msg.left_tactile_glove_states[i] = analog_read(p)
        for i, p in enumerate(RIGHT_GLOVE_STATE_PINS):
            sensor_msg.right_tactile_glove_states[i] = analog_read(p)

        # Update sync pulse state
        sensor_msg.sync_pulse_state = self.sync_pulse_control_pin_state
        sensor_msg.sync_pulse_last_time_on = (
            self.sync_pulse_last_time_on.to_msg()
        )
        sensor_msg.sync_pulse_last_time_off = (
            self.sync_pulse_last_time_off.to_msg()
        )

        self.sensor_pub.publish(sensor_msg)

    async def spin(self) -> None:
        """Spin simulated monkey loop"""
        wiggins_logger = self.get_logger().get_child("wiggins")
        await monkey_loop(**self.param("monkey_loop"), logger=wiggins_logger)


async def main_async(args=None):
    """Async entry point for the mock Teensy node.

    Args:
        args: Command line arguments (passed to rclpy.init).
    """
    # rclpy.init(args=args)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    # Parse non-ROS arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug", action="store_true", default=False)

    non_ros_args = rclpy.utilities.remove_ros_args(args)  # type: ignore
    args, _ = parser.parse_known_args(non_ros_args)

    if args.debug:
        print("Debug mode enabled")
        debugpy.listen(1302)
        print("Waiting for debugger to attach")
        debugpy.wait_for_client()
        print("Debugger attached")

    try:
        executor = AIOExecutor()
        mock_teensy = MockTeensy()
        executor.add_node(mock_teensy)

        try:
            future = executor.create_task(mock_teensy.spin())
            await executor.spin_until_future_complete(future)
        finally:
            print("Shutting down mock teensy")
            mock_teensy.destroy_node()
            print("Shutting down executor")
            executor.shutdown()
    finally:
        print("Shutting down rclpy")
        rclpy.try_shutdown()  # type: ignore


def main(args=None):
    """Entry point for the mock_teensy node."""
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        pass
