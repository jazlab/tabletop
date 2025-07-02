import argparse
import asyncio
import random
import time
from copy import copy

import debugpy
import rclpy
import rclpy.logging
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSPresetProfiles
from std_msgs.msg import String
from tabletop_interfaces.msg import TeensySensor
from tabletop_interfaces.srv import SetArmLock, SetReward, SetSmartglass
from tabletop_utils.aio_executor import AIOExecutor

from tabletop_server.nodes.base import BaseNode

monkey_logger = rclpy.logging.get_logger("wiggins")

# Define constants for digital pin states
HIGH = True
LOW = False

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

# Define pin assignments similar to main.cpp
# Solenoid for locking the arm
# Setting the lock pin to HIGH does not mean the arm is locked
# The arm is only locked when the locked state pin is HIGH
LEFT_ARM_LOCK_CONTROL_PIN = 1
RIGHT_ARM_LOCK_CONTROL_PIN = 2
SMARTGLASS_CONTROL_PIN = 3
REWARD_CONTROL_PIN = 4
SYNC_PULSE_CONTROL_PIN = 9
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


SENSOR_PERIOD_MS = 100
SYNC_PULSE_BASE_PERIOD_MS = 1000
SYNC_PULSE_DELAY_RANGE_MS = 200
SYNC_PULSE_DURATION_MS = 100


def digital_write(pin, value: bool):
    """Write a digital value to a pin."""
    assert (
        pin in digital_output_pin_states
    ), f"Pin {pin} not found in digital_output_pin_states"
    digital_output_pin_states[pin] = value


def digital_read(pin) -> bool:
    """Read a digital value from a pin."""
    assert (
        pin in digital_input_pin_states
    ), f"Pin {pin} not found in digital_input_pin_states"
    return digital_input_pin_states[pin]


def analog_read(pin) -> int:
    assert (
        pin in analog_input_pin_states
    ), f"Pin {pin} not found in analog_input_pin_states"
    return analog_input_pin_states[pin]


def _change_input_pin_state(pin, value: bool):
    assert (
        pin in digital_input_pin_states
    ), f"Pin {pin} not found in digital_input_pin_states"
    digital_input_pin_states[pin] = value


def _read_output_pin_state(pin) -> bool:
    assert (
        pin in digital_output_pin_states
    ), f"Pin {pin} not found in digital_output_pin_states"
    return digital_output_pin_states[pin]


async def _sleep_and_change_input_pin_state(
    pin, value: bool, delay_sec: float
):
    await asyncio.sleep(delay_sec)
    _change_input_pin_state(pin, value)


async def monkey_loop(
    min_arm_locked_delay_sec: float,
    max_arm_locked_delay_sec: float,
    safety_laser_broken_when_locked_prob: float,
    safety_laser_broken_when_unlocked_prob: float,
    loop_period_sec: float,
):
    """Simulate the monkey's actions.

    This function is responsible for simulating the monkey's actions, such as
    locking and unlocking the arms, and breaking the safety laser.

    If both arms are locked, break the safety laser with a small probability
    (to test arm lock failure handling)
    Otherwise, the safety laser is broken with a higher probability and
    the arm locked states are updated to reflect the arm lock control
    pins (with some delay to reflect the monkey taking time to put their
    arms in the arm lock)
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

        if digital_read(LEFT_ARM_LOCKED_STATE_PIN) and digital_read(
            RIGHT_ARM_LOCKED_STATE_PIN
        ):
            # If both arms are locked, break the safety laser with a small probability
            _change_input_pin_state(
                SAFETY_LASER_BROKEN_STATE_PIN,
                (random.uniform(0, 1) < safety_laser_broken_when_locked_prob),
            )
            if digital_read(SAFETY_LASER_BROKEN_STATE_PIN):
                monkey_logger.info("Wiggins has broken confinement!!!!!!!!")
        else:
            # Otherwise, break the safety laser with a higher probability
            _change_input_pin_state(
                SAFETY_LASER_BROKEN_STATE_PIN,
                random.uniform(0, 1) < safety_laser_broken_when_unlocked_prob,
            )

            # Update the left arm locked state based on the left arm lock pin
            if _read_output_pin_state(LEFT_ARM_LOCK_CONTROL_PIN) == HIGH:
                if digital_read(LEFT_ARM_LOCKED_STATE_PIN) == LOW:
                    delay = random.uniform(
                        min_arm_locked_delay_sec, max_arm_locked_delay_sec
                    )
                    left_arm_lock_task = asyncio.create_task(
                        _sleep_and_change_input_pin_state(
                            LEFT_ARM_LOCKED_STATE_PIN, HIGH, delay
                        )
                    )
            else:
                _change_input_pin_state(LEFT_ARM_LOCKED_STATE_PIN, LOW)

            # Update the right arm locked state based on the right arm lock pin
            if _read_output_pin_state(RIGHT_ARM_LOCK_CONTROL_PIN) == HIGH:
                if digital_read(RIGHT_ARM_LOCKED_STATE_PIN) == LOW:
                    delay = random.uniform(
                        min_arm_locked_delay_sec, max_arm_locked_delay_sec
                    )
                    right_arm_lock_task = asyncio.create_task(
                        _sleep_and_change_input_pin_state(
                            RIGHT_ARM_LOCKED_STATE_PIN, HIGH, delay
                        )
                    )
            else:
                _change_input_pin_state(RIGHT_ARM_LOCKED_STATE_PIN, LOW)

            if left_arm_lock_task is not None:
                await left_arm_lock_task
            if right_arm_lock_task is not None:
                await right_arm_lock_task

        # Sleep for the remaining time in the loop period
        delay = loop_period_sec - (time.time() - start_time)
        await asyncio.sleep(delay)


class MockTeensy(BaseNode):
    default_params = BaseNode.default_params | {
        "monkey_loop.min_arm_locked_delay_sec": 0.5,
        "monkey_loop.max_arm_locked_delay_sec": 0.8,
        "monkey_loop.safety_laser_broken_when_locked_prob": 0.1,
        "monkey_loop.safety_laser_broken_when_unlocked_prob": 0.5,
        "monkey_loop.loop_period_sec": 2.0,
    }

    def __init__(self):
        super().__init__("mock_teensy")

        # Log publisher
        self.log_pub = self.create_publisher(String, "/teensy/log", 10)

        # Monkey loop
        monkey_loop_kwargs = self.get_parameter_wrapper("monkey_loop")
        self.monkey_loop = asyncio.create_task(
            monkey_loop(**monkey_loop_kwargs)
        )

        # State variables
        self.sync_pulse_control_pin_state = LOW
        self.sync_pulse_last_time_on = self.get_clock().now()
        self.sync_pulse_last_time_off = self.get_clock().now()
        self.reward_control_pin_state = LOW
        self.smartglass_control_pin_state = LOW

        # Write the initial pin states
        digital_write(LEFT_ARM_LOCK_CONTROL_PIN, HIGH)
        digital_write(RIGHT_ARM_LOCK_CONTROL_PIN, HIGH)
        digital_write(
            SMARTGLASS_CONTROL_PIN, self.smartglass_control_pin_state
        )
        digital_write(REWARD_CONTROL_PIN, self.reward_control_pin_state)
        digital_write(
            SYNC_PULSE_CONTROL_PIN, self.sync_pulse_control_pin_state
        )

        # Publishers
        qos = copy(QoSPresetProfiles.SENSOR_DATA.value)
        qos.durability = QoSDurabilityPolicy.VOLATILE
        self.sensor_pub = self.create_publisher(
            TeensySensor,
            "/teensy/sensor",
            qos_profile=qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Services
        # qos = copy(QoSPresetProfiles.SERVICES_DEFAULT.value)
        # qos.liveliness = QoSLivelinessPolicy.AUTOMATIC
        self.set_arm_lock_service = self.create_service(
            SetArmLock,
            "/teensy/set_arm_lock",
            self.set_arm_lock_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.set_smartglass_service = self.create_service(
            SetSmartglass,
            "/teensy/set_smartglass",
            self.set_smartglass_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.set_reward_service = self.create_service(
            SetReward,
            "/teensy/set_reward",
            self.set_reward_callback,
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

    def log(self, message: str, severity: str = "INFO"):
        super().log(message, severity)
        if hasattr(self, "log_pub"):
            self.log_pub.publish(String(data=f"{severity}: {message}"))

    def sync_pulse_end_timer_callback(self):
        assert self.sync_pulse_control_pin_state == HIGH
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

    def sync_pulse_start_timer_callback(self):
        assert self.sync_pulse_control_pin_state == LOW
        digital_write(SYNC_PULSE_CONTROL_PIN, HIGH)
        self.sync_pulse_control_pin_state = HIGH
        self.sync_pulse_last_time_on = self.get_clock().now()

        self.sync_pulse_start_timer.cancel()

        self.sync_pulse_end_timer.reset()

        self.log(
            f"Sync pulse started for {SYNC_PULSE_DURATION_MS} ms",
            severity="DEBUG",
        )

    def sync_pulse_base_timer_callback(self):
        assert self.sync_pulse_control_pin_state == LOW
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

    def set_arm_lock_callback(
        self, request: SetArmLock.Request, response: SetArmLock.Response
    ):
        if not request.left_arm and not request.right_arm:
            response.success = False
            response.message = "No arm specified"
            self.log(response.message, severity="WARN")
            return response

        pin_state = HIGH if request.lock else LOW
        if request.left_arm:
            digital_write(LEFT_ARM_LOCK_CONTROL_PIN, pin_state)
            if not request.right_arm:
                message_arm = "Left arm"
        if request.right_arm:
            digital_write(RIGHT_ARM_LOCK_CONTROL_PIN, pin_state)
            if not request.left_arm:
                message_arm = "Right arm"

        if request.left_arm and request.right_arm:
            message_arm = "Both arms"

        response.success = True
        response.message = (
            f"{message_arm} {'locked' if request.lock else 'released'}"
        )
        self.log(response.message)

        return response

    def set_smartglass_callback(
        self, request: SetSmartglass.Request, response: SetSmartglass.Response
    ):
        pin_state = HIGH if request.reveal else LOW
        digital_write(SMARTGLASS_CONTROL_PIN, pin_state)
        response.success = True
        response.message = (
            f"Smartglass {'revealed' if request.reveal else 'occluded'}"
        )
        self.log(response.message)

        return response

    def reward_timer_callback(self):
        digital_write(REWARD_CONTROL_PIN, LOW)
        self.reward_active = False
        self.reward_timer.cancel()
        self.log("Reward finished")

    def set_reward_callback(
        self, request: SetReward.Request, response: SetReward.Response
    ):
        if not self.reward_timer.is_canceled():
            assert (
                self.reward_active
            ), "Reward is not active while reward timer is running"
            response.message = "Error: Reward already active!"
            response.success = False
            self.log(response.message, severity="WARN")
            return response

        digital_write(REWARD_CONTROL_PIN, HIGH)
        self.reward_active = True

        duration = Duration.from_msg(request.duration)
        self.reward_timer.timer_period_ns = duration.nanoseconds
        self.reward_timer.reset()

        # Set the response message
        response.success = True
        response.message = (
            f"Reward started for {duration.nanoseconds / 1e9:.2f} s"
        )
        self.log(response.message)

        return response

    def sensor_timer_callback(self):
        # Populate sensor message
        sensor_msg = TeensySensor()
        sensor_msg.timestamp = self.get_clock().now().to_msg()
        sensor_msg.is_safety_laser_broken = digital_read(
            SAFETY_LASER_BROKEN_STATE_PIN
        )
        sensor_msg.is_left_arm_locked = digital_read(LEFT_ARM_LOCKED_STATE_PIN)
        sensor_msg.is_right_arm_locked = digital_read(
            RIGHT_ARM_LOCKED_STATE_PIN
        )

        sensor_msg.is_reward_active = self.reward_control_pin_state == HIGH
        sensor_msg.is_smartglass_revealed = (
            self.smartglass_control_pin_state == HIGH
        )

        # Update tactile glove states
        for i, p in enumerate(LEFT_GLOVE_STATE_PINS):
            sensor_msg.left_tactile_glove_states[i] = analog_read(p)
        for i, p in enumerate(RIGHT_GLOVE_STATE_PINS):
            sensor_msg.right_tactile_glove_states[i] = analog_read(p)

        # Update sync pulse state
        sensor_msg.sync_pulse_state = self.sync_pulse_control_pin_state == HIGH
        sensor_msg.sync_pulse_last_time_on = (
            self.sync_pulse_last_time_on.to_msg()
        )
        sensor_msg.sync_pulse_last_time_off = (
            self.sync_pulse_last_time_off.to_msg()
        )

        self.sensor_pub.publish(sensor_msg)


async def main_async(args=None):
    rclpy.init(args=args)

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
            await executor.spin()
        finally:
            print("Shutting down mock teensy")
            mock_teensy.destroy_node()
            print("Shutting down executor")
            executor.shutdown()
    finally:
        print("Shutting down rclpy")
        rclpy.try_shutdown()  # type: ignore


def main(args=None):
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        pass
