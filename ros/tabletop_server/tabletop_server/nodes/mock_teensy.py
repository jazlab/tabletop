import asyncio
import random
from enum import Enum

import rclpy
from std_msgs.msg import String
from tabletop_msgs.msg import TeensySensor
from tabletop_msgs.srv import (
    GetArmDoor,
    GetHandFixation,
    GetReward,
    SetArmDoor,
    SetReward,
    SetSmartglass,
)

from tabletop_server.executor import AIOExecutor
from tabletop_server.nodes.base import BaseNode

DEFAULT_LOG_SEVERITY = "INFO"

# Define constants for digital pin states
HIGH = True
LOW = False

A0 = 14
A1 = 15
A2 = 16
A3 = 17
A4 = 18

# Define pin assignments similar to main.cpp
ARM_DOOR_OPEN_CONTROL_PIN = 1
ARM_DOOR_CLOSE_CONTROL_PIN = 2
SMARTGLASS_CONTROL_PIN = 3
REWARD_CONTROL_PIN = 4
HAND_FIXATION_STATE_PIN = 34
ARM_DOOR_CLOSED_STATE_PIN = 37
SYNC_PULSE_PIN = 9
GLOVE_STATE_PINS = [A0, A1, A2, A3, A4]

# Simulated state for digital pins
digital_pin_states = {
    ARM_DOOR_OPEN_CONTROL_PIN: LOW,
    ARM_DOOR_CLOSE_CONTROL_PIN: LOW,
    SMARTGLASS_CONTROL_PIN: LOW,
    REWARD_CONTROL_PIN: LOW,
    ARM_DOOR_CLOSED_STATE_PIN: random.choice([HIGH, LOW]),
    HAND_FIXATION_STATE_PIN: LOW,
    SYNC_PULSE_PIN: LOW,
}

analog_pin_states = {p: random.randint(0, 1023) for p in GLOVE_STATE_PINS}


SENSOR_PERIOD_MS = 10
SYNC_PULSE_BASE_PERIOD_MS = 1000
SYNC_PULSE_DELAY_RANGE_MS = 200
SYNC_PULSE_DURATION_MS = 100
ARM_DOOR_PERIOD_MS = 1000


class ArmDoorState(Enum):
    OPEN = 0
    OPENING = 1
    CLOSED = 2
    CLOSING = 3


def change_pin_state(pin, value: bool):
    """Change a pin state

    Used to change the state of a pin that is not directly controllable by the Teensy
    for simulation purposes.
    """
    digital_pin_states[pin] = value


def digital_write(pin, value: bool):
    digital_pin_states[pin] = value


def digital_read(pin) -> bool:
    return digital_pin_states[pin]


def analog_read(pin) -> int:
    # Simulate an analog read with a random value
    return analog_pin_states[pin]


async def monkey_loop(
    min_press_sec: float,
    max_press_sec: float,
    min_release_sec: float,
    max_release_sec: float,
):
    while True:
        if digital_read(HAND_FIXATION_STATE_PIN):
            delay = random.uniform(min_press_sec, max_press_sec)
        else:
            delay = random.uniform(min_release_sec, max_release_sec)
        await asyncio.sleep(delay)
        digital_write(
            HAND_FIXATION_STATE_PIN, not digital_read(HAND_FIXATION_STATE_PIN)
        )


class MockTeensy(BaseNode):
    default_params = BaseNode.default_params | {
        "simulate_hand_fixation_min_press_sec": 10.0,
        "simulate_hand_fixation_max_press_sec": 15.0,
        "simulate_hand_fixation_min_release_sec": 0.1,
        "simulate_hand_fixation_max_release_sec": 0.6,
    }

    def __init__(self):
        self.log_pub = None
        super().__init__("teensy")

        # Log publisher
        self.log_pub = self.create_publisher(String, "teensy/log", 10)

        # Simulation parameters
        self.simulate_hand_fixation_min_press_sec: float = (
            self.get_parameter_wrapper("simulate_hand_fixation_min_press_sec")
        )
        self.simulate_hand_fixation_max_press_sec: float = (
            self.get_parameter_wrapper("simulate_hand_fixation_max_press_sec")
        )
        self.simulate_hand_fixation_min_release_sec: float = (
            self.get_parameter_wrapper(
                "simulate_hand_fixation_min_release_sec"
            )
        )
        self.simulate_hand_fixation_max_release_sec: float = (
            self.get_parameter_wrapper(
                "simulate_hand_fixation_max_release_sec"
            )
        )

        # Monkey loop
        self.monkey_loop = asyncio.create_task(
            monkey_loop(
                self.simulate_hand_fixation_min_press_sec,
                self.simulate_hand_fixation_max_press_sec,
                self.simulate_hand_fixation_min_release_sec,
                self.simulate_hand_fixation_max_release_sec,
            )
        )

        # State variables
        self.hand_fixation_last_time_pressed_ms = int(self.time() * 1000)
        self.hand_fixation_last_time_released_ms = int(self.time() * 1000)

        self.sync_pulse_state = False
        self.sync_pulse_last_time_on_ms = int(self.time() * 1000)
        self.sync_pulse_last_time_off_ms = int(self.time() * 1000)

        self.reward_active = False

        if digital_read(ARM_DOOR_CLOSED_STATE_PIN):
            self.arm_door_state = ArmDoorState.CLOSED
        else:
            self.arm_door_state = ArmDoorState.OPEN

        # Publishers
        self.sensor_pub = self.create_publisher(
            TeensySensor, "teensy/sensors", 10
        )

        # Services
        self.set_arm_door_service = self.create_service(
            SetArmDoor,
            "teensy/set_arm_door",
            self.set_arm_door_callback,
        )
        self.get_arm_door_service = self.create_service(
            GetArmDoor,
            "teensy/get_arm_door",
            self.get_arm_door_callback,
        )

        self.set_smartglass_service = self.create_service(
            SetSmartglass,
            "teensy/set_smartglass",
            self.set_smartglass_callback,
        )

        self.set_reward_service = self.create_service(
            SetReward,
            "teensy/set_reward",
            self.set_reward_callback,
        )
        self.get_reward_service = self.create_service(
            GetReward,
            "teensy/get_reward",
            self.get_reward_callback,
        )

        self.get_hand_fixation_service = self.create_service(
            GetHandFixation,
            "teensy/get_hand_fixation",
            self.get_hand_fixation_callback,
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
        self.arm_door_timer = self.create_timer(
            ARM_DOOR_PERIOD_MS / 1000.0,
            self.arm_door_timer_callback,
            autostart=False,
        )

    def log(self, message: str, severity: str = DEFAULT_LOG_SEVERITY):
        super().log(message, severity)
        if self.log_pub is not None:
            self.log_pub.publish(String(data=message))

    def sensor_timer_callback(self):
        # Populate sensor message
        sensor_msg = TeensySensor()
        sensor_msg.arm_door_state = self.arm_door_state.value
        sensor_msg.arm_door_closed_state = digital_read(
            ARM_DOOR_CLOSED_STATE_PIN
        )
        sensor_msg.fixation_button_state = digital_read(
            HAND_FIXATION_STATE_PIN
        )

        # Update hand fixation timestamps
        if sensor_msg.fixation_button_state:
            self.hand_fixation_last_time_pressed_ms = int(self.time() * 1000)
        else:
            self.hand_fixation_last_time_released_ms = int(self.time() * 1000)

        # Update tactile glove states
        for i, p in enumerate(GLOVE_STATE_PINS):
            sensor_msg.tactile_glove_states[i] = analog_read(p)

        # Update sync pulse state
        sensor_msg.sync_pulse_state = self.sync_pulse_state
        sensor_msg.sync_pulse_last_time_on_ms = self.sync_pulse_last_time_on_ms
        sensor_msg.sync_pulse_last_time_off_ms = (
            self.sync_pulse_last_time_off_ms
        )

        self.sensor_pub.publish(sensor_msg)

    def sync_pulse_end_timer_callback(self):
        digital_write(SYNC_PULSE_PIN, LOW)
        self.sync_pulse_state = False
        self.sync_pulse_last_time_off_ms = int(self.time() * 1000)

        self.sync_pulse_end_timer.cancel()

        self.log(
            f"Sync pulse ended after {self.sync_pulse_last_time_off_ms - self.sync_pulse_last_time_on_ms} ms",
            severity="DEBUG",
        )

    def sync_pulse_start_timer_callback(self):
        digital_write(SYNC_PULSE_PIN, HIGH)
        self.sync_pulse_state = True
        self.sync_pulse_last_time_on_ms = int(self.time() * 1000)

        self.sync_pulse_start_timer.cancel()

        self.sync_pulse_end_timer.reset()

        self.log(
            f"Sync pulse started for {SYNC_PULSE_DURATION_MS} ms",
            severity="DEBUG",
        )

    def sync_pulse_base_timer_callback(self):
        assert not self.sync_pulse_state

        digital_write(SYNC_PULSE_PIN, LOW)
        self.sync_pulse_state = False

        # Generate a random delay between 0 and 200 ms
        delay = random.uniform(0, SYNC_PULSE_DELAY_RANGE_MS / 1000.0)
        self.log(
            f"Scheduling sync pulse start in {delay * 1000:.1f} ms",
            severity="DEBUG",
        )
        self.sync_pulse_start_timer.timer_period_ns = delay * 1e9
        self.sync_pulse_start_timer.reset()

        self.log(
            f"Sync pulse start timer scheduled for {delay * 1000:.1f} ms",
            severity="DEBUG",
        )

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

        duration_ms = request.duration_ms
        self.reward_timer.timer_period_ns = duration_ms * 1e6
        self.reward_timer.reset()

        # Set the response message
        response.success = True
        response.message = f"Reward started for {duration_ms} ms"
        self.log(response.message)
        return response

    def get_reward_callback(
        self, request: GetReward.Request, response: GetReward.Response
    ):
        response.is_active = self.reward_active
        response.success = True
        response.message = (
            f"Reward is {'active' if response.is_active else 'inactive'}"
        )
        self.log(response.message, severity="DEBUG")
        return response

    def arm_door_timer_callback(self):
        # Update the pin state
        digital_write(ARM_DOOR_OPEN_CONTROL_PIN, LOW)
        digital_write(ARM_DOOR_CLOSE_CONTROL_PIN, LOW)

        # Update the internal state
        match self.arm_door_state:
            case ArmDoorState.OPENING:
                self.arm_door_state = ArmDoorState.OPEN
            case ArmDoorState.CLOSING:
                self.arm_door_state = ArmDoorState.CLOSED
                change_pin_state(ARM_DOOR_CLOSED_STATE_PIN, HIGH)
            case _:
                assert False, "Arm door state is not OPENING or CLOSING when timer callback is called"

        self.arm_door_timer.cancel()

        self.log(f"Arm door reached state {self.arm_door_state.name}")

    def set_arm_door_callback(
        self, request: SetArmDoor.Request, response: SetArmDoor.Response
    ):
        assert (digital_read(ARM_DOOR_CLOSED_STATE_PIN) == HIGH) == (
            self.arm_door_state == ArmDoorState.CLOSED
        ), "Arm door state is not consistent with closed state pin"

        if not self.arm_door_timer.is_canceled():
            assert (
                self.arm_door_state
                in (
                    ArmDoorState.OPENING,
                    ArmDoorState.CLOSING,
                )
            ), "Arm door state is not OPENING or CLOSING when arm door timer is still running"
            assert (
                digital_read(ARM_DOOR_CLOSED_STATE_PIN) == LOW
            ), "Arm door closed state pin is not HIGH when arm door timer is still running"

        time_since_last_call_ms = self.arm_door_timer.time_since_last_call()

        if request.open:
            match self.arm_door_state:
                case ArmDoorState.OPEN:
                    response.success = True
                    response.message = "Arm door already open"
                    self.log(response.message)
                    return response
                case ArmDoorState.OPENING:
                    response.success = True
                    response.message = "Arm door is already opening"
                    self.log(response.message)
                    return response
                case ArmDoorState.CLOSED:
                    self.log("Arm door is closed, opening")
                    duration_ms = ARM_DOOR_PERIOD_MS
                    change_pin_state(ARM_DOOR_CLOSED_STATE_PIN, LOW)
                case ArmDoorState.CLOSING:
                    self.log("Arm door is closing, reversing")
                    duration_ms = time_since_last_call_ms

            digital_write(ARM_DOOR_OPEN_CONTROL_PIN, HIGH)
            digital_write(ARM_DOOR_CLOSE_CONTROL_PIN, LOW)
            self.arm_door_state = ArmDoorState.OPENING
        else:
            match self.arm_door_state:
                case ArmDoorState.OPEN:
                    self.log("Arm door is open, closing")
                    duration_ms = ARM_DOOR_PERIOD_MS
                case ArmDoorState.OPENING:
                    self.log("Arm door is opening, reversing")
                    duration_ms = time_since_last_call_ms
                case ArmDoorState.CLOSED:
                    response.success = True
                    response.message = "Arm door already closed"
                    self.log(response.message)
                    return response
                case ArmDoorState.CLOSING:
                    response.success = True
                    response.message = "Arm door is already closing"
                    self.log(response.message)
                    return response

            digital_write(ARM_DOOR_OPEN_CONTROL_PIN, LOW)
            digital_write(ARM_DOOR_CLOSE_CONTROL_PIN, HIGH)
            self.arm_door_state = ArmDoorState.CLOSING

        self.arm_door_timer.timer_period_ns = duration_ms * 1e6
        self.arm_door_timer.reset()

        # Set the response message
        response.success = True
        response.message = f"Arm door {'open' if request.open else 'close'} started for {duration_ms} ms"
        self.log(response.message)

        return response

    def get_arm_door_callback(
        self, request: GetArmDoor.Request, response: GetArmDoor.Response
    ):
        assert (digital_read(ARM_DOOR_CLOSED_STATE_PIN) == HIGH) == (
            self.arm_door_state == ArmDoorState.CLOSED
        ), "Arm door state is not consistent with closed state pin"

        response.is_closed = digital_read(ARM_DOOR_CLOSED_STATE_PIN)
        response.state = self.arm_door_state.value
        response.success = True
        response.message = f"Arm door closed state pin is {'HIGH' if response.is_closed else 'LOW'} and arm door state is {self.arm_door_state.name.lower()}"
        self.log(response.message, severity="DEBUG")
        return response

    def get_hand_fixation_callback(
        self,
        request: GetHandFixation.Request,
        response: GetHandFixation.Response,
    ):
        response.is_pressed = digital_read(HAND_FIXATION_STATE_PIN)
        response.last_time_pressed_ms = self.hand_fixation_last_time_pressed_ms
        response.last_time_released_ms = (
            self.hand_fixation_last_time_released_ms
        )
        response.success = True
        response.message = f"Hand fixation is {'pressed' if response.is_pressed else 'released'}"
        self.log(response.message, severity="DEBUG")

        return response

    def set_smartglass_callback(
        self, request: SetSmartglass.Request, response: SetSmartglass.Response
    ):
        digital_write(SMARTGLASS_CONTROL_PIN, request.reveal)
        response.success = True
        response.message = (
            f"Smartglass {'revealed' if request.reveal else 'occluded'}"
        )
        self.log(response.message)

        return response


async def main_async(args=None):
    rclpy.init(args=args)

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
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    except SystemExit:
        print("System exit")
    finally:
        print("Shutting down rclpy")
        rclpy.try_shutdown()


def main():
    asyncio.run(main_async())
