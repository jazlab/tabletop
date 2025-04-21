import asyncio
import random
import time
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
ARM_DOOR_CONTROL_PIN = 1
SMARTGLASS_CONTROL_PIN = 3
REWARD_CONTROL_PIN = 4
HAND_FIXATION_STATE_PIN = 34
ARM_DOOR_CLOSED_STATE_PIN = 37
SYNC_PULSE_PIN = 9
GLOVE_STATE_PINS = [A0, A1, A2, A3, A4]

# Simulated state for digital pins
digital_pin_states = {
    ARM_DOOR_CONTROL_PIN: LOW,
    SMARTGLASS_CONTROL_PIN: LOW,
    REWARD_CONTROL_PIN: LOW,
    ARM_DOOR_CLOSED_STATE_PIN: random.choice([HIGH, LOW]),
    HAND_FIXATION_STATE_PIN: random.choice([HIGH, LOW]),
    SYNC_PULSE_PIN: LOW,
}

analog_pin_states = {p: random.randint(0, 1023) for p in GLOVE_STATE_PINS}

ARM_DOOR_PERIOD_MS = 1000


class ArmDoorState(Enum):
    OPEN = 0
    CLOSED = 1
    OPENING = 2
    CLOSING = 3


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
        "simulate": True,
        "simulate_smartglass_max_delay_sec": 0.1,
        "simulate_arm_door_max_delay_sec": 0.5,
        "simulate_hand_fixation_min_press_sec": 3.0,
        "simulate_hand_fixation_max_press_sec": 5.0,
        "simulate_hand_fixation_min_release_sec": 0.1,
        "simulate_hand_fixation_max_release_sec": 0.6,
    }

    def __init__(self):
        # Initialize base node

        self.log_pub = None
        super().__init__("teensy")

        # Log publisher

        self.log_pub = self.create_publisher(String, "teensy/log", 10)

        # Simulation parameters

        self.simulate: bool = self.get_parameter_wrapper("simulate")
        self.simulate_arm_door_max_delay_sec: float = (
            self.get_parameter_wrapper("simulate_arm_door_max_delay_sec")
        )
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

        if self.simulate:
            self.monkey_loop = asyncio.create_task(
                monkey_loop(
                    self.simulate_hand_fixation_min_press_sec,
                    self.simulate_hand_fixation_max_press_sec,
                    self.simulate_hand_fixation_min_release_sec,
                    self.simulate_hand_fixation_max_release_sec,
                )
            )

        # State variables

        ## Timestamps for hand fixation button presses and releases
        self.hand_fixation_last_time_pressed_ms = int(time.time() * 1000)
        self.hand_fixation_last_time_released_ms = int(time.time() * 1000)

        ## Sync pulse state
        self.sync_pulse_state = False

        ## Reward state
        self.reward_state = False

        ## Arm door state
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

        # Periodic timers

        self.sensor_timer = self.create_timer(
            0.05, self.sensor_timer_callback
        )  # 50 ms period

        self.sync_base_timer = self.create_timer(
            1.0, self.sync_pulse_base_timer_callback
        )  # 1 second period

        # One-shot timers for delayed state updates

        self.simulate_arm_door_delay_timer = None
        self.simulate_smartglass_delay_timer = None
        self.reward_duration_timer = None
        self.sync_start_timer = None
        self.sync_end_timer = None

    def log(self, message: str, severity: str = DEFAULT_LOG_SEVERITY):
        super().log(message, severity)
        if self.log_pub is not None:
            self.log_pub.publish(String(data=message))

    def get_arm_door_callback(
        self, request: GetArmDoor.Request, response: GetArmDoor.Response
    ):
        response.is_closed = digital_read(ARM_DOOR_CLOSED_STATE_PIN)
        response.state = self.arm_door_state.value
        response.success = True
        response.message = f"Arm door is {self.arm_door_state.name} (closed_pin={response.is_closed})"
        self.log(response.message)
        return response

    def get_reward_callback(
        self, request: GetReward.Request, response: GetReward.Response
    ):
        response.success = True
        response.is_active = self.reward_state
        response.message = (
            f"Reward is {'active' if response.is_active else 'inactive'}"
        )
        self.log(response.message)
        return response

    def get_hand_fixation_callback(
        self,
        request: GetHandFixation.Request,
        response: GetHandFixation.Response,
    ):
        # Set the response message
        response.success = True
        response.is_pressed = digital_read(HAND_FIXATION_STATE_PIN)
        response.last_time_pressed_ms = self.hand_fixation_last_time_pressed_ms
        response.last_time_released_ms = (
            self.hand_fixation_last_time_released_ms
        )
        response.message = f"Hand fixation is {'pressed' if response.is_pressed else 'released'}"
        self.log(response.message)

        return response

    def set_smartglass_callback(
        self, request: SetSmartglass.Request, response: SetSmartglass.Response
    ):
        # Update the control pin
        digital_write(SMARTGLASS_CONTROL_PIN, request.is_revealed)

        # Set the response message
        response.success = True
        response.message = (
            f"Smartglass {'revealed' if request.is_revealed else 'occluded'}"
        )
        self.log(response.message)

        return response

    def set_arm_door_callback(
        self, request: SetArmDoor.Request, response: SetArmDoor.Response
    ):
        # Check if an arm door timer already exists (door is already moving)
        if self.simulate_arm_door_delay_timer is not None:
            response.message = "Arm door already opening or closing!"
            response.success = False
            self.log(response.message, severity="WARN")
            return response

        if request.open:  # Request to open the door
            if self.arm_door_state != ArmDoorState.CLOSED:
                response.message = (
                    "Error: Arm door state is not CLOSED before opening"
                )
                response.success = False
                self.log(response.message, severity="WARN")
                return response
            if not digital_read(ARM_DOOR_CLOSED_STATE_PIN):
                response.message = "Error: Arm door closed state pin is not HIGH before opening"
                response.success = False
                self.log(response.message, severity="WARN")
                return response
            # Update the control pin
            digital_write(ARM_DOOR_CONTROL_PIN, HIGH)
            self.arm_door_state = ArmDoorState.OPENING
            target_state = ArmDoorState.OPEN
            target_pin_state = LOW
            action = "open"
        else:  # Request to close the door
            if self.arm_door_state != ArmDoorState.OPEN:
                response.message = (
                    "Error: Arm door state is not OPEN before closing"
                )
                response.success = False
                self.log(response.message, severity="WARN")
                return response
            if digital_read(ARM_DOOR_CLOSED_STATE_PIN):
                response.message = "Error: Arm door closed state pin is not LOW before closing"
                response.success = False
                self.log(response.message, severity="WARN")
                return response
            # Update the control pin
            digital_write(
                ARM_DOOR_CONTROL_PIN, LOW
            )  # Control pin HIGH might mean open, LOW means close
            self.arm_door_state = ArmDoorState.CLOSING
            target_state = ArmDoorState.CLOSED
            target_pin_state = HIGH
            action = "close"

        # Schedule a timer to update the state pin after a random delay
        delay = random.uniform(0.0, self.simulate_arm_door_max_delay_sec)
        self.simulate_arm_door_delay_timer = self.create_timer(
            delay,
            lambda: self.arm_door_delay_timer_callback(
                target_state=target_state,
                target_pin_state=target_pin_state,
                delay=delay,
            ),
        )

        # Set the response message
        response.success = True
        response.message = f"Arm door {action} started"
        self.log(response.message)

        return response

    def arm_door_delay_timer_callback(
        self, target_state: ArmDoorState, target_pin_state: bool, delay: float
    ):
        # Update the pin state
        digital_write(ARM_DOOR_CLOSED_STATE_PIN, target_pin_state)
        # Update the internal state
        self.arm_door_state = target_state
        # Stop the control pin (assuming LOW stops the motor action)
        # digital_write(ARM_DOOR_CONTROL_PIN, LOW) # Might need separate OPEN/CLOSE pins or different logic

        timer = self.simulate_arm_door_delay_timer
        assert timer is not None
        timer.cancel()
        self.simulate_arm_door_delay_timer = None

        self.log(
            f"Arm door reached state {target_state.name} after {delay:.2f} seconds"
        )

    def set_reward_callback(
        self, request: SetReward.Request, response: SetReward.Response
    ):
        # Check if a reward timer already exists
        if self.reward_duration_timer is not None:
            msg = "Reward duration timer already exists"
            self.log(msg, severity="WARN")
            response.message = msg
            response.success = False
            return response

        # Update the control pin
        digital_write(REWARD_CONTROL_PIN, HIGH)
        self.reward_state = True
        # Schedule a timer to update the state pin after a random delay
        duration_ms = request.duration_ms
        duration_sec = duration_ms / 1000.0
        self.reward_duration_timer = self.create_timer(
            duration_sec,
            lambda: self.delay_timer_callback(
                timer_name="reward_duration_timer",
                pin=REWARD_CONTROL_PIN,
                state=LOW,
                delay=duration_sec,
            ),
        )

        # Set the response message
        response.success = True
        response.message = f"Reward started for {duration_ms} ms"
        self.log(response.message)
        return response

    def delay_timer_callback(
        self, timer_name: str, pin: int, state: bool, delay: float
    ):
        # Update the pin state
        digital_write(pin, state)
        if timer_name == "reward_duration_timer":
            self.reward_state = False
        # Note: Arm door state is now handled in its specific callback

        timer = getattr(self, timer_name)
        assert timer is not None
        timer.cancel()
        setattr(self, timer_name, None)

        self.log(f"{timer_name} finished after {delay:.2f} seconds")

    def read_sensors(self) -> TeensySensor:
        sensor_msg = TeensySensor()
        sensor_msg.arm_door_closed_state = digital_read(
            ARM_DOOR_CLOSED_STATE_PIN
        )
        sensor_msg.arm_door_state = self.arm_door_state.value
        sensor_msg.fixation_button_state = digital_read(
            HAND_FIXATION_STATE_PIN
        )
        glove_states = []
        for p in GLOVE_STATE_PINS:
            glove_states.append(analog_read(p))
        sensor_msg.tactile_glove_states = glove_states
        sensor_msg.sync_pulse_state = self.sync_pulse_state
        # Add missing fields from C++ version if necessary
        # sensor_msg.sync_pulse_last_time_on_ms = ...
        # sensor_msg.sync_pulse_last_time_off_ms = ...
        return sensor_msg

    def sensor_timer_callback(self):
        sensor_msg = self.read_sensors()

        # Check if the fixation button state has changed
        if sensor_msg.fixation_button_state:
            self.hand_fixation_last_time_pressed_ms = int(time.time() * 1000)
        else:
            self.hand_fixation_last_time_released_ms = int(time.time() * 1000)

        self.sensor_pub.publish(sensor_msg)

    def sync_pulse_base_timer_callback(self):
        # Only schedule a sync pulse if one is not already active
        assert self.sync_pulse_state is False
        # Generate a random delay between 0 and 200 ms
        delay = random.uniform(0, 0.2)
        self.get_logger().debug(
            f"Scheduling sync pulse start in {delay * 1000:.1f} ms"
        )
        # Create a one-shot timer for sync pulse start
        self.sync_start_timer = self.create_timer(
            delay, self.sync_pulse_start_callback
        )

    def sync_pulse_start_callback(self):
        if self.sync_start_timer:
            self.sync_start_timer.cancel()
            self.sync_start_timer = None
        digital_write(SYNC_PULSE_PIN, HIGH)
        self.sync_pulse_state = True
        self.get_logger().debug("Sync pulse started")
        # Create a one-shot timer to end the sync pulse after 100 ms
        self.sync_end_timer = self.create_timer(
            0.1, self.sync_pulse_end_callback
        )

    def sync_pulse_end_callback(self):
        if self.sync_end_timer:
            self.sync_end_timer.cancel()
            self.sync_end_timer = None
        digital_write(SYNC_PULSE_PIN, LOW)
        self.sync_pulse_state = False
        self.get_logger().debug("Sync pulse ended")


async def main_async(args=None):
    rclpy.init(args=args)

    try:
        executor = AIOExecutor()
        mock_teensy = MockTeensy()
        executor.add_node(mock_teensy)

        try:
            await executor.spin()
        finally:
            print("Shutting down executor")
            executor.shutdown()
            print("Shutting down mock teensy")
            mock_teensy.destroy_node()
    finally:
        print("Shutting down rclpy")
        rclpy.shutdown()


def main():
    asyncio.run(main_async())
