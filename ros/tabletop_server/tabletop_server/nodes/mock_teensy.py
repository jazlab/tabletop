import random
import time

import rclpy
from std_msgs.msg import String
from tabletop_msgs.msg import TeensySensor
from tabletop_msgs.srv import (
    GetArmDoor,
    GetHandFixation,
    GetReward,
    GetSmartglass,
    SetArmDoor,
    SetReward,
    SetSmartglass,
)

from tabletop_server.nodes.base import DEFAULT_LOG_SEVERITY, BaseNode

# Define constants for digital pin states
HIGH = True
LOW = False

A0 = 34
A1 = 35
A2 = 36
A3 = 37
A4 = 38

# Define pin assignments similar to main.cpp
ARM_DOOR_CONTROL_PIN = 1
SMARTGLASS_CONTROL_PIN = 2
REWARD_CONTROL_PIN = 3
ARM_DOOR_STATE_PIN = 4
SMARTGLASS_STATE_PIN = 5
REWARD_STATE_PIN = 6
HAND_FIXATION_STATE_PIN = 7
SYNC_PULSE_PIN = 9
GLOVE_STATE_PINS = [A0, A1, A2, A3, A4]

# Simulated state for digital pins
digital_pin_states = {
    ARM_DOOR_CONTROL_PIN: LOW,
    SMARTGLASS_CONTROL_PIN: LOW,
    REWARD_CONTROL_PIN: LOW,
    ARM_DOOR_STATE_PIN: random.choice([HIGH, LOW]),
    SMARTGLASS_STATE_PIN: random.choice([HIGH, LOW]),
    HAND_FIXATION_STATE_PIN: random.choice([HIGH, LOW]),
    SYNC_PULSE_PIN: LOW,
}

analog_pin_states = {p: random.randint(0, 1023) for p in GLOVE_STATE_PINS}


def digital_write(pin, value: bool):
    digital_pin_states[pin] = value


def digital_read(pin) -> bool:
    return digital_pin_states[pin]


def analog_read(pin) -> int:
    # Simulate an analog read with a random value
    return analog_pin_states[pin]


class MockTeensy(BaseNode):
    default_params = BaseNode.default_params | {
        "simulate": True,
        "simulate_smartglass_max_delay": 0.1,
        "simulate_arm_door_max_delay": 1.0,
        "simulate_hand_fixation_max_delay": 5.0,
    }

    def __init__(self):
        super().__init__("teensy")

        # Simulation parameters
        self.simulate: bool = self.get_parameter_wrapper("simulate")
        self.smartglass_max_delay: float = self.get_parameter_wrapper(
            "simulate_smartglass_max_delay"
        )
        self.arm_door_max_delay: float = self.get_parameter_wrapper(
            "simulate_arm_door_max_delay"
        )
        self.hand_fixation_max_delay: float = self.get_parameter_wrapper(
            "simulate_hand_fixation_max_delay"
        )

        # One-shot timers for delayed state updates
        self.simulate_arm_door_delay_timer = None
        self.simulate_smartglass_delay_timer = None
        self.reward_duration_timer = None
        self.sync_start_timer = None
        self.sync_end_timer = None

        # State variables

        # Timestamps for hand fixation button presses and releases
        self.hand_fixation_last_time_pressed = int(time.time() * 1000)
        self.hand_fixation_last_time_released = int(time.time() * 1000)

        # Last sensor message
        self.last_sensor_msg = self.read_sensors()

        # Sync pulse state
        self.sync_pulse_state = False

        # Create publishers
        self.sensor_pub = self.create_publisher(
            TeensySensor, "teensy/sensors", 10
        )
        self.log_pub = self.create_publisher(String, "teensy/log", 10)

        # Create services
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
        self.get_smartglass_service = self.create_service(
            GetSmartglass,
            "teensy/get_smartglass",
            self.get_smartglass_callback,
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

        # Create timers
        # Sensor update timer (simulate sensor_timer_callback from main.cpp)
        self.sensor_timer = self.create_timer(
            0.05, self.sensor_timer_callback
        )  # 50 ms period

        # Sync pulse base timer (simulate sync_pulse_base_timer from main.cpp)
        self.sync_base_timer = self.create_timer(
            1.0, self.sync_pulse_base_timer_callback
        )  # 1 second period

    def log(self, message: str, severity: str = DEFAULT_LOG_SEVERITY):
        super().log(message, severity)
        self.log_pub.publish(String(data=message))

    def get_smartglass_callback(
        self, request: GetSmartglass.Request, response: GetSmartglass.Response
    ):
        response.is_revealed = digital_read(SMARTGLASS_STATE_PIN)
        response.success = True
        response.message = f"Smartglass is {'revealed' if response.is_revealed else 'occluded'}"
        return response

    def get_arm_door_callback(
        self, request: GetArmDoor.Request, response: GetArmDoor.Response
    ):
        response.is_open = digital_read(ARM_DOOR_STATE_PIN)
        response.success = True
        response.message = (
            f"Arm door is {'open' if response.is_open else 'closed'}"
        )
        return response

    def get_reward_callback(
        self, request: GetReward.Request, response: GetReward.Response
    ):
        response.is_active = digital_read(REWARD_STATE_PIN)
        response.success = True
        response.message = (
            f"Reward is {'active' if response.is_active else 'inactive'}"
        )
        return response

    def get_hand_fixation_callback(
        self,
        request: GetHandFixation.Request,
        response: GetHandFixation.Response,
    ):
        response.is_pressed = digital_read(HAND_FIXATION_STATE_PIN)
        response.last_time_pressed = self.hand_fixation_last_time_pressed
        response.last_time_released = self.hand_fixation_last_time_released
        response.success = True
        response.message = f"Hand fixation is {'pressed' if response.is_pressed else 'released'}"
        return response

    def set_smartglass_callback(
        self, request: SetSmartglass.Request, response: SetSmartglass.Response
    ):
        # Update the control pin
        digital_write(SMARTGLASS_CONTROL_PIN, request.is_revealed)

        # Check if a smartglass timer already exists
        if self.simulate_smartglass_delay_timer is not None:
            response.message = "Smartglass timer already exists"
            response.success = False
            return response

        # Schedule a timer to update the state pin after a random delay
        delay = random.uniform(0.0, self.smartglass_max_delay)
        self.simulate_smartglass_delay_timer = self.create_timer(
            delay,
            lambda: self.delay_timer_callback(
                timer_name="smartglass_state_timer",
                pin=SMARTGLASS_STATE_PIN,
                state=request.is_revealed,
                delay=delay,
            ),
        )
        # Set the response message
        msg_str = f"Smartglass {'revealed' if request.is_revealed else 'occluded'} started"
        self.log(msg_str)
        response.message = msg_str
        response.success = True
        return response

    def set_arm_door_callback(
        self, request: SetArmDoor.Request, response: SetArmDoor.Response
    ):
        # Update the control pin
        digital_write(ARM_DOOR_CONTROL_PIN, request.is_open)

        # Check if an arm door timer already exists
        if self.simulate_arm_door_delay_timer is not None:
            response.message = "Arm door timer already exists"
            response.success = False
            return response

        # Schedule a timer to update the state pin after a random delay
        delay = random.uniform(0.0, self.arm_door_max_delay)
        self.simulate_arm_door_delay_timer = self.create_timer(
            delay,
            lambda: self.delay_timer_callback(
                timer_name="arm_door_state_timer",
                pin=ARM_DOOR_STATE_PIN,
                state=request.is_open,
                delay=delay,
            ),
        )

        msg_str = f"Arm door {'open' if request.is_open else 'closed'} started"
        self.log(msg_str)
        response.message = msg_str
        response.success = True
        return response

    def set_reward_callback(
        self, request: SetReward.Request, response: SetReward.Response
    ):
        duration_ms = request.duration_ms
        duration_sec = duration_ms / 1000.0
        digital_write(REWARD_CONTROL_PIN, True)

        # Check if a reward timer already exists
        if self.reward_duration_timer is not None:
            msg = "Reward duration timer already exists"
            self.log(msg, severity="WARN")
            response.message = msg
            response.success = False
            return response

        # Schedule a timer to update the state pin after a random delay
        self.reward_duration_timer = self.create_timer(
            duration_sec,
            lambda: self.delay_timer_callback(
                timer_name="reward_duration_timer",
                pin=REWARD_STATE_PIN,
                state=False,
                delay=duration_sec,
            ),
        )

        # Set the response message
        msg_str = f"Reward started for {duration_ms} ms"
        self.log(msg_str)
        response.message = msg_str
        response.success = True
        return response

    def delay_timer_callback(
        self, timer_name: str, pin: int, state: bool, delay: float
    ):
        # Update the pin state
        digital_write(pin, state)

        msg_str = f"{timer_name} delayed for {delay} seconds"
        self.log(msg_str)

        timer = getattr(self, timer_name)
        assert timer is not None
        timer.cancel()
        setattr(self, timer_name, None)

    def read_sensors(self) -> TeensySensor:
        sensor_msg = TeensySensor()
        sensor_msg.arm_door_state = digital_read(ARM_DOOR_STATE_PIN)
        sensor_msg.smartglass_state = digital_read(SMARTGLASS_STATE_PIN)
        sensor_msg.fixation_button_state = digital_read(
            HAND_FIXATION_STATE_PIN
        )
        glove_states = []
        for p in GLOVE_STATE_PINS:
            glove_states.append(analog_read(p))
        sensor_msg.tactile_glove_states = glove_states
        sensor_msg.sync_pulse_state = self.sync_pulse_state
        return sensor_msg

    def sensor_timer_callback(self):
        sensor_msg = self.read_sensors()

        # Check if the fixation button state has changed
        if sensor_msg.fixation_button_state:
            self.hand_fixation_last_time_pressed = int(time.time() * 1000)
        else:
            self.hand_fixation_last_time_released = int(time.time() * 1000)

        self.sensor_pub.publish(sensor_msg)
        self.last_sensor_msg = sensor_msg

    def sync_pulse_base_timer_callback(self):
        # Only schedule a sync pulse if one is not already active
        if not self.sync_pulse_state:
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
        digital_write(SYNC_PULSE_PIN, True)
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
        digital_write(SYNC_PULSE_PIN, False)
        self.sync_pulse_state = False
        self.get_logger().debug("Sync pulse ended")


def main(args=None):
    rclpy.init(args=args)

    try:
        executor: rclpy.Executor = rclpy.executors.SingleThreadedExecutor()  # type: ignore
        mock_teensy = MockTeensy()
        executor.add_node(mock_teensy)

        try:
            executor.spin()
        finally:
            print("Shutting down executor")
            executor.shutdown()
            print("Shutting down mock teensy")
            mock_teensy.destroy_node()
    finally:
        print("Shutting down rclpy")
        rclpy.shutdown()
