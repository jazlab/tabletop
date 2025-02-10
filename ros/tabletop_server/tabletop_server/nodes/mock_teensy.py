import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tabletop_msgs.msg import TeensySensor
from tabletop_msgs.srv import SetUint32


class MockTeensy(Node):
    def __init__(self):
        super().__init__("teensy")

        # Define pin assignments similar to main.cpp
        self.pin = {
            "arm_door_control": 1,
            "smartglass_control": 2,
            "reward_control": 3,
            "arm_door_state": 4,
            "smartglass_state": 5,
            "reward_state": 6,
            "hand_fixation_state": 7,
            "sync_pulse": 9,
            "glove_states": [10, 11, 12, 13, 14],
        }

        # Simulated state for digital pins
        self.pin_states = {}
        for p in [1, 2, 3, 4, 5, 6, 7, 9]:
            self.pin_states[p] = random.choice([True, False])

        # Sync pulse state
        self.sync_pulse_state = False
        self.sync_start_timer = None
        self.sync_end_timer = None

        # Reward timer for one-shot reward duration control
        self.reward_timer = None

        # Create publishers
        self.sensor_pub = self.create_publisher(TeensySensor, "sensors", 10)
        self.log_pub = self.create_publisher(String, "log", 10)

        # Create services
        self.arm_door_service = self.create_service(
            SetBool,
            "arm_door",
            self.arm_door_callback,
        )

        self.smartglass_service = self.create_service(
            SetBool,
            "smartglass",
            self.smartglass_callback,
        )

        self.reward_service = self.create_service(
            SetUint32,
            "reward",
            self.reward_callback,
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

    def digital_write(self, pin, value: bool):
        self.pin_states[pin] = value

    def digital_read(self, pin) -> bool:
        return self.pin_states.get(pin, False)

    def analog_read(self, pin) -> int:
        # Simulate an analog read with a random value
        return random.randint(0, 1023)

    def publish_log(self, message: str):
        msg = String()
        msg.data = message
        self.log_pub.publish(msg)

    def arm_door_callback(self, request, response):
        self.digital_write(self.pin["arm_door_control"], request.data)
        if request.data:
            msg_str = "Arm door opened"
        else:
            msg_str = "Arm door closed"
        self.get_logger().info(msg_str)
        response.message = msg_str
        response.success = True
        self.publish_log(msg_str)
        return response

    def smartglass_callback(self, request, response):
        self.digital_write(self.pin["smartglass_control"], request.data)
        if request.data:
            msg_str = "Smartglass revealed"
        else:
            msg_str = "Smartglass occluded"
        self.get_logger().info(msg_str)
        response.message = msg_str
        response.success = True
        self.publish_log(msg_str)
        return response

    def reward_callback(self, request, response):
        duration_ms = request.data
        duration_sec = duration_ms / 1000.0
        self.digital_write(self.pin["reward_control"], True)
        msg_str = f"Reward started for {duration_ms} ms"
        self.get_logger().info(msg_str)
        self.publish_log(msg_str)
        # Cancel any existing reward timer
        if self.reward_timer is not None:
            self.reward_timer.cancel()
            self.reward_timer = None
        # Create a one-shot timer for ending the reward
        self.reward_timer = self.create_timer(
            duration_sec, self.reward_timer_callback
        )
        response.message = msg_str
        response.success = True
        return response

    def reward_timer_callback(self):
        self.digital_write(self.pin["reward_control"], False)
        msg_str = "Reward ended"
        self.get_logger().info(msg_str)
        self.publish_log(msg_str)
        if self.reward_timer is not None:
            self.reward_timer.cancel()
            self.reward_timer = None

    def sensor_timer_callback(self):
        sensor_msg = TeensySensor()
        sensor_msg.arm_door_state = self.digital_read(
            self.pin["arm_door_state"]
        )
        sensor_msg.smartglass_state = self.digital_read(
            self.pin["smartglass_state"]
        )
        sensor_msg.fixation_button_state = self.digital_read(
            self.pin["hand_fixation_state"]
        )
        glove_states = []
        for p in self.pin["glove_states"]:
            glove_states.append(self.analog_read(p))
        sensor_msg.tactile_glove_states = glove_states
        sensor_msg.sync_pulse_state = self.sync_pulse_state
        self.sensor_pub.publish(sensor_msg)

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
        self.digital_write(self.pin["sync_pulse"], True)
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
        self.digital_write(self.pin["sync_pulse"], False)
        self.sync_pulse_state = False
        self.get_logger().debug("Sync pulse ended")


def main(args=None):
    rclpy.init(args=args)
    try:
        executor = rclpy.executors.SingleThreadedExecutor()
        mock_teensy = MockTeensy()
        executor.add_node(mock_teensy)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            mock_teensy.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
