import asyncio
import threading
from collections.abc import Callable
from copy import copy, deepcopy
from typing import Literal, Optional

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSPresetProfiles
from tabletop_interfaces.msg import TeensySensor
from tabletop_interfaces.srv import (
    SetArmLock,
    SetReward,
    SetSmartglass,
)

from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.nodes.base import BaseNode


def noop(msg: TeensySensor):
    """Default additional_subscription_callback for teensy message topic (does nothing)"""
    pass


class TeensyInterface(BaseInterface):
    def __init__(
        self,
        node: BaseNode,
        additional_subscription_callback: Optional[
            Callable[[TeensySensor], None]
        ] = None,
    ):
        """Initializes the TeensyInterface"""
        super().__init__(node, "teensy_interface")

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

        # Wait for ROS services
        self.log("Waiting for teensy services")
        self._set_arm_lock_client.wait_for_service()
        self._set_reward_client.wait_for_service()
        self._set_smartglass_client.wait_for_service()

        self.log("Teensy interface initialized")

    # Properties

    @property
    def last_teensy_sensor(self) -> TeensySensor:
        """Get the last teensy sensor."""
        with self._teensy_sensor_lock:
            return deepcopy(self._last_teensy_sensor)

    @property
    def safe_to_execute(self) -> bool:
        """Get the is safe to execute state."""
        max_sensor_delay = self.node.get_parameter_wrapper(
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
        """Check if the robot is safe to execute."""
        return (
            msg.is_left_arm_locked
            and msg.is_right_arm_locked
            and not msg.is_safety_laser_broken
        )

    def _teensy_sensor_callback(self, msg: TeensySensor):
        """Callback for the teensy sensor."""
        required_time = self.node.get_parameter_wrapper(
            "teensy.safe_to_execute.required_time"
        )

        # Determine if the monkey is safe
        with self._teensy_sensor_lock:
            current_time = self.node.ros_time()
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
    ):
        """Set the arm lock state via the /teensy/set_arm_lock service

        Args:
            arm: The arm to change the lock state of ('left', 'right', or 'both')
            lock: Whether to lock or release the arm

        Returns:
            True if the service call was successful, False otherwise
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
        """Lock both arms and wait until safe to execute

        Args:
            timeout: Timeout in seconds. If None, the default timeout from
                parameters is used.

        Returns:
            True if arms were locked and safety laser was unbroken within the timeout,
            False otherwise.
        """
        self.log("Locking arms and waiting until safe to execute")
        await self.set_arm_lock("both", lock=True)

        spin_period = self.node.get_parameter_wrapper("teensy.spin_period")
        try:
            async with asyncio.timeout(timeout):
                while not self.safe_to_execute:
                    await asyncio.sleep(spin_period)
            return True
        except TimeoutError:
            return False

    async def set_smartglass(self, reveal: bool):
        """Set the smartglass state."""
        self.log(f"Smartglass {'reveal' if reveal else 'occlude'}")
        await self.node.service_call_async(
            srv_request=SetSmartglass.Request(reveal=reveal),
            srv_client=self._set_smartglass_client,
        )

    async def set_reward(
        self, activate: bool, duration: Optional[int | float] = None
    ):
        """Set the reward state."""
        request = SetReward.Request(activate=activate)
        if activate:
            if duration is None or duration <= 0:
                raise ValueError(
                    "If activating, reward duration must not be None and must be greater than 0"
                )
            request.duration = Duration(seconds=duration).to_msg()
            self.log(f"Starting reward for {duration}s")
        else:
            if duration is not None or duration != 0:
                raise ValueError(
                    "If not activating, reward duration must be None or 0"
                )
            self.log("Stopping reward")

        await self.node.service_call_async(
            srv_request=request, srv_client=self._set_reward_client
        )

    async def start_reward_and_wait(self, duration: float):
        """Start reward and wait for it to finish."""
        await self.set_reward(activate=True, duration=duration)

        spin_period = self.node.get_parameter_wrapper("teensy.spin_period")

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
