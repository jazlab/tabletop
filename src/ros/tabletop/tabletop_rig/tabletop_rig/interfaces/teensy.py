import asyncio
import threading
from collections.abc import Callable
from copy import copy, deepcopy
from typing import Literal, Optional, cast

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
    ###########################################################################
    ########## Initialization #################################################
    ###########################################################################

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
        self.teensy_sub = self.node.create_subscription(
            TeensySensor,
            "/teensy/sensor",
            self.teensy_sensor_callback,
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
        self.set_arm_lock_client = self.node.create_client(
            SetArmLock,
            "/teensy/set_arm_lock",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.set_reward_client = self.node.create_client(
            SetReward,
            "/teensy/set_reward",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.set_smartglass_client = self.node.create_client(
            SetSmartglass,
            "/teensy/set_smartglass",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Wait for ROS services
        self.log("Waiting for teensy services")
        self.set_arm_lock_client.wait_for_service()
        self.set_reward_client.wait_for_service()
        self.set_smartglass_client.wait_for_service()

        self.log("Teensy interface initialized")

    def register_subscription_callback(
        self, callback: Callable[[TeensySensor], None]
    ):
        """Register additional callback for teensy sensor subscription

        Args:
            callback: Callable that takes TeensySensor message as argument and returns None
        """
        self._additional_subscription_callback = callback

    ###########################################################################
    ########## ROS Interface ##################################################
    ###########################################################################

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

    def teensy_sensor_callback(self, msg: TeensySensor):
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
    ) -> SetArmLock.Response:
        """Set the arm lock state."""
        if arm not in ["left", "right", "both"]:
            raise ValueError("Invalid arm: must be 'left', 'right', or 'both'")

        left = arm in ["left", "both"]
        right = arm in ["right", "both"]

        response = await self.node.service_call_async(
            srv_request=SetArmLock.Request(
                left_arm=left, right_arm=right, lock=lock
            ),
            srv_client=self.set_arm_lock_client,
        )
        return cast(SetArmLock.Response, response)

    async def arm_lock_and_wait(self, timeout: Optional[float] = None) -> bool:
        """Lock arms and wait for safety laser to be unbroken

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

    async def set_smartglass(self, reveal: bool) -> SetSmartglass.Response:
        """Set the smartglass state."""
        self.log(f"Smartglass {'reveal' if reveal else 'occlude'}")
        response = await self.node.service_call_async(
            srv_request=SetSmartglass.Request(reveal=reveal),
            srv_client=self.set_smartglass_client,
        )
        return cast(SetSmartglass.Response, response)

    async def set_reward(
        self, activate: bool, duration: Optional[int | float] = None
    ) -> SetReward.Response:
        """Set the reward state."""
        if activate:
            self.log(f"Starting reward for {duration}s")
        else:
            self.log("Stopping reward")

        request = SetReward.Request(activate=activate)
        if duration is not None:
            if duration < 0:
                raise ValueError("Duration must be greater than 0!")
            request.duration = Duration(seconds=duration).to_msg()

        response = await self.node.service_call_async(
            srv_request=request, srv_client=self.set_reward_client
        )
        return cast(SetReward.Response, response)

    async def reward_and_wait(self, duration: float):
        """Start reward and wait for it to be active."""
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
                return True
        except TimeoutError:
            assert False, "Reward still active after duration"
