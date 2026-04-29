"""Interface for Universal Robots dashboard control.

This module provides an interface to control a Universal Robots (UR) arm
through the UR Dashboard Server. The dashboard provides high-level robot
control including safety mode management, program loading/execution, and
error recovery.

The UR Dashboard Server exposes services for robot lifecycle management
that are essential for recovering from safety stops and protective stops.
"""

import asyncio
from typing import Optional, cast

from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.client import Client
from std_srvs.srv import Trigger
from ur_dashboard_msgs.action import SetMode
from ur_dashboard_msgs.msg import ProgramState, RobotMode, SafetyMode
from ur_dashboard_msgs.srv import (
    GetProgramState,
    GetRobotMode,
    GetSafetyMode,
    IsInRemoteControl,
    Load,
)

from tabletop_rig.exceptions import (
    ActionClientError,
    ServiceCallTimeoutError,
    ServiceCallUnsuccessfulError,
)
from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.nodes.base import AIOActionClient, BaseNode


class URInterface(BaseInterface):
    """Interface for Universal Robots nodes communication.

    Provides async methods to interact with the UR dashboard client and other
        UR robot driver nodes, such as:
    - Querying robot and safety modes
    - Loading programs and installations
    - Triggering dashboard commands (brake release, play, etc.)
    - Automated recovery sequences after safety events

    Service clients for every dashboard and state-helper endpoint are
    created at init time so that each call reuses the same long-lived
    client (enabling stable service introspection).
    """

    def __init__(
        self,
        node: BaseNode,
        name: str,
        *,
        simulate: bool,
        parameter_fallback_prefix: Optional[str] = None,
    ) -> None:
        """Initialize the UR interface.

        Args:
            ur_ns: ROS2 namespace of UR robot driver nodes
                (not including the node names)
            node: Parent ROS2 node for creating service clients.
        """
        super().__init__(
            node, name, parameter_fallback_prefix=parameter_fallback_prefix
        )

        self._simulate = simulate

        ur_ns = self.param("namespace")
        self._dashboard_ns = f"{ur_ns}/dashboard_client"
        self._state_helper_ns = f"{ur_ns}/ur_robot_state_helper"

        self.log(f"Waiting for {self._dashboard_ns} node")
        if not self.node.wait_for_node_blocking(self._dashboard_ns):
            raise RuntimeError(f"{self._dashboard_ns} node not available")

        self.log(f"Waiting for {self._state_helper_ns} node")
        if not self.node.wait_for_node_blocking(self._state_helper_ns):
            raise RuntimeError(f"{self._state_helper_ns} node not available")

        self._set_mode_client = AIOActionClient(
            node,
            SetMode,
            f"{self._state_helper_ns}/set_mode",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Parameter query service clients (one per node namespace)
        self._dashboard_get_parameters_client = self.node.create_client(
            GetParameters,
            f"{self._dashboard_ns}/get_parameters",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._state_helper_get_parameters_client = self.node.create_client(
            GetParameters,
            f"{self._state_helper_ns}/get_parameters",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Dashboard query service clients
        self._is_in_remote_control_client = self.node.create_client(
            IsInRemoteControl,
            f"{self._dashboard_ns}/is_in_remote_control",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._get_robot_mode_client = self.node.create_client(
            GetRobotMode,
            f"{self._dashboard_ns}/get_robot_mode",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._get_safety_mode_client = self.node.create_client(
            GetSafetyMode,
            f"{self._dashboard_ns}/get_safety_mode",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._get_program_state_client = self.node.create_client(
            GetProgramState,
            f"{self._dashboard_ns}/program_state",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Dashboard load service client
        self._load_program_client = self.node.create_client(
            Load,
            f"{self._dashboard_ns}/load_program",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Dashboard trigger service clients
        self._quit_client = self.node.create_client(
            Trigger,
            f"{self._dashboard_ns}/quit",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._connect_client = self.node.create_client(
            Trigger,
            f"{self._dashboard_ns}/connect",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._close_popup_client = self.node.create_client(
            Trigger,
            f"{self._dashboard_ns}/close_popup",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._close_safety_popup_client = self.node.create_client(
            Trigger,
            f"{self._dashboard_ns}/close_safety_popup",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._unlock_protective_stop_client = self.node.create_client(
            Trigger,
            f"{self._dashboard_ns}/unlock_protective_stop",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._stop_client = self.node.create_client(
            Trigger,
            f"{self._dashboard_ns}/stop",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._play_client = self.node.create_client(
            Trigger,
            f"{self._dashboard_ns}/play",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._restart_safety_client = self.node.create_client(
            Trigger,
            f"{self._dashboard_ns}/restart_safety",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self._connected = False

        self.log("UR interface initialized")

    async def _ensure_mock(
        self,
        srv_client: Client,
        node_ns: str,
        timeout: Optional[float] = None,
    ):
        self.log(
            f"Ensuring {node_ns} is running in {'mock' if self._simulate else 'real'} hardware mode"
        )

        response = cast(
            GetParameters.Response,
            await self.node.service_call_async(
                srv_request=GetParameters.Request(names=["is_mock"]),
                srv_client=srv_client,
                timeout=timeout,
            ),
        )
        values: list[ParameterValue] = list(response.values)
        assert len(values) <= 1

        is_mock = (
            len(values) == 1
            and values[0].type == ParameterType.PARAMETER_BOOL
            and values[0].bool_value
        )

        if self._simulate != is_mock:
            raise RuntimeError(
                f"simulate parameter is {self._simulate}, but {node_ns} node is "
                f"running in {'mock' if is_mock else 'real'} hardware mode. "
                f"Please ensure this node and the UR robot driver are launched "
                f"with the same robot_mode"
            )

    async def _trigger(
        self, srv_client: Client, timeout: Optional[float] = None
    ) -> Trigger.Response:
        """Call a dashboard Trigger service.

        Many dashboard commands (brake_release, play, close_popup, etc.)
        use the std_srvs/Trigger interface.

        Args:
            srv_client: The pre-built Trigger service client to call.

        Returns:
            The Trigger response with success status and message.
        """
        self.log(
            f"Triggering {srv_client.service_name} in UR Dashboard",
            severity="DEBUG",
        )
        response = await self.node.service_call_async(
            srv_request=Trigger.Request(),
            srv_client=srv_client,
            timeout=timeout,
        )
        return cast(Trigger.Response, response)

    async def _load_file(
        self,
        srv_client: Client,
        filename: str,
    ) -> Load.Response:
        """Load a program or installation file on the robot.

        Args:
            srv_client: The pre-built Load service client.
            filename: Path to the program file on the robot controller.

        Returns:
            The Load response with success status.
        """
        self.log(
            f"Loading {srv_client.service_name}: {filename} in UR Dashboard",
            severity="DEBUG",
        )
        response = await self.node.service_call_async(
            srv_request=Load.Request(filename=filename),
            srv_client=srv_client,
        )
        return cast(Load.Response, response)

    async def _is_in_remote_control(self) -> bool:
        """Check if the robot is in remote control mode.

        Remote control mode is required for programmatic dashboard control.
        If not in remote control, the operator must switch modes on the
        teach pendant.

        Returns:
            True if the robot is in remote control mode, False otherwise.
        """
        response = cast(
            IsInRemoteControl.Response,
            await self.node.service_call_async(
                srv_request=IsInRemoteControl.Request(),
                srv_client=self._is_in_remote_control_client,
            ),
        )
        return response.remote_control

    async def _wait_for_remote_control(
        self, timeout: Optional[float] = None
    ) -> None:
        async with asyncio.timeout(timeout):
            delay = self.param("check_remote_control_delay")
            remote_control = await self._is_in_remote_control()
            while not remote_control:
                self.log(
                    f"Dashboard is not in Remote Control mode, waiting {delay} "
                    f"seconds before retrying",
                    severity="WARN",
                )
                await asyncio.sleep(delay)
                remote_control = await self._is_in_remote_control()

    async def _set_robot_mode_running(
        self, stop_program: bool = True, play_program: bool = False
    ) -> None:
        """Set robot mode to RUNNING using SetMode action.

        Uses the UR robot state helper action to transition the robot
        to RUNNING mode. This handles brake release and power-on
        automatically.

        Raises:
            ActionError: If the SetMode action goal is
                not accepted or fails to complete successfully.
            RuntimeError: If the robot mode is not RUNNING after the
                action completes.
        """
        goal_handle = await self._set_mode_client.send_goal_async(
            SetMode.Goal(
                target_robot_mode=RobotMode.RUNNING,
                stop_program=stop_program,
                play_program=play_program,
            )
        )

        result: SetMode.Result = await self._set_mode_client.get_result_async(
            goal_handle
        )

        if not result.success:
            raise RuntimeError(
                f"UR SetMode action failed with message: {result.message}"
            )

        robot_mode = await self.get_robot_mode()
        if robot_mode.mode != RobotMode.RUNNING:
            raise RuntimeError(
                f"Robot mode should be RUNNING, actual mode: {robot_mode}"
            )

    async def get_robot_mode(self) -> RobotMode:
        """Query the current robot operating mode.

        Returns:
            RobotMode message indicating current state (RUNNING, IDLE,
            POWER_OFF, etc.).
        """
        response = cast(
            GetRobotMode.Response,
            await self.node.service_call_async(
                srv_request=GetRobotMode.Request(),
                srv_client=self._get_robot_mode_client,
            ),
        )
        return response.robot_mode

    async def get_safety_mode(self) -> SafetyMode:
        """Query the current robot safety mode.

        Returns:
            SafetyMode message indicating current state (NORMAL, REDUCED,
            PROTECTIVE_STOP, RECOVERY, etc.).
        """
        response = cast(
            GetSafetyMode.Response,
            await self.node.service_call_async(
                srv_request=GetSafetyMode.Request(),
                srv_client=self._get_safety_mode_client,
            ),
        )
        return response.safety_mode

    async def get_program_state(self) -> ProgramState:
        """Query the current program execution state.

        Returns:
            ProgramState message indicating current state (STOPPED, PLAYING,
            PAUSED, etc.).
        """
        response = cast(
            GetProgramState.Response,
            await self.node.service_call_async(
                srv_request=GetProgramState.Request(),
                srv_client=self._get_program_state_client,
            ),
        )
        return response.state

    async def is_ready(self) -> bool:
        # TODO: add logging and documentation
        if not self._connected:
            return False

        remote_control = await self._is_in_remote_control()
        if not remote_control:
            return False

        robot_mode = await self.get_robot_mode()
        if not robot_mode != RobotMode.RUNNING:
            return False

        program_state = await self.get_program_state()
        if not program_state != ProgramState.PLAYING:
            return False

        safety_mode = await self.get_safety_mode()
        if not safety_mode != SafetyMode.NORMAL:
            return False

        return True

    async def _reset_impl(self) -> None:
        """Execute full dashboard recovery sequence.

        Performs a comprehensive reset of the robot dashboard:
        1. Verify remote control mode is enabled
        2. Load the configured program (with reconnect on failure)
        3. Set robot mode to RUNNING via SetMode action
        4. Close any popup dialogs
        5. Unlock protective stops
        6. Start program execution
        7. Wait for NORMAL safety mode

        This is typically called after safety events, protective stops,
        or at startup to bring the robot to an operational state.

        Args:
            timeout: Maximum time for the entire reset sequence in seconds.
                If None, no timeout is applied.

        Raises:
            RuntimeError: If the dashboard is not in remote control mode.
            ServiceCallUnsuccessfulError: If a dashboard service call fails.
            ActionError: If the SetMode action fails.
        """
        self.log("Resetting dashboard")

        if not self._connected:
            await self._ensure_mock(
                self._dashboard_get_parameters_client, self._dashboard_ns
            )
            await self._ensure_mock(
                self._state_helper_get_parameters_client,
                self._state_helper_ns,
            )

            try:
                await self._trigger(self._quit_client)
            except ServiceCallUnsuccessfulError:
                pass

            try:
                await self._trigger(self._connect_client)
            except ServiceCallUnsuccessfulError as e:
                raise RuntimeError(
                    "Could not connect to dashboard client"
                ) from e

            await asyncio.sleep(1.0)

            await self._wait_for_remote_control()

            await self._load_file(
                self._load_program_client, self.param("program")
            )

            await self._set_robot_mode_running(
                stop_program=False, play_program=False
            )

            self._connected = True
        else:
            await self._wait_for_remote_control()

        # Close any popups and unlock protective stop
        await self._trigger(self._close_popup_client)
        await self._trigger(self._close_safety_popup_client)
        await self._trigger(self._unlock_protective_stop_client)

        await self._trigger(self._stop_client)

        await self._set_robot_mode_running(
            stop_program=False, play_program=False
        )
        await asyncio.sleep(0.5)
        await self._trigger(self._play_client)

        safety_mode = await self.get_safety_mode()
        delay = self.param("check_safety_mode_delay")
        while safety_mode.mode != SafetyMode.NORMAL:
            self.log(
                f"Safety mode is {safety_mode.mode}, retrying after {delay} seconds until NORMAL...",
                severity="WARN",
            )
            await asyncio.sleep(delay)
            safety_mode = await self.get_safety_mode()

        await asyncio.sleep(self.param("post_reset_delay"))

    async def reset(self, timeout: Optional[float] = None):
        """Execute full dashboard recovery sequence.

        Performs a comprehensive reset of the robot dashboard:
        1. Verify remote control mode is enabled
        2. Load the configured program (with reconnect on failure)
        3. Set robot mode to RUNNING via SetMode action
        4. Close any popup dialogs
        5. Unlock protective stops
        6. Start program execution
        7. Wait for NORMAL safety mode

        This is typically called after safety events, protective stops,
        or at startup to bring the robot to an operational state.

        Args:
            timeout: Maximum time for the entire reset sequence in seconds.
                If None, no timeout is applied.

        Raises:
            RuntimeError: If the dashboard is not in remote control mode.
            ServiceCallUnsuccessfulError: If a dashboard service call fails.
            ActionError: If the SetMode action fails.
        """
        max_attempts: int = self.param("max_reset_attempts")
        num_attempts_before_safety_restart: Optional[int] = self.param(
            "num_reset_attempts_before_safety_restart"
        )

        if (
            num_attempts_before_safety_restart is not None
            and num_attempts_before_safety_restart >= max_attempts
        ):
            raise ValueError(
                "num_reset_attempts_before_safety_restart parameter must be less than max_attempts"
            )

        async with asyncio.timeout(timeout):
            for i in range(max_attempts):
                try:
                    await self._reset_impl()
                    return
                except (
                    ServiceCallUnsuccessfulError,
                    ServiceCallTimeoutError,
                    ActionClientError,
                ) as e:
                    self.log(
                        f"Caught exception while resetting dashboard | {type(e).__name__}: {e}",
                        severity="WARN",
                    )

                    if (
                        num_attempts_before_safety_restart is not None
                        and i == num_attempts_before_safety_restart - 1
                    ):
                        await self._trigger(
                            self._restart_safety_client,
                            timeout=self.param("safety_restart_timeout"),
                        )

                    if i == max_attempts - 1:
                        raise

                    self.log("Retrying dashboard reset")

    def stop_program(self) -> None:
        self._stop_client.call_async(Trigger.Request())

    def destroy_interface(self):
        """Clean up SetMode action client"""
        self.log("Destroying URInterface")
        if hasattr(self, "_set_mode_client"):
            self._set_mode_client.destroy()
        super().destroy_interface()
