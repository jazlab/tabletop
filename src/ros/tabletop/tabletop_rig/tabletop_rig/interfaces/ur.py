"""Interface for Universal Robots dashboard control.

This module provides an interface to control a Universal Robots (UR) arm
through the UR Dashboard Server. The dashboard provides high-level robot
control including safety mode management, program loading/execution, and
error recovery.

The UR Dashboard Server exposes services for robot lifecycle management
that are essential for recovering from safety stops and protective stops.
"""

import asyncio
from collections.abc import Iterable
from typing import Optional, cast

import rclpy
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
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
    ServiceCallUnsuccessfulError,
    ServiceClientError,
)
from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.nodes.base import AIOActionClient, BaseNode

_ROBOT_MODE_MAP: dict[int, str] = {
    v: k
    for k, v in type(RobotMode)._Metaclass_RobotMode__constants.items()  # type: ignore
}
"""dict[int, str]: Maps RobotMode integer enumeration values to their string names.

This mapping is dynamically generated from the RobotMode message constants.
"""

_SAFETY_MODE_MAP: dict[int, str] = {
    v: k
    for k, v in type(SafetyMode)._Metaclass_SafetyMode__constants.items()  # type: ignore
}
"""dict[int, str]: Maps SafetyMode integer enumeration values to their string names.

This mapping is dynamically generated from the SafetyMode message constants.
"""


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

        Sets up clients for all dashboard services (query and trigger) and
        the SetMode action. Waits for dashboard_client and ur_robot_state_helper
        nodes. Reads 'namespace' parameter to determine node prefixes.

        Args:
            node: Parent ROS2 node for creating service clients.
            name: Interface name (used for parameter lookup and logging).
            simulate: If True, verifies nodes are in mock mode; if False,
                verifies they are in real hardware mode.
            parameter_fallback_prefix: Optional fallback prefix for parameter
                lookup (e.g., 'common_ur_interface').

        Raises:
            RuntimeError: If dashboard_client or ur_robot_state_helper nodes
                are not available, or if hardware mode mismatches simulate param.
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
            callback_group=ReentrantCallbackGroup(),
        )

        # Parameter query service clients (one per node namespace)
        self._dashboard_get_parameters_client = self.node.create_client(
            GetParameters,
            f"{self._dashboard_ns}/get_parameters",
            callback_group=ReentrantCallbackGroup(),
        )
        self._state_helper_get_parameters_client = self.node.create_client(
            GetParameters,
            f"{self._state_helper_ns}/get_parameters",
            callback_group=ReentrantCallbackGroup(),
        )

        # Dashboard query service clients
        self._is_in_remote_control_client = self.node.create_client(
            IsInRemoteControl,
            f"{self._dashboard_ns}/is_in_remote_control",
            callback_group=ReentrantCallbackGroup(),
        )
        self._get_robot_mode_client = self.node.create_client(
            GetRobotMode,
            f"{self._dashboard_ns}/get_robot_mode",
            callback_group=ReentrantCallbackGroup(),
        )
        self._get_safety_mode_client = self.node.create_client(
            GetSafetyMode,
            f"{self._dashboard_ns}/get_safety_mode",
            callback_group=ReentrantCallbackGroup(),
        )
        self._get_program_state_client = self.node.create_client(
            GetProgramState,
            f"{self._dashboard_ns}/program_state",
            callback_group=ReentrantCallbackGroup(),
        )

        # Dashboard load service client
        self._load_program_client = self.node.create_client(
            Load,
            f"{self._dashboard_ns}/load_program",
            callback_group=ReentrantCallbackGroup(),
        )

        # Dashboard trigger service clients
        self._quit_client = self.node.create_client(
            Trigger,
            f"{self._dashboard_ns}/quit",
            callback_group=ReentrantCallbackGroup(),
        )
        self._connect_client = self.node.create_client(
            Trigger,
            f"{self._dashboard_ns}/connect",
            callback_group=ReentrantCallbackGroup(),
        )
        self._close_popup_client = self.node.create_client(
            Trigger,
            f"{self._dashboard_ns}/close_popup",
            callback_group=ReentrantCallbackGroup(),
        )
        self._close_safety_popup_client = self.node.create_client(
            Trigger,
            f"{self._dashboard_ns}/close_safety_popup",
            callback_group=ReentrantCallbackGroup(),
        )
        self._unlock_protective_stop_client = self.node.create_client(
            Trigger,
            f"{self._dashboard_ns}/unlock_protective_stop",
            callback_group=ReentrantCallbackGroup(),
        )
        self._stop_client = self.node.create_client(
            Trigger,
            f"{self._dashboard_ns}/stop",
            callback_group=ReentrantCallbackGroup(),
        )
        self._play_client = self.node.create_client(
            Trigger,
            f"{self._dashboard_ns}/play",
            callback_group=ReentrantCallbackGroup(),
        )
        self._restart_safety_client = self.node.create_client(
            Trigger,
            f"{self._dashboard_ns}/restart_safety",
            callback_group=ReentrantCallbackGroup(),
        )

        self._connected = False
        self._stop_future: rclpy.Future | None = None

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

    async def _get_robot_mode(self) -> RobotMode:
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

    async def _get_safety_mode(self) -> SafetyMode:
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

    async def _get_program_state(self) -> ProgramState:
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

    async def _wait_for_remote_control(
        self, timeout: Optional[float] = None
    ) -> None:
        async with asyncio.timeout(timeout):
            delay = self.param("check_remote_control_delay")
            remote_control = await self._is_in_remote_control()
            while not remote_control:
                self.log(
                    f"Dashboard is not in Remote Control mode. "
                    f"You will have to set it to Remote Control mode "
                    f"manually on your robot's dashboard. "
                    f"Waiting {delay} seconds before checking again",
                    severity="WARN",
                )
                await asyncio.sleep(delay)
                remote_control = await self._is_in_remote_control()

    async def _wait_for_safety_mode(
        self,
        target_modes: Iterable[int],
        timeout: Optional[float] = None,
    ) -> None:
        async with asyncio.timeout(timeout):
            delay = self.param("check_safety_mode_delay")
            target_modes_strs = (_SAFETY_MODE_MAP[x] for x in target_modes)
            target_modes = set(target_modes)

            safety_mode = await self._get_safety_mode()
            while safety_mode.mode not in target_modes:
                self.log(
                    f"Current safety mode ({_SAFETY_MODE_MAP[safety_mode.mode]}) "
                    f"not in target safety modes ({target_modes_strs}). "
                    f"You may have to restore it to one of these modes manually "
                    f"on your robot's dashboard. "
                    f"Waiting {delay} seconds before checking again",
                    severity="WARN",
                )
                await asyncio.sleep(delay)
                safety_mode = await self._get_safety_mode()

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

        robot_mode = await self._get_robot_mode()
        if robot_mode.mode != RobotMode.RUNNING:
            raise RuntimeError(
                f"Robot mode should be RUNNING, actual mode: {robot_mode}"
            )

    async def _reconnect(self) -> None:
        """Re-establish a clean dashboard session.

        Quits any existing dashboard connection, reconnects, waits for
        remote-control mode, reloads the configured program, and sets the
        robot mode to RUNNING (without playing the program).

        Raises:
            RuntimeError: If the dashboard connect service call fails.
        """
        try:
            await self._trigger(self._quit_client)
        except ServiceCallUnsuccessfulError:
            pass

        try:
            await self._trigger(self._connect_client)
        except ServiceCallUnsuccessfulError as e:
            raise RuntimeError("Could not connect to dashboard client") from e

        await asyncio.sleep(1.0)

        await self._wait_for_remote_control()

        await self._load_file(self._load_program_client, self.param("program"))

        await self._set_robot_mode_running(
            stop_program=False, play_program=False
        )

    async def _reset_impl(self) -> None:
        """Execute full dashboard recovery sequence.

        Performs a comprehensive reset of the robot dashboard:
        1. (Re)connect to the dashboard and verify remote control mode
        2. If the safety mode is VIOLATION or FAULT, restart the safety
           controller (automatically after ``safety_restart_delay`` when
           ``safety_restart_enable`` is true, otherwise wait for the
           operator to restart safety manually) and wait for NORMAL
        3. Close any popup dialogs and unlock protective stops
        4. Stop the program, set robot mode to RUNNING, and replay it
        5. Wait for NORMAL safety mode

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
                self._state_helper_get_parameters_client, self._state_helper_ns
            )
            await self._reconnect()
            self._connected = True
        else:
            await self._wait_for_remote_control()

        safety_mode = await self._get_safety_mode()
        if safety_mode.mode in (SafetyMode.VIOLATION, SafetyMode.FAULT):
            self.log(
                f"SafetyMode is {_SAFETY_MODE_MAP[safety_mode.mode]}",
                severity="ERROR",
            )
            if self.param("safety_restart_enable"):
                delay = self.param("safety_restart_delay")
                timeout = self.param("safety_restart_timeout")
                self.log(
                    f"'safety_restart_enable' parameter is set to True, "
                    f"automatically restarting safety after {delay} seconds "
                    f"(if you don't want to restart, stop the program now!!!!!!!!!)",
                    severity="ERROR",
                )
                await asyncio.sleep(delay)
                self.log("Restarting safety!")
                await self._trigger(
                    self._restart_safety_client,
                    timeout=timeout,
                )
            else:
                self.log(
                    "'safety_restart_enable' parameter is set to False, "
                    "you will have manually restart safety on your "
                    "robot's dashboard",
                    severity="ERROR",
                )

            await self._wait_for_safety_mode((SafetyMode.NORMAL,))

            # TODO: See if this is necessary
            await self._reconnect()

        # Close any popups and unlock protective stop
        await self._trigger(self._close_popup_client)
        await self._trigger(self._close_safety_popup_client)
        await self._trigger(self._unlock_protective_stop_client)

        # Stop the currently running program, set robot mode to running,
        # then play the loaded program
        await self._trigger(self._stop_client)
        await self._set_robot_mode_running(
            stop_program=False, play_program=False
        )
        await asyncio.sleep(0.5)
        await self._trigger(self._play_client)

        # Wait for safety mode to return to NORMAL (should usually
        # already be NORMAL after setting robot mode to RUNNING)
        await self._wait_for_safety_mode((SafetyMode.NORMAL,))

    async def is_ready(self) -> bool:
        """Check if the UR robot is ready for programmatic control.

        Verifies that:
        1. reset() has been called (_connected flag set)
        2. Robot is in remote control mode
        3. Robot mode is RUNNING
        4. Program state is PLAYING
        5. Safety mode is NORMAL

        Logs specific reason if any condition fails.

        Returns:
            True if all conditions are met, False otherwise.
        """
        if not self._connected:
            self.log("UR not ready: Not yet connected")
            return False

        remote_control = await self._is_in_remote_control()
        if not remote_control:
            self.log("UR not ready: Not in remote control")
            return False

        robot_mode = await self._get_robot_mode()
        if robot_mode.mode != RobotMode.RUNNING:
            self.log(
                f"UR not ready: Robot mode not RUNNING, got {_ROBOT_MODE_MAP[robot_mode.mode]}"
            )
            return False

        program_state = await self._get_program_state()
        if program_state.state != ProgramState.PLAYING:
            self.log(
                f"UR not ready: Program state not PLAYING, got {program_state.state}"
            )
            return False

        safety_mode = await self._get_safety_mode()
        if safety_mode.mode != SafetyMode.NORMAL:
            self.log(
                f"UR not ready: Safety mode not NORMAL, got {_SAFETY_MODE_MAP[safety_mode.mode]}"
            )
            return False

        return True

    async def reset(self, timeout: Optional[float] = None):
        """Execute full dashboard recovery sequence.

        Performs a comprehensive reset of the robot dashboard:
        1. (Re)connect to the dashboard and verify remote control mode
        2. If the safety mode is VIOLATION or FAULT, restart the safety
           controller (automatically after ``safety_restart_delay`` when
           ``safety_restart_enable`` is true, otherwise wait for the
           operator to restart safety manually) and wait for NORMAL
        3. Close any popup dialogs and unlock protective stops
        4. Stop the program, set robot mode to RUNNING, and replay it
        5. Wait for NORMAL safety mode

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
        reset_retry_delay: float = self.param("reset_retry_delay")
        post_reset_delay: float = self.param("post_reset_delay")

        async with asyncio.timeout(timeout):
            for i in range(max_attempts):
                try:
                    await self._reset_impl()
                    break
                except (ServiceClientError, ActionClientError) as e:
                    self.log(
                        f"Caught exception while resetting dashboard | {type(e).__name__}: {e}",
                        severity="WARN",
                    )
                    if i == max_attempts - 1:
                        raise
                    self.log(
                        f"Retrying dashboard reset after {reset_retry_delay}s"
                    )
                    await asyncio.sleep(reset_retry_delay)

        self.log("URInterface successfully reset, ")
        await asyncio.sleep(post_reset_delay)

    def stop_program(self) -> None:
        """Call the UR dashboard stop service asynchronously.

        Initiates a non-blocking stop request via the dashboard_client/stop
        service. The result is not awaited.
        """
        self._stop_future = self._stop_client.call_async(Trigger.Request())

    def destroy_interface(self):
        """Clean up SetMode action client and other ROS resources."""
        self.log("Destroying URInterface")
        if hasattr(self, "_set_mode_client"):
            self._set_mode_client.destroy()
        super().destroy_interface()
