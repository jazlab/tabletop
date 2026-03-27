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
    ActionError,
    ServiceCallTimeoutError,
    ServiceCallUnsuccessfulError,
)
from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.nodes.base import AIOActionClient, BaseNode


class DashboardInterface(BaseInterface):
    """Interface for Universal Robots dashboard server communication.

    Provides async methods to interact with the UR dashboard for:
    - Querying robot and safety modes
    - Loading programs and installations
    - Triggering dashboard commands (brake release, play, etc.)
    - Automated recovery sequences after safety events

    The interface creates service clients on-demand rather than at init
    time to allow flexible service discovery.
    """

    def __init__(self, node: BaseNode) -> None:
        """Initialize the dashboard interface.

        Args:
            node: Parent ROS2 node for creating service clients.
        """
        super().__init__("dashboard_interface", node)

        self._connected = False

        if self.node.param("simulate"):
            self.log(
                "Waiting for mock_dashboard_client to indicate we are indeed simulating"
            )
            if not self.node.wait_for_node(
                "/mock_dashboard_client", timeout=1.0
            ):
                raise ValueError(
                    "simulate parameter is True, but mock_dashboard_client "
                    "node is not available. Please start the mock_dashboard_client "
                    "(e.g. by launching with robot_mode:=mock) or run this "
                    "node with simulate set to false (e.g. by launching with "
                    "robot_mode:=real)"
                )

        self._set_mode_client = AIOActionClient(
            node, SetMode, "/ur_robot_state_helper/set_mode"
        )

        # Wait for action server
        self.log("Waiting for Dashboard Client SetMode server")
        self._set_mode_client.wait_for_server()

        self.log("Dashboard interface initialized")

    async def _trigger(
        self, srv_name: str, timeout: Optional[float] = None
    ) -> Trigger.Response:
        """Call a dashboard Trigger service.

        Many dashboard commands (brake_release, play, close_popup, etc.)
        use the std_srvs/Trigger interface.

        Args:
            srv_name: Full service name (e.g., "/dashboard_client/play").

        Returns:
            The Trigger response with success status and message.
        """
        self.log(
            f"Triggering {srv_name} in UR Dashboard",
            severity="DEBUG",
        )
        response = await self.node.service_call_async(
            srv_request=Trigger.Request(),
            srv_type=Trigger,
            srv_name=srv_name,
            timeout=timeout,
        )
        return cast(Trigger.Response, response)

    async def _load_file(
        self,
        srv_name: str,
        filename: str,
    ) -> Load.Response:
        """Load a program or installation file on the robot.

        Args:
            srv_name: The load service name (e.g., "/dashboard_client/load_program").
            filename: Path to the program file on the robot controller.

        Returns:
            The Load response with success status.
        """
        self.log(
            f"Loading {srv_name}: {filename} in UR Dashboard",
            severity="DEBUG",
        )
        response = await self.node.service_call_async(
            srv_request=Load.Request(filename=filename),
            srv_type=Load,
            srv_name=srv_name,
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
                srv_type=IsInRemoteControl,
                srv_name="/dashboard_client/is_in_remote_control",
            ),
        )
        return response.remote_control

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
                srv_type=GetSafetyMode,
                srv_name="/dashboard_client/get_safety_mode",
            ),
        )
        return response.safety_mode

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
                srv_type=GetRobotMode,
                srv_name="/dashboard_client/get_robot_mode",
            ),
        )
        return response.robot_mode

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
                srv_type=GetProgramState,
                srv_name="/dashboard_client/program_state",
            ),
        )
        return response.state

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
        config = self.node.param("dashboard")

        if not self._connected:
            try:
                await self._trigger("/dashboard_client/quit")
            except ServiceCallUnsuccessfulError:
                pass

            try:
                await self._trigger("/dashboard_client/connect")
            except ServiceCallUnsuccessfulError as e:
                raise RuntimeError(
                    "Could not connect to dashboard client"
                ) from e

            await self._load_file(
                "/dashboard_client/load_program", config["program"]
            )

            await self._set_robot_mode_running(
                stop_program=False, play_program=False
            )

            self._connected = True

        remote_control = await self._is_in_remote_control()

        if not remote_control:
            raise RuntimeError(
                "Dashboard is not in Remote Control mode, please fix that immediately"
            )

        # Close any popups and unlock protective stop
        await self._trigger("/dashboard_client/close_popup")
        await self._trigger("/dashboard_client/close_safety_popup")
        await self._trigger("/dashboard_client/unlock_protective_stop")

        await self._trigger("/dashboard_client/stop")

        await self._set_robot_mode_running(
            stop_program=False, play_program=False
        )
        await asyncio.sleep(0.5)
        await self._trigger("/dashboard_client/play")

        safety_mode = await self.get_safety_mode()
        while safety_mode.mode != SafetyMode.NORMAL:
            self.log(
                f"Safety mode is {safety_mode.mode}, retrying after {config['play_retry_delay']} seconds until NORMAL...",
                severity="WARN",
            )
            await asyncio.sleep(config["play_retry_delay"])
            safety_mode = await self.get_safety_mode()

        await asyncio.sleep(2.0)

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
        max_attempts: int = self.node.param("dashboard.reset.max_attempts")
        num_attempts_before_safety_restart: Optional[int] = self.node.param(
            "dashboard.reset.num_attempts_before_safety_restart"
        )

        if (
            num_attempts_before_safety_restart is not None
            and num_attempts_before_safety_restart >= max_attempts
        ):
            raise ValueError(
                "num_attempts_before_safety_restart parameter must be less than max_attempts"
            )

        async with asyncio.timeout(timeout):
            for i in range(max_attempts):
                try:
                    await self._reset_impl()
                    return
                except (
                    ServiceCallUnsuccessfulError,
                    ServiceCallTimeoutError,
                    ActionError,
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
                            "/dashboard_client/restart_safety",
                            timeout=self.node.param(
                                "dashboard.reset.safety_restart_timeout"
                            ),
                        )

                    if i == max_attempts - 1:
                        raise

                    self.log("Retrying dashboard reset")

    def destroy_interface(self):
        """Clean up SetMode action client"""
        self.log("Destroying DashboardInterface")
        if hasattr(self, "_set_mode_client"):
            self._set_mode_client.destroy()
        super().destroy_interface()
