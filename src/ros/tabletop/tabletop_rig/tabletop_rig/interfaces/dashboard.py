import asyncio
from typing import Optional, cast

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
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
    ActionCallUnsuccessfulError,
    ServiceCallUnsuccessfulError,
)
from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.nodes.base import BaseNode


class DashboardInterface(BaseInterface):
    def __init__(self, node: BaseNode):
        """Initializes the DashboardInterface"""
        super().__init__(node, "dashboard_interface")

        self._init = False

        self._set_mode_client = ActionClient(
            node, SetMode, "/ur_robot_state_helper/set_mode"
        )

        # Wait for action server
        self.log("Waiting for response time server")
        self._set_mode_client.wait_for_server()

        self.log("Dashboard interface initialized")

    async def _trigger(self, srv_name: str) -> Trigger.Response:
        """Call a dashboard client Trigger service (asynchronous)."""
        self.log(
            f"Triggering {srv_name} in UR Dashboard",
            severity="DEBUG",
        )
        response = await self.node.service_call_async(
            srv_request=Trigger.Request(), srv_type=Trigger, srv_name=srv_name
        )
        return cast(Trigger.Response, response)

    async def _load_file(
        self,
        srv_name: str,
        filename: str,
    ) -> Load.Response:
        """Load a program or installation on the robot dashboard (asynchronous)."""
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

    async def _get_safety_mode(self) -> SafetyMode:
        """Get the safety mode from the dashboard client."""
        response = cast(
            GetSafetyMode.Response,
            await self.node.service_call_async(
                srv_request=GetSafetyMode.Request(),
                srv_type=GetSafetyMode,
                srv_name="/dashboard_client/get_safety_mode",
            ),
        )
        return response.safety_mode

    async def _get_robot_mode(self) -> RobotMode:
        """Get the robot mode from the dashboard client."""
        response = cast(
            GetRobotMode.Response,
            await self.node.service_call_async(
                srv_request=GetRobotMode.Request(),
                srv_type=GetRobotMode,
                srv_name="/dashboard_client/get_robot_mode",
            ),
        )
        return response.robot_mode

    async def _get_program_state(self) -> ProgramState:
        """Get the safety mode from the dashboard client."""
        response = cast(
            GetProgramState.Response,
            await self.node.service_call_async(
                srv_request=GetProgramState.Request(),
                srv_type=GetProgramState,
                srv_name="/dashboard_client/program_state",
            ),
        )
        return response.state

    async def _is_in_remote_control(self) -> bool:
        """Get the safety mode from the dashboard client."""
        response = cast(
            IsInRemoteControl.Response,
            await self.node.service_call_async(
                srv_request=IsInRemoteControl.Request(),
                srv_type=IsInRemoteControl,
                srv_name="/dashboard_client/is_in_remote_control",
            ),
        )
        return response.remote_control

    async def _reset(self, timeout: Optional[float] = None):
        """Call a sequence of dashboard client services to reset the dashboard (asynchronous)."""
        self.log("Resetting dashboard")
        config = self.node.param("dashboard")
        async with asyncio.timeout(timeout):
            while True:
                # Timeout included in wait_for_dashboard to stop the thread
                # from waiting longer than timeout
                await self._trigger("/dashboard_client/close_popup")
                await self._trigger("/dashboard_client/close_safety_popup")
                await self._trigger("/dashboard_client/unlock_protective_stop")
                await self._load_file(
                    "/dashboard_client/load_program", config["program"]
                )
                await self._trigger("/dashboard_client/brake_release")
                safety_mode = await self._get_safety_mode()
                robot_mode = await self._get_robot_mode()
                while (
                    safety_mode.mode != SafetyMode.NORMAL
                    and robot_mode.mode != RobotMode.RUNNING
                ):
                    self.log(
                        f"Safety mode is {safety_mode.mode}, retrying after {config['play_retry_delay']} seconds...",
                        severity="WARN",
                    )
                    await asyncio.sleep(config["play_retry_delay"])
                    safety_mode = await self._get_safety_mode()
                    robot_mode = await self._get_robot_mode()

                for _ in range(config["play_retries"]):
                    try:
                        await self._trigger("/dashboard_client/play")
                        return
                    except ServiceCallUnsuccessfulError:
                        self.log(
                            f"Failed attempt to play dashboard program, "
                            f"retrying after {config['play_retry_delay']} seconds...",
                            severity="WARN",
                        )
                        await asyncio.sleep(config["play_retry_delay"])

    async def reset_dashboard_2(
        self, timeout: Optional[float] = None, init: bool = False
    ):
        """Reset the UR Dashboard."""
        self.log("Resetting dashboard")
        async with asyncio.timeout(timeout):
            if init:
                await self._load_file(
                    "/dashboard_client/load_program",
                    self.node.param("dashboard.program"),
                )

            await self._trigger("/dashboard_client/close_popup")
            await self._trigger("/dashboard_client/close_safety_popup")
            await self._trigger("/dashboard_client/unlock_protective_stop")
            goal_handle = cast(
                ClientGoalHandle,
                await self._set_mode_client.send_goal_async(
                    SetMode.Goal(
                        target_robot_mode=RobotMode.RUNNING,
                        stop_program=True,
                        play_program=True,
                    )
                ),
            )
            if not goal_handle.accepted:
                raise ActionCallUnsuccessfulError(
                    "UR SetMode action goal not accepted"
                )

            try:
                response = await goal_handle.get_result_async()
            except asyncio.CancelledError:
                goal_handle.cancel_goal_async()
                raise

            if (
                response.status != GoalStatus.STATUS_SUCCEEDED
                or not response.result.success
            ):
                raise ActionCallUnsuccessfulError("UR SetMode action failed")

    async def _set_robot_mode_running(self):
        """Set robot mode to RUNNING"""
        goal_handle = cast(
            ClientGoalHandle,
            await self._set_mode_client.send_goal_async(
                SetMode.Goal(
                    target_robot_mode=RobotMode.RUNNING,
                    stop_program=True,
                    play_program=False,
                )
            ),
        )
        if not goal_handle.accepted:
            raise ActionCallUnsuccessfulError(
                "UR SetMode action goal not accepted"
            )

        try:
            response = await goal_handle.get_result_async()
        except asyncio.CancelledError:
            goal_handle.cancel_goal_async()
            raise

        if (
            response.status != GoalStatus.STATUS_SUCCEEDED
            or not response.result.success
        ):
            raise ActionCallUnsuccessfulError("UR SetMode action failed")

        robot_mode = await self._get_robot_mode()
        if robot_mode.mode != RobotMode.RUNNING:
            raise RuntimeError(
                f"Robot mode should be RUNNING, actual mode: {robot_mode}"
            )

    async def reset(self, timeout: Optional[float] = None):
        """Call a sequence of dashboard client services to reset the dashboard (asynchronous)."""
        self.log("Resetting dashboard")

        remote_control = await self._is_in_remote_control()
        if not remote_control:
            raise RuntimeError(
                "Dashboard is not in Remote Control mode, please fix that immediately"
            )
        config = self.node.param("dashboard")
        async with asyncio.timeout(timeout):
            # Load program
            try:
                await self._load_file(
                    "/dashboard_client/load_program", config["program"]
                )
            except ServiceCallUnsuccessfulError as e:
                self.log(
                    f"Failed to load program with error: {e}", severity="WARN"
                )
                self.log("Attempting to reconnect...")
                await self._trigger("/dashboard_client/connect")

            # Set RobotState to RUNNING
            await self._set_robot_mode_running()

            # Close any popups and unlock protective stop
            await self._trigger("/dashboard_client/close_popup")
            await self._trigger("/dashboard_client/close_safety_popup")
            await self._trigger("/dashboard_client/unlock_protective_stop")

            # Play program
            await self._trigger("/dashboard_client/play")

            safety_mode = await self._get_safety_mode()
            while safety_mode.mode != SafetyMode.NORMAL:
                self.log(
                    f"Safety mode is {safety_mode.mode}, retrying after {config['play_retry_delay']} seconds until NORMAL...",
                    severity="WARN",
                )
                await asyncio.sleep(config["play_retry_delay"])
                safety_mode = await self._get_safety_mode()

            return

            safety_mode = await self._get_safety_mode()
            while safety_mode.mode != SafetyMode.NORMAL:
                self.log(
                    f"Safety mode is {safety_mode.mode}, retrying after {config['play_retry_delay']} seconds...",
                    severity="WARN",
                )
                await asyncio.sleep(config["play_retry_delay"])
                safety_mode = await self._get_safety_mode()
                robot_mode = await self._get_robot_mode()

            while True:
                # Timeout included in wait_for_dashboard to stop the thread
                # from waiting longer than timeout

                await self._trigger("/dashboard_client/close_popup")
                await self._trigger("/dashboard_client/close_safety_popup")
                await self._trigger("/dashboard_client/unlock_protective_stop")
                await self._load_file(
                    "/dashboard_client/load_program", config["program"]
                )
                await self._trigger("/dashboard_client/brake_release")
                safety_mode = await self._get_safety_mode()
                robot_mode = await self._get_robot_mode()
                while (
                    safety_mode.mode != SafetyMode.NORMAL
                    and robot_mode.mode != RobotMode.RUNNING
                ):
                    self.log(
                        f"Safety mode is {safety_mode.mode}, retrying after {config['play_retry_delay']} seconds...",
                        severity="WARN",
                    )
                    await asyncio.sleep(config["play_retry_delay"])
                    safety_mode = await self._get_safety_mode()
                    robot_mode = await self._get_robot_mode()

                for _ in range(config["play_retries"]):
                    try:
                        await self._trigger("/dashboard_client/play")
                        return
                    except ServiceCallUnsuccessfulError:
                        self.log(
                            f"Failed attempt to play dashboard program, "
                            f"retrying after {config['play_retry_delay']} seconds...",
                            severity="WARN",
                        )
                        await asyncio.sleep(config["play_retry_delay"])
