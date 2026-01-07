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
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_dashboard_msgs.srv import GetRobotMode, GetSafetyMode
from ur_dashboard_msgs.srv import Load as DashboardLoad

from tabletop_rig.exceptions import ServiceCallUnsuccessfulError
from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.nodes.base import BaseNode


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
        super().__init__(node, "dashboard_interface")

        self.log("Dashboard interface initialized")

    async def dashboard_trigger(self, srv_name: str) -> Trigger.Response:
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
            srv_request=Trigger.Request(), srv_type=Trigger, srv_name=srv_name
        )
        return cast(Trigger.Response, response)

    async def dashboard_load(
        self,
        srv_name: str,
        filename: str,
    ) -> DashboardLoad.Response:
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
            srv_request=DashboardLoad.Request(filename=filename),
            srv_type=DashboardLoad,
            srv_name=srv_name,
        )
        return cast(DashboardLoad.Response, response)

    async def dashboard_get_safety_mode(self) -> SafetyMode:
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

    async def dashboard_get_robot_mode(self) -> RobotMode:
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

    async def reset(self, timeout: Optional[float] = None) -> None:
        """Execute full dashboard recovery sequence.

        Performs a comprehensive reset of the robot dashboard:
        1. Close any popup dialogs
        2. Unlock protective stops
        3. Load the configured program
        4. Release brakes
        5. Wait for NORMAL safety mode and RUNNING robot mode
        6. Start program execution with retries

        This is typically called after safety events, protective stops,
        or at startup to bring the robot to an operational state.

        Args:
            timeout: Maximum time for the entire reset sequence in seconds.
                If None, no timeout is applied.
        """
        self.log("Resetting dashboard")
        config = self.node.param("dashboard")
        async with asyncio.timeout(timeout):
            while True:
                # Timeout included in wait_for_dashboard to stop the thread
                # from waiting longer than timeout
                await self.dashboard_trigger("/dashboard_client/close_popup")
                await self.dashboard_trigger(
                    "/dashboard_client/close_safety_popup"
                )
                await self.dashboard_trigger(
                    "/dashboard_client/unlock_protective_stop"
                )
                await self.dashboard_load(
                    "/dashboard_client/load_program", config["program"]
                )
                await self.dashboard_trigger("/dashboard_client/brake_release")
                safety_mode = await self.dashboard_get_safety_mode()
                robot_mode = await self.dashboard_get_robot_mode()
                while (
                    safety_mode.mode != SafetyMode.NORMAL
                    or robot_mode.mode != RobotMode.RUNNING
                ):
                    self.log(
                        f"Safety mode is {safety_mode.mode}, retrying after {config['play_retry_delay']} seconds...",
                        severity="WARN",
                    )
                    await asyncio.sleep(config["play_retry_delay"])
                    safety_mode = await self.dashboard_get_safety_mode()
                    robot_mode = await self.dashboard_get_robot_mode()

                for _ in range(config["play_retries"]):
                    try:
                        await self.dashboard_trigger("/dashboard_client/play")
                        return
                    except ServiceCallUnsuccessfulError:
                        self.log(
                            f"Failed attempt to play dashboard program, "
                            f"retrying after {config['play_retry_delay']} seconds...",
                            severity="WARN",
                        )
                        await asyncio.sleep(config["play_retry_delay"])

    # async def reset_dashboard_2(
    #     self, timeout: Optional[float] = None, init: bool = False
    # ):
    #     """Reset the UR Dashboard."""
    #     self.log("Resetting dashboard")
    #     async with asyncio.timeout(timeout):
    #         if init:
    #             await self.dashboard_load(
    #                 "/dashboard_client/load_program",
    #                 self.param("dashboard.program"),
    #             )

    #         await self.dashboard_trigger("/dashboard_client/close_popup")
    #         await self.dashboard_trigger(
    #             "/dashboard_client/close_safety_popup"
    #         )
    #         await self.dashboard_trigger(
    #             "/dashboard_client/unlock_protective_stop"
    #         )
    #         goal_handle = cast(
    #             ClientGoalHandle,
    #             await self.set_mode_client.send_goal_async(
    #                 SetMode.Goal(
    #                     target_robot_mode=RobotMode.RUNNING,
    #                     stop_program=True,
    #                     play_program=True,
    #                 )
    #             ),
    #         )
    #         if not goal_handle.accepted:
    #             raise ActionCallUnsuccessfulError(
    #                 "UR SetMode action goal not accepted"
    #             )

    #         try:
    #             response = await goal_handle.get_result_async()
    #         except asyncio.CancelledError:
    #             goal_handle.cancel_goal_async()
    #             raise

    #         if (
    #             response.status != GoalStatus.STATUS_SUCCEEDED
    #             or not response.result.success
    #         ):
    #             raise ActionCallUnsuccessfulError("UR SetMode action failed")
