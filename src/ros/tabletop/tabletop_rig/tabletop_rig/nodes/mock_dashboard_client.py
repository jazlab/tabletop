"""Mock Universal Robots dashboard client for testing.

This module provides a ROS2 node that simulates the UR robot's dashboard
interface. It implements the same services as the real dashboard_client
node, allowing testing of the Commander node without a physical robot.

Services provided:
    ~/close_popup: Close UI popups.
    ~/close_safety_popup: Close safety-related popups.
    ~/unlock_protective_stop: Unlock after protective stop.
    ~/load_program: Load a URScript program.
    ~/load_installation: Load an installation file.
    ~/brake_release: Release robot brakes.
    ~/play: Start program execution.
    ~/stop: Stop program execution.
    ~/pause: Pause program execution.
    ~/connect: Connect to the robot.
    ~/quit: Disconnect from the robot.
    ~/get_safety_mode: Get current safety mode.
    ~/get_robot_mode: Get current robot mode.
    ~/program_state: Get current program state.
    ~/is_in_remote_control: Check if robot is in remote control.

Example:
    ros2 run tabletop_rig mock_dashboard_client
"""

import rclpy
from rclpy.executors import (
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.experimental import EventsExecutor
from std_srvs.srv import Trigger
from ur_dashboard_msgs.msg import ProgramState, RobotMode, SafetyMode
from ur_dashboard_msgs.srv import (
    GetProgramState,
    GetRobotMode,
    GetSafetyMode,
    IsInRemoteControl,
    Load,
)

from tabletop_rig.nodes.base import BaseNode


class MockDashboardClient(BaseNode):
    """Mock UR dashboard client node for testing and simulation.

    Provides stub implementations of all dashboard services that return
    success responses. The robot is always reported as being in RUNNING
    mode with NORMAL safety mode.

    This node is useful for:
    - Testing Commander node logic without hardware
    - Running simulations in RViz
    - Debugging motion planning workflows

    Attributes:
        close_popup_srv: Service for closing UI popups.
        close_safety_popup_srv: Service for closing safety popups.
        unlock_protective_stop_srv: Service for unlocking protective stop.
        load_program_srv: Service for loading URScript programs.
        load_installation_srv: Service for loading installation files.
        brake_release_srv: Service for releasing robot brakes.
        play_srv: Service for starting program execution.
        get_safety_mode_srv: Service for querying current safety mode.
        get_robot_mode_srv: Service for querying current robot mode.
        get_program_state_srv: Service for querying program state.
        is_in_remote_control_srv: Service for checking remote control mode.
    """

    def __init__(self):
        """Initialize the mock dashboard node and create all services."""
        super().__init__("dashboard_client")

        self.declare_parameter("is_mock", True, ignore_override=True)

        self.close_popup_srv = self.create_service(
            Trigger,
            "~/close_popup",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "close_popup",
            ),
        )

        self.close_safety_popup_srv = self.create_service(
            Trigger,
            "~/close_safety_popup",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "close_safety_popup",
            ),
        )

        self.unlock_protective_stop_srv = self.create_service(
            Trigger,
            "~/unlock_protective_stop",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "unlock_protective_stop",
            ),
        )

        self.load_program_srv = self.create_service(
            Load,
            "~/load_program",
            lambda request, response: self.load_callback(
                request,  # type: ignore
                response,  # type: ignore
                "load_program",
            ),
        )

        self.load_installation_srv = self.create_service(
            Load,
            "~/load_installation",
            lambda request, response: self.load_callback(
                request,  # type: ignore
                response,  # type: ignore
                "load_installation",
            ),
        )

        self.brake_release_srv = self.create_service(
            Trigger,
            "~/brake_release",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "brake_release",
            ),
        )

        self.play_srv = self.create_service(
            Trigger,
            "~/play",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "play",
            ),
        )

        self.stop_srv = self.create_service(
            Trigger,
            "~/stop",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "stop",
            ),
        )

        self.pause_srv = self.create_service(
            Trigger,
            "~/pause",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "pause",
            ),
        )

        self.connect_srv = self.create_service(
            Trigger,
            "~/connect",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "connect",
            ),
        )

        self.quit_srv = self.create_service(
            Trigger,
            "~/quit",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "quit",
            ),
        )

        self.get_safety_mode_srv = self.create_service(
            GetSafetyMode,
            "~/get_safety_mode",
            self.get_safety_mode_callback,  # type: ignore
        )

        self.get_robot_mode_srv = self.create_service(
            GetRobotMode,
            "~/get_robot_mode",
            self.get_robot_mode_callback,  # type: ignore
        )

        self.get_program_state_srv = self.create_service(
            GetProgramState,
            "~/program_state",
            self.get_program_state_callback,  # type: ignore
        )

        self.is_in_remote_control_srv = self.create_service(
            IsInRemoteControl,
            "~/is_in_remote_control",
            self.is_in_remote_control_callback,  # type: ignore
        )

        self.log("Mock Dashboard initialized")

    def trigger_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        service_name: str,
    ) -> Trigger.Response:
        """Generic callback for Trigger-type services.

        Used for close_popup, close_safety_popup, unlock_protective_stop,
        brake_release, and play services.

        Args:
            request: The trigger request (empty).
            response: The response to populate.
            service_name: Name of the service for logging.

        Returns:
            Populated response with success=True.
        """
        self.log(f"Received {service_name} trigger request")
        response.success = True
        response.message = f"Successfully processed {service_name}"
        return response

    def load_callback(
        self,
        request: Load.Request,
        response: Load.Response,
        service_name: str,
    ) -> Load.Response:
        """Handle load_program and load_installation service requests.

        Args:
            request: Request containing filename to load.
            response: The response to populate.
            service_name: Name of the service for logging.

        Returns:
            Populated response with success=True.
        """
        self.log(f"Received {service_name} load request: {request.filename}")
        response.success = True
        response.answer = f"Loading {request.filename}"
        return response

    def get_safety_mode_callback(
        self, request: GetSafetyMode.Request, response: GetSafetyMode.Response
    ) -> GetSafetyMode.Response:
        """Return the simulated safety mode.

        Always returns NORMAL safety mode.

        Args:
            request: The service request (empty).
            response: The response to populate.

        Returns:
            Response with safety_mode set to NORMAL.
        """
        self.log("Received GetSafetyMode request")
        response.safety_mode.mode = SafetyMode.NORMAL
        response.answer = "Safety mode is NORMAL"
        response.success = True
        return response

    def get_robot_mode_callback(
        self, request: GetRobotMode.Request, response: GetRobotMode.Response
    ) -> GetRobotMode.Response:
        """Return the simulated robot mode.

        Always returns RUNNING robot mode.

        Args:
            request: The service request (empty).
            response: The response to populate.

        Returns:
            Response with robot_mode set to RUNNING.
        """
        self.log("Received GetRobotMode request")
        response.robot_mode.mode = RobotMode.RUNNING
        response.answer = "Robot mode is RUNNING"
        response.success = True
        return response

    def get_program_state_callback(
        self,
        request: GetProgramState.Request,
        response: GetProgramState.Response,
    ) -> GetProgramState.Response:
        """Return the simulated program state.

        Always returns PLAYING program state.

        Args:
            request: The service request (empty).
            response: The response to populate.

        Returns:
            Response with state set to PLAYING.
        """
        self.log("Received GetRobotMode request")
        response.state.state = ProgramState.PLAYING
        response.answer = "Program state is PLAYING"
        response.success = True
        return response

    def is_in_remote_control_callback(
        self,
        request: IsInRemoteControl.Request,
        response: IsInRemoteControl.Response,
    ) -> IsInRemoteControl.Response:
        """Check if the robot is in remote control mode.

        Always returns True for remote control in the mock implementation.

        Args:
            request: The service request (empty).
            response: The response to populate.

        Returns:
            Response with remote_control set to True.
        """
        self.log("Received IsInRemoteControl request")
        response.remote_control = True
        response.answer = "Robot is in Remote Control"
        response.success = True
        return response


EXECUTOR_TYPE = "single-threaded"


def main(args=None):
    """Entry point for the mock_dashboard_client node."""
    rclpy.init(args=args)

    try:
        match EXECUTOR_TYPE:
            case "events":
                executor = EventsExecutor()
            case "single-threaded":
                executor = SingleThreadedExecutor()
            case "multi-threaded":
                executor = MultiThreadedExecutor()
            case _:
                raise ValueError(f"Unsupported EXECUTOR_TYPE: {EXECUTOR_TYPE}")

        mock_dashboard = MockDashboardClient()
        executor.add_node(mock_dashboard)

        try:
            executor.spin()
        finally:
            print("Shutting down mock dashboard")
            mock_dashboard.destroy_node()
            print("Shutting down executor")
            executor.shutdown()
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    except SystemExit:
        print("System exit")
    finally:
        print("Shutting down rclpy")
        rclpy.try_shutdown()  # type: ignore


if __name__ == "__main__":
    main()
