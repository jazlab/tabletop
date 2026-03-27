"""Mock Universal Robots dashboard client for testing.

This module provides a ROS2 node that simulates the UR robot's dashboard
interface. It implements the same services as the real dashboard_client
node, allowing testing of the Commander node without a physical robot.

Services provided:
    /dashboard_client/close_popup: Close UI popups
    /dashboard_client/close_safety_popup: Close safety-related popups
    /dashboard_client/unlock_protective_stop: Unlock after protective stop
    /dashboard_client/load_program: Load a URScript program
    /dashboard_client/load_installation: Load an installation file
    /dashboard_client/brake_release: Release robot brakes
    /dashboard_client/play: Start program execution
    /dashboard_client/get_safety_mode: Get current safety mode
    /dashboard_client/get_robot_mode: Get current robot mode
    /dashboard_client/is_in_remote_control: Check if robot is in remote control

Actions provided:
    /ur_robot_state_helper/set_mode: Change robot operational mode

Example:
    ros2 run tabletop_rig mock_dashboard
"""

import rclpy
from rclpy.action.server import ActionServer, ServerGoalHandle
from rclpy.executors import (
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.experimental import EventsExecutor
from std_srvs.srv import Trigger
from ur_dashboard_msgs.action import SetMode
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_dashboard_msgs.srv import (
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
    """

    def __init__(self):
        """Initialize the mock dashboard node and create all services."""
        super().__init__("mock_dashboard_client")
        self.close_popup_srv = self.create_service(
            Trigger,
            "/dashboard_client/close_popup",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "close_popup",
            ),
        )

        self.close_safety_popup_srv = self.create_service(
            Trigger,
            "/dashboard_client/close_safety_popup",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "close_safety_popup",
            ),
        )

        self.unlock_protective_stop_srv = self.create_service(
            Trigger,
            "/dashboard_client/unlock_protective_stop",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "unlock_protective_stop",
            ),
        )

        self.load_program_srv = self.create_service(
            Load,
            "/dashboard_client/load_program",
            lambda request, response: self.load_callback(
                request,  # type: ignore
                response,  # type: ignore
                "load_program",
            ),
        )

        self.load_installation_srv = self.create_service(
            Load,
            "/dashboard_client/load_installation",
            lambda request, response: self.load_callback(
                request,  # type: ignore
                response,  # type: ignore
                "load_installation",
            ),
        )

        self.brake_release_srv = self.create_service(
            Trigger,
            "/dashboard_client/brake_release",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "brake_release",
            ),
        )

        self.play_srv = self.create_service(
            Trigger,
            "/dashboard_client/play",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "play",
            ),
        )

        self.stop_srv = self.create_service(
            Trigger,
            "/dashboard_client/stop",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "stop",
            ),
        )

        self.pause_srv = self.create_service(
            Trigger,
            "/dashboard_client/pause",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "pause",
            ),
        )

        self.connect_srv = self.create_service(
            Trigger,
            "/dashboard_client/connect",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "connect",
            ),
        )

        self.quit_srv = self.create_service(
            Trigger,
            "/dashboard_client/quit",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,  # type: ignore
                "quit",
            ),
        )

        self.get_safety_mode_srv = self.create_service(
            GetSafetyMode,
            "/dashboard_client/get_safety_mode",
            self.get_safety_mode_callback,  # type: ignore
        )

        self.get_robot_mode_srv = self.create_service(
            GetRobotMode,
            "/dashboard_client/get_robot_mode",
            self.get_robot_mode_callback,  # type: ignore
        )

        self.is_in_remote_control_srv = self.create_service(
            IsInRemoteControl,
            "/dashboard_client/is_in_remote_control",
            self.is_in_remote_control_callback,  # type: ignore
        )

        self.set_mode_server = ActionServer(
            self,
            SetMode,
            "/ur_robot_state_helper/set_mode",
            self.set_mode_callback,
        )

        self.log("Mock Dashboard initialized")

    def set_mode_callback(
        self, goal_handle: ServerGoalHandle
    ) -> SetMode.Result:
        """Handle SetMode action requests.

        Args:
            goal_handle: The action goal handle from ROS2.

        Returns:
            SetMode.Result with success=True.
        """
        self.log("Received SetMode request")
        self.log_ros_msg(goal_handle.request)
        goal_handle.succeed()
        return SetMode.Result(success=True, message="Success")

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

    def destroy_node(self):
        if hasattr(self, "set_mode_server"):
            self.set_mode_server.destroy()
        super().destroy_node()


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
