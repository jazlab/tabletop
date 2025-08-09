import rclpy
from rclpy.action.server import ActionServer, ServerGoalHandle
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from ur_dashboard_msgs.action import SetMode
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_dashboard_msgs.srv import GetRobotMode, GetSafetyMode
from ur_dashboard_msgs.srv import Load as DashboardLoad

from tabletop_server.nodes.base import BaseNode


class MockDashboard(BaseNode):
    """
    A ROS2 node that mimics the Universal Robots dashboard client.
    Provides the dashboard services that are used by the Commander node.
    """

    def __init__(self):
        super().__init__("mock_dashboard")

        # Create all the dashboard services with their callbacks
        self.close_popup_srv = self.create_service(
            Trigger,
            "/dashboard_client/close_popup",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,
                "close_popup",
            ),
        )

        self.close_safety_popup_srv = self.create_service(
            Trigger,
            "/dashboard_client/close_safety_popup",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,
                "close_safety_popup",
            ),
        )

        self.unlock_protective_stop_srv = self.create_service(
            Trigger,
            "/dashboard_client/unlock_protective_stop",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,
                "unlock_protective_stop",
            ),
        )

        self.load_program_srv = self.create_service(
            DashboardLoad,
            "/dashboard_client/load_program",
            lambda request, response: self.load_callback(
                request,  # type: ignore
                response,
                "load_program",
            ),
        )

        self.load_installation_srv = self.create_service(
            DashboardLoad,
            "/dashboard_client/load_installation",
            lambda request, response: self.load_callback(
                request,  # type: ignore
                response,
                "load_installation",
            ),
        )

        self.brake_release_srv = self.create_service(
            Trigger,
            "/dashboard_client/brake_release",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,
                "brake_release",
            ),
        )

        self.play_srv = self.create_service(
            Trigger,
            "/dashboard_client/play",
            lambda request, response: self.trigger_callback(
                request,  # type: ignore
                response,
                "play",
            ),
        )

        self.get_safety_mode_srv = self.create_service(
            GetSafetyMode,
            "/dashboard_client/get_safety_mode",
            self.get_safety_mode_callback,
        )

        self.get_robot_mode_srv = self.create_service(
            GetRobotMode,
            "/dashboard_client/get_robot_mode",
            self.get_robot_mode_callback,
        )

        self.set_mode_server = ActionServer(
            self,
            SetMode,
            "/ur_robot_state_helper/set_mode",
            self.set_mode_callback,
        )

        self.log("Mock Dashboard initialized")

    def set_mode_callback(self, goal_handle: ServerGoalHandle):
        """Callback for SetMode action server."""
        self.log("Received SetMode request")
        self.log_ros_msg(goal_handle.request)
        goal_handle.succeed()
        return SetMode.Result(success=True, message="Success")

    def trigger_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        service_name: str,
    ):
        """Generic callback for Trigger services."""
        self.log(f"Received {service_name} trigger request")
        response.success = True
        response.message = f"Successfully processed {service_name}"
        return response

    def load_callback(
        self,
        request: DashboardLoad.Request,
        response: DashboardLoad.Response,
        service_name: str,
    ):
        """Callback for Load service."""
        self.log(f"Received {service_name} load request: {request.filename}")
        response.success = True
        response.answer = f"Loading {request.filename}"
        return response

    def get_safety_mode_callback(
        self, request: GetSafetyMode.Request, response: GetSafetyMode.Response
    ):
        """Callback for GetSafetyMode service."""
        self.log("Received GetSafetyMode request")
        response.safety_mode.mode = SafetyMode.NORMAL
        response.answer = "Safety mode is NORMAL"
        response.success = True
        return response

    def get_robot_mode_callback(
        self, request: GetRobotMode.Request, response: GetRobotMode.Response
    ):
        """Callback for GetRobotMode service."""
        self.log("Received GetRobotMode request")
        response.robot_mode.mode = RobotMode.RUNNING
        response.answer = "Robot mode is RUNNING"
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    try:
        executor = SingleThreadedExecutor()
        mock_dashboard = MockDashboard()
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
