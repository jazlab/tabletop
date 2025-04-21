import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import Load

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
                request, response, "close_popup"
            ),
        )

        self.close_safety_popup_srv = self.create_service(
            Trigger,
            "/dashboard_client/close_safety_popup",
            lambda request, response: self.trigger_callback(
                request, response, "close_safety_popup"
            ),
        )

        self.unlock_protective_stop_srv = self.create_service(
            Trigger,
            "/dashboard_client/unlock_protective_stop",
            lambda request, response: self.trigger_callback(
                request, response, "unlock_protective_stop"
            ),
        )

        self.load_program_srv = self.create_service(
            Load,
            "/dashboard_client/load_program",
            lambda request, response: self.load_callback(
                request, response, "load_program"
            ),
        )

        self.load_installation_srv = self.create_service(
            Load,
            "/dashboard_client/load_installation",
            lambda request, response: self.load_callback(
                request, response, "load_installation"
            ),
        )

        self.brake_release_srv = self.create_service(
            Trigger,
            "/dashboard_client/brake_release",
            lambda request, response: self.trigger_callback(
                request, response, "brake_release"
            ),
        )

        self.play_srv = self.create_service(
            Trigger,
            "/dashboard_client/play",
            lambda request, response: self.trigger_callback(
                request, response, "play"
            ),
        )

        self.log("Mock Dashboard initialized")

    def trigger_callback(self, request, response, service_name: str):
        """
        Generic callback for Trigger services.

        Args:
            request: The Trigger request (empty)
            response: The Trigger response to populate

        Returns:
            The populated Trigger response
        """
        self.log(f"Mock Dashboard received request: {service_name}")
        response.success = True
        response.message = f"Successfully processed {service_name}"
        return response

    def load_callback(self, request, response, service_name: str):
        """
        Callback for Load service.

        Args:
            request: The Load request with filename
            response: The Load response to populate

        Returns:
            The populated Load response
        """
        self.log(f"Mock Dashboard {service_name}: {request.filename}")
        response.success = True
        response.answer = f"Loading {request.filename}"
        return response


def main(args=None):
    rclpy.init(args=args)

    try:
        executor: rclpy.Executor = SingleThreadedExecutor()
        mock_dashboard = MockDashboard()
        executor.add_node(mock_dashboard)

        try:
            executor.spin()
        finally:
            print("Shutting down executor")
            executor.shutdown()
            print("Shutting down mock dashboard")
            mock_dashboard.destroy_node()
    finally:
        print("Shutting down rclpy")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
