"""Mock Universal Robots robot state helper for testing.

This module provides a ROS2 node that simulates the UR robot state helper
interface. It implements a simple dummy SetMode action server, allowing
testing of the Commander node without a physical robot.

Actions provided:
    ~/set_mode: Change robot operational mode.

Example:
    ros2 run tabletop_rig mock_robot_state_helper
"""

import rclpy
from rclpy.action.server import ActionServer, ServerGoalHandle
from rclpy.executors import (
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.experimental import EventsExecutor
from ur_dashboard_msgs.action import SetMode

from tabletop_rig.nodes.base import BaseNode


class MockRobotStateHelper(BaseNode):
    """Mock Universal Robots robot state helper for testing.

    Simulates the UR robot state helper interface providing a minimal
    SetMode action server that always succeeds. This allows testing the
    Commander node without a physical robot.

    Attributes:
        set_mode_server: Action server for the SetMode action.
    """

    def __init__(self):
        super().__init__("robot_state_helper")

        self.declare_parameter("is_mock", True, ignore_override=True)

        self.set_mode_server = ActionServer(
            self,
            SetMode,
            "~/set_mode",
            self.set_mode_callback,
        )

        self.log("Mock Robot State Helper initialized")

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

    def destroy_node(self):
        if hasattr(self, "set_mode_server"):
            self.set_mode_server.destroy()
        super().destroy_node()


EXECUTOR_TYPE = "single-threaded"


def main(args=None):
    """Entry point for the mock robot state helper node."""
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

        mock_robot_state_helper = MockRobotStateHelper()
        executor.add_node(mock_robot_state_helper)

        try:
            executor.spin()
        finally:
            print("Shutting down mock robot state helper")
            mock_robot_state_helper.destroy_node()
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
