"""Base interface class for ROS2 node interfaces.

This module provides the foundational BaseInterface class that all other
interfaces in the tabletop_rig package inherit from. It establishes the
pattern of interfaces being associated with a parent node and having
their own namespaced logger.

The interface pattern separates concerns by grouping related functionality
(e.g., MoveIt operations, hardware communication) into distinct classes
that operate on behalf of a node.
"""

from rclpy.impl.rcutils_logger import RcutilsLogger

from tabletop_rig.nodes.base import BaseNode
from tabletop_rig.utils.logging import LoggerMixin


class BaseInterface(LoggerMixin):
    """Base class for interfaces that operate on behalf of a ROS2 node.

    Interfaces encapsulate specific functionality (motion planning, hardware
    communication, etc.) and operate using a parent node's ROS2 resources
    (publishers, subscribers, services, etc.). Each interface gets its own
    child logger for namespaced log output.

    This class implements LoggerMixin to provide convenient logging methods.

    Attributes:
        _node: The parent ROS2 node this interface operates on behalf of.
        _logger: Child logger for this interface's log output.

    Example:
        class MyInterface(BaseInterface):
            def __init__(self, node: BaseNode):
                super().__init__(node, "my_interface")
                # Create ROS resources using self._node
                self._pub = self._node.create_publisher(...)

            def do_something(self):
                self.log("Doing something", severity="DEBUG")
    """

    def __init__(self, node: BaseNode, logger_name: str = "interface") -> None:
        """Initialize the interface with a parent node.

        Args:
            node: The parent ROS2 node that owns this interface. The interface
                will use the node's resources for ROS communication.
            logger_name: Name suffix for this interface's logger. The full
                logger name will be "{node_name}.{logger_name}".
        """
        self._node = node
        self._logger = node.get_logger().get_child(logger_name)

    def get_logger(self) -> RcutilsLogger:
        """Return the logger instance for this interface.

        Required by LoggerMixin to provide logging functionality.

        Returns:
            The RcutilsLogger instance for this interface.
        """
        return self._logger

    @property
    def node(self) -> BaseNode:
        """The parent node this interface operates on behalf of.

        Returns:
            The BaseNode instance that owns this interface.
        """
        return self._node
