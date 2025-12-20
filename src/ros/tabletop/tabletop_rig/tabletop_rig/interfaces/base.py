from rclpy.impl.rcutils_logger import RcutilsLogger

from tabletop_rig.nodes.base import BaseNode
from tabletop_rig.utils.logging import LoggerMixin


class BaseInterface(LoggerMixin):
    def __init__(self, node: BaseNode, logger_name: str = "interface"):
        """Initializes the Node Interface

        Args:
            node: Parent ROS node
            logger_name: Name to give logger
        """
        self._node = node
        self._logger = node.get_logger().get_child(logger_name)

    def get_logger(self) -> RcutilsLogger:
        """Get the logger instance"""
        return self._logger

    @property
    def node(self) -> BaseNode:
        """Get the parent node instance"""
        return self._node
