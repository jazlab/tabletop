from rclpy.impl.rcutils_logger import RcutilsLogger

from tabletop_rig.nodes.base import BaseNode
from tabletop_rig.utils.logging import LoggerMixin


class BaseInterface(LoggerMixin):
    def __init__(self, node: BaseNode, logger_name: str):
        self.node = node
        self.logger_name = logger_name

    def get_logger(self) -> RcutilsLogger:
        """Gets child logger of parent node"""
        return self.node.get_logger().get_child(self.logger_name)
