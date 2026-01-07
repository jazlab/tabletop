"""Logging utilities and mixins for ROS2 nodes.

This module provides logging functionality that integrates with ROS2's logging
system while providing graceful fallbacks when the ROS context is unavailable.
It includes utilities for converting ROS messages to dictionaries for logging
and a mixin class that adds convenience logging methods to any class.

Typical usage:
    class MyNode(rclpy.node.Node, LoggerMixin):
        def some_method(self):
            self.log("Processing data", severity="DEBUG")
            self.log_ros_msg(some_ros_msg, title="Received message")
"""

from abc import ABCMeta, abstractmethod
from collections.abc import Mapping
from typing import Any, Optional

import rclpy
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.impl.rcutils_logger import RcutilsLogger

from tabletop_py.utils.common import is_iterable, yaml_dump_string

# ROS message utilities


def msg_to_dict(msg: Any) -> dict[str, Any] | list[Any] | Any:
    """Recursively convert a ROS message to a Python dictionary.

    This function handles nested ROS messages, lists, and mappings,
    converting them all to standard Python types suitable for serialization
    or logging.

    Args:
        msg: The ROS message, mapping, iterable, or primitive value to convert.
            ROS messages are detected by the presence of a
            `get_fields_and_field_types` method.

    Returns:
        The converted value:
        - Mappings become dict[str, Any] with recursively converted values
        - Iterables become list[Any] with recursively converted items
        - ROS messages become dict[str, Any] with field names as keys
        - Primitives are returned unchanged
    """
    if isinstance(msg, Mapping):
        return {k: msg_to_dict(v) for k, v in msg.items()}
    elif is_iterable(msg):
        return [msg_to_dict(item) for item in msg]
    elif hasattr(msg, "get_fields_and_field_types"):
        return {
            field: msg_to_dict(getattr(msg, field))
            for field in msg.get_fields_and_field_types().keys()
        }
    else:
        return msg


# Logging utilities


class LoggerMixin(metaclass=ABCMeta):
    """Mixin class that adds convenience logging methods to ROS-node-like classes.

    This mixin provides a unified logging interface that works with ROS2's
    logging system when available, with automatic fallback to standard output
    when the ROS context is shut down. Classes using this mixin must implement
    the `get_logger` method to provide access to a ROS logger.

    The mixin supports severity-based logging with string or enum severity
    levels, and provides special support for logging ROS messages as
    human-readable YAML.

    Example:
        class MyInterface(LoggerMixin):
            def __init__(self, node: rclpy.node.Node):
                self._node = node

            def get_logger(self) -> RcutilsLogger:
                return self._node.get_logger()

            def do_something(self):
                self.log("Starting operation", severity="DEBUG")
    """

    @abstractmethod
    def get_logger(self) -> RcutilsLogger:
        """Return the ROS logger instance for this object.

        Returns:
            The RcutilsLogger instance to use for logging.
        """
        ...

    def log(
        self, message: Any, severity: str | LoggingSeverity = "INFO", **kwargs
    ) -> bool:
        """Log a message with the specified severity level.

        When the ROS context is active, this delegates to the appropriate
        ROS logging method. When ROS is shut down, it falls back to printing
        to stdout if the message meets the severity threshold.

        Args:
            message: The message to log. Will be converted to string.
            severity: Log level as string ("DEBUG", "INFO", "WARN", "ERROR",
                "FATAL") or LoggingSeverity enum. Defaults to "INFO".
            **kwargs: Additional keyword arguments passed to the ROS logger
                (e.g., `throttle_duration_sec` for throttled logging).

        Returns:
            True if the message was logged (met severity threshold),
            False otherwise.

        Raises:
            ValueError: If an invalid severity string is provided.
        """
        if not isinstance(severity, LoggingSeverity):
            severity = LoggingSeverity[severity]

        if rclpy.ok():  # type: ignore
            match severity:
                case LoggingSeverity.DEBUG:
                    return self.get_logger().debug(message, **kwargs)
                case LoggingSeverity.INFO:
                    return self.get_logger().info(message, **kwargs)
                case LoggingSeverity.WARN:
                    return self.get_logger().warning(message, **kwargs)
                case LoggingSeverity.ERROR:
                    return self.get_logger().error(message, **kwargs)
                case LoggingSeverity.FATAL:
                    return self.get_logger().fatal(message, **kwargs)
                case _:
                    raise ValueError(f"Invalid severity: {severity}")
        elif severity >= self.get_logger().get_effective_level():
            print(f"{severity.name}: {message}")
            return True
        else:
            return False

    @property
    def log_level(self) -> LoggingSeverity:
        """The current effective logging severity level.

        Returns:
            The minimum LoggingSeverity that will be logged.
        """
        return self.get_logger().get_effective_level()

    def log_ros_msg(
        self,
        msg: Any,
        title: Optional[str] = None,
        severity: str | LoggingSeverity = "INFO",
        yaml_width: int = 120,
    ) -> bool:
        """Log a ROS message as a formatted YAML string.

        Converts the ROS message to a dictionary and formats it as YAML
        for human-readable logging output. Only logs if the severity
        meets the current threshold.

        Args:
            msg: The ROS message to log. Must have `get_fields_and_field_types`
                method or be a mapping/iterable.
            title: Optional title to prepend to the YAML output.
            severity: Log level as string or LoggingSeverity enum.
                Defaults to "INFO".
            yaml_width: Maximum line width for YAML formatting. Defaults to 120.

        Returns:
            True if the message was logged, False if filtered by severity.
        """
        if not isinstance(severity, LoggingSeverity):
            severity = LoggingSeverity[severity]

        if severity >= self.log_level:
            string = yaml_dump_string(msg_to_dict(msg), width=yaml_width)
            if title is not None:
                string = f"{title}:\n{string}"
            success = self.log(string, severity=severity)
            assert success
            return True
        else:
            return False
