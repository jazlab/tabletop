from abc import ABCMeta, abstractmethod
from collections.abc import Mapping
from typing import (
    Any,
    Optional,
)

import rclpy
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.impl.rcutils_logger import RcutilsLogger

from tabletop_py.utils.common import is_iterable, yaml_dump_string

# ROS message utilities


def msg_to_dict(msg: Any) -> dict[str, Any] | list[Any] | Any:
    """Convert a ROS message to a dictionary."""
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
    """Adds convenience functions to 'ROS-node-like' class"""

    @abstractmethod
    def get_logger(self) -> RcutilsLogger: ...

    def log(
        self, message: Any, severity: str | LoggingSeverity = "INFO", **kwargs
    ):
        """
        Log a message with the given severity.
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
        """Get the log severity."""
        return self.get_logger().get_effective_level()

    def log_ros_msg(
        self,
        msg: Any,
        title: Optional[str] = None,
        severity: str | LoggingSeverity = "INFO",
        yaml_width: int = 120,
    ) -> bool:
        """Log a ROS message as a YAML string."""
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
