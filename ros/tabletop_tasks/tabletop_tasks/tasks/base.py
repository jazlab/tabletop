"""Base task module."""

from abc import ABC, abstractmethod
from typing import Any

import rclpy.logging
from rclpy.impl.logging_severity import LoggingSeverity
from tabletop_server.nodes import Commander

logger = rclpy.logging.get_logger("tabletop_task")


class BaseTask(ABC):
    """Abstract base class for all tasks."""

    def __init__(self, commander: Commander) -> None:
        """Initialize base task.

        Args:
            commander: Commander instance for interacting with the system
            **kwargs: Additional task-specific arguments
        """
        self._commander = commander

    @property
    def commander(self) -> Commander:
        """Get the commander instance."""
        return self._commander

    @property
    def log_level(self) -> LoggingSeverity:
        """Get the log severity."""
        return logger.get_effective_level()

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
                    logger.debug(message, **kwargs)
                case LoggingSeverity.INFO:
                    logger.info(message, **kwargs)
                case LoggingSeverity.WARN:
                    logger.warning(message, **kwargs)
                case LoggingSeverity.ERROR:
                    logger.error(message, **kwargs)
                case LoggingSeverity.FATAL:
                    logger.fatal(message, **kwargs)
                case _:
                    raise ValueError(f"Invalid severity: {severity}")
        elif severity >= self.log_level:
            print(f"{severity.name}: {message}")
            return True
        else:
            return False

    @abstractmethod
    async def run(self) -> None:
        """Run the task to completion."""
        raise NotImplementedError("Tasks must implement run() method")
