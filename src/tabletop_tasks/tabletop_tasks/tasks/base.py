"""Base task module."""

from abc import ABC, abstractmethod
from collections.abc import Mapping
from typing import Any

import rclpy.logging
from rclpy.impl.logging_severity import LoggingSeverity
from rpyutils.import_c_library import importlib

from tabletop_server.nodes import Commander
from tabletop_tasks.trial_generators.base import BaseTrialGenerator

logger = rclpy.logging.get_logger("tabletop_task")


DEFAULT_NOTE = {
    "name": "C",
    "octave": 4,
    "velocity": 127,
    "channel": 0,
}


class BaseTask(ABC):
    """Abstract base class for all tasks."""

    def __init__(
        self,
        commander: Commander,
        trial_generator: BaseTrialGenerator | Mapping[str, Any],
    ):
        """Initialize base task.

        Args:
            commander: Commander instance for interacting with the system
            **kwargs: Additional task-specific arguments
        """
        self._commander = commander

        # Create trial_generator if necessary
        if isinstance(trial_generator, Mapping):
            self._trial_generator = getattr(
                importlib.import_module("tabletop_tasks.trial_generators"),
                trial_generator["class"],
            )(commander, **trial_generator["kwargs"])
            assert isinstance(self._trial_generator, BaseTrialGenerator)
        else:
            self._trial_generator = trial_generator

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
                    return logger.debug(message, **kwargs)
                case LoggingSeverity.INFO:
                    return logger.info(message, **kwargs)
                case LoggingSeverity.WARN:
                    return logger.warning(message, **kwargs)
                case LoggingSeverity.ERROR:
                    return logger.error(message, **kwargs)
                case LoggingSeverity.FATAL:
                    return logger.fatal(message, **kwargs)
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
