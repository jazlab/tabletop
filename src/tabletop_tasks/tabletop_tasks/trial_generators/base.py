"""Foraging task."""

from abc import abstractmethod
from dataclasses import dataclass
from typing import Any, Literal, NamedTuple

import rclpy.logging
from geometry_msgs.msg import PoseStamped
from rclpy.impl.logging_severity import LoggingSeverity

from tabletop_server.nodes import Commander

logger = rclpy.logging.get_logger("trial_generator")


class TrialSpec(NamedTuple):
    """Specification for a Foraging task trial."""

    object_id: str
    object_pose: PoseStamped
    arm: Literal["left", "right", "both"]
    occlude: bool


@dataclass(slots=True)
class TrialFeedback:
    """Feedback for a Foraging task trial."""

    reaction_time: float | None = None
    timeout: bool | None = None


class BaseTrialGenerator:
    """
    Base class for trial generators.

    Trial generators are generators that generate trial specs.
    """

    def __init__(self, commander: Commander):
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
    def __next__(self) -> TrialSpec:
        """Generate a new trial."""
        raise NotImplementedError

    @abstractmethod
    def send(self, feedback: TrialFeedback):
        """Get trial feedback."""
        raise NotImplementedError

    def __iter__(self):
        return self
