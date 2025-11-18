from abc import ABCMeta, abstractmethod
from typing import Literal, NamedTuple

import rclpy.logging
from geometry_msgs.msg import PoseStamped
from rclpy.impl.rcutils_logger import RcutilsLogger
from tabletop_rig.nodes import Commander
from tabletop_rig.utils.logging import LoggerMixin


class TrialSpec(NamedTuple):
    """Specification for a Foraging task trial."""

    object_id: str
    object_pose: PoseStamped
    arm: Literal["left", "right", "both"]
    occlude: bool


class TrialFeedback(NamedTuple):
    """Feedback for a Foraging task trial."""

    reaction_time: float | None
    timeout: bool | None


class BaseTrialGenerator(LoggerMixin, metaclass=ABCMeta):
    """Abstract base class for trial generators.

    Trial generators are Python generators that produce TrialSpecs and receive
    TrialFeedback.
    """

    def __init__(
        self, commander: Commander, logger_name: str = "trial_generator"
    ):
        self._commander = commander
        self._logger = rclpy.logging.get_logger(logger_name)

    def get_logger(self) -> RcutilsLogger:
        """Get the logger instance"""
        return self._logger

    @property
    def commander(self) -> Commander:
        """Get the commander instance."""
        return self._commander

    @abstractmethod
    def __next__(self) -> TrialSpec:
        """Generate a new trial."""

    @abstractmethod
    def send(self, feedback: TrialFeedback):
        """Get trial feedback."""

    def __iter__(self):
        return self
