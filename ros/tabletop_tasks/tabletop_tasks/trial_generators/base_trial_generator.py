"""Foraging task."""

from abc import abstractmethod
from typing import NamedTuple

from geometry_msgs.msg import PoseStamped


# TrialSpec contains the specification for a foraging task trial
class TrialSpec(NamedTuple):
    object_id: str
    object_pose: PoseStamped
    occlude: bool


class BaseTrialGenerator:
    """
    Base class for trial generators.

    Trial generators are generators that generate trial specs.
    """

    @abstractmethod
    def __next__(self) -> TrialSpec:
        """Generate a new trial."""
        raise NotImplementedError

    @abstractmethod
    def send(self, **trial_feedback: dict) -> None:
        """Get trial feedback."""
        raise NotImplementedError

    def __iter__(self):
        return self
