"""Foraging task."""

from abc import abstractmethod
from collections.abc import Generator
from typing import Any, NamedTuple

from geometry_msgs.msg import Pose


# TrialSpec contains the specification for a foraging task trial
class TrialSpec(NamedTuple):
    object_id: str
    object_pose: Pose
    occlude: bool


class BaseTrialGenerator():
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
