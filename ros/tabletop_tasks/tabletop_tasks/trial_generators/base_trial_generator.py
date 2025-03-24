"""Foraging task."""

from abc import abstractmethod
from typing import NamedTuple

from geometry_msgs.msg import PoseStamped
from tabletop_server.nodes import Commander


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

    def __init__(self, commander: Commander):
        self._commander = commander

    @property
    def commander(self) -> Commander:
        return self._commander

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
