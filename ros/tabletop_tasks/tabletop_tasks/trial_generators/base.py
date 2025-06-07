"""Foraging task."""

from abc import abstractmethod
from dataclasses import dataclass
from typing import NamedTuple

from geometry_msgs.msg import PoseStamped
from tabletop_server.nodes import Commander


class TrialSpec(NamedTuple):
    """Specification for a Foraging task trial."""

    object_id: str
    object_pose: PoseStamped
    occlude: bool


@dataclass(slots=True)
class TrialFeedback:
    """Feedback for a Foraging task trial."""

    next_trial_spec: bool = False
    broke_fixation: bool = False
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

    @abstractmethod
    def __next__(self) -> TrialSpec:
        """Generate a new trial."""
        raise NotImplementedError

    @abstractmethod
    def send(self, feedback: TrialFeedback) -> None:
        """Get trial feedback."""
        raise NotImplementedError

    def __iter__(self):
        return self
