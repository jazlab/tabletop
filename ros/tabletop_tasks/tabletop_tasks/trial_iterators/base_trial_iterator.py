"""Foraging task."""

from abc import abstractmethod
from collections.abc import Generator, Iterator
from typing import Any, NamedTuple

from geometry_msgs.msg import Pose


# TrialSpec contains the specification for a foraging task trial
class TrialSpec(NamedTuple):
    object_id: str
    object_pose: Pose
    occlude: bool


class BaseTrialIterator(Iterator[TrialSpec]):
    """
    Base class for trial iterators.

    Trial iterators are iterators that generate trial specs.
    """

    @abstractmethod
    def __iter__(self) -> TrialSpec:
        """Generate a new trial."""
        raise NotImplementedError
