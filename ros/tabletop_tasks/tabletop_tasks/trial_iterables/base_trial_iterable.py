"""Foraging task."""

from abc import abstractmethod
from collections.abc import Generator, Iterable
from typing import Any, NamedTuple

from geometry_msgs.msg import Pose


# TrialSpec contains the specification for a foraging task trial
class TrialSpec(NamedTuple):
    object_id: str
    object_pose: Pose
    occlude: bool


class BaseTrialIterable(Iterable[TrialSpec]):
    """
    Base class for trial iterables.

    Trial iterables are iterables that generate trial specs.
    """

    @abstractmethod
    def __iter__(self) -> Generator[TrialSpec, Any, None]:
        """Generate a new trial."""
        raise NotImplementedError
