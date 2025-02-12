"""Foraging task."""

import abc
import collections
from typing import Any, Generator

# TrialSpec contains the specification for a foraging task trial
TrialSpec = collections.namedtuple(
    "TrialSpec",
    [
        "object_id",
        "object_pose",
        "occlude",
    ],
)


class BaseTrialGenerator(abc.ABC):
    """Base class for trial generators."""

    @abc.abstractmethod
    def __iter__(self) -> Generator[TrialSpec, Any, None]:
        """Generate a new trial."""
        raise NotImplementedError
