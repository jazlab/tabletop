"""Foraging task."""

import abc
import collections
import enum
import time

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
    def __call__(self) -> TrialSpec:
        """Generate a new trial."""
        raise NotImplementedError
    
    @abc.abstractmethod
    def feedback(self,
                 trial_spec: TrialSpec,
                 broke_fixation: bool,
                 reaction_time: float,
                 timeout: float,
                 ) -> None:
        """Provide feedback for trial."""
        raise NotImplementedError