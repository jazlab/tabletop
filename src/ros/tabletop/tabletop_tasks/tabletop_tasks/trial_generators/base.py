"""Base classes for trial generation in behavioral experiments.

This module defines the core abstractions for generating trial sequences
in behavioral experiments. Trial generators produce TrialSpec objects that
define what happens in each trial, and receive TrialFeedback after each
trial completes.

The generator pattern allows for:
- Adaptive trial sequences based on subject performance
- Block-structured designs with condition switching
- Random or ordered trial selection

Classes:
    TrialSpec: Immutable specification for a single trial.
    TrialFeedback: Immutable feedback from a completed trial.
    BaseTrialGenerator: Abstract base class for all trial generators.

Example:
    class MyGenerator(BaseTrialGenerator):
        def __next__(self) -> TrialSpec:
            return TrialSpec(...)

        def send(self, feedback: TrialFeedback):
            # Adapt based on feedback
            pass
"""

import dataclasses
from abc import ABCMeta, abstractmethod
from typing import Literal

from geometry_msgs.msg import PoseStamped
from rclpy.impl.rcutils_logger import RcutilsLogger
from tabletop_rig.nodes import Commander
from tabletop_rig.utils.logging import LoggerMixin


@dataclasses.dataclass(frozen=True)
class TrialSpec:
    """Specification for a single experimental trial.

    Defines all parameters needed to execute one trial, including
    which object to present, where to present it, and behavioral
    constraints.

    Attributes:
        object_id: Identifier for the object to present (e.g., "cup_1").
        object_pose: Target pose for object presentation.
        arm: Which arm(s) the subject should use for response.
        occlude: Whether to occlude the smartglass during delay period.
    """

    trial_number: int
    object_id: str = ""
    group_name: str = ""
    object_pose: PoseStamped = dataclasses.field(default_factory=PoseStamped)
    arm: Literal["left", "right", "both"] = "both"
    occlude: bool = False


@dataclasses.dataclass
class TrialFeedback:
    """Feedback from a completed trial.

    Contains behavioral measures from the trial that can be used
    by adaptive trial generators to modify subsequent trials.

    Attributes:
        reaction_time: Subject's reaction time in seconds, or None if
            no response was recorded.
        timeout: True if the trial timed out without a response.
    """

    reaction_time: float | None = None
    timeout: bool | None = None


class BaseTrialGenerator(LoggerMixin, metaclass=ABCMeta):
    """Abstract base class for trial generators.

    Trial generators implement the Python generator protocol to produce
    a sequence of TrialSpec objects. They also receive TrialFeedback
    via the send() method, enabling adaptive trial sequences.

    Subclasses must implement __next__() to generate trials and
    send() to process feedback.

    Attributes:
        commander: Reference to the Commander node for accessing
            robot capabilities (e.g., creating pose messages).

    Example:
        generator = MyTrialGenerator(commander, num_trials=100)
        for trial_spec in generator:
            feedback = await task.run_trial(trial_spec)
            generator.send(feedback)
    """

    def __init__(self, name: str, commander: Commander):
        """Initialize the trial generator.

        Args:
            commander: Commander instance for robot interaction.
            logger_name: Name for the ROS logger.
        """
        self._commander = commander
        self._logger = commander.get_logger().get_child(name)

    def get_logger(self) -> RcutilsLogger:
        """Get the logger instance.

        Returns:
            ROS logger for this generator.
        """
        return self._logger

    @property
    def commander(self) -> Commander:
        """Get the commander instance.

        Returns:
            The Commander node reference.
        """
        return self._commander

    @property
    @abstractmethod
    def group_names(self) -> list[str]: ...

    @abstractmethod
    def __next__(self) -> TrialSpec:
        """Generate the next trial specification.

        Returns:
            TrialSpec for the next trial.

        Raises:
            StopIteration: When no more trials should be generated.
        """

    @abstractmethod
    def send(self, trial_spec: TrialSpec, feedback: TrialFeedback | None):
        """Process feedback from a completed trial.

        Called after each trial completes with behavioral measures.
        Can be used to adapt subsequent trial generation.

        Args:
            trial_spec: Original trial spec that feedback corresponds to.
            feedback: Behavioral feedback from the completed trial.
        """

    def __iter__(self):
        """Return self as iterator.

        Returns:
            Self, implementing the iterator protocol.
        """
        return self


class DefaultTrialGenerator(BaseTrialGenerator):
    """Placeholder trial generator that yields a single None trial.

    Used by tasks that don't require a trial generator but still
    need to conform to the task execution interface.
    """

    def __init__(self, commander: Commander):
        super().__init__("default_trial_generator", commander)
        self._trial_count = 0

    def __next__(self) -> TrialSpec:
        """Return None once, then stop iteration.

        Returns:
            None on first call.

        Raises:
            StopIteration: On subsequent calls.
        """
        trial_spec = TrialSpec(trial_number=self._trial_count)
        self._trial_count += 1
        return trial_spec

    @property
    def group_names(self) -> list[str]:
        return []

    def send(self, trial_spec: TrialSpec, feedback: TrialFeedback | None):
        """Accept and ignore trial feedback.

        Args:
            trial_spec: Unused trial spec.
            feedback: Unused trial feedback.
        """
        pass
