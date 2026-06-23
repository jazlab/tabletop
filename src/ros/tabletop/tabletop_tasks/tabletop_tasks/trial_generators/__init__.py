"""Trial generators for producing experimental trial sequences.

This subpackage contains trial generator classes that produce sequences
of TrialSpec objects for behavioral experiments. Generators implement
the Python iterator protocol and can optionally adapt based on feedback.

Available Generators:
    BaseTrialGenerator: Abstract base class for all generators.
    OrderedChoiceAlternating: Deterministic cycling through parameter
        combinations, alternating between robot groups (left/right).
    RandomChoiceAlternating: Random sampling from parameter distributions,
        alternating between robot groups (left/right).

Data Classes:
    TrialSpec: Specification for a single trial (object, pose, arm, occlusion).
    TrialFeedback: Behavioral feedback from completed trials.

Example:
    from tabletop_tasks.trial_generators import (
        RandomChoiceAlternating,
        TrialSpec,
    )
    generator = RandomChoiceAlternating(commander, object_ids=["cup_1"], ...)
    for trial_spec in generator:
        feedback = await task.run_trial(trial_spec)
        generator.send(trial_spec, feedback)
"""

from .base import BaseTrialGenerator, TrialFeedback, TrialSpec
from .ordered_choice_alternating import OrderedChoiceAlternating
from .random_choice_alternating import RandomChoiceAlternating

__all__ = [
    "BaseTrialGenerator",
    "OrderedChoiceAlternating",
    "RandomChoiceAlternating",
    "TrialFeedback",
    "TrialSpec",
]
