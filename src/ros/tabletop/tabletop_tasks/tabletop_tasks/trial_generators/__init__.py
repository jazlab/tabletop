"""Trial generators for producing experimental trial sequences.

This subpackage contains trial generator classes that produce sequences
of TrialSpec objects for behavioral experiments. Generators implement
the Python iterator protocol and can optionally adapt based on feedback.

Available Generators:
    BaseTrialGenerator: Abstract base class for all generators.
    RandomChoice: Random sampling from parameter distributions.
    OrderedChoice: Deterministic cycling through parameter combinations.
    BlockedCupDrawer: Adaptive block-structured generator for cup/drawer tasks.

Data Classes:
    TrialSpec: Specification for a single trial (object, pose, arm, occlusion).
    TrialFeedback: Behavioral feedback from completed trials.

Example:
    from tabletop_tasks.trial_generators import RandomChoice, TrialSpec
    generator = RandomChoice(commander, object_ids=["cup_1"], ...)
    for trial_spec in generator:
        feedback = await task.run_trial(trial_spec)
        generator.send(feedback)
"""

from .base import BaseTrialGenerator, TrialFeedback, TrialSpec
from .blocked_cup_drawer import BlockedCupDrawer
from .ordered_choice import OrderedChoice
from .ordered_choice_alternating import OrderedChoiceAlternating
from .random_choice import RandomChoice
from .random_choice_alternating import RandomChoiceAlternating

__all__ = [
    "BaseTrialGenerator",
    "BlockedCupDrawer",
    "OrderedChoice",
    "OrderedChoiceAlternating",
    "RandomChoice",
    "RandomChoiceAlternating",
    "TrialFeedback",
    "TrialSpec",
]
