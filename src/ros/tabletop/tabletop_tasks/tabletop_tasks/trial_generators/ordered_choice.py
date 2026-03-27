"""Ordered choice trial generator for behavioral experiments.

This module provides a trial generator that cycles through all
combinations of trial parameters in a deterministic order. Uses
itertools.product to create a full factorial design.

This generator is non-adaptive - feedback is ignored and does not
influence subsequent trial generation.

Example:
    generator = OrderedChoice(
        commander=commander,
        object_ids=["cup_1", "cup_2"],
        poses=[{"position": [0.5, 0, 0.3]}],
        arms=["left", "right"],
        occlude=[True, False],
        num_trials=100,
    )
"""

import itertools
from collections.abc import Mapping
from typing import Any, Literal

from tabletop_rig.nodes import Commander
from tabletop_rig.utils.ros import pose_stamped_msg

from tabletop_tasks.trial_generators.base import (
    BaseTrialGenerator,
    TrialFeedback,
    TrialSpec,
)


class OrderedChoice(BaseTrialGenerator):
    """Trial generator with deterministic ordered cycling.

    Generates trials by cycling through all combinations of parameters
    in a fixed order. Creates a full factorial design using itertools.product
    and iterates through combinations, wrapping when necessary.

    Parameter order in the product (fastest to slowest varying):
    occlude -> pose -> arm -> object_id

    This generator does not adapt based on feedback.

    Attributes:
        _object_ids: List of object identifiers.
        _poses: List of PoseStamped objects.
        _arms: List of arm assignments.
        _occlude: List of occlusion states.
        _num_trials: Total number of trials to generate.
        _trial_counter: Current trial count.
        _parameter_grid: Precomputed list of all parameter combinations.
    """

    def __init__(
        self,
        commander: Commander,
        object_ids: list[str],
        poses: list[Mapping[str, Any]],
        arms: list[Literal["left", "right", "both"]],
        occlude: list[bool],
        num_trials: int,
        skip_failed: bool = True,
    ):
        """Initialize the ordered choice generator.

        Args:
            commander: Commander instance for robot interaction.
            object_ids: List of object IDs to cycle through.
            poses: List of pose dictionaries (passed to pose_stamped_msg).
            arms: List of arm assignments to cycle through.
            occlude: List of occlusion states to cycle through.
            num_trials: Total number of trials to generate.

        Raises:
            ValueError: If num_trials is less than 1.
        """
        super().__init__("ordered_choice_trial_generator", commander)

        self._object_ids = object_ids
        self._poses = [pose_stamped_msg(**pose) for pose in poses]
        self._arms = arms
        self._occlude = occlude

        if num_trials < 1:
            raise ValueError("num_trials must be at least 1")
        self._num_trials = num_trials
        self._trial_counter = 0

        # Precompute all combinations for deterministic ordering
        self._parameter_grid = list(
            itertools.product(
                self._occlude, self._poses, self._arms, self._object_ids
            )
        )

        # Store last TrialSpec in case we want to redo it
        self._skip_failed = skip_failed
        self._last_trial_spec: TrialSpec | None = None

    def __next__(self) -> TrialSpec:
        """Generate the next trial in sequence.

        Cycles through the parameter grid, wrapping to the beginning
        when all combinations have been used.

        Returns:
            TrialSpec with the next parameter combination.

        Raises:
            StopIteration: When num_trials have been generated.
        """
        if not self._skip_failed and self._last_trial_spec is not None:
            return self._last_trial_spec

        if self._trial_counter >= self._num_trials:
            raise StopIteration

        occlude, pose, arm, object_id = self._parameter_grid[
            self._trial_counter % len(self._parameter_grid)
        ]
        self._last_trial_spec = TrialSpec(
            object_id=object_id,
            object_pose=pose,
            arm=arm,
            occlude=occlude,
        )

        self._trial_counter += 1

        return self._last_trial_spec

    def send(self, feedback: TrialFeedback):
        """Process trial feedback (no-op for ordered generator).

        This generator does not adapt based on feedback.

        Args:
            feedback: Unused trial feedback.
        """
        self._last_trial_spec = None
