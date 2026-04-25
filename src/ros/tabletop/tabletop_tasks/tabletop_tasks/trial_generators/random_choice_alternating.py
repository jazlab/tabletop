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

from collections.abc import Mapping
from typing import Any, Literal

import numpy as np
from tabletop_rig.nodes import Commander
from tabletop_rig.utils.ros import pose_stamped_msg

from tabletop_tasks.trial_generators.base import (
    BaseTrialGenerator,
    TrialFeedback,
    TrialSpec,
)


class RandomChoiceAlternating(BaseTrialGenerator):
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
        grouped_object_ids: dict[str, list[str]],
        poses: list[Mapping[str, Any]],
        arms: list[Literal["left", "right", "both"]],
        occlude_prob: float,
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
        super().__init__(
            "ordered_choice_alternating_trial_generator", commander
        )

        # Check if all objects are reachable
        for group_name, object_ids in grouped_object_ids.items():
            not_reachable = set(
                object_ids
            ) - self.commander.reachable_object_ids(group_name)
            if len(not_reachable) > 0:
                raise ValueError(
                    f"group_object_ids contains objects not reachable by robot {group_name}: {not_reachable}"
                )

        # Check that num_trials is valid
        if num_trials < 1:
            raise ValueError("num_trials must be at least 1")

        self._grouped_object_ids = grouped_object_ids
        self._largest_group = max(len(x) for x in grouped_object_ids.values())
        self._group_names = list(grouped_object_ids.keys())
        self._poses = [pose_stamped_msg(**pose) for pose in poses]
        self._arms = arms
        self._occlude_prob = occlude_prob
        self._num_trials = num_trials
        self._skip_failed = skip_failed

        self._trial_counter = 0
        self._last_trial_spec: dict[str, TrialSpec | None] = {
            x: None for x in self._group_names
        }
        self._last_feedback_group: str | None = None
        self._next_group_idx: int = 0

    @property
    def group_names(self) -> list[str]:
        return self._group_names

    def __next__(self) -> TrialSpec:
        """Generate the next trial in sequence.

        Cycles through the parameter grid, wrapping to the beginning
        when all combinations have been used.

        Returns:
            TrialSpec with the next parameter combination.

        Raises:
            StopIteration: When num_trials have been generated.
        """

        group_name: str
        if self._last_feedback_group is not None:
            group_name = self._last_feedback_group
            self._last_feedback_group = None
        else:
            group_name = self._group_names[self._next_group_idx]
            self._next_group_idx = (self._next_group_idx + 1) % len(
                self._group_names
            )

        last_trial_spec = self._last_trial_spec[group_name]
        if last_trial_spec is not None:
            return last_trial_spec

        if self._trial_counter >= self._num_trials:
            raise StopIteration

        object_pose = np.random.choice(self._poses)  # type: ignore[arg-type]
        object_id = np.random.choice(self._grouped_object_ids[group_name])
        arm = np.random.choice(self._arms)
        occlude = np.random.choice(
            [True, False], p=[self._occlude_prob, 1 - self._occlude_prob]
        )
        trial_spec = TrialSpec(
            trial_number=self._trial_counter,
            object_id=object_id,
            group_name=group_name,
            object_pose=object_pose,
            arm=arm,
            occlude=occlude,
        )

        self._last_trial_spec[group_name] = trial_spec
        self._trial_counter += 1

        return trial_spec

    def send(self, trial_spec: TrialSpec, feedback: TrialFeedback | None):
        """Process trial feedback.

        This generator does not adapt based on feedback.

        Args:
            trial_spec: Unused original trial spec.
            feedback: Unused trial feedback.
        """
        if trial_spec is not None:
            last_trial_spec = self._last_trial_spec[trial_spec.group_name]
            assert (
                last_trial_spec is not None
                and trial_spec.trial_number == last_trial_spec.trial_number
            )
            if feedback is not None or self._skip_failed:
                self._last_trial_spec[trial_spec.group_name] = None

            self._last_feedback_group = trial_spec.group_name
