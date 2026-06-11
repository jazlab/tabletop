"""Random choice trial generator for behavioral experiments.

This module provides a trial generator that randomly samples trial
parameters from specified distributions. Each trial independently
samples object ID, pose, arm assignment, and occlusion.

This generator is non-adaptive - feedback is ignored and does not
influence subsequent trial generation.

Example:
    generator = RandomChoice(
        commander=commander,
        object_ids=["cup_1", "cup_2"],
        poses=[{"position": [0.5, 0, 0.3]}],
        arms=["left", "right"],
        occlude_prob=0.5,
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


class RandomChoice(BaseTrialGenerator):
    """Trial generator with random independent sampling.

    Generates trials by randomly sampling each parameter independently:
    - Object ID: uniform random from provided list
    - Pose: uniform random from provided list
    - Arm: uniform random from provided list
    - Occlude: Bernoulli with specified probability

    This generator does not adapt based on feedback.

    Attributes:
        _object_ids: List of object identifiers to sample from.
        _poses: List of PoseStamped objects to sample from.
        _arms: List of arm assignments to sample from.
        _occlude_prob: Probability of occlusion on each trial.
        _num_trials: Total number of trials to generate.
        _trial_counter: Current trial count.
    """

    def __init__(
        self,
        commander: Commander,
        object_ids: list[str],
        poses: list[Mapping[str, Any]],
        arms: list[Literal["left", "right", "both"]],
        occlude_prob: float,
        num_trials: int,
        skip_failed: bool = True,
    ):
        """Initialize the random choice generator.

        Args:
            commander: Commander instance for robot interaction.
            object_ids: List of object IDs to randomly select from.
            poses: List of pose dictionaries (passed to pose_stamped_msg).
            arms: List of arm assignments to randomly select from.
            occlude_prob: Probability of smartglass occlusion (0.0 to 1.0).
            num_trials: Total number of trials to generate.

        Raises:
            ValueError: If num_trials is less than 1.
        """
        super().__init__("random_choice_trial_generator", commander)

        self._object_ids = object_ids
        self._poses = [pose_stamped_msg(**pose) for pose in poses]
        self._arms = arms

        self._occlude_prob = occlude_prob

        if num_trials < 1:
            raise ValueError("num_trials must be at least 1")
        self._num_trials = num_trials
        self._trial_counter = 0

        # Store last TrialSpec in case we want to redo it
        self._skip_failed = skip_failed
        self._last_trial_spec: TrialSpec | None = None

    def __next__(self) -> TrialSpec:
        """Generate the next random trial.

        Returns:
            TrialSpec with randomly sampled parameters.

        Raises:
            StopIteration: When num_trials have been generated.
        """
        if not self._skip_failed and self._last_trial_spec is not None:
            return self._last_trial_spec

        if self._trial_counter >= self._num_trials:
            raise StopIteration

        # Sample object pose
        object_pose = np.random.choice(self._poses)  # type: ignore[arg-type]

        # Sample object id
        object_id = np.random.choice(self._object_ids)

        # Sample arm
        arm = np.random.choice(self._arms)

        # Sample occlude
        occlude = np.random.choice(
            [True, False], p=[self._occlude_prob, 1 - self._occlude_prob]
        )

        # Make trial spec
        self._last_trial_spec = TrialSpec(
            trial_number=self._trial_counter,
            object_id=object_id,
            object_pose=object_pose,
            arm=arm,
            occlude=occlude,
        )
        self._trial_counter += 1
        return self._last_trial_spec

    def send(self, trial_spec: TrialSpec, feedback: TrialFeedback):
        """Process trial feedback.

        This generator does not adapt based on feedback. Clears the last
        trial spec on successful feedback. If skip_failed is False and
        feedback is None, retains the trial for retry.

        Args:
            trial_spec: Original trial spec (unused).
            feedback: Trial feedback (unused - only status checked).
        """
        pass
