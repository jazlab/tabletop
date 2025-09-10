"""Block-structured cup/drawer trial generator."""

from collections.abc import Mapping
from typing import Any, Literal

import numpy as np
from tabletop_server.nodes import Commander

from tabletop_tasks.trial_generators.base import (
    BaseTrialGenerator,
    TrialFeedback,
    TrialSpec,
)


class RandomChoice(BaseTrialGenerator):
    """
    Random choice trial generator.

    Randomly chooses an object from a list of objects and a pose from a list of poses.

    Args:
        object_ids: List of object ids to choose from.
        poses: List of poses to choose from.
        num_trials: Number of trials to generate.
    """

    def __init__(
        self,
        commander: Commander,
        object_ids: list[str],
        poses: list[Mapping[str, Any]],
        arms: list[Literal["left", "right", "both"]],
        occlude_prob: float,
        num_trials: int,
    ):
        super().__init__(commander)

        self._object_ids = object_ids
        self._poses = [
            self.commander.create_pose_stamped(**pose) for pose in poses
        ]
        self._arms = arms

        self._occlude_prob = occlude_prob

        if num_trials < 1:
            raise ValueError("num_trials must be at least 1")
        self._num_trials = num_trials
        self._trial_counter = 0

    def __next__(self) -> TrialSpec:
        """Return a trial."""
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
        trial_spec = TrialSpec(
            object_id=object_id,
            object_pose=object_pose,
            arm=arm,
            occlude=occlude,
        )

        self._trial_counter += 1

        return trial_spec

    def send(self, feedback: TrialFeedback):
        """Send feedback."""
        pass
