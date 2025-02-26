"""Block-structured cup/drawer trial generator."""

from typing import Any

import numpy as np
from tabletop_utils.ros import pose_stamped_msg

from tabletop_tasks.trial_generators import BaseTrialGenerator, TrialSpec


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
        object_ids: list[str],
        poses: list[dict[str, Any]],
        num_trials: int,
    ):
        self._object_ids = object_ids
        self._poses = [pose_stamped_msg(**pose) for pose in poses]
        self._num_trials = num_trials
        self._trial_counter = 0

    def __next__(self) -> TrialSpec:
        """Return a trial."""
        if self._trial_counter >= self._num_trials:
            raise StopIteration

        # Sample object pose
        object_pose = np.random.choice(self._poses)  # type: ignore

        # Sample object id
        object_id = np.random.choice(self._object_ids)

        # Make trial spec
        trial_spec = TrialSpec(
            object_id=object_id,
            object_pose=object_pose,
            occlude=True,
        )

        self._trial_counter += 1

        return trial_spec

    def send(self, value: dict[str, Any]) -> None:
        raise NotImplementedError(
            "RandomObject trial generator does not support sending feedback"
        )
