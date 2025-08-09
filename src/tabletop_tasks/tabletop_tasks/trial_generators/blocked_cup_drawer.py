"""Block-structured cup/drawer trial generator."""

from collections.abc import Mapping
from random import randrange
from typing import Any

import numpy as np

from tabletop_server.nodes import Commander
from tabletop_tasks.trial_generators.base import BaseTrialGenerator, TrialSpec


class BlockedCupDrawer(BaseTrialGenerator):
    """Base class for trial generators."""

    def __init__(
        self,
        commander: Commander,
        poses: list[Mapping[str, Any]],
        correct_trials_per_block: int = 10,
    ):
        super().__init__(commander)
        self._correct_trials_per_block = correct_trials_per_block

        # Setup cup and drawer object ids
        self._object_ids = {
            "cup": ["cup_1", "cup_2", "cup_3", "cup_4"],
            "drawer": ["drawer_1", "drawer_2", "drawer_3", "drawer_4"],
        }
        self._block_keys = list(self._object_ids.keys())

        # Setup poses. Each trial, a random pose will be sampled from these.
        self._poses = [
            self._commander.create_pose_stamped(**pose) for pose in poses
        ]

        # Initialize generator
        self._num_correct = 0
        self._block_index = randrange(len(self._block_keys))

    def send(
        self, broke_fixation: bool, timeout: bool, **unused_kwargs: dict
    ) -> None:
        """Update generator based on feedback."""

        # Increment number of correct trials if necessary
        if not broke_fixation and not timeout:
            self._num_correct += 1

        # Update block index if necessary
        if self._num_correct >= self._correct_trials_per_block:
            self._num_correct = 0
            self._block_index = (self._block_index + 1) % len(self._block_keys)

    def __next__(self) -> TrialSpec:
        """Return a trial."""
        # Sample object pose
        object_pose = np.random.choice(self._poses)  # type: ignore

        # Sample object id
        block_key = self._block_keys[self._block_index]
        object_id = np.random.choice(self._object_ids[block_key])

        # Make trial spec
        trial_spec = TrialSpec(
            object_id=object_id,
            object_pose=object_pose,
            occlude=True,
        )

        return trial_spec
