"""Block-structured cup/drawer trial iterator."""

from random import randrange
from typing import Any, Iterator

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion

from tabletop_tasks.trial_iterators import BaseTrialIterator, TrialSpec


class BlockedCupDrawer(BaseTrialIterator):
    """Base class for trial iterators."""

    def __init__(
        self,
        poses: list[dict[str, float]],
        correct_trials_per_block: int = 10,
    ):
        self._correct_trials_per_block = correct_trials_per_block

        # Setup cup and drawer object ids
        self._object_ids = {
            "cup": ["cup_1", "cup_2", "cup_3", "cup_4"],
            "drawer": ["drawer_1", "drawer_2", "drawer_3", "drawer_4"],
        }
        self._block_keys = list(self._object_ids.keys())

        # Setup poses. Each trial, a random pose will be sampled from these.
        self._poses = poses
        
        # Initialize iterator
        self._num_correct = 0
        self._block_index = randrange(len(self._block_keys))
        
    def feedback(self, broke_fixaation: bool, timeout: bool, **unused_kwargs: dict) -> None:
        """Update iterator based on feedback."""
        del unused_kwargs
        
        # Increment number of correct trials if necessary
        if not broke_fixation and not timeout:
            self._num_correct += 1

        # Update block index if necessary
        if self._num_correct >= self._correct_trials_per_block:
            self._num_correct = 0
            self._block_index = (self._block_index + 1) % len(self._block_keys)
        
    def __iter__(self) -> Iterator[TrialSpec]:
        """Return self."""
        return self

    def __next__(self) -> TrialSpec:
        """Return a trial."""
        # Sample object pose
        object_pose = np.random.choice(self._poses)
        object_pose = Pose(
            position=Point(
                x=object_pose["x"], y=object_pose["y"], z=object_pose["z"]
            ),
            orientation=Quaternion(x=0, y=0, z=0, w=1),
        )

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
