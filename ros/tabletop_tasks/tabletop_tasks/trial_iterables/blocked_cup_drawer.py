"""Block-structured cup/drawer trial generator."""

from random import randrange
from typing import Any, Generator

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion

from tabletop_tasks.trial_generators import BaseTrialGenerator, TrialSpec


class BlockedCupDrawer(BaseTrialGenerator):
    """Base class for trial generators."""

    def __init__(
        self,
        pose_sets: dict[str, list[float]],
        correct_trials_per_block: int = 10,
    ):
        self._correct_trials_per_block = correct_trials_per_block

        # Setup cup and drawer object ids
        self._object_ids = {
            "cup": ["cup_1", "cup_2", "cup_3", "cup_4"],
            "drawer": ["drawer_1", "drawer_2", "drawer_3", "drawer_4"],
        }
        self._block_keys = list(self._object_ids.keys())

        # Setup positions and orientations. Each trial, a random position and
        # orientation will be sampled from these sets.
        self._pose_sets = pose_sets

    def __iter__(self) -> Generator[TrialSpec, dict[str, Any], None]:
        """Generate trials."""
        num_correct = 0
        block_index = randrange(len(self._block_keys))

        while num_correct < self._correct_trials_per_block:
            # Sample object pose
            object_pose = {
                k: np.random.choice(v) for k, v in self._pose_sets.items()
            }

            object_pose = Pose(
                position=Point(
                    x=object_pose["x"], y=object_pose["y"], z=object_pose["z"]
                ),
                orientation=Quaternion(x=0, y=0, z=0, w=1),
            )

            # Sample object type
            block_key = self._block_keys[block_index]
            object_id = np.random.choice(self._object_ids[block_key])

            # Make trial spec
            trial_spec = TrialSpec(
                object_id=object_id,
                object_pose=object_pose,
                occlude=True,
            )

            # Get feedback
            feedback = yield trial_spec

            # Process feedback
            correct = (
                not feedback["broke_fixation"] and not feedback["timeout"]
            )
            if correct:
                num_correct += 1

        raise StopIteration
