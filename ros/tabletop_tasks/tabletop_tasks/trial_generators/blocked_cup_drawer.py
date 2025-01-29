"""Block-structured cup/drawer trial generator."""

import numpy as np

from tabletop_tasks.trial_generators import BaseTrialGenerator, TrialSpec


class BlockedCupDrawer(BaseTrialGenerator):
    """Base class for trial generators."""

    def __init__(self, correct_trials_per_block: int = 10):
        self._correct_trials_per_block = correct_trials_per_block
        self._block_index = 0
        self._correct_since_block_start = 0

        # Setup cup and drawer object ids
        self._object_ids = {
            "cup": ["cup_1", "cup_2", "cup_3", "cup_4"],
            "drawer": ["drawer_1", "drawer_2", "drawer_3", "drawer_4"],
        }
        self._block_keys = list(self._object_ids.keys())

        # Setup positions and orientations. Each trial, a random position and
        # orientation will be sampled from these sets.
        self._pose_sets = {
            "x": [-0.1, 0.1],
            "y": [-0.1, 0.1],
            "z": [0.0],
            "theta": [0.0, np.pi / 2],
            "phi": [0.0, np.pi / 2],
            "psi": [0.0],
        }

    def __call__(self) -> TrialSpec:
        """Generate a new trial."""
        if self._correct_since_block_start > self._correct_trials_per_block:
            self._correct_since_block_start = 0

        # Sample object pose
        object_pose = {
            k: np.random.choice(v) for k, v in self._pose_sets.items()
        }

        # Sample object type
        block_key = self._block_keys[self._block_index]
        object_id = np.random.choice(self._object_ids[block_key])

        # Make trial spec
        trial_spec = TrialSpec(
            object_id=object_id,
            object_pose=object_pose,
            occlude=True,
        )

        return trial_spec

    def feedback(
        self,
        trial_spec: TrialSpec,
        broke_fixation: bool,
        reaction_time: float,
        timeout: float,
    ) -> None:
        """Provide feedback for trial."""
        del trial_spec
        del reaction_time
        success = not broke_fixation and not timeout
        if success:
            self._correct_since_block_start += 1

        # Start new block if necessary
        if self._correct_since_block_start >= self._correct_trials_per_block:
            self._correct_since_block_start = 0

            # Sample a random new block index
            new_block_index = np.random.choice(len(self._block_keys) - 1)
            if new_block_index >= self._block_index:
                new_block_index += 1
            self._block_index = new_block_index
