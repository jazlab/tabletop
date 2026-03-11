"""Block-structured cup/drawer trial generator for behavioral experiments.

This module provides an adaptive trial generator that alternates between
blocks of cup trials and drawer trials. Block switching occurs after a
specified number of correct (non-timeout) trials.

This generator is adaptive - feedback is used to track correct trials
and trigger block transitions.

Example:
    generator = BlockedCupDrawer(
        commander=commander,
        poses=[{"position": [0.5, 0, 0.3]}],
        correct_trials_per_block=10,
    )
"""

from collections.abc import Mapping
from random import randrange
from typing import Any

import numpy as np
from geometry_msgs.msg import PoseStamped
from tabletop_rig.nodes import Commander
from tabletop_rig.utils.ros import pose_stamped_msg

from tabletop_tasks.trial_generators.base import (
    BaseTrialGenerator,
    TrialFeedback,
    TrialSpec,
)


class BlockedCupDrawer(BaseTrialGenerator):
    """Adaptive block-structured trial generator for cup/drawer tasks.

    Generates trials in blocks, alternating between "cup" and "drawer"
    object categories. Within each block, objects are randomly sampled
    from the current category. Block transitions occur after a specified
    number of correct (non-timeout) trials.

    This implements a common behavioral paradigm for studying category
    learning and set-shifting.

    Attributes:
        _correct_trials_per_block: Number of correct trials before switching.
        _object_ids: Dict mapping block keys to lists of object IDs.
        _block_keys: List of block category names ("cup", "drawer").
        _poses: List of PoseStamped objects to sample from.
        _num_correct: Count of correct trials in current block.
        _block_index: Index of current block in _block_keys.
    """

    def __init__(
        self,
        commander: Commander,
        poses: list[PoseStamped | Mapping[str, Any]],
        correct_trials_per_block: int = 10,
    ):
        """Initialize the blocked cup/drawer generator.

        Args:
            commander: Commander instance for robot interaction.
            poses: List of pose dictionaries (passed to pose_stamped_msg).
            correct_trials_per_block: Number of correct trials required
                before switching to the next block.
        """
        super().__init__("blocked_cup_drawer_trial_generator", commander)
        self._correct_trials_per_block = correct_trials_per_block

        # Setup cup and drawer object ids
        self._object_ids = {
            "cup": ["cup_1", "cup_2", "cup_3", "cup_4"],
            "drawer": ["drawer_1", "drawer_2", "drawer_3", "drawer_4"],
        }
        self._block_keys = list(self._object_ids.keys())

        # Setup poses. Each trial, a random pose will be sampled from these.
        self._poses = [
            pose if isinstance(pose, PoseStamped) else pose_stamped_msg(**pose)
            for pose in poses
        ]

        # Initialize generator state
        self._num_correct = 0
        self._block_index = randrange(len(self._block_keys))

    def send(self, feedback: TrialFeedback) -> None:
        """Update generator state based on trial feedback.

        Increments the correct trial counter when timeout is True.
        Switches to the next block when the criterion is reached.

        Note:
            The logic here checks feedback.timeout - verify this matches
            the intended experimental design.

        Args:
            feedback: Feedback from the completed trial.
        """
        # Increment counter based on timeout flag
        if feedback.timeout:
            self._num_correct += 1

        # Update block index if criterion reached
        if self._num_correct >= self._correct_trials_per_block:
            self._num_correct = 0
            self._block_index = (self._block_index + 1) % len(self._block_keys)

    def __next__(self) -> TrialSpec:
        """Generate the next trial from the current block.

        Randomly samples a pose and an object from the current block's
        category. All trials use both arms and have occlusion enabled.

        Returns:
            TrialSpec with randomly sampled parameters from current block.
        """
        # Sample object pose
        object_pose = np.random.choice(self._poses)  # type: ignore

        # Sample object id from current block
        block_key = self._block_keys[self._block_index]
        object_id = np.random.choice(self._object_ids[block_key])

        # Make trial spec (always uses both arms and occlusion)
        trial_spec = TrialSpec(
            object_id=object_id,
            object_pose=object_pose,
            arm="both",
            occlude=True,
        )

        return trial_spec
