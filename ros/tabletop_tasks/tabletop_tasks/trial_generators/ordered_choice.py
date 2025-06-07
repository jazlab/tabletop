"""Block-structured cup/drawer trial generator."""

from collections.abc import Mapping
from itertools import product
from typing import Any

from tabletop_server.nodes import Commander

from tabletop_tasks.trial_generators.base import (
    BaseTrialGenerator,
    TrialFeedback,
    TrialSpec,
)


class OrderedChoice(BaseTrialGenerator):
    """Ordered choice trial generator.

    Chooses an object from a list of objects and a pose from a list of poses in order.

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
        occlude: bool | str,
        num_trials: int,
    ):
        super().__init__(commander)

        self._object_ids = object_ids
        self._poses = [
            self.commander.create_pose_stamped(**pose) for pose in poses
        ]

        if isinstance(occlude, str):
            if occlude != "both":
                raise ValueError("occlude must be a boolean or 'both'")
            self._occlude = [False, True]
        else:
            self._occlude = [occlude]

        if num_trials < 1:
            raise ValueError("num_trials must be at least 1")
        self._num_trials = num_trials
        self._trial_counter = 0

        self._parameter_grid = list(
            product(self._occlude, self._poses, self._object_ids)
        )

    def __next__(self) -> TrialSpec:
        """Return a trial."""
        if self._trial_counter >= self._num_trials:
            raise StopIteration

        # Sample object pose
        occlude, pose, object_id = self._parameter_grid[
            self._trial_counter % len(self._parameter_grid)
        ]

        # Make trial spec
        trial_spec = TrialSpec(
            object_id=object_id, object_pose=pose, occlude=occlude
        )

        return trial_spec

    def send(self, feedback: TrialFeedback) -> None:
        """Send feedback."""
        self._trial_counter += 1
