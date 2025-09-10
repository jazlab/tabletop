"""Block-structured cup/drawer trial generator."""

import itertools
from collections.abc import Mapping
from typing import Any, Literal

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
        arms: list[Literal["left", "right", "both"]],
        occlude: list[bool],
        num_trials: int,
    ):
        super().__init__(commander)

        self._object_ids = object_ids
        self._poses = [
            self.commander.create_pose_stamped(**pose) for pose in poses
        ]
        self._arms = arms
        self._occlude = occlude

        if num_trials < 1:
            raise ValueError("num_trials must be at least 1")
        self._num_trials = num_trials
        self._trial_counter = 0

        self._parameter_grid = list(
            itertools.product(
                self._occlude, self._poses, self._arms, self._object_ids
            )
        )

    def __next__(self) -> TrialSpec:
        """Return a trial."""
        if self._trial_counter >= self._num_trials:
            raise StopIteration

        occlude, pose, arm, object_id = self._parameter_grid[
            self._trial_counter % len(self._parameter_grid)
        ]
        trial_spec = TrialSpec(
            object_id=object_id,
            object_pose=pose,
            arm=arm,
            occlude=occlude,
        )

        self._trial_counter += 1

        return trial_spec

    def send(self, feedback: TrialFeedback):
        """Send feedback."""
        pass
