"""Block-structured cup/drawer trial generator."""

from collections.abc import Mapping
from typing import Any

from tabletop_server.nodes import Commander

from tabletop_tasks.trial_generators.base import BaseTrialGenerator, TrialSpec


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
        occlude: list[bool],
        poses: list[Mapping[str, Any]],
        num_trials: int,
    ):
        super().__init__(commander)
        self._object_ids = object_ids
        self._poses = [
            self.commander.create_pose_stamped(**pose) for pose in poses
        ]
        self._occlude = occlude
        self._num_trials = num_trials
        self._trial_counter = 0

    def __next__(self) -> TrialSpec:
        """Return a trial."""
        if self._trial_counter >= self._num_trials:
            raise StopIteration

        # Sample object pose
        object_pose = self._poses[self._trial_counter % len(self._poses)]

        # Sample object id
        object_id = self._object_ids[
            self._trial_counter % len(self._object_ids)
        ]

        # Sample occlude
        occlude = self._occlude[self._trial_counter % len(self._occlude)]

        # Make trial spec
        trial_spec = TrialSpec(
            object_id=object_id,
            object_pose=object_pose,
            occlude=occlude,
        )

        self._trial_counter += 1

        return trial_spec

    def send(self, **unused_kwargs: dict) -> None:
        """Send feedback."""
        pass
