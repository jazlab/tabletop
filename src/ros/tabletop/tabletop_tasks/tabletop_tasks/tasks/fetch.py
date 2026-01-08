"""Fetch task for moving objects to specified poses.

This module provides a simple task that moves a held object to
a target pose. Unlike ForagingTask, FetchTask does not include
stimulus presentation, response collection, or reward phases.

This task is useful for:
- Object placement experiments
- Testing motion planning
- Simple pick-and-place demonstrations

Example:
    generator = RandomChoice(commander, ...)
    task = FetchTask(commander, generator)
    await task.run()
"""

from collections.abc import Mapping
from typing import Any

from geometry_msgs.msg import PoseStamped
from tabletop_rig.nodes import Commander

from tabletop_tasks.tasks.base import BaseTask
from tabletop_tasks.trial_generators.base import (
    BaseTrialGenerator,
    TrialFeedback,
    TrialSpec,
)


class FetchTask(BaseTask):
    """Task for moving objects to target poses.

    Executes simple motion-only trials where the robot moves a held
    object to a pose specified by the trial generator. Does not include
    any behavioral components like stimulus presentation or response
    collection.

    This task requires a trial generator (will raise ValueError if
    trial_spec is None).
    """

    def __init__(
        self,
        commander: Commander,
        trial_generator: BaseTrialGenerator | Mapping[str, Any],
    ):
        """Initialize the fetch task.

        Args:
            commander: Commander instance for robot interaction.
            trial_generator: Generator producing TrialSpec objects with
                target poses, or a config dict for dynamic instantiation.
        """
        super().__init__(commander, trial_generator, logger_name="fetch_task")

    async def prepare(self, pose: PoseStamped):
        """Move the held object to the target pose.

        Uses linear planning to move the end effector (with attached
        object) to the specified pose.

        Args:
            pose: Target pose for the object.
        """
        self.log("Prepare phase")
        await self.commander.plan_and_execute(
            goal=pose, planning_pipeline="linear"
        )

    async def run_trial(
        self, trial_spec: TrialSpec | None
    ) -> TrialFeedback | None:
        """Execute a single fetch trial.

        Moves the object to the pose specified in the trial spec.
        Does not return feedback as there is no behavioral response.

        Args:
            trial_spec: Specification containing the target object pose.

        Returns:
            None (no behavioral feedback for fetch trials).

        Raises:
            ValueError: If trial_spec is None.
        """
        if trial_spec is None:
            raise ValueError("trial_spec should not be None for fetch task")

        self.log(f"Fetch task trial spec: {trial_spec}")
        await self.prepare(trial_spec.object_pose)
