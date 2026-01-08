"""Present object task for object presentation experiments.

This module provides a task that presents objects at specified poses
without behavioral response collection. Similar to FetchTask but
returns empty feedback to the trial generator.

This task is useful for:
- Object familiarization phases
- Passive viewing experiments
- Testing object presentation mechanics

Example:
    generator = OrderedChoice(commander, ...)
    task = PresentObjectTask(commander, generator)
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


class PresentObjectTask(BaseTask):
    """Task for presenting objects at specified poses.

    Moves objects to target poses as specified by the trial generator.
    Unlike ForagingTask, does not include stimulus/delay/response phases
    or reward delivery. Returns empty TrialFeedback to allow trial
    generators to track trial completion.

    This task requires a trial generator (will raise ValueError if
    trial_spec is None).
    """

    def __init__(
        self,
        commander: Commander,
        trial_generator: BaseTrialGenerator | Mapping[str, Any],
    ):
        """Initialize the present object task.

        Args:
            commander: Commander instance for robot interaction.
            trial_generator: Generator producing TrialSpec objects with
                target poses, or a config dict for dynamic instantiation.
        """
        super().__init__(
            commander, trial_generator, logger_name="present_task"
        )

    ############################################################
    # Phases
    ############################################################

    async def present(self, pose: PoseStamped):
        """Move the object to the presentation pose.

        Uses linear planning to move the end effector (with attached
        object) to the specified presentation pose.

        Args:
            pose: Target pose for object presentation.
        """
        self.log("Present phase")
        await self.commander.plan_and_execute(
            goal=pose, planning_pipeline="linear"
        )

    async def run_trial(self, trial_spec: TrialSpec | None) -> TrialFeedback:
        """Execute a single presentation trial.

        Moves the object to the specified pose and returns empty
        feedback. No behavioral response is collected.

        Args:
            trial_spec: Specification containing the target object pose.

        Returns:
            Empty TrialFeedback (no behavioral measures collected).

        Raises:
            ValueError: If trial_spec is None.
        """
        if trial_spec is None:
            raise ValueError(
                "trial_spec should not be None for present object task"
            )

        self.log(f"Present object task trial spec: {trial_spec}")
        await self.present(trial_spec.object_pose)
        return TrialFeedback()
