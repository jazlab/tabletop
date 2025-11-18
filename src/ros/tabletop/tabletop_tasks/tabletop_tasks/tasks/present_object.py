"""Present object task."""

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
    def __init__(
        self,
        commander: Commander,
        trial_generator: BaseTrialGenerator | Mapping[str, Any],
    ):
        super().__init__(
            commander, trial_generator, logger_name="present_task"
        )

    ############################################################
    # Phases
    ############################################################

    async def present(self, pose: PoseStamped):
        """Present object."""
        self.log("Present phase")
        await self.commander.plan_and_execute(
            goal=pose, planning_pipeline="linear"
        )

    async def run_trial(self, trial_spec: TrialSpec | None) -> TrialFeedback:
        """Run a trial."""
        if trial_spec is None:
            raise ValueError("trial_spec should not be None for foraging task")

        self.log(f"Foraging task trial spec: {trial_spec}")
        await self.present(trial_spec.object_pose)
        return TrialFeedback()
