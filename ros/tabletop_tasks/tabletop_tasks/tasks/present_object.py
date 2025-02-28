"""Foraging task."""

import asyncio
import enum
import importlib

from tabletop_server.nodes import Commander

from tabletop_tasks.tasks.base_task import BaseTask
from tabletop_tasks.trial_generators.base_trial_generator import (
    BaseTrialGenerator,
)


class PresentObjectState(enum.Enum):
    """PresentObject state."""

    IDLE = 0
    FETCH = 1
    STIMULUS = 2
    RETURN = 3
    DELAY = 4
    FINISHED = 5


class PresentObjectTask(BaseTask):
    def __init__(
        self,
        commander: Commander,
        trial_generator: BaseTrialGenerator | dict,
        delay_duration_s: float = 0.5,
        stimulus_duration_s: float = 0.5,
    ):
        super().__init__(commander)

        # Logging
        self.log(
            "RetrieveObjectTask(\n"
            f"  trial_generator={trial_generator},\n"
            f"  stimulus_duration_s={stimulus_duration_s},\n"
            f"  delay_duration_s={delay_duration_s},\n"
            ")"
        )

        # Create trial_generator if necessary
        if isinstance(trial_generator, dict):
            trial_generator_module_name = trial_generator["module"]
            trial_generator_module = f"tabletop_tasks.trial_generators.{trial_generator_module_name}"
            trial_generator_class = trial_generator["class"]
            trial_generator_kwargs = trial_generator["kwargs"]
            trial_generator_tmp: BaseTrialGenerator = getattr(
                importlib.import_module(trial_generator_module),
                trial_generator_class,
            )(**trial_generator_kwargs)
        else:
            trial_generator_tmp = trial_generator

        self._trial_generator = trial_generator_tmp
        self._stimulus_duration_s = stimulus_duration_s
        self._delay_duration_s = delay_duration_s
        self._state = PresentObjectState.IDLE

    async def _fetch(self):
        """Fetch object for trial."""
        self.log("Fetching new trial spec")

        # Sample new trial
        self._trial_spec = next(self._trial_generator)

        # Fetch object
        object_id = self._trial_spec.object_id
        object_pose = self._trial_spec.object_pose
        self._return_pose = await self.commander.fetch_object_async(
            object_id, object_pose
        )

        self._state = PresentObjectState.STIMULUS

    async def _stimulus(self):
        """Present stimulus."""
        self.log("Presenting stimulus")

        await asyncio.sleep(self._stimulus_duration_s)

        self._state = PresentObjectState.RETURN

    async def _delay(self):
        """Delay phase."""
        self.log("Delay phase")

        await asyncio.sleep(self._delay_duration_s)

        self._state = PresentObjectState.RETURN

    async def _return(self):
        """Return object."""
        self.log("Returning object")

        # Give feedback to trial generator
        self.log("Giving feedback to trial generator")

        # Return object
        await self.commander.return_object_async(
            self._trial_spec.object_id, self._return_pose
        )

        # Transition to fetch state
        self._state = PresentObjectState.IDLE

    async def run(self):
        """Run a trial."""
        while True:
            async with self.commander.planning_context_manager_async():
                match self._state:
                    case PresentObjectState.IDLE:
                        self._state = PresentObjectState.FETCH
                    case PresentObjectState.FETCH:
                        await self._fetch()
                    case PresentObjectState.STIMULUS:
                        await self._stimulus()
                    case PresentObjectState.RETURN:
                        await self._return()
                    case PresentObjectState.DELAY:
                        await self._delay()
                    case PresentObjectState.FINISHED:
                        self.log("RetrieveObjectTask finished")
                        return
                    case _:
                        raise ValueError(f"Invalid state: {self._state}")
