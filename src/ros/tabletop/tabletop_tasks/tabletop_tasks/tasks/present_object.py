"""Present object task."""

import asyncio
import enum
import importlib
from collections.abc import Mapping
from typing import Any

from tabletop_server.nodes import Commander

from tabletop_tasks.tasks.base import BaseTask
from tabletop_tasks.trial_generators import BaseTrialGenerator


class PresentObjectState(enum.Enum):
    """PresentObject state."""

    IDLE = 0
    NEXT_TRIAL_SPEC = 1
    FETCH = 2
    STIMULUS = 3
    RETURN = 4
    DELAY = 5
    FINISHED = 6


class PresentObjectTask(BaseTask):
    def __init__(
        self,
        commander: Commander,
        trial_generator: BaseTrialGenerator | Mapping[str, Any],
        stimulus_duration_sec: float = 0.5,
        delay_duration_sec: float = 0.5,
    ):
        super().__init__(commander)

        # Logging
        self.log(
            "PresentObjectTask(\n"
            f"  trial_generator={trial_generator},\n"
            f"  stimulus_duration_sec={stimulus_duration_sec},\n"
            f"  delay_duration_sec={delay_duration_sec},\n"
            ")"
        )

        # Create trial_generator if necessary
        if isinstance(trial_generator, Mapping):
            trial_generator_tmp: BaseTrialGenerator = getattr(
                importlib.import_module("tabletop_tasks.trial_generators"),
                trial_generator["class"],
            )(commander, **trial_generator["kwargs"])
        else:
            trial_generator_tmp = trial_generator

        self._trial_generator = trial_generator_tmp
        self._stimulus_duration_sec = stimulus_duration_sec
        self._delay_duration_sec = delay_duration_sec
        self._state = PresentObjectState.IDLE

    def _next_trial_spec(self):
        """Get next trial spec."""
        try:
            self._trial_spec = next(self._trial_generator)
        except StopIteration:
            self.log("Trial generator finished")
            self._state = PresentObjectState.FINISHED
        else:
            self._state = PresentObjectState.FETCH

    async def _fetch(self):
        """Fetch object for trial."""
        self.log("Fetching object")

        # Fetch object
        object_id = self._trial_spec.object_id
        object_pose = self._trial_spec.object_pose
        await self.commander.fetch_and_present_object(object_id, object_pose)

        self._state = PresentObjectState.STIMULUS

    async def _stimulus(self):
        """Present stimulus."""
        self.log("Presenting stimulus")

        await asyncio.sleep(self._stimulus_duration_sec)

        self._state = PresentObjectState.DELAY

    async def _delay(self):
        """Delay phase."""
        self.log("Delay phase")

        await asyncio.sleep(self._delay_duration_sec)

        self._state = PresentObjectState.RETURN

    async def _return(self):
        """Return object."""
        self.log("Returning object")

        # Return object
        await self.commander.unpresent_and_return_object()

        # Transition to fetch state
        self._state = PresentObjectState.NEXT_TRIAL_SPEC

    async def run(self):
        """Run a trial."""
        while True:
            async with self.commander.context_manager():
                try:
                    while True:
                        match self._state:
                            case PresentObjectState.IDLE:
                                self._state = (
                                    PresentObjectState.NEXT_TRIAL_SPEC
                                )
                            case PresentObjectState.NEXT_TRIAL_SPEC:
                                self._next_trial_spec()
                            case PresentObjectState.FETCH:
                                await self._fetch()
                            case PresentObjectState.STIMULUS:
                                await self._stimulus()
                            case PresentObjectState.DELAY:
                                await self._delay()
                            case PresentObjectState.RETURN:
                                await self._return()
                            case PresentObjectState.FINISHED:
                                self.log("PresentObjectTask finished")
                                return
                            case _:
                                raise ValueError(
                                    f"Invalid state: {self._state}"
                                )
                finally:
                    # If an error occurs, reset to the fetch state so the next
                    # attempt will continue for the same trial
                    self._state = PresentObjectState.FETCH
