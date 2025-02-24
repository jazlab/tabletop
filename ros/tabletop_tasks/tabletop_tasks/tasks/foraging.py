"""Foraging task."""

import asyncio
import enum
import importlib
import time
from typing import Any

from tabletop_server.nodes import Commander

from tabletop_tasks.tasks.base_task import BaseTask
from tabletop_tasks.trial_generators.base_trial_generator import (
    BaseTrialGenerator,
)


class ForagingState(enum.Enum):
    """Foraging state."""

    IDLE = 0
    FETCH = 1
    FIXATION = 2
    STIMULUS = 3
    DELAY = 4
    RESPONSE = 5
    REVEAL = 6
    RETURN = 7
    FINISHED = 8


class ForagingTask(BaseTask):
    def __init__(
        self,
        commander: Commander,
        trial_generator: BaseTrialGenerator | dict,
        fixation_duration_s: float = 0.5,
        stimulus_duration_s: float = 0.5,
        delay_duration_s: float = 0.5,
        reward_duration_s: float = 0.1,
        reveal_duration_s: float = 0.5,
        response_timeout_s: float = 10.0,
        fixation_timeout_s: float = 100.0,
    ):
        super().__init__(commander)

        # Logging
        self.log(
            "ForagingTask(\n"
            f"  trial_generator={trial_generator},\n"
            f"  fixation_duration_s={fixation_duration_s},\n"
            f"  stimulus_duration_s={stimulus_duration_s},\n"
            f"  delay_duration_s={delay_duration_s},\n"
            f"  response_timeout_s={response_timeout_s},\n"
            f"  fixation_timeout_s={fixation_timeout_s},\n"
            f"  reward_duration_s={reward_duration_s},\n"
            f"  reveal_duration_s={reveal_duration_s},\n"
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
        self._fixation_duration_s = fixation_duration_s
        self._stimulus_duration_s = stimulus_duration_s
        self._delay_duration_s = delay_duration_s
        self._response_timeout_s = response_timeout_s
        self._fixation_timeout_s = fixation_timeout_s
        self._reward_duration_s = reward_duration_s
        self._reveal_duration_s = reveal_duration_s
        self._state = ForagingState.IDLE

    def broke_fixation(self):
        self._trial_feedback["broke_fixation"] = True
        self.log("Hand fixation broken")
        self._state = ForagingState.RETURN

    async def _fetch(self):
        """Fetch object for trial."""
        self.log("Fetching new trial spec")

        # Sample new trial
        self._trial_spec = next(self._trial_generator)

        self._trial_feedback: dict[str, Any] = dict(
            broke_fixation=False,
            reaction_time=None,
            timeout=None,
        )

        # Make smartglass opaque
        await self.commander.smartglass_occlude_async()

        # Fetch object
        object_id = self._trial_spec.object_id
        object_pose = self._trial_spec.object_pose
        await self.commander.fetch_object_async(object_id, object_pose)

        # Transition to fixation state
        self._state = ForagingState.FIXATION

    async def _fixation(self):
        """Wait for hand fixation."""
        self.log("Fixation phase")

        # Wait for hand fixation onset
        await self.commander.wait_for_hand_fixation_on_async(
            self._fixation_timeout_s
        )

        # Wait for hand fixation duration
        if self.commander.wait_for_hand_fixation_off_async(
            self._fixation_duration_s
        ):
            self.log("Hand fixation acquired")
            self._state = ForagingState.STIMULUS
        else:
            self._state = ForagingState.FIXATION

        return

    async def _stimulus(self):
        """Present stimulus."""
        self.log("Presenting stimulus")

        # Reveal stimulus
        await self.commander.smartglass_reveal_async()

        # Wait for stimulus duration, terminating if hand fixation is broken
        if self.commander.wait_for_hand_fixation_off_async(
            self._stimulus_duration_s
        ):
            self._state = ForagingState.DELAY
        else:
            self.broke_fixation()

        return

    async def _delay(self):
        """Delay phase."""
        self.log("Delay phase")

        # Occlude smartglass if necessary
        if self._trial_spec.occlude:
            await self.commander.smartglass_occlude_async()

        # Wait for delay duration, terminating if hand fixation is broken
        if self.commander.wait_for_hand_fixation_off_async(
            self._delay_duration_s
        ):
            self._state = ForagingState.RESPONSE
        else:
            self.broke_fixation()

        return

    async def _response(self):
        """Response phase."""
        self.log("Response phase")

        # Open arm door
        await self.commander.arm_door_open_async()

        # Wait for response
        response_start_time = time.time()
        try:
            async with asyncio.timeout(self._response_timeout_s):
                await self.commander.wait_for_flic_button_async()

            # Response received
            reaction_time = time.time() - response_start_time
            self._trial_feedback["reaction_time"] = reaction_time
            self.log(f"Reaction time: {reaction_time}")
            await self.commander.reward_async(self._reward_duration_s)
            self._state = ForagingState.REVEAL
        except asyncio.TimeoutError:
            # Response not received
            self.log("Response timeout")
            self._trial_feedback["timeout"] = True
            self._state = ForagingState.RETURN

    async def _reveal(self):
        """Reveal object."""
        self.log("Reveal phase")

        # Reveal object
        await self.commander.smartglass_reveal_async()

        # Wait for reveal duration
        await asyncio.sleep(self._reveal_duration_s)

        # Transition to return state
        self._state = ForagingState.RETURN

    async def _return(self):
        """Return object."""
        self.log("Returning object")

        # Give feedback to trial generator
        self.log("Giving feedback to trial generator")
        self._trial_generator.send(**self._trial_feedback)

        # Close arm door
        arm_door_future = self.commander.arm_door_close_async()

        # Occlude smartglass
        smartglass_future = self.commander.smartglass_occlude_async()

        # Return object
        return_future = self.commander.return_object_async(
            self._trial_spec.object_id
        )

        await asyncio.gather(arm_door_future, smartglass_future, return_future)

        # Transition to fetch state
        self._state = ForagingState.FETCH

    async def run(self):
        """Run a trial."""
        while True:
            match self._state:
                case ForagingState.IDLE:
                    self._state = ForagingState.FETCH
                case ForagingState.FETCH:
                    await self._fetch()
                case ForagingState.FIXATION:
                    await self._fixation()
                case ForagingState.STIMULUS:
                    await self._stimulus()
                case ForagingState.DELAY:
                    await self._delay()
                case ForagingState.RESPONSE:
                    await self._response()
                case ForagingState.REVEAL:
                    await self._reveal()
                case ForagingState.RETURN:
                    await self._return()
                case ForagingState.FINISHED:
                    self.log("Foraging task finished")
                    return
                case _:
                    raise ValueError(f"Invalid state: {self._state}")
