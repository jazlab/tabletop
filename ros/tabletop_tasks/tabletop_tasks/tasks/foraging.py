"""Foraging task."""

import asyncio
import enum
import importlib
import time
from typing import Any, Optional

from tabletop_server.nodes import Commander

from tabletop_tasks.tasks.base_task import BaseTask
from tabletop_tasks.trial_generators.base_trial_generator import (
    BaseTrialGenerator,
)


class ForagingState(enum.Enum):
    """Foraging state."""

    IDLE = 0
    FETCH = 1
    STIMULUS = 2
    DELAY = 3
    RESPONSE = 4
    REVEAL = 5
    RETURN = 6
    FINISHED = 7


class ForagingTask(BaseTask):
    def __init__(
        self,
        commander: Commander,
        trial_generator: BaseTrialGenerator | dict,
        fixation_duration_ms: float = 500,
        stimulus_duration_ms: float = 500,
        delay_duration_ms: float = 500,
        response_timeout_s: float = 10.0,
        reward_duration_ms: float = 100,
        reveal_duration_ms: float = 500,
        logger: Any = None,
    ):
        super().__init__(commander)
        
        # Logging
        self.log(
            "ForagingTask(\n"
            f"  trial_generator={trial_generator},\n"
            f"  fixation_duration_ms={fixation_duration_ms},\n"
            f"  stimulus_duration_ms={stimulus_duration_ms},\n"
            f"  delay_duration_ms={delay_duration_ms},\n"
            f"  response_timeout_s={response_timeout_s},\n"
            f"  reward_duration_ms={reward_duration_ms},\n"
            f"  reveal_duration_ms={reveal_duration_ms},\n"
            ")"
        )
        
        # Create trial_generator if necessary
        if isinstance(trial_generator, dict):
            trial_generator_module_name = trial_generator["module"]
            trial_generator_module = f"tabletop_tasks.trial_generators.{trial_generator_module_name}"
            trial_generator_class = trial_generator["class"]
            trial_generator_kwargs = trial_generator["kwargs"]
            trial_generator = getattr(importlib.import_module(trial_generator_module), trial_generator_class)(
                **trial_generator_kwargs)
        
        # print("\n\n\n\n\n\n\n\n\n\n")
        # raise Exception("test")
        
        self._trial_generator = trial_generator
        print("trial_generator: ", trial_generator)
        self._fixation_duration_s = fixation_duration_ms / 1000
        self._stimulus_duration_s = stimulus_duration_ms / 1000
        self._delay_duration_s = delay_duration_ms / 1000
        self._response_timeout_s = response_timeout_s
        self._reward_duration_ms = reward_duration_ms
        self._reveal_duration_s = reveal_duration_ms / 1000
        self._state = ForagingState.IDLE
        
    async def maintain_hand_fixation(self, duration: float) -> None:
        t_start = time.time()
        while time.time() - t_start < duration:
            hand_fixation_duration = self.commander.hand_fixation_duration()
            if hand_fixation_duration < time.time() - t_start:
                return False
            await asyncio.sleep(0.01)
        return True

    async def _fetch(self):
        """Fetch object for trial."""
        self.log("Fetching new trial spec")
        
        # Sample new trial
        self._trial_spec = next(self._trial_generator)
        
        self._trial_feedback = dict(
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

        # Wait for hand fixation
        self.log("Waiting for hand fixation")
        hand_fixated = False
        while not hand_fixated:
            hand_fixation_duration = self.commander.hand_fixation_duration()
            if hand_fixation_duration > self._fixation_duration_s:
                hand_fixated = True
            else:
                await asyncio.sleep(0.01)
        self.log("Hand fixation acquired")
        
        # Transition to stimulus state
        self._state = ForagingState.STIMULUS

    async def _stimulus(self):
        """Present stimulus."""
        self.log("Presenting stimulus")
        
        # Reveal stimulus
        await self.commander.smartglass_reveal_async()

        # Wait for stimulus duration, terminating early if hand fixation is
        # broken
        maintained_hand_fixation = await self.maintain_hand_fixation(
            self._stimulus_duration_s
        )
        if maintained_hand_fixation:
            self._state = ForagingState.DELAY
        else:
            self._trial_feedback["broke_fixation"] = True
            self.log("Hand fixation broken")
            self._state = ForagingState.RETURN

    async def _delay(self):
        """Delay phase."""
        self.log("Delay phase")
        
        # Occlude smartglass if necessary
        if self._trial_spec.occlude:
            await self.commander.smartglass_occlude_async()

        # Wait for delay duration, terminating early if hand fixation is broken
        maintained_hand_fixation = await self.maintain_hand_fixation(
            self._delay_duration_s
        )
        if maintained_hand_fixation:
            self._state = ForagingState.RESPONSE
        else:
            self._trial_feedback["broke_fixation"] = True
            self.log("Hand fixation broken")
            self._state = ForagingState.RETURN

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
            reaction_time = time.time() - response_start_time
            self._trial_feedback["reaction_time"] = reaction_time
            self.log(f"Reaction time: {reaction_time}")
            await self.commander.reward_async(self._reward_duration_ms)
            self._state = ForagingState.REVEAL
        except asyncio.TimeoutError:
            self.log("Response timeout")
            self._trial_feedback["timeout"] = True
            self._state = ForagingState.RETURN

    async def _reveal(self):
        """Reveal object."""
        self.log("Reveal phase")
        
        # Reveal object
        self.commander.smartglass_reveal()

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
                    commander.log("Foraging task finished")
                    return
                case _:
                    raise ValueError(f"Invalid state: {self._state}")
