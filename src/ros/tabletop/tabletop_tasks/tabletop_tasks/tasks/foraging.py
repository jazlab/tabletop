"""Foraging task for behavioral experiments with delayed response.

This module provides a complete behavioral task implementing the
delayed match-to-sample paradigm commonly used in primate research.
Each trial consists of distinct phases:

1. Prepare: Move object to presentation pose (smartglass occluded)
2. Stimulus: Reveal object for brief viewing period
3. Delay: Optionally occlude view during delay period
4. Response: Release arms and wait for subject response
5. Reveal: Show object again after correct response (reward phase)

The task collects reaction time data and delivers rewards (juice + optional
sound) for correct responses.

Example:
    generator = OrderedChoiceAlternating(commander, ...)
    task = ForagingTask(
        commander=commander,
        trial_generator=generator,
        stimulus_duration=1.0,
        delay_duration=2.0,
        reward_duration=0.5,
        reward_sound=True,
        reveal_duration=1.0,
        response_timeout=10.0,
    )
    await task.run()
"""

import asyncio
from collections.abc import Mapping
from typing import Any, Literal, Optional

from tabletop_rig.nodes import Commander
from tabletop_rig.nodes.commander import ManipulationContextManager

from tabletop_tasks.tasks.base import BaseObjectInteractionTask
from tabletop_tasks.trial_generators.base import (
    BaseTrialGenerator,
    TrialFeedback,
    TrialSpec,
)


class ForagingTask(BaseObjectInteractionTask):
    """Full behavioral task with stimulus, delay, and response phases.

    Implements a delayed match-to-sample paradigm with configurable
    timing parameters. Trials follow a fixed phase sequence with
    optional occlusion during the delay period.

    This task requires a trial generator and uses the standard
    trial loop from BaseTask.run().

    Attributes:
        stimulus_duration: Duration to show stimulus (seconds).
        delay_duration: Duration of delay period (seconds).
        response_timeout: Maximum time to wait for response (seconds).
        reward_duration: Duration of reward delivery (seconds).
        reward_sound: Whether to play sound with reward.
        reveal_duration: Duration to show object after response (seconds).
        sound_kwargs: Keyword arguments for play_sound().
    """

    def __init__(
        self,
        commander: Commander,
        trial_generator: BaseTrialGenerator | Mapping[str, Any],
        stimulus_duration: float,
        delay_duration: float,
        reward_duration: float,
        reward_sound: bool,
        reveal_duration: float,
        response_timeout: float,
        sound_kwargs: Optional[Mapping[str, Any]] = None,
    ):
        """Initialize the foraging task.

        Args:
            commander: Commander instance for robot interaction.
            trial_generator: Generator producing TrialSpec objects,
                or a config dict for dynamic instantiation.
            stimulus_duration: Time to display stimulus (seconds).
            delay_duration: Duration of delay period (seconds).
            reward_duration: Duration of reward delivery (seconds).
            reward_sound: Whether to play sound during reward.
            reveal_duration: Time to show object after response (seconds).
            response_timeout: Maximum wait time for response (seconds).
            sound_kwargs: Optional keyword arguments for play_sound()
                (default empty dict).
        """
        super().__init__("foraging_task", commander, trial_generator)

        self.stimulus_duration = stimulus_duration
        self.delay_duration = delay_duration
        self.response_timeout = response_timeout
        self.reward_duration = reward_duration
        self.reward_sound = reward_sound
        self.reveal_duration = reveal_duration
        self.sound_kwargs = sound_kwargs if sound_kwargs is not None else {}

    ############################################################
    # Trial Phases
    ############################################################

    async def stimulus(self):
        """Reveal the stimulus to the subject.

        Reveals the smartglass to make the object visible, then waits
        for the configured stimulus duration. This is the encoding
        phase where the subject views the object.
        """
        self.log("Stimulus phase")
        await self.commander.reveal_smartglass()
        await asyncio.sleep(self.stimulus_duration)

    async def delay(self, occlude: bool):
        """Wait during the delay period with optional occlusion.

        Implements the delay period between stimulus presentation and
        response. If occlusion is enabled, the smartglass is occluded
        to prevent the subject from viewing the object.

        Args:
            occlude: Whether to occlude the smartglass during delay.
        """
        self.log("Delay phase")
        if occlude:
            await self.commander.occlude_smartglass()
        await asyncio.sleep(self.delay_duration)

    async def response(
        self, object_id: str, arm: Literal["left", "right", "both"]
    ) -> TrialFeedback:
        """Collect behavioral response and deliver reward.

        Releases the specified arm(s) and waits for a button press.
        If a response is received within the timeout, delivers juice
        reward and optional sound feedback.

        Reaction time is measured using the ROS2 clock for consistency
        with other ROS2 timestamps.

        Args:
            arm: Which arm(s) to release for response ("left", "right",
                or "both").

        Returns:
            TrialFeedback containing reaction_time and timeout status.
            On timeout, reaction_time is None and timeout is True.
        """
        self.log("Response phase")
        await self.commander.release_arm(arm)

        start_time = self.commander.ros_time()

        # Wait for response from monkey button press
        if response_time := await self.commander.flic_response_time(
            object_id, timeout=self.response_timeout
        ):
            # Calculate reaction time from response time
            reaction_time = response_time - start_time

            # Reward monkey and play sound
            async with asyncio.TaskGroup() as tg:
                if self.reward_sound:
                    tg.create_task(
                        self.commander.play_sound(**self.sound_kwargs)
                    )
                tg.create_task(
                    self.commander.start_reward_and_wait(self.reward_duration)
                )
                self.log(f"Reaction time: {reaction_time}")

            return TrialFeedback(reaction_time=reaction_time, timeout=False)
        else:
            # No response within timeout
            self.log("Response timeout")
            return TrialFeedback(reaction_time=None, timeout=True)

    async def reveal(self):
        """Reveal the object after a correct response.

        Shows the object again after reward delivery, allowing the
        subject to see the result of their choice. Only called on
        non-timeout trials.
        """
        self.log("Reveal phase")
        await self.commander.reveal_smartglass()
        await asyncio.sleep(self.reveal_duration)

    async def run_trial(
        self, trial_spec: TrialSpec, manipulator: ManipulationContextManager
    ) -> TrialFeedback:
        """Execute a single foraging trial.

        Runs through all trial phases in sequence:
        1. Prepare: Position object at target pose
        2. Stimulus: Show object for encoding
        3. Delay: Wait with optional occlusion
        4. Response: Collect response and deliver reward
        5. Reveal: Show object again (only if response received)

        Args:
            trial_spec: Specification for this trial containing object
                pose, arm assignment, and occlusion setting.

        Returns:
            TrialFeedback with reaction time and timeout status.

        Raises:
            ValueError: If trial_spec is None.
        """
        self.log(f"Foraging task trial spec: {trial_spec}")

        # Execute trial phases in sequence
        await self.stimulus()
        await self.delay(trial_spec.occlude)
        feedback = await self.response(trial_spec.object_id, trial_spec.arm)

        # Only reveal on successful (non-timeout) trials
        if not feedback.timeout and self.reveal_duration > 0:
            await self.reveal()

        return feedback
