"""Base task classes for behavioral experiments.

This module provides the abstract base class for all experimental tasks
in the TableTop system. Tasks orchestrate the sequence of robot actions,
subject interactions, and trial logic for behavioral experiments.

The task architecture follows a standard trial-based structure:
1. Prepare trial (fetch and position object)
2. Run trial (present stimulus, collect response)
3. Reset trial (return object to home position)

Tasks work with trial generators to produce sequences of trials,
and can be configured via YAML configuration files.

Classes:
    NullTrialGenerator: Placeholder generator that yields a single None trial.
    BaseTask: Abstract base class for all experimental tasks.

Example:
    class MyTask(BaseTask):
        async def run_trial(self, trial_spec):
            await self.commander.reveal_smartglass()
            response = await self.commander.flic_response_time()
            return TrialFeedback(reaction_time=response)
"""

import asyncio
from abc import ABCMeta, abstractmethod
from collections.abc import Mapping
from typing import Any, Optional

import rclpy.logging
from rclpy.impl.rcutils_logger import RcutilsLogger
from rpyutils.import_c_library import importlib
from tabletop_rig.nodes import Commander
from tabletop_rig.utils.logging import LoggerMixin

from tabletop_tasks.trial_generators.base import (
    BaseTrialGenerator,
    TrialFeedback,
    TrialSpec,
)

#: Default MIDI note configuration for sound feedback.
#: Used by tasks that play sounds during reward delivery.
DEFAULT_NOTE = {
    "name": "C",
    "octave": 4,
    "velocity": 127,
    "channel": 0,
}


class NullTrialGenerator:
    """Placeholder trial generator that yields a single None trial.

    Used by tasks that don't require a trial generator but still
    need to conform to the task execution interface.
    """

    def __init__(self):
        """Initialize the null generator."""
        self.returned = False

    def __next__(self) -> None:
        """Return None once, then stop iteration.

        Returns:
            None on first call.

        Raises:
            StopIteration: On subsequent calls.
        """
        if self.returned:
            raise StopIteration

        self.returned = True
        return None

    def send(self, *args, **kwargs):
        """Accept and ignore trial feedback.

        Args:
            *args: Ignored positional arguments.
            **kwargs: Ignored keyword arguments.
        """
        pass


class BaseTask(LoggerMixin, metaclass=ABCMeta):
    """Abstract base class for all TableTop experimental tasks.

    Tasks define the logic for running behavioral experiments, including
    stimulus presentation, response collection, and reward delivery.
    Subclasses must implement run_trial() to define trial-specific logic.

    The default run() implementation provides a standard trial loop that:
    1. Locks arms and occludes smartglass for safety
    2. Prepares each trial (fetches and positions object)
    3. Runs the trial and collects feedback
    4. Resets the trial (returns object)

    Subclasses can override run() for custom experiment structures.

    Attributes:
        commander: Reference to the Commander node for robot control.
        _trial_generator: Generator producing TrialSpec objects.
        _logger: ROS logger for this task.
    """

    def __init__(
        self,
        commander: Commander,
        trial_generator: BaseTrialGenerator | Mapping[str, Any] | None = None,
        logger_name: Optional[str] = None,
    ):
        """Initialize the base task.

        Args:
            commander: Commander instance for robot and peripheral control.
            trial_generator: Either a BaseTrialGenerator instance, or a dict
                with "class" and "kwargs" keys for dynamic instantiation.
                If None, tasks must handle trial generation themselves.
            logger_name: Optional name for the ROS logger (currently unused).

        Raises:
            ValueError: If trial_generator dict specifies a class that isn't
                a BaseTrialGenerator subclass.
        """
        self._commander = commander

        self._logger = rclpy.logging.get_logger("tabletop_task")

        # Create trial_generator from config dict if necessary
        if isinstance(trial_generator, Mapping):
            self._trial_generator = getattr(
                importlib.import_module("tabletop_tasks.trial_generators"),
                trial_generator["class"],
            )(commander, **trial_generator["kwargs"])
            if not isinstance(self._trial_generator, BaseTrialGenerator):
                raise ValueError(
                    "trial_generator class must be an instance of BaseTrialGenerator"
                )
        elif trial_generator is not None:
            self._trial_generator = trial_generator

    def get_logger(self) -> RcutilsLogger:
        """Get the task logger instance.

        Returns:
            ROS logger for this task.
        """
        return self._logger

    @property
    def commander(self) -> Commander:
        """Get the commander instance.

        Returns:
            The Commander node reference for robot control.
        """
        return self._commander

    async def _occlude_and_lock(self):
        """Occlude smartglass and lock arms concurrently.

        Ensures safety before robot motion by blocking the subject's
        view and constraining their arms. Waits for both operations
        to complete.
        """
        arm_lock_task = asyncio.create_task(
            self.commander.lock_arms_and_wait()
        )
        smartglass_task = asyncio.create_task(
            self.commander.occlude_smartglass()
        )

        await arm_lock_task
        await smartglass_task

    async def _prepare_trial(self, trial_spec: TrialSpec):
        """Prepare the object for a trial.

        Fetches the specified object and moves it to the pre-presentation
        position.

        Args:
            trial_spec: Trial specification containing the object ID.
        """
        await self.commander.fetch_object(trial_spec.object_id)
        await self.commander.pre_present_object()

    async def _reset_trial(self):
        """Reset the object after a trial.

        Resets the object to its home configuration and returns it
        to its storage position.
        """
        await self.commander.reset_object()
        await self.commander.return_object()

    @abstractmethod
    async def run_trial(
        self, trial_spec: TrialSpec | None
    ) -> TrialFeedback | None:
        """Run a single trial.

        Subclasses must implement this method to define the trial-specific
        logic including stimulus presentation, response collection, and
        reward delivery.

        Args:
            trial_spec: Specification for the current trial, or None if
                no trial generator was provided.

        Returns:
            TrialFeedback with behavioral measures from the trial,
            or None if no feedback should be sent to the generator.
        """

    async def run(self) -> None:
        """Run the complete task.

        Executes the standard trial loop: for each trial from the
        generator, prepares the trial, runs it, collects feedback,
        and resets before the next trial.

        The loop runs indefinitely, restarting the trial generator
        when exhausted. Override this method for custom task structures.
        """
        while True:
            async with self.commander:
                await self._occlude_and_lock()
                for trial_spec in self._trial_generator:
                    # Prepare object for trial
                    await self._prepare_trial(trial_spec)

                    # Run trial and send feedback to trial generator
                    feedback = await self.run_trial(trial_spec)
                    if feedback is not None:
                        self._trial_generator.send(feedback)

                    # Occlude smartglass and lock arms before moving
                    await self._occlude_and_lock()

                    # Reset object before next trial
                    await self._reset_trial()
