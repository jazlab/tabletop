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
from collections import deque
from collections.abc import Mapping
from typing import Any

from rclpy.impl.rcutils_logger import RcutilsLogger
from rpyutils.import_c_library import importlib
from tabletop_rig.nodes import Commander
from tabletop_rig.utils.logging import LoggerMixin

from tabletop_tasks.trial_generators.base import (
    BaseTrialGenerator,
    DefaultTrialGenerator,
    TrialFeedback,
    TrialSpec,
)


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
        name: str,
        commander: Commander,
        trial_generator: BaseTrialGenerator | Mapping[str, Any] | None = None,
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

        self._logger = commander.get_logger().get_child(name)

        # Create trial_generator from config dict if necessary
        if trial_generator is None:
            self._trial_generator = DefaultTrialGenerator(commander)
        elif isinstance(trial_generator, BaseTrialGenerator):
            self._trial_generator = trial_generator
        if isinstance(trial_generator, Mapping):
            self._trial_generator: BaseTrialGenerator = getattr(
                importlib.import_module("tabletop_tasks.trial_generators"),
                trial_generator["class"],
            )(commander, **trial_generator["kwargs"])
            if not isinstance(self._trial_generator, BaseTrialGenerator):
                raise ValueError(
                    "trial_generator class must be an instance of BaseTrialGenerator"
                )

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

    @abstractmethod
    async def run(self) -> None:
        """Run the complete task.

        Subclasses must implement this method, which is called in run.py and
        which defines the task runtime.

        Users must remember to use the Commander as an asynchronous context
        manager (`async with self.commander`) before calling any commander
        methods.
        """


class BaseObjectInteractionTask(BaseTask, metaclass=ABCMeta):
    """TODO"""

    def __init__(
        self,
        name: str,
        commander: Commander,
        trial_generator: BaseTrialGenerator | Mapping[str, Any] | None = None,
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

        self._logger = commander.get_logger().get_child(name)

        # Create trial_generator from config dict if necessary
        if trial_generator is None:
            self._trial_generator = DefaultTrialGenerator(commander)
        elif isinstance(trial_generator, BaseTrialGenerator):
            self._trial_generator = trial_generator
        if isinstance(trial_generator, Mapping):
            self._trial_generator: BaseTrialGenerator = getattr(
                importlib.import_module("tabletop_tasks.trial_generators"),
                trial_generator["class"],
            )(commander, **trial_generator["kwargs"])
            if not isinstance(self._trial_generator, BaseTrialGenerator):
                raise ValueError(
                    "trial_generator class must be an instance of BaseTrialGenerator"
                )

        self._trial_condition = asyncio.Condition()
        self._trial_queue = deque()

    @abstractmethod
    async def run_trial(self, trial_spec: TrialSpec) -> TrialFeedback:
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

    async def _occlude_and_lock(self):
        """Occlude smartglass and lock arms concurrently.

        Ensures safety before robot motion by blocking the subject's
        view and constraining their arms. Waits for both operations
        to complete.
        """
        async with asyncio.TaskGroup() as tg:
            tg.create_task(self.commander.lock_arms_and_wait())
            tg.create_task(self.commander.occlude_smartglass())

    async def _trial_coro(self, trial_spec: TrialSpec) -> TrialFeedback:
        feedback = await self.run_trial(trial_spec)
        await self._occlude_and_lock()
        return feedback

    async def _run_one_trial(
        self, trial_spec: TrialSpec, ready_event: asyncio.Event
    ) -> TrialFeedback:
        prev_object_id = self.commander.current_manipulation_id(
            trial_spec.group_name
        )

        if prev_object_id is not None:
            await self.commander.reset_object(
                prev_object_id, trial_spec.group_name
            )
            if trial_spec.object_id != prev_object_id:
                await self.commander.return_object(
                    prev_object_id, trial_spec.group_name
                )

        await self.commander.fetch_object(
            trial_spec.object_id, trial_spec.group_name
        )
        await ready_event.wait()
        await self.commander.present_object(
            trial_spec.object_id, trial_spec.group_name
        )
        await self.commander.plan_and_move(
            goal=trial_spec.object_pose,
            group_name=trial_spec.group_name,
            planning_pipeline="linear",
        )

        feedback = await self.run_trial(trial_spec)

        await self._occlude_and_lock()
        await self.commander.unpresent_object(
            trial_spec.object_id, trial_spec.group_name
        )

        return feedback

    async def _run_trials_simultaneously(self, tg: asyncio.TaskGroup):
        # Occlude smartglass and lock arms before starting
        await self._occlude_and_lock()

        active_trial: asyncio.Task[TrialFeedback] | None = None
        active_spec: TrialSpec | None = None
        next_spec: TrialSpec | None

        try:
            next_spec = next(self._trial_generator)
        except StopIteration:
            raise RuntimeError("Trial generator empty before starting task")

        while not (next_spec is None and active_trial is None):
            # If you wrote a good trial generator, the next trial will use a
            # different robot from the one performing the active, in which
            # case we can fetch the next object.
            # Otherwise we must wait until the active trial has finished, so we
            # skip starting the next trial until we've
            next_trial: asyncio.Task[TrialFeedback] | None = None
            next_ready_event: asyncio.Event | None = None
            if next_spec is not None:
                if (
                    active_spec is None
                    or active_spec.group_name != next_spec.group_name
                ):
                    next_ready_event = asyncio.Event()
                    next_trial = tg.create_task(
                        self._run_one_trial(next_spec, next_ready_event)
                    )
                else:
                    assert active_trial is not None
                    self.log(
                        f"The next trial spec requested the robot "
                        f"{next_spec.group_name}, but this robot is "
                        f"already being used for the active trial.",
                        severity="WARN",
                    )

            # Wait for active trial to finish, send trial feedback, then
            # occlude smartglass, lock arms, and unpresent active object
            if active_trial is not None:
                assert active_spec is not None

                if not active_trial.done():
                    self.log("Waiting for active trial to complete")

                feedback = await active_trial
                self._trial_generator.send(active_spec, feedback)

                active_trial = None
                active_spec = None

            if next_trial is not None:
                assert next_ready_event is not None
                assert next_spec is not None

                next_ready_event.set()

                active_trial = next_trial
                active_spec = next_spec

                try:
                    next_spec = next(self._trial_generator)
                except StopIteration:
                    next_spec = None

    async def _run_trials(self, tg: asyncio.TaskGroup) -> None:
        # Occlude smartglass and lock arms before starting
        await self._occlude_and_lock()

        active_trial: asyncio.Task[TrialFeedback] | None = None
        active_spec: TrialSpec | None = None
        next_spec: TrialSpec | None

        try:
            next_spec = next(self._trial_generator)
        except StopIteration:
            raise RuntimeError("Trial generator empty before starting task")

        while not (next_spec is None and active_trial is None):
            # If you wrote a good trial generator, the next trial will use a
            # different robot from the one performing the active, in which
            # case we can fetch the next object.
            # Otherwise we must wait until the active trial has finished, so we
            # skip starting the next trial until we've
            skip_next: bool = False
            if next_spec is not None:
                if (
                    active_spec is None
                    or active_spec.group_name != next_spec.group_name
                ):
                    await self.commander.fetch_object(
                        next_spec.object_id, next_spec.group_name
                    )
                else:
                    assert active_trial is not None
                    self.log(
                        f"The next trial spec requested the robot "
                        f"{next_spec.group_name}, but this robot is "
                        f"already being used for the active trial.",
                        severity="WARN",
                    )
                    skip_next = True

            # Wait for active trial to finish, send trial feedback, then
            # occlude smartglass, lock arms, and unpresent active object
            prev_spec: TrialSpec | None = None
            if active_trial is not None:
                assert active_spec is not None

                if not active_trial.done():
                    self.log("Waiting for active trial to complete")

                feedback = await active_trial
                self._trial_generator.send(active_spec, feedback)

                await self.commander.unpresent_object(
                    active_spec.object_id, active_spec.group_name
                )

                prev_spec = active_spec
                active_trial = None
                active_spec = None

            # Present next object and move to goal pose, then start the new
            # active trial and get the new next trial spec
            if not skip_next and next_spec is not None:
                await self.commander.present_object(
                    next_spec.object_id, next_spec.group_name
                )
                await self.commander.plan_and_move(
                    goal=next_spec.object_pose,
                    group_name=next_spec.group_name,
                    planning_pipeline="linear",
                )

                # Start trial
                active_spec = next_spec
                active_trial = tg.create_task(self._trial_coro(active_spec))

                try:
                    next_spec = next(self._trial_generator)
                except StopIteration:
                    next_spec = None

            # Reset the previous object and return it if there is no next
            # trial or the next object is different from the previous
            if prev_spec is not None:
                await self.commander.reset_object(
                    prev_spec.object_id, prev_spec.group_name
                )
                if (
                    next_spec is None
                    or next_spec.object_id != prev_spec.object_id
                ):
                    await self.commander.return_object(
                        prev_spec.object_id, prev_spec.group_name
                    )

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
                async with asyncio.TaskGroup() as tg:
                    await self._run_trials_simultaneously(tg)
                    return
