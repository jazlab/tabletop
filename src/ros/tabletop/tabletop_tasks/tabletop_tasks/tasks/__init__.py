"""Experimental task implementations for behavioral experiments.

This subpackage contains task classes that define the logic for running
different types of behavioral experiments. All tasks inherit from BaseTask
and implement the run_trial() method.

Available Tasks:
    BaseTask: Abstract base class for all tasks.
    DummyTask: Placeholder task for testing.
    FetchTask: Simple object movement task.
    PresentObjectTask: Object presentation without response collection.
    ForagingTask: Full delayed match-to-sample paradigm with reward.
    SmoothPursuitTask: Eye tracking with moving target trajectory.

Example:
    from tabletop_tasks.tasks import ForagingTask
    task = ForagingTask(commander, trial_generator, ...)
    await task.run()
"""

from .base import BaseObjectInteractionTask, BaseTask
from .cache_benchmark import CacheBenchmarkTask
from .dummy import DummyTask
from .foraging import ForagingTask
from .present import PresentTask
from .smooth_pursuit import SmoothPursuitTask

__all__ = [
    "BaseObjectInteractionTask",
    "BaseTask",
    "CacheBenchmarkTask",
    "DummyTask",
    "ForagingTask",
    "PresentTask",
    "SmoothPursuitTask",
]
