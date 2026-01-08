"""TableTop Tasks package for behavioral experiment execution.

This package provides tasks and trial generators for running behavioral
experiments with the TableTop robotic system. Tasks define the experimental
logic, while trial generators produce sequences of trial specifications.

The main entry point is run_tasks(), which loads task configurations from
YAML files and executes them sequentially.

Subpackages:
    tasks: Experimental task implementations (ForagingTask, etc.)
    trial_generators: Trial sequence generators (RandomChoice, etc.)

Example:
    from tabletop_tasks import run_tasks
    await run_tasks(commander, "config/foraging.yaml")
"""

from .run import run_tasks

__all__ = ["run_tasks"]
