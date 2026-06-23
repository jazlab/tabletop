"""Task runner for executing experimental tasks from YAML configuration.

This module provides the entry point for running behavioral experiment tasks.
Tasks are dynamically loaded from YAML configuration files that specify
the task class and its initialization parameters.

The run_tasks coroutine is designed to be called by the Commander node
after it has completed initialization. It reads the task configuration,
instantiates each task, and runs them in sequence.

Configuration file format:
    tasks:
      - class: ForagingTask
        kwargs:
          trial_generator:
            class: OrderedChoiceAlternating
            kwargs:
              poses: [...]
              num_trials: 100
          stimulus_duration: 1.0
          delay_duration: 2.0
          ...

Example:
    # Called by Commander coroutine mechanism
    await run_tasks(commander, "/path/to/config.yaml")
"""

import importlib

import yaml
from tabletop_rig.nodes import Commander

from tabletop_py.utils.common import yaml_dump_string
from tabletop_tasks.tasks.base import BaseTask


async def run_tasks(commander: Commander, config_file: str) -> None:
    """Run experimental tasks from a YAML configuration file.

    Loads task configurations from the specified YAML file and executes
    each task in sequence. Tasks are dynamically instantiated using
    importlib based on the class name specified in the configuration.

    Each task receives the commander instance and any additional keyword
    arguments specified in the configuration.

    Args:
        commander: The Commander node instance for robot and peripheral
            control. Passed to each task's constructor.
        config_file: Path to the YAML configuration file containing
            task definitions.

    Note:
        Tasks are run sequentially - each task's run() method must
        complete before the next task begins. The configuration file
        should contain a 'tasks' key with a list of task definitions,
        each having 'class' and 'kwargs' keys.
    """
    print("Running tasks")

    # Load task configuration from YAML file
    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    print(f"Tasks config: {yaml_dump_string(config, width=80)}")

    # Dynamically instantiate and run each configured task
    try:
        for task_config in config["tasks"]:
            task: BaseTask = getattr(
                importlib.import_module("tabletop_tasks.tasks"),
                task_config["class"],
            )(commander=commander, **task_config["kwargs"])

            print(f"Running task: {task_config['class']}")
            await task.run()
    except:
        raise
