import importlib

import yaml

from tabletop_server.nodes import Commander
from tabletop_tasks.tasks.base import BaseTask
from tabletop_utils.common import yaml_dump_string


async def run_tasks(commander: Commander, config_file: str) -> None:
    print("Running tasks")

    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    commander.log(f"Tasks config: {yaml_dump_string(config, width=80)}")
    # Use importlib to create instances of task class
    for task_config in config["tasks"]:
        task: BaseTask = getattr(
            importlib.import_module("tabletop_tasks.tasks"),
            task_config["class"],
        )(commander=commander, **task_config["kwargs"])

        commander.log(f"Running task: {task_config['class']}")
        await task.run()
