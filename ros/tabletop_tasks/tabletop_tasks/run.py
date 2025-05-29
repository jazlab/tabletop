import importlib
import traceback
from collections.abc import Mapping
from typing import Any

from tabletop_server.nodes import Commander

from tabletop_tasks.tasks.base import BaseTask


async def run_tasks(commander: Commander, config: Mapping[str, Any]) -> None:
    print("Running tasks")
    try:
        # Initialize the commander (must be done after starting the executor)
        commander.init_dashboard()

        commander.log(f"Loading task config from: {config}")
        # Use importlib to create instances of task class
        for task_config in config["tasks"]:
            task: BaseTask = getattr(
                importlib.import_module("tabletop_tasks.tasks"),
                task_config["class"],
            )(commander=commander, **task_config["kwargs"])

            commander.log(f"Running task: {task_config['class']}")
            await task.run()
    except Exception as e:
        print("Error running tasks:")
        print(f"{type(e).__name__}: {e}")
        traceback.print_exc()
        raise e
