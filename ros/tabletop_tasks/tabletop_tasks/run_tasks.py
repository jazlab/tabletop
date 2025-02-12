import asyncio
import importlib

import rclpy
import yaml
from tabletop_server.nodes import Commander

from tabletop_tasks.utils import without_keys


async def _run(commander):
    config_file = commander.get_parameter("task_runner_yaml").value
    config = yaml.safe_load(config_file)

    for task_config in config["tasks"]:
        task_module = task_config["module"]
        task_constructor = task_config["constructor"]
        task_kwargs = without_keys(task_config, ["module", "constructor"])
        task = getattr(importlib.import_module(task_module), task_constructor)(
            commander=commander, **task_kwargs
        )

        await task.run()


def run(commander):
    asyncio.run(_run(commander))


def main(args=None):
    rclpy.init(args=args)
    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        commander = Commander(executor)

        executor.add_node(commander)
        executor.create_task(run, commander)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            commander.destroy_node()
    finally:
        rclpy.shutdown()
