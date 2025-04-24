import asyncio
import importlib
import traceback
from collections.abc import Mapping
from concurrent.futures import ThreadPoolExecutor
from typing import Any

import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from tabletop_server.nodes import Commander

from tabletop_tasks.tasks.base_task import BaseTask


async def run(commander: Commander, config: Mapping[str, Any]) -> None:
    print("Running tasks")
    try:
        commander.log(f"Loading task config from: {config}")

        # Use importlib to create instances of task class
        for task_config in config["tasks"]:
            task: BaseTask = getattr(
                importlib.import_module("tabletop_tasks.tasks"),
                task_config["class"],
            )(commander=commander, **task_config["kwargs"])
            await task.run()
    except Exception as e:
        print("Error running tasks:")
        print(f"{type(e).__name__}: {e}")
        traceback.print_exc()
        raise e


def main(args=None):
    rclpy.init(args=args)

    non_ros_args = rclpy.utilities.remove_ros_args(args)  # type: ignore
    config_file = non_ros_args[1]

    with open(config_file, "r") as f:
        run_config = yaml.safe_load(f)

    try:
        commander = Commander()
        executor = SingleThreadedExecutor()
        executor.add_node(commander)

        try:
            with ThreadPoolExecutor(max_workers=1) as tpe:
                run_future = tpe.submit(
                    asyncio.run, run(commander, run_config)
                )
                run_future.add_done_callback(lambda _: executor.shutdown())
                executor.spin()
        finally:
            print("Shutting down executor")
            executor.shutdown()
            print("Shutting down commander")
            commander.destroy_node()
    finally:
        print("Shutting down rclpy")
        rclpy.shutdown()
