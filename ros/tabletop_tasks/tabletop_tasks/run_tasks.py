import asyncio
import importlib
import traceback
from collections.abc import Mapping
from typing import Any

import rclpy
import yaml
from tabletop_server.nodes import Commander

from tabletop_tasks.tasks.base_task import BaseTask


async def run(commander: Commander, config: Mapping[str, Any]) -> None:
    print("Running tasks")
    try:
        commander.log(f"Loading task config from: {config}")

        for task_config in config["tasks"]:
            task_module_name = task_config["module"]
            task_module = f"tabletop_tasks.tasks.{task_module_name}"
            task_class = task_config["class"]
            task_kwargs = task_config["kwargs"]

            # Warning: This is a potential security risk, but we trust the task
            # config file
            # Use importlib to create instance of task class
            task: BaseTask = getattr(
                importlib.import_module(task_module), task_class
            )(commander=commander, **task_kwargs)

            await task.run()
    except Exception as e:
        print("Error running tasks:")
        print(f"{type(e).__name__}: {e}")
        traceback.print_exc()
        raise e


def main(args=None) -> None:
    rclpy.init(args=args)

    non_ros_args = rclpy.utilities.remove_ros_args(args)  # type: ignore
    task_config_file = non_ros_args[1]

    with open(task_config_file, "r") as f:
        task_config = yaml.safe_load(f)

    print(f"Task config: {task_config}")

    # parser = argparse.ArgumentParser()
    # parser.add_argument(
    #     "--task_config_file",
    #     type=str,
    #     default="tasks.yaml",
    #     help="Path to the YAML config file for running tasks",
    # )
    # print(f"non_ros_args: {non_ros_args[1:]}")
    # args = parser.parse_args(" ".join(non_ros_args))
    try:
        executor: rclpy.Executor = rclpy.executors.MultiThreadedExecutor()  # type: ignore
        commander = Commander()
        executor.add_node(commander)

        future = executor.create_task(asyncio.run, run(commander, task_config))

        try:
            executor.spin_until_future_complete(future)
        finally:
            print("Shutting down executor")
            executor.shutdown()
            print("Shutting down commander")
            commander.destroy_node()
    finally:
        print("Shutting down rclpy")
        rclpy.shutdown()
