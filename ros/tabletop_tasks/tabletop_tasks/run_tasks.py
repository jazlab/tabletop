import argparse
import asyncio
import importlib

import rclpy
import yaml
from tabletop_server.nodes import Commander

from tabletop_tasks.tasks.base_task import BaseTask
from tabletop_tasks.utils import without_keys


async def run(commander: Commander, task_config_file: str) -> None:
    config = yaml.safe_load(task_config_file)

    for task_config in config["tasks"]:
        task_module = task_config["module"]
        task_constructor = task_config["constructor"]
        task_kwargs = without_keys(task_config, ["module", "constructor"])

        # Warning: This is a potential security risk, but we trust the task
        # config file
        task: BaseTask = getattr(
            importlib.import_module(task_module), task_constructor
        )(commander=commander, **task_kwargs)

        await task.run()


def main(args=None) -> None:
    assert args is None
    rclpy.init(args=args)

    non_ros_args = rclpy.utilities.remove_ros_args(args)  # type: ignore

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "task_config_file",
        type=str,
        help="Path to the YAML config file for running tasks",
    )
    args = parser.parse_args(non_ros_args)
    try:
        executor: rclpy.Executor = rclpy.executors.MultiThreadedExecutor()  # type: ignore
        commander = Commander()
        executor.add_node(commander)

        future = commander.create_rclpy_task(
            asyncio.run, run(commander, args.task_config_file)
        )

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
