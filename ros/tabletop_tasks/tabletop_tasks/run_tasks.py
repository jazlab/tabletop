import argparse
import asyncio
import importlib

import rclpy
import yaml
from tabletop_server.nodes import Commander

from tabletop_tasks.tasks.base_task import BaseTask
from tabletop_tasks.utils import without_keys


async def run(commander: Commander, task_config_file: str) -> None:
    print("Running tasks")
    try:
        config = yaml.safe_load(task_config_file)
        print('\n\n\ntest\n\n\n')
        raise ValueError('test')

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
    except Exception as e:
        print(f"Error running tasks: {e}")
        raise e


def main(args=None) -> None:
    rclpy.init(args=args)

    non_ros_args = rclpy.utilities.remove_ros_args(args)  # type: ignore
    task_config_file = non_ros_args[1]
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

        future = executor.create_task(asyncio.run, run(commander, task_config_file))

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