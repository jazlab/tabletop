import asyncio

import rclpy
import yaml
from tabletop_server.commander import Commander

from tabletop_tasks.tasks import ForagingTask, SmoothPursuit


async def _run(commander):
    config_file = commander.get_parameter("task_generator_yaml").value
    config = yaml.safe_load(config_file)

    for task_config in config["tasks"]:
        match task_config["type"]:
            case "smooth_pursuit":
                task = SmoothPursuit(
                    commander=commander,
                    **task_config,
                )
            case "foraging":
                task = ForagingTask(
                    commander=commander,
                    **task_config,
                )
            case _:
                raise ValueError(f"Unknown task type: {task_config['type']}")

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
