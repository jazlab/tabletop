"""Base task module."""

import asyncio

from tabletop_rig.nodes import Commander

from tabletop_tasks.tasks.base import BaseTask


class DummyTask(BaseTask):
    """Dummy task."""

    def __init__(self, commander: Commander) -> None:
        super().__init__(commander, logger_name="dummy_task")

    async def run(self):
        async with self.commander:
            while True:
                await asyncio.sleep(1.0)
