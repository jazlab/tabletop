"""Base task module."""

import asyncio

from tabletop_server.nodes import Commander

from tabletop_tasks.tasks.base_task import BaseTask


class DummyTask(BaseTask):
    """Dummy task."""

    def __init__(self, commander: Commander) -> None:
        self._commander = commander

    async def run(self):
        while True:
            self.log("Dummy task running")
            await asyncio.sleep(10)
