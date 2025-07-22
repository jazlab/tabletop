"""Base task module."""

import asyncio
import time

from tabletop_server.nodes import Commander

from tabletop_tasks.tasks.base import BaseTask


class DummyTask(BaseTask):
    """Dummy task."""

    def __init__(self, commander: Commander) -> None:
        self._commander = commander

    async def run(self):
        async with self.commander as com:
            while True:
                start = time.time()
                await com.reward_and_wait(1.0)
                print(f"Reward took {time.time() - start:.2f} seconds")
                start = time.time()
                await asyncio.sleep(1.0)
                print(f"Sleep took {time.time() - start:.2f} seconds")
