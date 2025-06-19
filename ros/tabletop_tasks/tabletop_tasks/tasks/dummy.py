"""Base task module."""

from tabletop_server.nodes import Commander

from tabletop_tasks.tasks.base import BaseTask


class DummyTask(BaseTask):
    """Dummy task."""

    def __init__(self, commander: Commander) -> None:
        self._commander = commander

    async def run(self):
        while True:
            async with self.commander as com:
                await com.fetch_object("small_object_23")
                com._pre_return_cache_kwargs = None
                await com.return_object()
