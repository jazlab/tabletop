"""Dummy task for testing and placeholder purposes.

This module provides a minimal task implementation that does nothing
but keep the commander context alive. Useful for testing the task
infrastructure or as a placeholder during development.

Example:
    task = DummyTask(commander)
    await task.run()  # Runs indefinitely, sleeping each second
"""

import asyncio

from tabletop_rig.nodes import Commander

from tabletop_tasks.tasks.base import BaseTask


class DummyTask(BaseTask):
    """Minimal placeholder task that runs indefinitely.

    This task maintains an active commander context while doing no
    actual work. It's useful for:
    - Testing the task infrastructure
    - Keeping robot connections alive during debugging
    - Serving as a template for new task implementations

    Unlike other tasks, DummyTask does not use a trial generator
    and overrides run() to provide its own infinite loop.
    """

    def __init__(self, commander: Commander) -> None:
        """Initialize the dummy task.

        Args:
            commander: Commander instance for robot interaction.
        """
        super().__init__(commander, logger_name="dummy_task")

    async def run_trial(self, trial_spec):
        """Not implemented for dummy task.

        This method exists only to satisfy the abstract base class
        requirement. DummyTask overrides run() directly.

        Args:
            trial_spec: Unused trial specification.

        Returns:
            None (never called).
        """
        pass

    async def run(self):
        """Run the dummy task indefinitely.

        Maintains an active commander context while sleeping in
        one-second intervals. Never terminates on its own.
        """
        async with self.commander:
            while True:
                await asyncio.sleep(1.0)
