"""Base task module."""

import asyncio
from abc import ABC, abstractmethod
from collections.abc import Callable, Coroutine
from typing import Optional

from tabletop_server.nodes import Commander


def create_task_wrapper(self, coro: Coroutine) -> asyncio.Task:
    tg: Optional[asyncio.TaskGroup] = self.tg
    if tg is None:
        return asyncio.create_task(coro)
    else:
        return self.tg.create_task(coro)


def asyncio_task_decorator(coro_fn: Callable[..., Coroutine]):
    """
    Decorator for methods that should be run in the current asyncio.TaskGroup.

    This decorator is designed for BaseNode methods. It will only work for
    methods whose first argument is `self` and whose class has an
    `asyncio.TaskGroup` attribute named `tg`.
    """

    def wrapper(self, *args, **kwargs) -> asyncio.Task:
        coro = coro_fn(self, *args, **kwargs)
        return create_task_wrapper(self, coro)

    return wrapper


class BaseTask(ABC):
    """Abstract base class for all tasks."""

    def __init__(self, commander: Commander) -> None:
        """Initialize base task.

        Args:
            commander: Commander instance for interacting with the system
            **kwargs: Additional task-specific arguments
        """
        self._commander = commander

        self._commander.init_commander()

    @property
    def commander(self) -> Commander:
        """Get the commander instance."""
        return self._commander

    def log(self, message: str, severity: str = "INFO") -> None:
        """Log a message."""
        self.commander.log(message, severity)

    def schedule_sleep(self, delay: float) -> asyncio.Task:
        """Schedule a sleep."""
        return self.commander.schedule(asyncio.sleep(delay))  # type: ignore

    @abstractmethod
    async def run(self) -> None:
        """Run the task to completion."""
        raise NotImplementedError("Tasks must implement run() method")
