"""Base task module."""

from abc import ABC, abstractmethod
from typing import Any

from tabletop_server.nodes import Commander


class BaseTask(ABC):
    """Abstract base class for all tasks."""

    def __init__(self, commander: Commander, **kwargs: Any) -> None:
        """Initialize base task.

        Args:
            commander: Commander instance for interacting with the system
            **kwargs: Additional task-specific arguments
        """
        self._commander = commander

    @property
    def commander(self) -> Commander:
        """Get the commander instance."""
        return self._commander

    @abstractmethod
    async def run(self) -> None:
        """Run the task to completion."""
        raise NotImplementedError("Tasks must implement run() method")
