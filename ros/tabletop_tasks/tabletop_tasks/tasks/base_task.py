"""Base task module."""

from abc import ABC, abstractmethod

from tabletop_server.nodes import Commander


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

    @abstractmethod
    async def run(self) -> None:
        """Run the task to completion."""
        raise NotImplementedError("Tasks must implement run() method")
