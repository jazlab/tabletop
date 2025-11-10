import asyncio
from collections.abc import (
    Callable,
    Coroutine,
)
from typing import Any


def asyncio_task_decorator[T](
    coro_fn: Callable[..., Coroutine[None, None, T]],
) -> Callable[..., asyncio.Task[T]]:
    """
    Decorator for methods that should be run in the current asyncio.TaskGroup.

    This decorator is designed for BaseNode methods. It will only work for
    methods whose first argument is `self` and whose class has an
    `asyncio.TaskGroup` attribute named `tg`.

    WARNING: If a task raises an exception, all tasks in the TaskGroup will
    be cancelled. As a result, you should not use this decorator for coroutines
    that are expected to raise exceptions (e.g. you cannot catch exceptions of tasks).

    Args:
        coro_fn: The coroutine function to decorate.

    Returns:
        The decorated function which returns an asyncio.Task.
    """

    def wrapper(*args: Any, **kwargs: Any) -> asyncio.Task:
        """Wrapper function that creates and returns an asyncio.Task.

        Args:
            self: The instance of the class.
            *args: Positional arguments for the coroutine function.
            **kwargs: Keyword arguments for the coroutine function.

        Returns:
            The created asyncio.Task.
        """
        coro = coro_fn(*args, **kwargs)
        return asyncio.create_task(coro)

    return wrapper
