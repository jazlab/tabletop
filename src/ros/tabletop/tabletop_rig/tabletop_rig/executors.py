# pyright: reportIncompatibleMethodOverride=false
"""Custom ROS2 executors with asyncio integration.

This module provides executor implementations that bridge ROS2's callback-based
execution model with Python's asyncio event loop, enabling truly concurrent
async/await patterns in ROS2 nodes.

The standard ROS2 executors block during `spin()`, which prevents integration
with asyncio coroutines. These custom executors solve this by:

1. Using non-blocking waits with asyncio sleep intervals (SimpleAIOExecutor)
2. Running wait operations in a thread pool with asyncio integration (AIOExecutor)

Classes:
    SimpleAIOExecutor: Basic asyncio-compatible executor using polling.
    AIOExecutor: Full-featured asyncio executor with thread pool support.
    TestExecutor: Simple wrapper for debugging executor behavior.

Example:
    async def main():
        rclpy.init()
        node = MyNode()
        executor = AIOExecutor()
        executor.add_node(node)

        # Spin in background while doing other async work
        spin_task = asyncio.create_task(executor.spin())
        await some_async_operation()
        executor.shutdown()
        await spin_task
"""

import asyncio
import inspect
from collections.abc import AsyncGenerator, Callable
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from typing import Any, Optional

from rclpy.client import Client
from rclpy.exceptions import InvalidHandle
from rclpy.executors import (
    ConditionReachedException,
    Executor,
    ExternalShutdownException,
    ShutdownException,
    SingleThreadedExecutor,
)
from rclpy.guard_condition import GuardCondition
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy.task import Future, Task
from rclpy.timer import Timer
from rclpy.utilities import timeout_sec_to_nsec
from rclpy.waitable import NumberOfEntities, Waitable

WAIT_SET_CLEANUP_TIMEOUT_SEC = 0.1
"""float: Timeout for cleaning up wait sets when cancelling operations."""


class AIOFutureDoneError(Exception):
    """Raised when an asyncio future completes during wait operations.

    This exception is used internally to break out of the wait loop when
    an asyncio future that was being monitored completes, allowing the
    executor to process its result.
    """


class SimpleAIOExecutor(SingleThreadedExecutor):
    """Simple asyncio-compatible executor using polling.

    This executor provides basic asyncio compatibility by using non-blocking
    spin_once calls with small sleep intervals. It's simpler than AIOExecutor
    but may have slightly higher CPU usage due to polling.

    The spin method is an async coroutine that can be awaited, allowing
    other async tasks to run concurrently.

    Example:
        executor = SimpleAIOExecutor()
        executor.add_node(node)
        await executor.spin()  # Runs until shutdown
    """

    async def spin(self) -> None:
        """Spin the executor asynchronously until shutdown.

        Continuously processes callbacks using non-blocking spin_once calls
        with brief async sleeps to yield to the event loop.
        """
        while self._context.ok() and not self._is_shutdown:
            self.spin_once(timeout_sec=0)
            await asyncio.sleep(1e-4)


class AIOExecutor(Executor):
    """Full-featured asyncio-compatible ROS2 executor.

    This executor provides seamless integration between ROS2 callbacks and
    Python's asyncio event loop. It uses a thread pool to perform blocking
    ROS2 wait operations while allowing the asyncio event loop to continue
    running.

    Key features:
    - All spin methods are async coroutines
    - Supports both coroutine and regular callbacks
    - Optional multi-threading for parallel callback execution
    - Proper cleanup on shutdown

    Attributes:
        _multi_threaded: Whether parallel callback execution is enabled.
        _tpe: Thread pool executor for blocking operations.
        _aio_futures: List of pending asyncio futures being monitored.
        _spin_interval: Sleep interval between spin iterations.
    """

    def __init__(
        self,
        *args: Any,
        multi_threaded: bool = False,
        max_workers: int = 1,
        spin_interval: float = 1e-3,
        **kwargs: Any,
    ) -> None:
        """Initialize the asyncio executor.

        Args:
            *args: Arguments passed to the base Executor constructor.
            multi_threaded: If True, non-coroutine callbacks execute in the
                thread pool, allowing parallel execution. If False, callbacks
                execute sequentially.
            max_workers: Maximum thread pool size. Only meaningful when
                multi_threaded is True.
            spin_interval: Time in seconds between spin loop iterations.
            **kwargs: Keyword arguments passed to the base Executor constructor.

        Raises:
            ValueError: If max_workers > 1 but multi_threaded is False.
        """
        super().__init__(*args, **kwargs)
        if not multi_threaded and max_workers > 1:
            raise ValueError(
                "max_workers must be 1 if multi_threaded is False"
            )
        self._multi_threaded = multi_threaded
        self._tpe = ThreadPoolExecutor(max_workers=max_workers)
        self._aio_futures: list[asyncio.Future] = []
        self._spin_interval = spin_interval

    def _waitables_ready(self, wait_set: Any) -> bool:
        """Check if any entities in the wait set are ready.

        Args:
            wait_set: The ROS2 wait set to check.

        Returns:
            True if any subscription, guard condition, timer, client,
            service, or event is ready for processing.
        """
        for entity_type in [
            "subscription",
            "guard_condition",
            "timer",
            "client",
            "service",
            "event",
        ]:
            if len(wait_set.get_ready_entities(entity_type)) > 0:
                return True
        return False

    async def _wait_for_ready_callbacks(
        self,
        timeout_sec: Optional[float] = None,
        nodes: Optional[list[Node]] = None,
        condition: Callable[[], bool] = lambda: False,
    ) -> AsyncGenerator[tuple[Task, Any, Node | None], None]:
        """Async generator yielding callbacks ready for execution.

        This is the core method that waits for ROS2 entities (subscriptions,
        timers, services, etc.) to become ready and yields task handlers for
        each ready callback.

        The wait operation runs in a thread pool to avoid blocking the asyncio
        event loop. The method also monitors any pending asyncio futures and
        breaks out of the wait when they complete.

        Args:
            timeout_sec: Maximum time to wait in seconds. Block forever if None
                or negative. Return immediately if 0.
            nodes: Specific nodes to process. If None, processes all nodes
                registered with the executor.
            condition: Callable returning True to trigger early return via
                ConditionReachedException.

        Yields:
            Tuples of (task_handler, entity, node) for each ready callback.

        Raises:
            ShutdownException: If the executor was shut down.
            ConditionReachedException: If the condition callable returns True.
            AIOFutureDoneError: If a monitored asyncio future completes.
        """
        timeout_nsec = timeout_sec_to_nsec(timeout_sec)

        yielded_work = False
        while (
            not yielded_work
            and not self._is_shutdown
            and not condition()
            and not any(future.done() for future in self._aio_futures)
        ):
            # Refresh "all" nodes in case executor was woken by a node being added or removed
            nodes_to_use = nodes
            if nodes is None:
                nodes_to_use = self.get_nodes()

            # Yield tasks in-progress before waiting for new work
            tasks = None
            with self._tasks_lock:
                tasks = list(self._tasks)
            if tasks:
                for task, entity, node in reversed(tasks):
                    if (
                        not task.executing()
                        and not task.done()
                        and (node is None or node in nodes_to_use)  # type: ignore
                    ):
                        yielded_work = True
                        yield task, entity, node
                with self._tasks_lock:
                    # Get rid of any tasks that are done
                    self._tasks = list(
                        filter(lambda t_e_n: not t_e_n[0].done(), self._tasks)
                    )
                    # Get rid of any tasks that are cancelled
                    self._tasks = list(
                        filter(
                            lambda t_e_n: not t_e_n[0].cancelled(), self._tasks
                        )
                    )

            # Gather entities that can be waited on
            subscriptions: list[Subscription] = []
            guards: list[GuardCondition] = []
            timers: list[Timer] = []
            clients: list[Client] = []
            services: list[Service] = []
            waitables: list[Waitable] = []
            for node in nodes_to_use:  # type: ignore
                subscriptions.extend(
                    filter(self.can_execute, node.subscriptions)
                )
                timers.extend(filter(self.can_execute, node.timers))
                clients.extend(filter(self.can_execute, node.clients))
                services.extend(filter(self.can_execute, node.services))
                node_guards = filter(self.can_execute, node.guards)
                waitables.extend(filter(self.can_execute, node.waitables))
                # retrigger a guard condition that was triggered but not handled
                for gc in node_guards:
                    if gc._executor_triggered:
                        gc.trigger()
                    guards.append(gc)

            guards.append(self._guard)  # type: ignore
            guards.append(self._sigint_gc)  # type: ignore

            entity_count = NumberOfEntities(
                len(subscriptions),
                len(guards),
                len(timers),
                len(clients),
                len(services),
            )

            # Construct a wait set
            with ExitStack() as context_stack:
                sub_handles = []
                for sub in subscriptions:
                    try:
                        context_stack.enter_context(sub.handle)
                        sub_handles.append(sub.handle)
                    except InvalidHandle:
                        entity_count.num_subscriptions -= 1

                client_handles = []
                for cli in clients:
                    try:
                        context_stack.enter_context(cli.handle)
                        client_handles.append(cli.handle)
                    except InvalidHandle:
                        entity_count.num_clients -= 1

                service_handles = []
                for srv in services:
                    try:
                        context_stack.enter_context(srv.handle)
                        service_handles.append(srv.handle)
                    except InvalidHandle:
                        entity_count.num_services -= 1

                timer_handles = []
                for tmr in timers:
                    try:
                        context_stack.enter_context(tmr.handle)
                        timer_handles.append(tmr.handle)
                    except InvalidHandle:
                        entity_count.num_timers -= 1

                guard_handles = []
                for gc in guards:
                    try:
                        context_stack.enter_context(gc.handle)
                        guard_handles.append(gc.handle)
                    except InvalidHandle:
                        entity_count.num_guard_conditions -= 1

                for waitable in waitables:
                    try:
                        context_stack.enter_context(waitable)
                        entity_count += waitable.get_num_entities()
                    except InvalidHandle:
                        pass

                context_stack.enter_context(self._context.handle)  # type: ignore

                wait_set = _rclpy.WaitSet(
                    entity_count.num_subscriptions,
                    entity_count.num_guard_conditions,
                    entity_count.num_timers,
                    entity_count.num_clients,
                    entity_count.num_services,
                    entity_count.num_events,
                    self._context.handle,
                )

                wait_set.clear_entities()
                for sub_handle in sub_handles:
                    wait_set.add_subscription(sub_handle)
                for cli_handle in client_handles:
                    wait_set.add_client(cli_handle)
                for srv_capsule in service_handles:
                    wait_set.add_service(srv_capsule)
                for tmr_handle in timer_handles:
                    wait_set.add_timer(tmr_handle)
                for gc_handle in guard_handles:
                    wait_set.add_guard_condition(gc_handle)
                for waitable in waitables:
                    waitable.add_to_wait_set(wait_set)

                # Wait for something to become ready
                future = None
                try:
                    future = self._tpe.submit(wait_set.wait, timeout_nsec)
                    await asyncio.wrap_future(future)
                except asyncio.CancelledError:
                    # Wake the executor to join the thread
                    self.wake()
                    if future is not None:
                        if not future.cancel():
                            future.result(timeout=WAIT_SET_CLEANUP_TIMEOUT_SEC)
                    raise

                if self._is_shutdown:
                    raise ShutdownException()
                if not self._context.ok():
                    raise ExternalShutdownException()

                # get ready entities
                subs_ready = wait_set.get_ready_entities("subscription")
                guards_ready = wait_set.get_ready_entities("guard_condition")
                timers_ready = wait_set.get_ready_entities("timer")
                clients_ready = wait_set.get_ready_entities("client")
                services_ready = wait_set.get_ready_entities("service")

                # Mark all guards as triggered before yielding since they're auto-taken
                for gc in guards:
                    if gc.handle.pointer in guards_ready:
                        gc._executor_triggered = True

                # Check waitables before wait set is destroyed
                for node in nodes_to_use:  # type: ignore
                    for wt in node.waitables:
                        # Only check waitables that were added to the wait set
                        if wt in waitables and wt.is_ready(wait_set):
                            if wt.callback_group.can_execute(wt):
                                handler = self._make_handler(
                                    wt, node, self._take_waitable
                                )
                                yielded_work = True
                                yield handler, wt, node

            # Process ready entities one node at a time
            for node in nodes_to_use:  # type: ignore
                for tmr in node.timers:
                    if tmr.handle.pointer in timers_ready:
                        # Check timer is ready to workaround rcl issue with cancelled timers
                        if tmr.handle.is_timer_ready():
                            if tmr.callback_group.can_execute(tmr):
                                handler = self._make_handler(
                                    tmr, node, self._take_timer
                                )
                                yielded_work = True
                                yield handler, tmr, node

                for sub in node.subscriptions:
                    if sub.handle.pointer in subs_ready:
                        if sub.callback_group.can_execute(sub):
                            handler = self._make_handler(
                                sub, node, self._take_subscription
                            )
                            yielded_work = True
                            yield handler, sub, node

                for gc in node.guards:
                    if gc._executor_triggered:
                        if gc.callback_group.can_execute(gc):
                            handler = self._make_handler(
                                gc, node, self._take_guard_condition
                            )
                            yielded_work = True
                            yield handler, gc, node

                for client in node.clients:
                    if client.handle.pointer in clients_ready:
                        if client.callback_group.can_execute(client):
                            handler = self._make_handler(
                                client, node, self._take_client
                            )
                            yielded_work = True
                            yield handler, client, node

                for srv in node.services:
                    if srv.handle.pointer in services_ready:
                        if srv.callback_group.can_execute(srv):
                            handler = self._make_handler(
                                srv, node, self._take_service
                            )
                            yielded_work = True
                            yield handler, srv, node

        if self._is_shutdown:
            raise ShutdownException()
        if condition():
            raise ConditionReachedException()
        if any(future.done() for future in self._aio_futures):
            raise AIOFutureDoneError()

    async def wait_for_ready_callbacks(
        self, *args: Any, **kwargs: Any
    ) -> tuple[Task, Any, Node | None]:
        """Wait for and return a single ready callback.

        Wraps the async generator `_wait_for_ready_callbacks` to return
        one callback at a time. Manages the generator lifecycle internally,
        creating a new one when arguments change or the previous is exhausted.

        Args:
            *args: Passed to _wait_for_ready_callbacks.
            **kwargs: Passed to _wait_for_ready_callbacks.

        Returns:
            Tuple of (task_handler, entity, node) for the next ready callback.
        """
        while True:
            if (
                self._cb_iter is None
                or self._last_args != args
                or self._last_kwargs != kwargs
            ):
                # Create a new generator
                self._last_args = args
                self._last_kwargs = kwargs
                self._cb_iter = self._wait_for_ready_callbacks(*args, **kwargs)

            try:
                return await self._cb_iter.__anext__()
            except StopAsyncIteration:
                # Generator ran out of work
                self._cb_iter = None

    @staticmethod
    async def _call_task(handler: Task) -> None:
        """Execute a task handler, supporting both coroutines and regular callables.

        Acquires the task lock to prevent concurrent execution, runs the handler,
        and stores the result or exception in the task.

        Args:
            handler: The Task object wrapping the callback to execute.
        """
        if (
            not handler._pending()
            or handler._executing
            or not handler._task_lock.acquire(blocking=False)
        ):
            return
        try:
            if not handler._pending():
                return
            handler._executing = True

            # Execute a coroutine
            try:
                if inspect.iscoroutine(handler._handler):
                    result = await handler._handler
                else:
                    assert handler._handler is not None
                    result = handler._handler(
                        *handler._args,
                        **handler._kwargs,  # type: ignore
                    )
                handler.set_result(result)
            except Exception as e:
                handler.set_exception(e)
            finally:
                handler._complete_task()
                handler._executing = False
        finally:
            handler._task_lock.release()

    async def _spin_once_impl(
        self,
        timeout_sec: Optional[float] = None,
        wait_condition: Callable[[], bool] = lambda: False,
    ) -> None:
        """Internal implementation of spin_once.

        Waits for a ready callback and executes it. Coroutine callbacks run
        as asyncio tasks; regular callbacks run in the thread pool if
        multi_threaded is enabled.

        Args:
            timeout_sec: Maximum wait time in seconds.
            wait_condition: Callable for early termination condition.
        """
        try:
            handler, _, _ = await self.wait_for_ready_callbacks(
                timeout_sec, None, wait_condition
            )
        except (
            ExternalShutdownException,
            ShutdownException,
            ConditionReachedException,
        ):
            return
        except AIOFutureDoneError:
            pass
        else:
            # Create an asyncio future to run the handler
            if (
                inspect.iscoroutine(handler._handler)
                or not self._multi_threaded
            ):
                future = asyncio.create_task(self._call_task(handler))
            else:
                future = asyncio.wrap_future(self._tpe.submit(handler))

            # Wake the executor when the future is done to handle any exceptions
            future.add_done_callback(lambda _: self.wake())
            self._aio_futures.append(future)

        # Raise any exceptions from futures that are done
        if len(self._aio_futures) > 0:
            pending_futures: list[asyncio.Future] = []
            for future in self._aio_futures:
                if future.done():
                    self._aio_futures.remove(future)
                    future.result()  # raise any exceptions
                else:
                    pending_futures.append(future)
            self._aio_futures = pending_futures

    async def _spin_impl(
        self,
        timeout_sec: Optional[float] = None,
        wait_condition: Callable[[], bool] = lambda: False,
    ) -> None:
        """Internal implementation of continuous spinning.

        Repeatedly calls _spin_once_impl until shutdown, timeout, or the
        wait condition is met.

        Args:
            timeout_sec: Maximum total spin time in seconds.
            wait_condition: Callable for early termination condition.
        """
        try:
            async with asyncio.timeout(timeout_sec):
                while (
                    self._context.ok()
                    and not self._is_shutdown
                    and not wait_condition()
                ):
                    await self._spin_once_impl(timeout_sec, wait_condition)
        except TimeoutError:
            pass

    async def spin_once(self, timeout_sec: Optional[float] = None) -> None:
        """Process a single callback asynchronously.

        Args:
            timeout_sec: Maximum time to wait for a callback in seconds.
        """
        await self._spin_once_impl(timeout_sec)

    async def spin_once_until_future_complete(
        self, future: Future, timeout_sec: Optional[float] = None
    ) -> None:
        """Process callbacks until a ROS future completes.

        Args:
            future: The ROS Future to wait for.
            timeout_sec: Maximum time to wait in seconds.
        """
        future.add_done_callback(lambda x: self.wake())
        await self._spin_once_impl(timeout_sec, future.done)

    async def spin(self) -> None:
        """Spin the executor asynchronously until shutdown.

        Continuously processes callbacks until the ROS context is invalid
        or shutdown is requested.
        """
        await self._spin_impl()

    async def spin_until_future_complete(
        self, future: Future, timeout_sec: Optional[float] = None
    ) -> None:
        """Spin until a ROS future completes or timeout.

        Args:
            future: The ROS Future to wait for.
            timeout_sec: Maximum time to wait in seconds.
        """
        future.add_done_callback(lambda x: self.wake())
        await self._spin_impl(timeout_sec, future.done)

    def shutdown(self, timeout_sec: Optional[float] = None) -> bool:
        """Shutdown the executor and clean up resources.

        Shuts down the thread pool, cancels pending asyncio futures,
        and calls the parent shutdown method.

        Args:
            timeout_sec: Maximum time to wait for shutdown in seconds.

        Returns:
            True if shutdown completed successfully.
        """
        success = super().shutdown(timeout_sec)

        if self._multi_threaded:
            self._tpe.shutdown(wait=False, cancel_futures=True)

        for future in self._aio_futures:
            future.cancel()
        del self._aio_futures

        return success


class TestExecutor(SingleThreadedExecutor):
    """Simple executor wrapper for debugging purposes.

    Wraps the standard SingleThreadedExecutor with print statements
    to indicate when spinning starts and stops.
    """

    def spin(self) -> None:
        """Spin with debug output.

        Prints messages before and after the spin operation for debugging.
        """
        print("Spinning")
        super().spin()
        print("Spinning done")
