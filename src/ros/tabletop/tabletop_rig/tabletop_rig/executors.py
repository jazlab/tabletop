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

import abc
import asyncio
import concurrent.futures
import contextlib
import inspect
import threading
from collections.abc import AsyncGenerator, Callable
from contextlib import (
    ExitStack,
)
from typing import TYPE_CHECKING, Any, Optional, Protocol

import rclpy.task
from rclpy.client import Client
from rclpy.exceptions import InvalidHandle
from rclpy.executors import (
    ConditionReachedException,
    Executor,
    ExternalShutdownException,
    MultiThreadedExecutor,
    ShutdownException,
    TimeoutException,
    TimeoutObject,
)
from rclpy.guard_condition import GuardCondition
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from rclpy.utilities import timeout_sec_to_nsec
from rclpy.waitable import NumberOfEntities, Waitable

if TYPE_CHECKING:
    from rclpy.node import Node


class WaitableEntityType(Protocol):
    callback_group: Any
    _executor_event: Any


async def _call_in_tpe(
    tpe: concurrent.futures.ThreadPoolExecutor,
    fn: Callable[..., Any],
    *args: Any,
    **kwargs: Any,
) -> Any:
    """Submit a blocking function to a thread pool and wait asynchronously.

    Wraps ThreadPoolExecutor.submit with asyncio.wrap_future to allow
    blocking operations to run in a separate thread while the event loop
    continues processing other tasks.

    Args:
        tpe: The ThreadPoolExecutor to submit work to.
        fn: The callable to execute.
        *args: Positional arguments for fn.
        **kwargs: Keyword arguments for fn.

    Returns:
        The result of fn(*args, **kwargs).
    """
    return await asyncio.wrap_future(tpe.submit(fn, *args, **kwargs))


def _call_task_fn(task: rclpy.task.Task) -> Any:
    """Execute a synchronous ROS task handler safely with locking.

    Acquires the task lock and executes the non-coroutine handler,
    setting the result or exception on the task. Does nothing if the
    task is not pending or already executing.

    Args:
        task: The ROS task to execute.

    Returns:
        The handler result, or None if the task was not pending.

    Raises:
        ValueError: If task._handler is a coroutine (should use
            _call_task_coro instead).
        Any exception raised by the handler.
    """
    if (
        not task._pending()
        or task._executing
        or not task._task_lock.acquire(blocking=False)
    ):
        return
    try:
        if inspect.iscoroutine(task._handler):
            raise ValueError(
                "task._handler should not be a coroutine function"
            )
        if not task._pending():
            return
        task._executing = True
        try:
            result = task._handler(*task._args, **task._kwargs)  # type: ignore
            task.set_result(result)
            return result
        except Exception as e:
            task.set_exception(e)
            raise
        finally:
            task._complete_task()
            task._executing = False
    finally:
        task._task_lock.release()


async def _call_task_coro(task: rclpy.task.Task) -> Any:
    """Execute a coroutine ROS task handler safely with locking.

    Acquires the task lock and awaits the coroutine handler, setting
    the result or exception on the task. Does nothing if the task is
    not pending or already executing.

    Args:
        task: The ROS task with a coroutine handler to execute.

    Returns:
        The coroutine result, or None if the task was not pending.

    Raises:
        ValueError: If task._handler is not a coroutine (should use
            _call_task_fn instead).
        Any exception raised by the coroutine.
    """
    if (
        not task._pending()
        or task._executing
        or not task._task_lock.acquire(blocking=False)
    ):
        return
    try:
        if not inspect.iscoroutine(task._handler):
            raise ValueError("task._handler should be a coroutine function")
        if not task._pending():
            return
        task._executing = True
        try:
            result = await task._handler
            task.set_result(result)
            return result
        except Exception as e:
            task.set_exception(e)
            raise
        finally:
            task._complete_task()
            task._executing = False
    finally:
        task._task_lock.release()


class ErrorHandlingMultiThreadedExecutor(MultiThreadedExecutor):
    """MultiThreadedExecutor variant that collects and raises task exceptions.

    Unlike the base MultiThreadedExecutor which silently discards exceptions
    from submitted futures, this variant collects them and raises an
    ExceptionGroup on shutdown or when all futures are processed.
    """

    def _spin_once_impl(
        self,
        timeout_sec: Optional[float | TimeoutObject] = None,
        wait_condition: Callable[[], bool] = lambda: False,
    ) -> None:
        """Spin once, collecting exceptions from all completed futures.

        Args:
            timeout_sec: Maximum time to wait for callbacks.
            wait_condition: Stop spinning if this callable returns True.

        Raises:
            ExceptionGroup: If any future completed with an exception.
        """
        try:
            handler, entity, node = self.wait_for_ready_callbacks(
                timeout_sec,
                None,
                condition=lambda: (
                    any(f.done() for f in self._futures) or wait_condition()
                ),
            )
        except (
            ExternalShutdownException,
            ShutdownException,
            TimeoutException,
            ConditionReachedException,
        ):
            pass
        else:
            self._executor.submit(handler).add_done_callback(
                lambda _: self.wake()
            )
            self._futures.append(handler)

        # make a copy of the list that we iterate over while modifying it
        # (https://stackoverflow.com/q/1207406/3753684)
        excs: list[Exception] = []
        for future in self._futures[:]:
            if future.done():
                self._futures.remove(future)
                try:
                    future.result()  # raise any exceptions
                except Exception as e:
                    excs.append(e)
        if len(excs) > 0:
            raise ExceptionGroup("Unhandled Task exceptions", excs)

    def shutdown(self, timeout_sec: float | None = None) -> bool:
        """Shut down the thread pool and base executor.

        Args:
            timeout_sec: Maximum time to wait for threads to finish.

        Returns:
            True if shutdown completed within timeout.
        """
        self._executor.shutdown()
        return super().shutdown(timeout_sec)


class _BaseAIOExecutor(Executor, metaclass=abc.ABCMeta):
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
    """

    def __init__(
        self,
        *args: Any,
        multi_threaded: bool = False,
        max_workers: int = 1,
        eager_task_factory: bool = True,
        **kwargs: Any,
    ) -> None:
        """Initialize the asyncio executor.

        Creates a base ROS2 Executor and sets up parameters for async spinning.
        When spin() is called, the executor will use asyncio.TaskGroup to manage
        coroutines and a ThreadPoolExecutor for blocking ROS wait operations.

        Args:
            *args: Arguments passed to the base Executor constructor.
            multi_threaded: If True, non-coroutine ROS callbacks execute in the
                thread pool, allowing parallel execution. If False (default),
                callbacks execute sequentially in the event loop.
            max_workers: Maximum thread pool size. Only meaningful when
                multi_threaded is True. Defaults to 1 (sequential execution).
            eager_task_factory: If True, use asyncio.eager_task_factory to
                execute tasks eagerly instead of deferring. Defaults to True.
            **kwargs: Keyword arguments passed to the base Executor constructor.

        Raises:
            ValueError: If max_workers > 1 but multi_threaded is False.
        """
        super().__init__(*args, **kwargs)
        if not multi_threaded and max_workers > 1:
            raise ValueError(
                "max_workers must be 1 if multi_threaded is False"
            )
        self._multi_threaded: bool = multi_threaded
        self._max_workers: int = max_workers
        self._eager_task_factory: bool = eager_task_factory

    @contextlib.contextmanager
    def _spin_context_manager(self):
        """Context manager for ROS executor spin lifecycle.

        Calls _enter_spin() on entry and _exit_spin() on exit to manage
        the executor's internal state during spinning.
        """
        self._enter_spin()
        try:
            yield
        finally:
            self._exit_spin()

    @contextlib.contextmanager
    def _eager_task_context_manager(self):
        """Context manager for eager task execution.

        Temporarily replaces the asyncio task factory with
        asyncio.eager_task_factory to execute tasks eagerly instead of
        deferring them. Restores the original factory on exit.
        """
        loop = asyncio.get_running_loop()
        old_task_factory = loop.get_task_factory()
        loop.set_task_factory(asyncio.eager_task_factory)
        try:
            yield
        finally:
            loop.set_task_factory(old_task_factory)

    @contextlib.asynccontextmanager
    async def _spin_context_stack(self):
        """Set up context for spinning with TaskGroup and ThreadPoolExecutor.

        Enters the ROS executor spin context, optionally activates eager task
        factory, creates an asyncio.TaskGroup for managing async work, and
        creates a ThreadPoolExecutor for blocking ROS wait operations.

        Yields:
            An AsyncExitStack managing all the above contexts.
        """
        async with contextlib.AsyncExitStack() as stack:
            stack.enter_context(self._spin_context_manager())
            if self._eager_task_factory:
                stack.enter_context(self._eager_task_context_manager())
            stack.callback(lambda: delattr(self, "_tg"))
            self._tg = await stack.enter_async_context(asyncio.TaskGroup())
            stack.callback(lambda: delattr(self, "_tpe"))
            self._tpe = stack.enter_context(
                concurrent.futures.ThreadPoolExecutor(
                    max_workers=self._max_workers
                )
            )
            yield stack

    def _schedule_or_call(self, handler: rclpy.task.Task):
        """Schedule a ROS task handler for execution.

        Routes the handler based on its type:
        - Coroutine handlers: create asyncio task with _call_task_coro
        - Sync handlers with multi_threaded=True: create asyncio task that
          runs the handler in the thread pool via _call_in_tpe
        - Sync handlers with multi_threaded=False: execute immediately via
          _call_task_fn

        Args:
            handler: The ROS task to schedule.
        """
        if asyncio.iscoroutine(handler._handler):
            self._tg.create_task(_call_task_coro(handler))
        elif self._multi_threaded:
            self._tg.create_task(
                _call_in_tpe(self._tpe, _call_task_fn(handler))
            )
        else:
            _call_task_fn(handler)

    @abc.abstractmethod
    async def _spin_impl(
        self,
        timeout_sec: Optional[float] = None,
        wait_condition: Callable[[], bool] = lambda: False,
    ) -> None:
        """Internal implementation of continuous spinning.

        Starts a work waiter thread then creates asyncio Tasks for each rclpy
        handler. Runs until shutdown, timeout, or the wait condition is met.

        Args:
            timeout_sec: Maximum total spin time in seconds.
            wait_condition: Callable for early termination condition.
        """

    async def spin(self) -> None:
        """Spin the executor asynchronously until shutdown.

        Continuously processes ROS callbacks and executes scheduled tasks
        until the ROS context is invalid or shutdown is requested.
        Bridges ROS2's callback model with asyncio's event loop.
        """
        await self._spin_impl()

    async def spin_until_future_complete(
        self, future: rclpy.task.Future, timeout_sec: Optional[float] = None
    ) -> None:
        """Spin until a ROS future completes or timeout expires.

        Registers a callback on the future to wake the executor when it
        completes, then spins with a wait condition that checks the future.

        Args:
            future: The ROS Future to wait for.
            timeout_sec: Maximum time to wait in seconds. If None, waits
                indefinitely.

        Raises:
            TimeoutError: If timeout expires before future completes.
        """
        future.add_done_callback(lambda _: self.wake())
        await self._spin_impl(
            timeout_sec=timeout_sec,
            wait_condition=lambda: future.done() or future.cancelled(),
        )


class _AIOExecutor(_BaseAIOExecutor):
    """Asyncio executor using queue-based producer-consumer pattern.

    Spawns a producer thread that calls the blocking
    wait_for_ready_callbacks() and queues results for the main event
    loop to process via _spin_impl.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Initialize the queue-based executor."""
        super().__init__(*args, **kwargs)
        self._queue: asyncio.Queue[
            tuple[rclpy.task.Task, WaitableEntityType, "Node"]
        ] = asyncio.Queue()

    def _queue_producer(
        self,
        timeout_sec: Optional[float] = None,
        wait_condition: Callable[[], bool] = lambda: False,
        *,
        loop: asyncio.AbstractEventLoop,
        cancel_event: threading.Event,
    ) -> None:
        """Blocking producer thread function.

        Repeatedly calls wait_for_ready_callbacks and queues the result
        using loop.call_soon_threadsafe. Exits when cancel_event is set
        or the wait condition becomes true.

        Args:
            timeout_sec: Timeout for wait_for_ready_callbacks.
            wait_condition: Exit when this callable returns True.
            loop: The asyncio event loop to queue results to.
            cancel_event: Threading event to signal producer to exit.
        """
        try:
            while not cancel_event.is_set():
                ready = self.wait_for_ready_callbacks(
                    timeout_sec=timeout_sec,
                    nodes=None,
                    condition=lambda: (
                        cancel_event.is_set() or wait_condition()
                    ),
                )
                loop.call_soon_threadsafe(self._queue.put_nowait, ready)
        except BaseException as e:
            print(f"{type(e).__name__} in AIOExecutor._queue_producer: {e}")
            raise

    async def _spin_impl(
        self,
        timeout_sec: Optional[float] = None,
        wait_condition: Callable[[], bool] = lambda: False,
    ) -> None:
        """Spin using queue-based producer-consumer pattern.

        Spawns a producer task that runs _queue_producer in the thread pool,
        continuously queuing ready handlers. The main loop gets handlers from
        the queue and schedules them for execution.

        Args:
            timeout_sec: Timeout for wait_for_ready_callbacks.
            wait_condition: Stop spinning if this callable returns True.
        """
        async with self._spin_context_stack():
            loop = asyncio.get_running_loop()
            cancel_event = threading.Event()
            self._tg.create_task(
                _call_in_tpe(
                    self._tpe,
                    self._queue_producer,
                    timeout_sec=timeout_sec,
                    wait_condition=wait_condition,
                    loop=loop,
                    cancel_event=cancel_event,
                )
            )
            try:
                while (
                    self._context.ok()
                    and not wait_condition()
                    and not self._is_shutdown
                ):
                    handler, _, _ = await self._queue.get()
                    self._schedule_or_call(handler)
            finally:
                cancel_event.set()
                self.wake()


class _AIOExecutorOptimized(_BaseAIOExecutor):
    """Asyncio executor with optimized async wait-set handling.

    Instead of spawning a producer thread, this executor directly awaits
    wait_for_ready_callbacks by running it in the thread pool with
    asyncio.wrap_future. This allows more direct integration with the
    event loop.
    """

    async def _wait_for_ready_callbacks_async(
        self,
        timeout_sec: Optional[float | TimeoutObject] = None,
        nodes: Optional[list["Node"]] = None,
        condition: Callable[[], bool] = lambda: False,
    ) -> AsyncGenerator[
        tuple[rclpy.task.Task, Optional[WaitableEntityType], Optional["Node"]],
        None,
    ]:
        """Async generator yielding ready handlers from the wait set.

        Constructs a wait set from all executable ROS entities and waits
        on it (blocking in thread pool), then yields ready handlers one
        at a time. Lazily regenerates the wait set if called with new args.

        Args:
            timeout_sec: Timeout for wait set wait.
            nodes: Nodes to check for executable entities. If None, checks
                all registered nodes.
            condition: Stop waiting if this callable returns True.

        Yields:
            Tuples of (handler, entity, node) for ready callbacks.

        Raises:
            TimeoutException: If timeout expires.
            ShutdownException: If ROS context is shutdown.
            ExternalShutdownException: If ROS context becomes invalid.
        """
        timeout_timer = None
        timeout_nsec = timeout_sec_to_nsec(
            timeout_sec.timeout
            if isinstance(timeout_sec, TimeoutObject)
            else timeout_sec
        )
        if timeout_nsec > 0:
            timeout_timer = Timer(
                None,  # type: ignore
                None,  # type: ignore
                timeout_nsec,
                self._clock,
                context=self._context,  # type: ignore
            )

        yielded_work = False
        while not yielded_work and not self._is_shutdown and not condition():
            # Refresh "all" nodes in case executor was woken by a node being added or removed
            nodes_to_use = nodes
            if nodes is None:
                nodes_to_use = self.get_nodes()
            assert nodes_to_use is not None

            # Yield tasks in-progress before waiting for new work
            with self._tasks_lock:
                # Get rid of any tasks that are done or cancelled
                for task in list(self._pending_tasks.keys()):
                    if task.done() or task.cancelled():
                        del self._pending_tasks[task]

                ready_tasks_count = len(self._ready_tasks)
            for _ in range(ready_tasks_count):
                task = self._ready_tasks.popleft()
                task_data = self._pending_tasks[task]
                node = task_data.source_node
                if node is None or node in nodes_to_use:
                    entity = task_data.source_entity
                    yielded_work = True
                    yield task, entity, node  # type: ignore
                else:
                    # Asked not to execute these tasks, so don't do them yet
                    with self._tasks_lock:
                        self._ready_tasks.append(task)
            # Gather entities that can be waited on
            subscriptions: list[Subscription] = []
            guards: list[GuardCondition] = []
            timers: list[Timer] = []
            clients: list[Client] = []
            services: list[Service] = []
            waitables: list[Waitable] = []
            for node in nodes_to_use:
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
            if timeout_timer is not None:
                timers.append(timeout_timer)

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
            wait_set = None
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
                # future = None
                try:
                    future = self._tpe.submit(wait_set.wait, timeout_nsec)
                    await asyncio.wrap_future(future)
                except asyncio.CancelledError:
                    # Wake the executor to join the thread
                    self.wake()
                    # if future is not None:
                    #     if not future.cancel():
                    #         future.result(timeout=WAIT_SET_CLEANUP_TIMEOUT_SEC)
                    raise
                except BaseException as e:
                    print(e)
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
                for node in nodes_to_use:
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
            for node in nodes_to_use:
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

            # Check timeout timer
            if timeout_nsec == 0 or (
                timeout_timer is not None
                and timeout_timer.handle.pointer in timers_ready
            ):
                raise TimeoutException()
        if self._is_shutdown:
            raise ShutdownException()
        if condition():
            raise ConditionReachedException()

    async def wait_for_ready_callbacks_async(
        self, *args, **kwargs
    ) -> tuple[
        rclpy.task.Task, Optional[WaitableEntityType], Optional["Node"]
    ]:
        """Get the next ready handler from the wait set.

        Caches and reuses the async generator from
        _wait_for_ready_callbacks_async, recreating it if arguments change.
        Lazily recreates when the generator is exhausted.

        Args:
            *args: Arguments passed to _wait_for_ready_callbacks_async.
            **kwargs: Keyword arguments passed to _wait_for_ready_callbacks_async.

        Returns:
            Tuple of (handler, entity, node) for a ready callback.

        Raises:
            TimeoutException, ShutdownException, etc. from the wait set.
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
                self._cb_iter = self._wait_for_ready_callbacks_async(
                    *args, **kwargs
                )
            try:
                return await self._cb_iter.__anext__()
            except StopAsyncIteration:
                # Generator ran out of work
                self._cb_iter = None

    async def _spin_impl(
        self,
        timeout_sec: Optional[float] = None,
        wait_condition: Callable[[], bool] = lambda: False,
    ) -> None:
        """Spin using optimized async wait-set handling.

        Continuously awaits wait_for_ready_callbacks_async (which runs
        wait_set.wait in the thread pool) and schedules ready handlers.
        More efficient than _AIOExecutor's separate producer thread.

        Args:
            timeout_sec: Timeout for wait set wait.
            wait_condition: Stop spinning if this callable returns True.
        """
        async with self._spin_context_stack():
            while (
                self._context.ok()
                and not wait_condition()
                and not self._is_shutdown
            ):
                handler, _, _ = await self.wait_for_ready_callbacks_async(
                    timeout_sec=timeout_sec,
                    nodes=None,
                    condition=wait_condition,
                )
                self._schedule_or_call(handler)


AIOExecutor = _AIOExecutorOptimized  # Default to optimized variant
