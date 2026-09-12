"""Regression tests for task-scoped synchronization pulses."""

import asyncio
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from tabletop_interfaces.msg import TaskEvent
from tabletop_tasks.run import run_tasks


class FakeCommander:
    def __init__(self) -> None:
        self.sync_calls: list[tuple[bool, int | float | None]] = []
        self.start_state_checks = 0
        self.task_events: list[tuple[int, dict]] = []
        self.eyelink_waits: list[float] = []

    def validate_task_start_state(self) -> None:
        self.start_state_checks += 1

    async def set_sync_pulses(
        self, enable: bool, duration: int | float | None = None
    ) -> None:
        self.sync_calls.append((enable, duration))

    async def wait_for_eyelink_recorder(self, timeout: float) -> None:
        self.eyelink_waits.append(timeout)

    def publish_task_event(self, event_type: int, **kwargs) -> None:
        self.task_events.append((event_type, kwargs))

    def log(self, *_args, **_kwargs) -> None:
        pass


def _write_config(tmp_path) -> str:
    config = tmp_path / "task.yaml"
    config.write_text("tasks:\n  - class: FakeTask\n    kwargs: {}\n")
    return str(config)


def test_sync_pulses_span_only_successful_task_execution(tmp_path) -> None:
    commander = FakeCommander()

    class FakeTask:
        def __init__(self, commander, **_kwargs) -> None:
            self.commander = commander

        async def run(self) -> None:
            assert self.commander.sync_calls == [(True, None)]

    with patch(
        "tabletop_tasks.run.importlib.import_module",
        return_value=SimpleNamespace(FakeTask=FakeTask),
    ):
        asyncio.run(run_tasks(commander, _write_config(tmp_path)))

    assert commander.sync_calls == [(True, None), (False, None)]
    assert commander.start_state_checks == 1
    assert [event[0] for event in commander.task_events] == [
        TaskEvent.TASK_STARTED,
        TaskEvent.TASK_ENDED,
    ]
    assert commander.task_events[0][1]["session_id"]


def test_sync_pulses_stop_when_task_raises(tmp_path) -> None:
    commander = FakeCommander()

    class FakeTask:
        def __init__(self, commander, **_kwargs) -> None:
            self.commander = commander

        async def run(self) -> None:
            assert self.commander.sync_calls == [(True, None)]
            raise RuntimeError("synthetic task failure")

    with (
        patch(
            "tabletop_tasks.run.importlib.import_module",
            return_value=SimpleNamespace(FakeTask=FakeTask),
        ),
        pytest.raises(RuntimeError, match="synthetic task failure"),
    ):
        asyncio.run(run_tasks(commander, _write_config(tmp_path)))

    assert commander.sync_calls == [(True, None), (False, None)]
    assert commander.start_state_checks == 1
    assert [event[0] for event in commander.task_events] == [
        TaskEvent.TASK_STARTED,
        TaskEvent.TASK_ENDED,
    ]


def test_start_state_failure_prevents_sync_and_task_start(tmp_path) -> None:
    commander = FakeCommander()

    def fail_start_state_check() -> None:
        commander.start_state_checks += 1
        raise RuntimeError("unsafe startup pose")

    commander.validate_task_start_state = fail_start_state_check

    with pytest.raises(RuntimeError, match="unsafe startup pose"):
        asyncio.run(run_tasks(commander, _write_config(tmp_path)))

    assert commander.start_state_checks == 1
    assert commander.sync_calls == []
    assert commander.task_events == []


def test_eyelink_recorder_is_confirmed_before_sync_and_task(
    tmp_path, monkeypatch
) -> None:
    commander = FakeCommander()
    monkeypatch.setenv("TABLETOP_REQUIRE_EYELINK_RECORDER", "true")
    monkeypatch.setenv("TABLETOP_EYELINK_RECORDER_TIMEOUT", "7.5")

    class FakeTask:
        def __init__(self, commander, **_kwargs) -> None:
            self.commander = commander

        async def run(self) -> None:
            assert self.commander.eyelink_waits == [7.5]
            assert self.commander.sync_calls == [(True, None)]

    with patch(
        "tabletop_tasks.run.importlib.import_module",
        return_value=SimpleNamespace(FakeTask=FakeTask),
    ):
        asyncio.run(run_tasks(commander, _write_config(tmp_path)))

    assert commander.eyelink_waits == [7.5]


def test_eyelink_recorder_failure_prevents_sync_and_task(
    tmp_path, monkeypatch
) -> None:
    commander = FakeCommander()
    monkeypatch.setenv("TABLETOP_REQUIRE_EYELINK_RECORDER", "true")

    async def fail_recorder_check(timeout: float) -> None:
        commander.eyelink_waits.append(timeout)
        raise RuntimeError("no rosbag recorder")

    commander.wait_for_eyelink_recorder = fail_recorder_check

    with pytest.raises(RuntimeError, match="no rosbag recorder"):
        asyncio.run(run_tasks(commander, _write_config(tmp_path)))

    assert commander.eyelink_waits == [10.0]
    assert commander.sync_calls == []
    assert commander.task_events == []
