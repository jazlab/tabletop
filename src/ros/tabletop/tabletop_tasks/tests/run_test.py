"""Regression tests for task-scoped synchronization pulses."""

import asyncio
import os
import time
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import yaml
from tabletop_interfaces.msg import TaskEvent
from tabletop_tasks.run import _wait_for_flir_preflight, run_tasks


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


def test_camera_only_report_cannot_release_recorded_task(
    tmp_path, monkeypatch
):
    session = tmp_path / "latest"
    session.mkdir()
    report_path = session / "flir_preflight.yaml"
    report_path.write_text(yaml.safe_dump({"status": "PASS"}))
    monkeypatch.setenv("ROS_BAG_DIR", str(tmp_path))
    monkeypatch.setenv("TABLETOP_TASK_LAUNCH_EPOCH", "0")
    monkeypatch.setenv("TABLETOP_FLIR_PREFLIGHT_TIMEOUT", "0.01")
    with pytest.raises(RuntimeError, match="did not pass"):
        asyncio.run(_wait_for_flir_preflight(FakeCommander()))
    report_path.write_text(
        yaml.safe_dump(
            {"status": "PASS", "robot_recording_preflight": {"status": "PASS"}}
        )
    )
    monkeypatch.setenv("TABLETOP_FLIR_PREFLIGHT_TIMEOUT", "1")
    asyncio.run(_wait_for_flir_preflight(FakeCommander()))


def test_recorded_task_waits_for_fresh_combined_preflight(
    tmp_path, monkeypatch
):
    session = tmp_path / "latest"
    session.mkdir()
    report = session / "flir_preflight.yaml"
    report.write_text(
        yaml.safe_dump(
            {"status": "PASS", "robot_recording_preflight": {"status": "PASS"}}
        )
    )
    epoch = time.time()
    os.utime(report, (epoch - 5, epoch - 5))
    monkeypatch.setenv("ROS_BAG_DIR", str(tmp_path))
    monkeypatch.setenv("TABLETOP_TASK_LAUNCH_EPOCH", str(epoch))
    monkeypatch.setenv("TABLETOP_REQUIRE_FLIR_PREFLIGHT", "true")
    monkeypatch.setenv("TABLETOP_REQUIRE_EYELINK_RECORDER", "true")
    monkeypatch.setenv("TABLETOP_FLIR_PREFLIGHT_TIMEOUT", "2")
    commander = FakeCommander()

    class FakeTask:
        def __init__(self, commander, **_kwargs):
            self.commander = commander

        async def run(self):
            assert self.commander.eyelink_waits == [10.0]
            assert self.commander.sync_calls == [(True, None)]

    async def exercise():
        task = asyncio.create_task(
            run_tasks(commander, _write_config(tmp_path))
        )
        try:
            await asyncio.sleep(0.03)
            assert not task.done()  # Previous session's PASS is insufficient.
            assert commander.sync_calls == []
            assert commander.task_events == []
            report.write_text(yaml.safe_dump({"status": "PASS"}))
            await asyncio.sleep(0.15)
            assert not task.done()  # Camera PASS without recorder readiness.
            assert commander.eyelink_waits == []
            assert commander.sync_calls == []
            assert commander.task_events == []
            report.write_text(
                yaml.safe_dump(
                    {
                        "status": "PASS",
                        "robot_recording_preflight": {"status": "PASS"},
                    }
                )
            )
            await asyncio.wait_for(task, timeout=2)
        finally:
            if not task.done():
                task.cancel()
                with pytest.raises(asyncio.CancelledError):
                    await task

    with patch(
        "tabletop_tasks.run.importlib.import_module",
        return_value=SimpleNamespace(FakeTask=FakeTask),
    ):
        asyncio.run(exercise())
    assert [event[0] for event in commander.task_events] == [
        TaskEvent.TASK_STARTED,
        TaskEvent.TASK_ENDED,
    ]


@pytest.mark.parametrize("report_status", [None, "FAIL", "stale"])
def test_missing_failed_or_stale_preflight_never_starts_task(
    tmp_path, monkeypatch, report_status
):
    session = tmp_path / "latest"
    session.mkdir()
    epoch = time.time()
    if report_status is not None:
        report = session / "flir_preflight.yaml"
        report.write_text(
            yaml.safe_dump(
                {
                    "status": "FAIL" if report_status == "FAIL" else "PASS",
                    "robot_recording_preflight": {"status": "PASS"},
                    "failures": ["synthetic readiness failure"],
                }
            )
        )
        if report_status == "stale":
            os.utime(report, (epoch - 5, epoch - 5))
    monkeypatch.setenv("ROS_BAG_DIR", str(tmp_path))
    monkeypatch.setenv("TABLETOP_TASK_LAUNCH_EPOCH", str(epoch))
    monkeypatch.setenv("TABLETOP_REQUIRE_FLIR_PREFLIGHT", "true")
    monkeypatch.setenv("TABLETOP_FLIR_PREFLIGHT_TIMEOUT", "0.01")
    commander = FakeCommander()
    with pytest.raises(RuntimeError, match="FLIR.*preflight"):
        asyncio.run(run_tasks(commander, _write_config(tmp_path)))
    assert commander.sync_calls == []
    assert commander.task_events == []


def test_waiting_for_preflight_remains_cancellable_without_outputs(
    tmp_path, monkeypatch
):
    (tmp_path / "latest").mkdir()
    monkeypatch.setenv("ROS_BAG_DIR", str(tmp_path))
    monkeypatch.setenv("TABLETOP_REQUIRE_FLIR_PREFLIGHT", "true")
    monkeypatch.setenv("TABLETOP_FLIR_PREFLIGHT_TIMEOUT", "10")
    commander = FakeCommander()

    async def exercise():
        task = asyncio.create_task(
            run_tasks(commander, _write_config(tmp_path))
        )
        await asyncio.sleep(0.02)
        assert not task.done()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await asyncio.wait_for(task, timeout=1)

    asyncio.run(exercise())
    assert commander.sync_calls == []
    assert commander.task_events == []
