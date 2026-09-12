import numpy as np
import pytest
from tabletop_interfaces.msg import TaskEvent
from tabletop_rig.nodes.session_dashboard import (
    CAMERAS,
    HEIGHT,
    WIDTH,
    SessionHealthState,
    StopControlSnapshot,
    render_dashboard,
)


def _event(
    event_type,
    stamp,
    *,
    session_id="session-1",
    trial=-1,
    group="",
    object_id="",
):
    message = TaskEvent()
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
    message.event_type = event_type
    message.session_id = session_id
    message.trial_number = trial
    message.group_name = group
    message.object_id = object_id
    return message


def test_session_metrics_persist_until_next_start(tmp_path):
    path = tmp_path / "health.json"
    state = SessionHealthState(path)
    state.record_event(_event(TaskEvent.TASK_STARTED, 100.0), now=1.0)

    state.record_event(
        _event(TaskEvent.TRIAL_DEPRESENTING, 110.0, group="left_manipulator")
    )
    state.record_event(
        _event(
            TaskEvent.TRIAL_PRESENTED,
            114.0,
            trial=1,
            group="right_manipulator",
        )
    )
    state.record_event(
        _event(TaskEvent.TRIAL_DEPRESENTING, 120.0, group="right_manipulator")
    )
    state.record_event(
        _event(
            TaskEvent.TRIAL_PRESENTED, 127.0, trial=2, group="left_manipulator"
        )
    )
    state.record_event(
        _event(TaskEvent.TRIAL_DEPRESENTING, 130.0, group="left_manipulator")
    )
    state.record_event(
        _event(
            TaskEvent.TRIAL_PRESENTED,
            139.0,
            trial=3,
            group="right_manipulator",
        )
    )
    state.record_event(
        _event(TaskEvent.OBJECT_MISS, 142.0, object_id="object_4")
    )
    state.record_event(_event(TaskEvent.TASK_ENDED, 150.0), now=2.0)

    snapshot = state.snapshot(now=3.0, now_epoch=500.0)
    assert snapshot.status == "COMPLETE"
    assert snapshot.duration_seconds == pytest.approx(50.0)
    assert snapshot.trial_count == 3
    assert snapshot.object_misses == 1
    assert snapshot.average_iti_seconds == pytest.approx(20.0 / 3.0)
    assert snapshot.left_lap_seconds == pytest.approx(17.0)
    assert snapshot.right_lap_seconds == pytest.approx(19.0)

    restored = SessionHealthState(path).snapshot()
    assert restored.session_id == "session-1"
    assert restored.trial_count == 3
    assert restored.object_misses == 1

    state.record_event(
        _event(TaskEvent.TASK_STARTED, 200.0, session_id="session-2"),
        now=4.0,
    )
    fresh = state.snapshot(now=5.0, now_epoch=201.0)
    assert fresh.status == "RUNNING"
    assert fresh.session_id == "session-2"
    assert fresh.trial_count == 0
    assert not path.exists()


def test_transport_loss_baseline(tmp_path):
    state = SessionHealthState(tmp_path / "health.json")
    topic = "/cam_sync/left_front_top_cam/meta"
    state.record_transport_log(
        f"Messages lost on transport layer for topic '{topic}'. Total lost: 5"
    )
    state.record_event(_event(TaskEvent.TASK_STARTED, 100.0), now=1.0)
    state.record_transport_log(
        f"Messages lost on transport layer for topic '{topic}'. Total lost: 8"
    )

    camera = CAMERAS[0]
    state.record_component(camera, "metadata", now=1.3)
    state.record_transport_log(
        f"Messages lost on transport layer for topic '{topic}'. Total lost: 2"
    )

    row = state.snapshot(now=1.4, now_epoch=101.0).rows[camera]
    assert row.transport_lost == 5
    assert not hasattr(row, "estimated_lost")
    assert row.streaming == "LIVE"


def test_ttl_miss_requires_full_minute_without_a_match(tmp_path):
    state = SessionHealthState(tmp_path / "health.json")
    state.record_event(_event(TaskEvent.TASK_STARTED, 100.0), now=1.0)
    state.record_reference_rise(101.0)
    state.record_ttl_state("eyelink", False, 101.02)
    state.record_ttl_state("eyelink", True, 101.03)
    for reference in range(102, 161):
        state.record_reference_rise(float(reference))

    row = state.snapshot(now=2.0, now_epoch=160.5).rows["eyelink"]
    assert row.ttl_status == "INTACT"
    assert "past 60 s" in row.ttl_detail

    row = state.snapshot(now=3.0, now_epoch=161.5).rows["eyelink"]
    assert row.ttl_status == "MISS"
    assert row.ttl_detail == "No matched pulses in past 60 s"


def test_ttl_waits_during_initial_health_window(tmp_path):
    state = SessionHealthState(tmp_path / "health.json")
    state.record_event(_event(TaskEvent.TASK_STARTED, 100.0), now=1.0)
    state.record_reference_rise(101.0)

    row = state.snapshot(now=2.0, now_epoch=120.0).rows["eyelink"]
    assert row.ttl_status == "WAITING"
    assert "Checking TTL" in row.ttl_detail


def test_disk_capacity_and_render(tmp_path):
    state = SessionHealthState(tmp_path / "health.json")
    state.update_disk(tmp_path)
    snapshot = state.snapshot(now=1.0, now_epoch=1.0)

    assert snapshot.disk.path == str(tmp_path)
    assert snapshot.disk.total > 0
    assert snapshot.disk.free >= 0

    frame = render_dashboard(snapshot)
    assert frame.shape == (HEIGHT, WIDTH, 3)


def test_stop_control_render_states(tmp_path):
    state = SessionHealthState(tmp_path / "health.json")
    state.record_event(_event(TaskEvent.TASK_STARTED, 100.0), now=1.0)
    snapshot = state.snapshot(now=2.0, now_epoch=101.0)

    ready = render_dashboard(
        snapshot, StopControlSnapshot(online=True, pending=False)
    )
    pending = render_dashboard(
        snapshot, StopControlSnapshot(online=True, pending=True)
    )

    assert ready.shape == (HEIGHT, WIDTH, 3)
    assert pending.shape == (HEIGHT, WIDTH, 3)
    assert not np.array_equal(ready, pending)
