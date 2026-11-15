"""Capture completion must not depend on CSV, and failures cannot report success."""

import json
import threading
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest
from std_srvs.srv import Trigger
from tabletop_rig.nodes import eyelink


def finalizing_node(tmp_path):
    calls = []
    node = SimpleNamespace(
        _retrieval_lock=threading.Lock(),
        _retrieving=True,
        _last_finalization_error=None,
        _active_session_dir=str(tmp_path),
        _last_retrieval_stats=eyelink.EyelinkRetrievalStats(
            samples=3, published_samples=3, batches=1
        ),
        simulate=False,
        log=Mock(),
        tracker=SimpleNamespace(
            stopRecording=lambda: calls.append("tracker_stop")
        ),
        _stop_sample_retrieval_loop=lambda: calls.append("drain_and_flush"),
    )
    edf = tmp_path / "eyelink_received" / "last.edf"

    def transfer(*, save):
        assert save
        edf.parent.mkdir()
        edf.write_bytes(b"saved EDF")
        calls.append("edf_saved")
        return str(edf)

    def queue(path):
        assert path == str(edf)
        report = json.loads((tmp_path / "eyelink_integrity.json").read_text())
        assert report["status"] == "complete" and report["edf_saved"]
        assert report["stats"]["batches"] == 1
        calls.append("csv_queued_after_report")

    node._close_data_file = transfer
    node._queue_csv_conversion = Mock(side_effect=queue)
    node._write_retrieval_report = MethodType(
        eyelink.Eyelink._write_retrieval_report, node
    )
    node.stop_retrieval = MethodType(eyelink.Eyelink.stop_retrieval, node)
    return node, calls


def test_stop_acknowledges_drain_edf_report_without_waiting_for_csv(tmp_path):
    node, calls = finalizing_node(tmp_path)
    response = eyelink.Eyelink.stop_recording_callback(
        node, None, Trigger.Response()
    )
    assert response.success
    assert "CSV is separate" in response.message
    assert calls == [
        "drain_and_flush",
        "tracker_stop",
        "edf_saved",
        "csv_queued_after_report",
    ]
    # An acknowledged stop is idempotent, not a duplicate EDF/CSV job.
    assert node.stop_retrieval(force=True)
    assert len(calls) == 4


@pytest.mark.parametrize("phase", ["drain", "transfer", "report"])
def test_critical_failure_returns_false_and_does_not_queue_csv(
    tmp_path, phase
):
    node, calls = finalizing_node(tmp_path)
    failure = Mock(side_effect=RuntimeError(f"{phase} failed"))
    if phase == "drain":
        node._stop_sample_retrieval_loop = failure
    elif phase == "transfer":
        node._close_data_file = failure
    else:
        node._write_retrieval_report = Mock(return_value=False)
    response = eyelink.Eyelink.stop_recording_callback(
        node, None, Trigger.Response()
    )
    assert not response.success
    node._queue_csv_conversion.assert_not_called()
    if phase == "drain":
        assert (
            node._retrieving
        )  # Cannot start a second loop over a pending drain.
        assert "tracker_stop" not in calls
    else:
        assert not node.stop_retrieval(
            force=True
        )  # Repeat cannot hide a failure.


def test_conversion_queue_failure_does_not_invalidate_saved_capture(tmp_path):
    node, _ = finalizing_node(tmp_path)
    node._queue_csv_conversion.side_effect = RuntimeError("worker unavailable")
    response = eyelink.Eyelink.stop_recording_callback(
        node, None, Trigger.Response()
    )
    assert response.success
    assert json.loads((tmp_path / "eyelink_integrity.json").read_text())[
        "edf_saved"
    ]


@pytest.mark.parametrize(
    "failure", ["exception", "negative", "empty", "size_mismatch"]
)
def test_failed_transfer_never_leaves_final_edf(tmp_path, failure):
    def receive(_name, path):
        Path(path).write_bytes(b"" if failure == "empty" else b"partial")
        if failure == "exception":
            raise RuntimeError("connection failed")
        return (
            -1
            if failure == "negative"
            else 100
            if failure == "size_mismatch"
            else 0
        )

    node = SimpleNamespace(
        tracker=SimpleNamespace(
            setOfflineMode=Mock(),
            closeDataFile=Mock(),
            receiveDataFile=receive,
        ),
        _active_session_dir=str(tmp_path),
        session_bag_dir=str(tmp_path),
        log=Mock(),
    )
    with pytest.raises(eyelink.DataFileReceiveError):
        eyelink.Eyelink._close_data_file(node, save=True)
    assert not (tmp_path / "eyelink_received/last.edf").exists()
    assert not list((tmp_path / "eyelink_received").glob("*.part"))


def test_transfer_refuses_to_overwrite_an_existing_edf(tmp_path):
    path = tmp_path / "eyelink_received/last.edf"
    path.parent.mkdir()
    path.write_bytes(b"original")
    tracker = SimpleNamespace(
        setOfflineMode=Mock(), closeDataFile=Mock(), receiveDataFile=Mock()
    )
    node = SimpleNamespace(
        tracker=tracker,
        _active_session_dir=str(tmp_path),
        session_bag_dir=str(tmp_path),
        log=Mock(),
    )
    with pytest.raises(
        eyelink.DataFileReceiveError, match="refusing to overwrite"
    ):
        eyelink.Eyelink._close_data_file(node, save=True)
    assert path.read_bytes() == b"original"
    tracker.receiveDataFile.assert_not_called()


def test_stop_waits_for_actual_rclpy_future_completion_before_result():
    future = Mock()
    completed = threading.Event()
    future.done.side_effect = completed.is_set
    future.cancelled.return_value = False
    future.result.side_effect = lambda: (
        completed.is_set() or pytest.fail("read pending future")
    )
    node = SimpleNamespace(
        stop_sample_retrieval_event=threading.Event(),
        _recording_active=threading.Event(),
        sample_retrieval_future=future,
        log=Mock(),
    )
    timer = threading.Timer(0.03, completed.set)
    timer.start()
    try:
        eyelink.Eyelink._stop_sample_retrieval_loop(node)
    finally:
        timer.join()
    assert completed.is_set()
    future.result.assert_called_once()
    assert not hasattr(node, "sample_retrieval_future")


def test_cancelled_drain_is_not_acknowledged_as_complete():
    future = Mock()
    future.done.return_value = False
    future.cancelled.return_value = True
    node = SimpleNamespace(
        stop_sample_retrieval_event=threading.Event(),
        _recording_active=threading.Event(),
        sample_retrieval_future=future,
        log=Mock(),
    )
    with pytest.raises(RuntimeError, match="drain did not complete"):
        eyelink.Eyelink._stop_sample_retrieval_loop(node)
    future.result.assert_not_called()
    assert node.sample_retrieval_future is future


def test_unfinalized_capture_blocks_new_recording_before_tracker_overwrite():
    node = SimpleNamespace(
        _retrieval_lock=threading.Lock(),
        _retrieving=False,
        _last_finalization_error="EDF transfer failed",
        log=Mock(),
        tracker=Mock(),
        _open_data_file=Mock(),
    )
    with pytest.raises(RuntimeError, match="overwrite the tracker EDF"):
        eyelink.Eyelink.start_retrieval(node)
    node._open_data_file.assert_not_called()
    assert not node.tracker.mock_calls


def test_pending_drain_timeout_does_not_read_or_delete_future(monkeypatch):
    future = Mock()
    future.done.return_value = False
    future.cancelled.return_value = False
    node = SimpleNamespace(
        stop_sample_retrieval_event=threading.Event(),
        _recording_active=threading.Event(),
        sample_retrieval_future=future,
        log=Mock(),
    )
    times = iter((0.0, 11.0))
    monkeypatch.setattr(eyelink.time, "monotonic", lambda: next(times))
    with pytest.raises(RuntimeError, match="drain did not complete"):
        eyelink.Eyelink._stop_sample_retrieval_loop(node)
    future.result.assert_not_called()
    assert node.sample_retrieval_future is future
