import json
import threading
import time
from pathlib import Path
from unittest.mock import Mock

import pytest
from tabletop_rig.utils import eyelink_csv


def source(tmp_path, name="session"):
    folder = tmp_path / name
    edf = folder / "eyelink_received" / "last.edf"
    edf.parent.mkdir(parents=True)
    edf.write_bytes(b"source EDF")
    (folder / "eyelink_integrity.json").write_text(
        '{"status":"complete","edf_saved":true}'
    )
    return edf


def test_success_only_publishes_complete_csv_and_keeps_capture_report(
    tmp_path,
):
    edf = source(tmp_path)
    edf.with_suffix(".csv").write_text("old CSV")

    def converter(path, *, output_path):
        assert Path(path) == edf
        assert edf.with_suffix(".csv").read_text() == "old CSV"
        Path(output_path).write_text("time,input\n0,127\n")

    assert eyelink_csv.convert(edf, converter) == edf.with_suffix(".csv")
    assert edf.read_bytes() == b"source EDF"
    assert edf.with_suffix(".csv").read_text() == "time,input\n0,127\n"
    assert (
        json.loads(
            (edf.parent.parent / "eyelink_conversion.json").read_text()
        )["status"]
        == "complete"
    )
    assert (
        json.loads((edf.parent.parent / "eyelink_integrity.json").read_text())[
            "status"
        ]
        == "complete"
    )
    assert not list(edf.parent.glob(".csv-*"))


@pytest.mark.parametrize(
    "failure", [RuntimeError("bad converter"), KeyboardInterrupt()]
)
def test_conversion_failure_preserves_edf_prior_csv_and_capture_status(
    tmp_path, failure
):
    edf = source(tmp_path)
    edf.with_suffix(".csv").write_text("old CSV")

    def converter(path, *, output_path):
        Path(output_path).write_text("partial")
        raise failure

    with pytest.raises(type(failure)):
        eyelink_csv.convert(edf, converter)
    assert edf.read_bytes() == b"source EDF"
    assert edf.with_suffix(".csv").read_text() == "old CSV"
    report = json.loads(
        (edf.parent.parent / "eyelink_conversion.json").read_text()
    )
    assert report["status"] == (
        "interrupted" if isinstance(failure, KeyboardInterrupt) else "error"
    )
    assert json.loads(
        (edf.parent.parent / "eyelink_integrity.json").read_text()
    )["edf_saved"]
    assert not list(edf.parent.glob(".csv-*"))


def test_no_output_is_a_conversion_failure_not_success(tmp_path):
    edf = source(tmp_path)
    with pytest.raises(RuntimeError, match="no CSV data"):
        eyelink_csv.convert(edf, lambda *_args, **_kwargs: None)
    assert not edf.with_suffix(".csv").exists()


def wait_until(predicate):
    deadline = time.monotonic() + 3
    while not predicate():
        assert time.monotonic() < deadline, (
            "worker did not reach expected state"
        )
        time.sleep(0.01)


def test_worker_waits_for_bag_close_serializes_jobs_and_pins_session(
    tmp_path, monkeypatch
):
    first = source(tmp_path, "first")
    second = source(tmp_path, "second")
    (first.parent.parent / "bag").mkdir()
    (second.parent.parent / "bag").mkdir()
    (second.parent.parent / "bag/metadata.yaml").write_text("closed")
    finished = [threading.Event(), threading.Event()]
    launched = []

    def launch(command, **kwargs):
        index = len(launched)
        launched.append((command, kwargs))
        process = Mock(returncode=0)
        process.poll.side_effect = lambda: (
            0 if finished[index].is_set() else None
        )
        return process

    monkeypatch.setattr(eyelink_csv.subprocess, "Popen", launch)
    worker = eyelink_csv.EyeLinkCSVWorker()
    try:
        worker.submit(first)
        worker.submit(second)
        time.sleep(0.05)
        assert not launched
        assert (
            json.loads(
                (first.parent.parent / "eyelink_conversion.json").read_text()
            )["status"]
            == "waiting_bag_close"
        )
        (first.parent.parent / "bag/metadata.yaml").write_text("closed")
        wait_until(lambda: len(launched) == 1)
        assert launched[0][0][-1] == str(first)
        assert launched[0][1]["env"]["OMP_NUM_THREADS"] == "1"
        finished[0].set()
        wait_until(lambda: len(launched) == 2)
        assert launched[1][0][-1] == str(second)
        finished[1].set()
        worker._jobs.join()
    finally:
        for event in finished:
            event.set()
        worker.close()


def test_stopping_waiting_worker_is_bounded_and_preserves_edf(tmp_path):
    edf = source(tmp_path)
    (edf.parent.parent / "bag").mkdir()
    worker = eyelink_csv.EyeLinkCSVWorker()
    worker.submit(edf)
    worker.close()
    assert not worker._thread.is_alive()
    assert edf.read_bytes() == b"source EDF"
    assert (
        json.loads(
            (edf.parent.parent / "eyelink_conversion.json").read_text()
        )["status"]
        == "interrupted"
    )


def test_child_crash_reports_conversion_failure_not_capture_loss(
    tmp_path, monkeypatch
):
    edf = source(tmp_path)
    process = Mock(returncode=1)
    process.poll.return_value = 1
    monkeypatch.setattr(
        eyelink_csv.subprocess, "Popen", lambda *_a, **_kw: process
    )
    worker = eyelink_csv.EyeLinkCSVWorker()
    try:
        worker.submit(edf)
        wait_until(
            lambda: (
                json.loads(
                    (edf.parent.parent / "eyelink_conversion.json").read_text()
                )["status"]
                == "error"
            )
        )
    finally:
        worker.close()
    assert edf.read_bytes() == b"source EDF"
    assert json.loads(
        (edf.parent.parent / "eyelink_integrity.json").read_text()
    )["edf_saved"]


def test_service_shutdown_interrupts_only_its_csv_child(tmp_path, monkeypatch):
    edf = source(tmp_path)
    process = Mock(returncode=None, pid=12345)
    process.poll.return_value = None
    launched = threading.Event()

    def launch(*_args, **_kwargs):
        launched.set()
        return process

    signals = Mock()
    monkeypatch.setattr(eyelink_csv.subprocess, "Popen", launch)
    monkeypatch.setattr(eyelink_csv.os, "killpg", signals)
    worker = eyelink_csv.EyeLinkCSVWorker()
    worker.submit(edf)
    assert launched.wait(2)
    worker.close()
    assert not worker._thread.is_alive()
    signals.assert_called_once_with(12345, eyelink_csv.signal.SIGTERM)
    assert edf.read_bytes() == b"source EDF"
    assert (
        json.loads(
            (edf.parent.parent / "eyelink_conversion.json").read_text()
        )["status"]
        == "interrupted"
    )
