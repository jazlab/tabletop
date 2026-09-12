import sys
from argparse import Namespace
from pathlib import Path
from types import SimpleNamespace

import yaml
from tabletop_rig.utils import diagnostic_rosbag_recorder


def test_flir_preflight_uses_separate_python_process(monkeypatch):
    observed_command = None

    def fake_run(command, *, check, timeout):
        nonlocal observed_command
        observed_command = command
        session_dir = Path(command[command.index("--session-dir") + 1])
        with (session_dir / "flir_preflight.yaml").open("w") as file:
            yaml.safe_dump({"status": "PASS", "failures": []}, file)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(diagnostic_rosbag_recorder.subprocess, "run", fake_run)
    args = Namespace(
        flir_preflight_timeout=20.0,
        flir_preflight_window=2.0,
        flir_min_rate_hz=115.0,
    )

    report = diagnostic_rosbag_recorder._run_flir_preflight_subprocess(args)

    assert report["status"] == "PASS"
    assert observed_command[:3] == [
        sys.executable,
        "-m",
        "tabletop_rig.utils.flir_preflight",
    ]


def test_flir_preflight_retries_until_third_attempt_passes(monkeypatch):
    reports = iter(
        [
            {"status": "FAIL", "failures": ["first"]},
            {"status": "FAIL", "failures": ["second"]},
            {"status": "PASS", "failures": []},
        ]
    )
    calls = 0

    def fake_preflight(_args):
        nonlocal calls
        calls += 1
        return next(reports)

    monkeypatch.setattr(
        diagnostic_rosbag_recorder,
        "_run_flir_preflight_subprocess",
        fake_preflight,
    )
    args = Namespace(flir_preflight_attempts=3)

    report = diagnostic_rosbag_recorder._run_flir_preflight_with_retries(args)

    assert calls == 3
    assert report["status"] == "PASS"
    assert report["attempts_used"] == 3
    assert report["attempts_allowed"] == 3
    assert [entry["status"] for entry in report["attempt_history"]] == [
        "FAIL",
        "FAIL",
        "PASS",
    ]


def test_flir_preflight_stops_after_first_success(monkeypatch):
    calls = 0

    def fake_preflight(_args):
        nonlocal calls
        calls += 1
        return {"status": "PASS", "failures": []}

    monkeypatch.setattr(
        diagnostic_rosbag_recorder,
        "_run_flir_preflight_subprocess",
        fake_preflight,
    )

    report = diagnostic_rosbag_recorder._run_flir_preflight_with_retries(
        Namespace(flir_preflight_attempts=3)
    )

    assert calls == 1
    assert report["attempts_used"] == 1


def test_recorder_keeps_late_configured_topics_discoverable(monkeypatch):
    captured = {}

    class FakeRecorder:
        def __init__(self, storage, record, log_level, node_name):
            captured["record"] = record
            captured["node_name"] = node_name

    monkeypatch.setattr(diagnostic_rosbag_recorder, "Recorder", FakeRecorder)
    args = Namespace(
        output="/tmp/test_bag",
        storage="mcap",
        max_bag_size=0,
        max_cache_size=0,
        all_topics=False,
        all_services=False,
        topics=["/eyelink/sample_array"],
        services=[],
        use_sim_time=False,
        qos_profile_overrides_path=None,
        log_level="info",
        per_topic_loss_debug=False,
    )

    diagnostic_rosbag_recorder._make_recorder(args)

    assert captured["node_name"] == "rosbag2_recorder"
    assert captured["record"].include_unpublished_topics is True
    assert captured["record"].is_discovery_disabled is False
