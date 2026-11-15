import sys
import re
from argparse import Namespace
from pathlib import Path
from types import SimpleNamespace

import yaml
import pytest
from tabletop_rig.utils import diagnostic_rosbag_recorder


@pytest.mark.parametrize("same_host", [True, False])
def test_nas_dds_identity_guard_never_restarts_drivers(monkeypatch, same_host):
    monkeypatch.setenv("TABLETOP_RECORDING_BACKEND", "nas")
    own = bytes([1,15,2,3])+bytes(20)
    remote = own if same_host else bytes([1,15,9,9])+bytes(20)
    class Context:
        def ok(self): return True
    class Node:
        def get_name(self): return "nas_dds_identity_preflight"
        def create_subscription(self,*a): pass
        def get_subscriptions_info_by_topic(self,*a):
            return [SimpleNamespace(node_name=self.get_name(),endpoint_gid=own)]
        def get_publishers_info_by_topic(self,*a):
            return [SimpleNamespace(node_name="source",endpoint_gid=remote)]
        def destroy_node(self): pass
    class Executor:
        def __init__(self,**kw): pass
        def add_node(self,*a): pass
        def spin_once(self,**kw): pass
        def shutdown(self): pass
    monkeypatch.setattr(diagnostic_rosbag_recorder,"Context",Context)
    monkeypatch.setattr(diagnostic_rosbag_recorder,"SingleThreadedExecutor",Executor)
    monkeypatch.setattr(diagnostic_rosbag_recorder.rclpy,"init",lambda **kw:None)
    monkeypatch.setattr(diagnostic_rosbag_recorder.rclpy,"shutdown",lambda **kw:None)
    monkeypatch.setattr(diagnostic_rosbag_recorder.rclpy,"create_node",lambda *a,**kw:Node())
    monkeypatch.setattr(diagnostic_rosbag_recorder.rclpy,"get_rmw_implementation_identifier",lambda:"rmw_fastrtps_cpp")
    monkeypatch.setattr(diagnostic_rosbag_recorder.subprocess,"run",lambda *a,**kw:pytest.fail("must not restart anything"))
    args=Namespace(topics=["/joint_states"],all_topics=False)
    if same_host:
        assert diagnostic_rosbag_recorder._check_local_dds_hosts(args)["status"] == "PASS"
    else:
        with pytest.raises(RuntimeError,match="no robot/controller restart"):
            diagnostic_rosbag_recorder._check_local_dds_hosts(args)


def test_eyelink_finalization_default_is_bounded_ninety_seconds(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["recorder", "--output", "/tmp/test_bag"])
    assert diagnostic_rosbag_recorder._parse_args().eyelink_finalize_timeout == 90.0


def test_eyelink_finalization_timeout_remains_overridable(monkeypatch):
    monkeypatch.setattr(sys, "argv", [
        "recorder", "--output", "/tmp/test_bag", "--eyelink-finalize-timeout", "120",
    ])
    assert diagnostic_rosbag_recorder._parse_args().eyelink_finalize_timeout == 120.0


def test_recorder_launch_allows_finalization_and_flush():
    import runpy
    from launch import LaunchContext

    launch_path = Path(__file__).resolve().parents[1] / "launch/rosbag.launch.py"
    namespace = runpy.run_path(str(launch_path))
    arguments = {arg.name: arg for arg in namespace["declare_arguments"]()}
    grace = "".join(
        value.perform(LaunchContext())
        for value in arguments["rosbag_sigterm_timeout"].default_value
    )
    assert float(grace) == 180.0


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
    assert captured["record"].topics == []
    assert re.fullmatch(captured["record"].regex, "/eyelink/sample_array")


def test_topic_allowlist_escapes_names_and_keeps_services(monkeypatch):
    captured = {}

    def fake_recorder(storage, record, *args):
        captured["record"] = record

    monkeypatch.setattr(diagnostic_rosbag_recorder, "Recorder", fake_recorder)
    args = Namespace(
        output="/tmp/test_bag", storage="mcap", max_bag_size=0,
        max_cache_size=0, all_topics=False, all_services=False,
        topics=["/joint_states", "/dynamic_joint_states", "/camera.v1"],
        services=["/teensy/set_reward"], use_sim_time=False,
        qos_profile_overrides_path=None, log_level="info",
        per_topic_loss_debug=False,
    )
    diagnostic_rosbag_recorder._make_recorder(args)
    options = captured["record"]
    assert not options.topics
    assert options.services == ["/teensy/set_reward/_service_event"]
    for topic in args.topics:
        assert re.fullmatch(options.regex, topic)
    for topic in ("/joint_states_extra", "/unrelated", "/cameraXv1"):
        assert not re.fullmatch(options.regex, topic)


@pytest.mark.parametrize("recorder_present", [False, True])
def test_robot_gate_requires_live_samples_and_recorder(
    monkeypatch, recorder_present
):
    # Samples alone cannot pass a missing recorder endpoint. Conversely,
    # discovery of an endpoint alone cannot pass a silent source.
    class FakeContext:
        def ok(self):
            return False

    class FakeNode:
        def create_subscription(self, *args):
            if not recorder_present:
                for _ in range(3):
                    args[2](None)

        def get_subscriptions_info_by_topic(self, topic):
            return [SimpleNamespace(node_name="rosbag2_recorder")] if recorder_present else []

        def destroy_node(self):
            pass

    class FakeExecutor:
        def __init__(self, **kwargs):
            pass

        def add_node(self, node):
            pass

        def spin_once(self, **kwargs):
            pass

        def shutdown(self):
            pass

    monkeypatch.setattr(diagnostic_rosbag_recorder, "Context", FakeContext)
    monkeypatch.setattr(diagnostic_rosbag_recorder.rclpy, "init", lambda **kw: None)
    monkeypatch.setattr(diagnostic_rosbag_recorder.rclpy, "create_node", lambda *a, **kw: FakeNode())
    monkeypatch.setattr(diagnostic_rosbag_recorder, "SingleThreadedExecutor", FakeExecutor)
    with pytest.raises(RuntimeError, match="Robot recording preflight failed"):
        diagnostic_rosbag_recorder._wait_for_robot_recording(
            Namespace(all_topics=False, topics=["/joint_states", "/dynamic_joint_states"]),
            timeout_sec=0.01,
        )
