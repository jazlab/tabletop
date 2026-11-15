"""Exercise Jazzy's real DDS discovery with service events and late topics.

Each test runs in a separate process/domain, without hardware or rig commands.
"""

import os
import subprocess
import sys
import threading
import time
from argparse import Namespace
from collections import Counter
from pathlib import Path

import pytest


@pytest.mark.parametrize("legacy", [True, False])
def test_late_joint_stream_survives_service_discovery(tmp_path, legacy):
    result = subprocess.run(
        [sys.executable, str(Path(__file__).resolve()), str(tmp_path / "bag"), str(int(legacy))],
        env={**os.environ, "ROS_DOMAIN_ID": "187", "ROS_LOCALHOST_ONLY": "1"},
        capture_output=True, text=True, timeout=40,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def _scenario(bag_path, legacy):
    import rclpy
    from control_msgs.msg import DynamicJointState
    from rosbag2_py import Recorder, RecordOptions, SequentialReader, StorageOptions, ConverterOptions
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from tabletop_rig.utils.diagnostic_rosbag_recorder import (
        _make_recorder, _wait_for_robot_recording,
    )

    rclpy.init()
    node = rclpy.create_node("recording_discovery_test_source")
    early = node.create_publisher(JointState, "/joint_states", 10)
    event = node.create_publisher(Trigger.Event, "/test_action/_service_event", 10)
    other = node.create_publisher(JointState, "/unrequested_joint_states", 10)
    stop = threading.Event()
    began = time.monotonic()

    def publish():
        late = None
        while not stop.wait(0.02):
            early.publish(JointState(name=["test_joint"], position=[1.0]))
            event.publish(Trigger.Event())
            other.publish(JointState())
            if time.monotonic() - began > 4.0:
                if late is None:
                    late = node.create_publisher(DynamicJointState, "/dynamic_joint_states", 10)
                late.publish(DynamicJointState(joint_names=["test_joint"]))

    thread = threading.Thread(target=publish)
    thread.start()
    args = Namespace(
        output=bag_path, storage="mcap", max_bag_size=0,
        max_cache_size=0, all_topics=False, all_services=False,
        topics=["/joint_states", "/dynamic_joint_states"],
        services=["/test_action"], use_sim_time=False,
        qos_profile_overrides_path=None, log_level="info",
        per_topic_loss_debug=False,
    )
    if legacy:
        options = RecordOptions()
        options.topics = args.topics
        options.services = ["/test_action/_service_event"]
        options.include_unpublished_topics = True
        options.disable_keyboard_controls = True
        options.rmw_serialization_format = "cdr"
        recorder = Recorder(StorageOptions(uri=bag_path, storage_id="mcap"), options, "info", "rosbag2_recorder")
    else:
        recorder = _make_recorder(args)
    try:
        recorder.start_spin()
        time.sleep(1.0)
        recorder.record()
        if legacy:
            with pytest.raises(RuntimeError, match="Robot recording preflight failed"):
                _wait_for_robot_recording(args, timeout_sec=7.0)
        else:
            report = _wait_for_robot_recording(args, timeout_sec=10.0)
            assert report["status"] == "PASS"
            assert all(count >= 3 for count in report["sample_counts"].values())
        time.sleep(0.5)
    finally:
        recorder.stop()
        recorder.stop_spin()
        stop.set()
        thread.join()
        node.destroy_node()
        rclpy.shutdown()

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id="mcap"), ConverterOptions("", ""))
    counts = Counter()
    while reader.has_next():
        topic, _, _ = reader.read_next()
        counts[topic] += 1
    assert counts["/joint_states"] > 10, counts
    assert counts["/test_action/_service_event"] > 10, counts
    assert counts["/unrequested_joint_states"] == 0, counts
    if legacy:
        assert counts["/dynamic_joint_states"] == 0, counts
    else:
        assert counts["/dynamic_joint_states"] > 10, counts
    print(f"legacy={legacy}, bag_counts={dict(counts)}", flush=True)


if __name__ == "__main__":
    _scenario(sys.argv[1], bool(int(sys.argv[2])))
