"""Rosbag recorder with exact per-topic DDS loss diagnostics.

The stock ROS 2 CLI only accepts a global log level.  This wrapper keeps the
normal Python recorder lifecycle (including clean SIGINT/SIGTERM shutdown), but
enables DEBUG only for the rosbag2 recorder logger.  Rosbag2 then reports the
cumulative transport-layer loss count for each affected topic without flooding
the session log with DEBUG output from every ROS client-library operation.
"""

import argparse
import os
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
import yaml
from control_msgs.msg import DynamicJointState
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import LoggingSeverity, set_logger_level
from rclpy.qos import qos_profile_sensor_data
from ros2bag.api import (
    convert_service_to_service_event_topic,
    convert_yaml_to_qos_profile,
)
from ros2cli.node import NODE_NAME_PREFIX
from rosbag2_py import Recorder, RecordOptions, StorageOptions
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tabletop_interfaces.msg import EyelinkBatch

from tabletop_rig.utils.flir_preflight import (
    write_flir_preflight_report,
)
from tabletop_rig.utils.session_integrity import generate_session_report
from tabletop_rig.utils.recording_storage import recorder_qos_profiles, validate_recording_environment


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record an MCAP bag with per-topic DDS loss diagnostics"
    )
    parser.add_argument("--output", required=True)
    parser.add_argument("--storage", default="mcap")
    parser.add_argument("--log-level", default="info")
    parser.add_argument("--max-bag-size", type=int, default=0)
    parser.add_argument("--max-cache-size", type=int, default=0)
    parser.add_argument("--topics", nargs="*", default=[])
    parser.add_argument("--services", nargs="*", default=[])
    parser.add_argument("--all-topics", action="store_true")
    parser.add_argument("--all-services", action="store_true")
    parser.add_argument("--use-sim-time", action="store_true")
    parser.add_argument("--per-topic-loss-debug", action="store_true")
    parser.add_argument("--qos-profile-overrides-path")
    parser.add_argument("--dds-identity-preflight-only", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--flir-preflight", action="store_true")
    parser.add_argument("--flir-preflight-timeout", type=float, default=20.0)
    parser.add_argument("--flir-preflight-window", type=float, default=4.0)
    parser.add_argument("--flir-min-rate-hz", type=float, default=115.0)
    parser.add_argument("--flir-preflight-attempts", type=int, default=3)
    parser.add_argument("--flir-recorder-warmup", type=float, default=3.0)
    parser.add_argument(
        "--discovery-soak",
        type=float,
        default=2.0,
        help="Seconds to let the recorder DDS participant discover publishers before recording",
    )
    parser.add_argument(
        "--eyelink-finalize-timeout",
        type=float,
        default=90.0,
        help="Bounded seconds for EyeLink drain, EDF transfer, and integrity report before bag close (CSV is separate)",
    )
    return parser.parse_args()


def _make_recorder(args: argparse.Namespace) -> Recorder:
    storage_options = StorageOptions(
        uri=args.output,
        storage_id=args.storage,
        max_bagfile_size=args.max_bag_size,
        max_cache_size=args.max_cache_size,
    )

    record_options = RecordOptions()
    record_options.all_topics = args.all_topics
    record_options.all_services = args.all_services
    # Jazzy 0.26.10 stops discovery when total subscriptions == len(topics),
    # counting service events too. A service can therefore replace a missing
    # data topic in that count. Select the same finite set with an anchored,
    # escaped regex and leave topics empty, disabling that early-exit branch.
    # Services stay explicit: their event topic names differ from the service
    # names matched by TopicFilter's regex path.
    if args.topics and not args.all_topics:
        record_options.regex = (
            "^(?:"
            + "|".join(re.escape(topic) for topic in dict.fromkeys(args.topics))
            + ")$"
        )
    record_options.services = convert_service_to_service_event_topic(
        args.services
    )
    record_options.rmw_serialization_format = "cdr"
    record_options.node_prefix = NODE_NAME_PREFIX
    record_options.use_sim_time = args.use_sim_time
    record_options.disable_keyboard_controls = True
    # Keep explicitly configured topics eligible even when their publisher is
    # late to the graph, and keep discovery enabled for normal endpoint
    # matching. This is particularly important for EyeLink sample batches.
    record_options.include_unpublished_topics = True
    record_options.is_discovery_disabled = False
    if args.qos_profile_overrides_path:
        with open(args.qos_profile_overrides_path, encoding="utf-8") as file:
            record_options.topic_qos_profile_overrides = (
                convert_yaml_to_qos_profile(recorder_qos_profiles(
                    yaml.safe_load(file),
                    nas=os.environ.get("TABLETOP_RECORDING_BACKEND", "local") == "nas",
                ))
            )

    recorder = Recorder(
        storage_options,
        record_options,
        args.log_level,
        "rosbag2_recorder",
    )
    if args.per_topic_loss_debug:
        # Keep global INFO; promote only rosbag2 for per-topic loss reports.
        set_logger_level("rosbag2_recorder", LoggingSeverity.DEBUG)
    return recorder


def _wait_for_robot_recording(
    args: argparse.Namespace, *, timeout_sec: float = 15.0
) -> dict:
    """Confirm live robot samples and actual recorder endpoints before motion.

    This short-lived probe checks delivery to a fresh subscriber and the
    recorder's subscriptions. Final bag counts remain the end-to-end audit.
    It never publishes or commands anything and adds no steady-state workload.
    """
    required = {
        topic: message_type
        for topic, message_type in (
            ("/joint_states", JointState),
            ("/dynamic_joint_states", DynamicJointState),
        )
        if args.all_topics or topic in args.topics
    }
    if os.environ.get("TABLETOP_RECORDING_BACKEND", "local") == "nas" and (
        args.all_topics or "/eyelink/sample_array" in args.topics
    ):
        # An advertised EyeLink publisher alone is insufficient: a container
        # with an old /bags bind can fail only when retrieval starts.
        required["/eyelink/sample_array"] = EyelinkBatch
    if not required:
        return {"status": "PASS", "sample_counts": {}, "recorder_topics": []}
    context = Context()
    node = None
    executor = None
    counts = dict.fromkeys(required, 0)
    recorder_topics = set()
    try:
        rclpy.init(context=context)
        node = rclpy.create_node("robot_recording_preflight", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        for topic, message_type in required.items():

            def receive(_message, topic=topic):
                counts[topic] += 1

            node.create_subscription(
                message_type, topic, receive, qos_profile_sensor_data
            )
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            recorder_topics = {
                topic
                for topic in required
                if any(
                    "rosbag2_recorder" in endpoint.node_name
                    for endpoint in node.get_subscriptions_info_by_topic(topic)
                )
            }
            if recorder_topics == set(required) and all(
                count >= 3 for count in counts.values()
            ):
                return {
                    "status": "PASS",
                    "sample_counts": counts,
                    "recorder_topics": sorted(recorder_topics),
                }
        raise RuntimeError(
            "Robot recording preflight failed: "
            f"sample_counts={counts}, missing recorder subscriptions="
            f"{sorted(set(required) - recorder_topics)}"
        )
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)


def _run_flir_preflight_subprocess(
    args: argparse.Namespace,
) -> dict:
    """Run preflight under a separate PID so DDS SHM ports cannot be reused."""
    with tempfile.TemporaryDirectory(
        prefix="tabletop_flir_preflight_"
    ) as temporary_dir:
        command = [
            sys.executable,
            "-m",
            "tabletop_rig.utils.flir_preflight",
            "--session-dir",
            temporary_dir,
            "--timeout",
            str(args.flir_preflight_timeout),
            "--sample-window",
            str(args.flir_preflight_window),
            "--minimum-rate",
            str(args.flir_min_rate_hz),
        ]
        try:
            result = subprocess.run(
                command,
                check=False,
                timeout=args.flir_preflight_timeout + 10.0,
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("FLIR preflight process timed out") from exc

        report_path = os.path.join(temporary_dir, "flir_preflight.yaml")
        if not os.path.exists(report_path):
            raise RuntimeError(
                "FLIR preflight process did not produce a readiness report "
                f"(exit code {result.returncode})"
            )
        with open(report_path, encoding="utf-8") as file:
            return yaml.safe_load(file)


def _run_flir_preflight_with_retries(
    args: argparse.Namespace,
) -> dict:
    """Run bounded, independent FLIR checks until one passes."""
    attempts = int(args.flir_preflight_attempts)
    if attempts < 1:
        raise ValueError("flir_preflight_attempts must be at least 1")

    history = []
    report = None
    for attempt in range(1, attempts + 1):
        print(
            f"Running FLIR recording preflight attempt "
            f"{attempt}/{attempts}...",
            flush=True,
        )
        try:
            report = _run_flir_preflight_subprocess(args)
        except RuntimeError as exc:
            report = {"status": "FAIL", "failures": [str(exc)]}

        history.append(
            {
                "attempt": attempt,
                "status": report.get("status", "FAIL"),
                "failures": list(report.get("failures", [])),
            }
        )
        if report.get("status") == "PASS":
            break
        if attempt < attempts:
            failures = "; ".join(report.get("failures", []))
            print(
                f"FLIR preflight attempt {attempt}/{attempts} failed; "
                f"retrying: {failures}",
                flush=True,
            )

    assert report is not None
    report = dict(report)
    report["attempts_used"] = len(history)
    report["attempts_allowed"] = attempts
    report["attempt_history"] = history
    return report


def _finalize_eyelink(timeout_sec: float) -> bool:
    """Ask EyeLink to drain samples and transfer EDF before rosbag closes."""
    deadline = time.monotonic() + timeout_sec
    context = Context()
    node = None
    executor = None
    try:
        rclpy.init(context=context)
        node = rclpy.create_node("rosbag_eyelink_finalizer", context=context)
        client = node.create_client(Trigger, "/eyelink/stop_recording")
        discovery_timeout = min(10.0, max(timeout_sec, 0.0))
        if not client.wait_for_service(timeout_sec=discovery_timeout):
            print(
                "EyeLink finalization service unavailable after "
                f"{discovery_timeout:.1f}s; continuing bag shutdown",
                flush=True,
            )
            return False
        print("Finalizing EyeLink before closing rosbag...", flush=True)
        future = client.call_async(Trigger.Request())
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        remaining_timeout = max(0.0, deadline - time.monotonic())
        executor.spin_until_future_complete(
            future, timeout_sec=remaining_timeout
        )
        if not future.done() or future.result() is None:
            print("EyeLink finalization timed out", flush=True)
            return False
        result = future.result()
        print(result.message, flush=True)
        return bool(result.success)
    except Exception as exc:
        print(f"EyeLink finalization failed: {exc}", flush=True)
        return False
    finally:
        if executor is not None and node is not None:
            executor.remove_node(node)
        if node is not None:
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)


def _check_local_dds_hosts(args, timeout_sec: float = 10.0) -> dict:
    """Catch NIC-hotplug host-ID mismatches before creating recording subscriptions.

    This rig's ROS acquisition drivers are local. Never restart a robot driver
    here: reject startup and ask the operator for a coherent idle bring-up.
    """
    if os.environ.get("TABLETOP_RECORDING_BACKEND", "local") != "nas":
        return {"status": "NOT_REQUIRED"}
    if "fastrtps" not in rclpy.get_rmw_implementation_identifier():
        return {"status": "NOT_APPLICABLE"}
    wanted = {"/joint_states", "/dynamic_joint_states", "/markers", "/teensy/sensor", "/eyelink/sample_array"}
    wanted.update(topic for topic in args.topics if topic.startswith("/cam_sync/") and topic.endswith("/meta"))
    wanted = wanted if args.all_topics else wanted.intersection(args.topics)
    context = Context()
    node = None
    executor = None
    try:
        rclpy.init(context=context)
        node = rclpy.create_node("nas_dds_identity_preflight", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        node.create_subscription(JointState, "/joint_states", lambda _msg: None, qos_profile_sensor_data)
        deadline = time.monotonic()+timeout_sec
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            own = [ep for ep in node.get_subscriptions_info_by_topic("/joint_states") if ep.node_name == node.get_name()]
            if not own:
                continue
            prefix = bytes(own[0].endpoint_gid[:4])
            endpoints = {topic:node.get_publishers_info_by_topic(topic) for topic in wanted}
            mismatch = {topic:[ep.node_name for ep in eps if bytes(ep.endpoint_gid[:4]) != prefix]
                        for topic,eps in endpoints.items()}
            mismatch = {topic:names for topic,names in mismatch.items() if names}
            if mismatch:
                raise RuntimeError("NAS DDS host identities differ after a network-interface change: "
                    f"{mismatch}. Stop tasks and relaunch the acquisition stack with the NAS adapter connected; "
                    "no robot/controller restart has been attempted automatically.")
            if all(endpoints.values()):
                return {"status":"PASS", "host_prefix":prefix.hex('.'), "topics":sorted(wanted)}
        raise RuntimeError("NAS DDS preflight timed out waiting for local acquisition publishers")
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)


def main() -> int:
    args = _parse_args()
    if args.dds_identity_preflight_only:
        _check_local_dds_hosts(args)
        return 0
    validate_recording_environment(os.path.dirname(args.output))
    if os.environ.get("TABLETOP_RECORDING_BACKEND", "local") == "nas":
        # Like the camera probe, exit the graph participant in a separate PID
        # before constructing the actual recorder (avoid SHM GUID/port reuse).
        command = [sys.executable, "-m", "tabletop_rig.utils.diagnostic_rosbag_recorder", "--output", args.output,
                   "--dds-identity-preflight-only"]
        if args.all_topics:
            command.append("--all-topics")
        elif args.topics:
            command.extend(["--topics", *args.topics])
        subprocess.run(command, check=True, timeout=25)
    preflight_report = None
    if args.flir_preflight:
        preflight_report = _run_flir_preflight_with_retries(args)
        if preflight_report["status"] != "PASS":
            report_path = write_flir_preflight_report(
                os.path.dirname(args.output), preflight_report
            )
            failures = "; ".join(preflight_report["failures"])
            raise RuntimeError(
                f"FLIR preflight failed ({report_path}): {failures}"
            )

    # The preflight child has exited completely before the recorder participant
    # is created, preventing Fast DDS SHM port/GUID reuse in this process.
    recorder = _make_recorder(args)
    stop_requested = threading.Event()
    previous_sigint_handler = signal.getsignal(signal.SIGINT)
    previous_sigterm_handler = signal.getsignal(signal.SIGTERM)

    def request_stop(_signum, _frame):
        stop_requested.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    spin_started = False
    record_started = False
    try:
        recorder.start_spin()
        spin_started = True

        # The recorder participant now exists, but no data subscriptions have
        # been created yet. A short soak lets persistent hardware nodes become
        # visible before rosbag evaluates the explicit topic list.
        if args.discovery_soak > 0:
            time.sleep(args.discovery_soak)

        recorder.record()
        record_started = True
        if preflight_report is not None:
            time.sleep(args.flir_recorder_warmup)
            # Publish the task-start gate only after both joint-state streams
            # are live and the recorder has created their subscriptions.
            try:
                preflight_report["robot_recording_preflight"] = (
                    _wait_for_robot_recording(args)
                )
                print("Robot recording preflight PASS", flush=True)
            except RuntimeError as exc:
                preflight_report["status"] = "FAIL"
                preflight_report.setdefault("failures", []).append(str(exc))
                write_flir_preflight_report(
                    os.path.dirname(args.output), preflight_report
                )
                raise
            report_path = write_flir_preflight_report(
                os.path.dirname(args.output), preflight_report
            )
            print(
                f"FLIR preflight PASS; recorder warmup complete: {report_path}",
                flush=True,
            )
        # start_spin() installs ROS signal handling; restore our explicit handlers.
        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        while not stop_requested.wait(0.1):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        if record_started:
            if not _finalize_eyelink(args.eyelink_finalize_timeout):
                print(
                    "WARNING: EyeLink capture finalization was not verified; "
                    "closing bag after bounded wait. Check eyelink_integrity.json "
                    "and preserve the tracker EDF before starting another session.",
                    flush=True,
                )
            recorder.stop()
        if spin_started:
            recorder.stop_spin()
        signal.signal(signal.SIGINT, previous_sigint_handler)
        signal.signal(signal.SIGTERM, previous_sigterm_handler)

    try:
        generate_session_report(os.path.dirname(args.output))
    except Exception as exc:
        print(
            f"Could not generate session integrity report: {exc}",
            flush=True,
        )

    print(f"Recording stopped cleanly: {args.output}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
