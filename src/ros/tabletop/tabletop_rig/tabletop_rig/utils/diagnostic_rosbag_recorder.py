"""Rosbag recorder with exact per-topic DDS loss diagnostics.

The stock ROS 2 CLI only accepts a global log level.  This wrapper keeps the
normal Python recorder lifecycle (including clean SIGINT/SIGTERM shutdown), but
enables DEBUG only for the rosbag2 recorder logger.  Rosbag2 then reports the
cumulative transport-layer loss count for each affected topic without flooding
the session log with DEBUG output from every ROS client-library operation.
"""

import argparse
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
import yaml
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import LoggingSeverity, set_logger_level
from ros2bag.api import (
    convert_service_to_service_event_topic,
    convert_yaml_to_qos_profile,
)
from ros2cli.node import NODE_NAME_PREFIX
from rosbag2_py import Recorder, RecordOptions, StorageOptions
from std_srvs.srv import Trigger

from tabletop_rig.utils.flir_preflight import (
    write_flir_preflight_report,
)
from tabletop_rig.utils.session_integrity import generate_session_report


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
        default=30.0,
        help="Seconds to wait for EyeLink drain and EDF transfer before bag close",
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
    record_options.topics = args.topics
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
                convert_yaml_to_qos_profile(yaml.safe_load(file))
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


def main() -> int:
    args = _parse_args()
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
            _finalize_eyelink(args.eyelink_finalize_timeout)
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
