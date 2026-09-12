"""Bounded FLIR readiness check and one-time calibration capture."""

from __future__ import annotations

import argparse
import os
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
import yaml
from flir_camera_msgs.msg import ImageMetaData
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage

CAMERAS = (
    "left_front_top_cam",
    "left_back_top_cam",
    "left_bottom_cam",
    "right_front_top_cam",
    "right_back_top_cam",
    "right_bottom_cam",
)


def _camera_info_to_dict(message: CameraInfo) -> dict[str, Any]:
    """Convert the constant calibration payload to plain YAML-safe values."""
    return {
        "frame_id": message.header.frame_id,
        "height": int(message.height),
        "width": int(message.width),
        "distortion_model": message.distortion_model,
        "d": [float(value) for value in message.d],
        "k": [float(value) for value in message.k],
        "r": [float(value) for value in message.r],
        "p": [float(value) for value in message.p],
        "binning_x": int(message.binning_x),
        "binning_y": int(message.binning_y),
        "roi": {
            "x_offset": int(message.roi.x_offset),
            "y_offset": int(message.roi.y_offset),
            "height": int(message.roi.height),
            "width": int(message.roi.width),
            "do_rectify": bool(message.roi.do_rectify),
        },
    }


def evaluate_preflight(
    images_seen: set[str],
    camera_info: dict[str, dict[str, Any]],
    metadata_rates_hz: dict[str, float],
    min_rate_hz: float,
) -> tuple[str, list[str]]:
    """Evaluate observable readiness without moving or commanding the rig."""
    failures: list[str] = []
    for camera in CAMERAS:
        if camera not in images_seen:
            failures.append(f"{camera}: no compressed image received")
        if camera not in camera_info:
            failures.append(f"{camera}: no CameraInfo received")
        rate = metadata_rates_hz.get(camera, 0.0)
        if rate < min_rate_hz:
            failures.append(
                f"{camera}: metadata rate {rate:.1f} Hz is below "
                f"{min_rate_hz:.1f} Hz"
            )
    return ("PASS" if not failures else "FAIL", failures)


class _FlirPreflightNode(Node):
    def __init__(self, *, context: Context) -> None:
        super().__init__("flir_recording_preflight", context=context)
        self.images_seen: set[str] = set()
        self.camera_info: dict[str, dict[str, Any]] = {}
        self.metadata_stamps_ns = {camera: [] for camera in CAMERAS}
        self.metadata_counts = {camera: 0 for camera in CAMERAS}
        self._image_subscriptions = []
        self._meta_subscriptions = []
        self._info_subscriptions = []

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        metadata_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=32,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        info_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        for camera in CAMERAS:
            prefix = f"/cam_sync/{camera}"
            self._image_subscriptions.append(
                self.create_subscription(
                    CompressedImage,
                    f"{prefix}/image_raw/compressed",
                    lambda _message, name=camera: self.images_seen.add(name),
                    image_qos,
                )
            )
            self._meta_subscriptions.append(
                self.create_subscription(
                    ImageMetaData,
                    f"{prefix}/meta",
                    lambda message, name=camera: self._record_metadata(name, message),
                    metadata_qos,
                )
            )
            self._info_subscriptions.append(
                self.create_subscription(
                    CameraInfo,
                    f"{prefix}/camera_info",
                    lambda message, name=camera: self.camera_info.setdefault(
                        name, _camera_info_to_dict(message)
                    ),
                    info_qos,
                )
            )

    def _record_metadata(
        self, camera: str, message: ImageMetaData
    ) -> None:
        self.metadata_counts[camera] += 1
        stamp = message.header.stamp
        self.metadata_stamps_ns[camera].append(
            int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        )

    def release_large_and_redundant_subscriptions(self) -> None:
        """Stop consuming images and repeated calibration after first samples."""
        for subscription in self._image_subscriptions:
            self.destroy_subscription(subscription)
        self._image_subscriptions.clear()
        for subscription in self._info_subscriptions:
            self.destroy_subscription(subscription)
        self._info_subscriptions.clear()


def run_flir_preflight(
    *,
    timeout_sec: float = 20.0,
    sample_window_sec: float = 2.0,
    min_rate_hz: float = 115.0,
) -> dict[str, Any]:
    """Verify all cameras, briefly sample their rates, and capture calibration."""
    context = Context()
    rclpy.init(context=context)
    node = _FlirPreflightNode(context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    deadline = time.monotonic() + timeout_sec

    try:
        while (
            len(node.images_seen) < len(CAMERAS)
            or len(node.camera_info) < len(CAMERAS)
            or any(count == 0 for count in node.metadata_counts.values())
        ):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            executor.spin_once(timeout_sec=min(0.1, remaining))

        node.release_large_and_redundant_subscriptions()
        # Let executor queues and DDS discovery settle after removing the large
        # image subscriptions. Otherwise the first rate window can measure the
        # cleanup backlog instead of steady-state metadata delivery.
        settle_deadline = min(deadline, time.monotonic() + 1.0)
        while time.monotonic() < settle_deadline:
            executor.spin_once(
                timeout_sec=min(0.1, settle_deadline - time.monotonic())
            )
        node.metadata_counts = {camera: 0 for camera in CAMERAS}
        node.metadata_stamps_ns = {camera: [] for camera in CAMERAS}
        sample_start = time.monotonic()
        sample_deadline = min(deadline, sample_start + sample_window_sec)
        while time.monotonic() < sample_deadline:
            executor.spin_once(
                timeout_sec=min(0.1, sample_deadline - time.monotonic())
            )
        elapsed = max(time.monotonic() - sample_start, 1e-9)
        source_rates = {}
        for camera, stamps in node.metadata_stamps_ns.items():
            deltas = [
                current - previous
                for previous, current in zip(stamps, stamps[1:])
                if current > previous
            ]
            median_delta = statistics.median(deltas) if deltas else 0
            source_rates[camera] = (
                1_000_000_000.0 / median_delta if median_delta else 0.0
            )
        receive_rates = {
            camera: count / elapsed
            for camera, count in node.metadata_counts.items()
        }
        readiness_rates = {
            camera: min(source_rates[camera], receive_rates[camera])
            for camera in CAMERAS
        }
        status, failures = evaluate_preflight(
            node.images_seen, node.camera_info, readiness_rates, min_rate_hz
        )
        return {
            "status": status,
            "checked_at": datetime.now(timezone.utc).isoformat(),
            "sample_window_sec": elapsed,
            "minimum_metadata_rate_hz": min_rate_hz,
            "metadata_rates_hz": {
                camera: round(rate, 3)
                for camera, rate in source_rates.items()
            },
            "metadata_receive_rates_hz": {
                camera: round(rate, 3)
                for camera, rate in receive_rates.items()
            },
            "metadata_messages_received": node.metadata_counts,
            "images_received": sorted(node.images_seen),
            "camera_info": node.camera_info,
            "failures": failures,
        }
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)


def write_flir_preflight_report(
    session_dir: str | os.PathLike[str], report: dict[str, Any]
) -> Path:
    """Atomically publish readiness so task startup cannot observe partial YAML."""
    path = Path(session_dir).resolve() / "flir_preflight.yaml"
    temporary_path = path.with_suffix(".yaml.tmp")
    with temporary_path.open("w", encoding="utf-8") as file:
        yaml.safe_dump(report, file, sort_keys=True)
    os.replace(temporary_path, path)
    return path


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check all six FLIR streams without commanding the rig"
    )
    parser.add_argument("--session-dir", required=True)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--sample-window", type=float, default=2.0)
    parser.add_argument("--minimum-rate", type=float, default=115.0)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    report = run_flir_preflight(
        timeout_sec=args.timeout,
        sample_window_sec=args.sample_window,
        min_rate_hz=args.minimum_rate,
    )
    report_path = write_flir_preflight_report(args.session_dir, report)
    print(f"FLIR preflight {report['status']}: {report_path}", flush=True)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
