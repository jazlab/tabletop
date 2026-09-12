"""Fast post-session checks for completeness of recorded task data."""

import argparse
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

CAMERAS = (
    "left_front_top_cam",
    "left_back_top_cam",
    "left_bottom_cam",
    "right_front_top_cam",
    "right_back_top_cam",
    "right_bottom_cam",
)
REQUIRED_ROBOT_TOPICS = (
    "/joint_states",
    "/dynamic_joint_states",
    "/left_io_and_status_controller/io_states",
    "/left_tcp_pose_broadcaster/pose",
    "/right_tcp_pose_broadcaster/pose",
    "/left_force_torque_sensor_broadcaster/wrench",
    "/right_force_torque_sensor_broadcaster/wrench",
)
REQUIRED_OPTITRACK_TOPICS = ("/markers", "/rigid_bodies")
REQUIRED_TASK_TOPICS = ("/tabletop/task_event",)
EYELINK_REPORT_WAIT_SEC = 30.0


def _topic_counts(session_dir: Path) -> dict[str, int]:
    metadata_path = session_dir / "bag" / "metadata.yaml"
    with metadata_path.open(encoding="utf-8") as file:
        info = yaml.safe_load(file)["rosbag2_bagfile_information"]
    return {
        item["topic_metadata"]["name"]: int(item["message_count"])
        for item in info["topics_with_message_count"]
    }


def _add_check(
    checks: list[dict[str, Any]], level: str, name: str, detail: str
) -> None:
    checks.append({"level": level, "name": name, "detail": detail})


def _wait_for_eyelink_report(
    session_path: Path, timeout_sec: float = EYELINK_REPORT_WAIT_SEC
) -> None:
    """Allow bounded EyeLink EDF/CSV finalization to finish before reporting."""
    report_path = session_path / "eyelink_integrity.json"
    if report_path.exists() or timeout_sec <= 0:
        return
    if _topic_counts(session_path).get("/eyelink/sample_array", 0) <= 0:
        return

    deadline = time.monotonic() + timeout_sec
    while not report_path.exists():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(0.1, remaining))


def build_session_report(
    session_dir: str | os.PathLike[str],
) -> dict[str, Any]:
    """Build a quick count-based integrity report without scanning MCAP data."""
    session_path = Path(session_dir).resolve()
    counts = _topic_counts(session_path)
    checks: list[dict[str, Any]] = []
    preflight_path = session_path / "flir_preflight.yaml"
    if preflight_path.exists():
        with preflight_path.open(encoding="utf-8") as file:
            flir_preflight = yaml.safe_load(file) or {}
    else:
        flir_preflight = {}
    calibration = flir_preflight.get("camera_info", {})
    preflight_ok = flir_preflight.get("status") == "PASS"

    for camera in CAMERAS:
        prefix = f"/cam_sync/{camera}"
        image_count = counts.get(f"{prefix}/image_raw/compressed", 0)
        if image_count == 0:
            image_count = counts.get(f"{prefix}/image_raw", 0)
        meta_count = counts.get(f"{prefix}/meta", 0)
        unmatched_image_count = counts.get(
            f"{prefix}/image_raw_unmatched/compressed", 0
        )
        if unmatched_image_count == 0:
            unmatched_image_count = counts.get(
                f"{prefix}/image_raw_unmatched", 0
            )
        unmatched_meta_count = counts.get(f"{prefix}/meta_unmatched", 0)
        if not image_count and not unmatched_image_count:
            _add_check(checks, "ERROR", camera, "no recorded images")
            continue
        meta_delta = image_count - meta_count
        unmatched_meta_delta = unmatched_image_count - unmatched_meta_count
        calibration_saved = camera in calibration
        level = (
            "PASS"
            if meta_delta == 0
            and unmatched_meta_delta == 0
            and unmatched_image_count == 0
            and calibration_saved
            and preflight_ok
            else "WARN"
        )
        _add_check(
            checks,
            level,
            camera,
            f"images={image_count}, meta={meta_count} "
            f"(image-meta={meta_delta}), "
            f"archived_unmatched_images={unmatched_image_count}, "
            f"archived_unmatched_meta={unmatched_meta_count} "
            f"(image-meta={unmatched_meta_delta}), "
            f"camera_info_snapshot={calibration_saved}, "
            f"preflight={flir_preflight.get('status', 'absent')}",
        )

    eyelink_path = session_path / "eyelink_integrity.json"
    if eyelink_path.exists():
        with eyelink_path.open(encoding="utf-8") as file:
            eyelink = json.load(file)
        stats = eyelink.get("stats") or {}
        published = int(stats.get("batches", 0))
        source_samples = int(stats.get("samples", 0))
        published_samples = int(stats.get("published_samples", source_samples))
        unpublished_samples = source_samples - published_samples
        recorded = counts.get("/eyelink/sample_array", 0)
        shortfall = published - recorded
        tracker_missing = int(stats.get("estimated_missing_samples", 0))
        status_ok = eyelink.get("status") == "complete"
        edf_saved = bool(eyelink.get("edf_saved"))
        level = (
            "PASS"
            if status_ok
            and edf_saved
            and shortfall == 0
            and tracker_missing == 0
            and unpublished_samples == 0
            else "WARN"
        )
        _add_check(
            checks,
            level,
            "eyelink",
            f"published_batches={published}, recorded_batches={recorded}, "
            f"shortfall={shortfall}, tracker_missing={tracker_missing}, "
            f"source_samples={source_samples}, "
            f"unpublished_samples={unpublished_samples}, "
            f"edf_saved={edf_saved}",
        )
    else:
        _add_check(
            checks,
            "WARN",
            "eyelink",
            "source integrity report absent (EyeLink may not have recorded)",
        )

    for topic in REQUIRED_TASK_TOPICS:
        count = counts.get(topic, 0)
        _add_check(
            checks,
            "PASS" if count else "ERROR",
            topic,
            f"messages={count}",
        )
    for topic in REQUIRED_ROBOT_TOPICS:
        count = counts.get(topic, 0)
        _add_check(
            checks,
            "PASS" if count else "ERROR",
            topic,
            f"messages={count}",
        )
    for topic in REQUIRED_OPTITRACK_TOPICS:
        count = counts.get(topic, 0)
        _add_check(
            checks,
            "PASS" if count else "WARN",
            topic,
            f"messages={count}",
        )

    overall = "PASS"
    if any(check["level"] == "ERROR" for check in checks):
        overall = "ERROR"
    elif any(check["level"] == "WARN" for check in checks):
        overall = "WARN"
    return {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "session_dir": str(session_path),
        "overall": overall,
        "checks": checks,
    }


def generate_session_report(
    session_dir: str | os.PathLike[str],
    *,
    eyelink_report_timeout_sec: float = EYELINK_REPORT_WAIT_SEC,
) -> dict[str, Any]:
    """Build, atomically save, and print a concise integrity report."""
    _wait_for_eyelink_report(
        Path(session_dir).resolve(), eyelink_report_timeout_sec
    )
    report = build_session_report(session_dir)
    report_path = Path(session_dir).resolve() / "session_integrity.json"
    temporary_path = report_path.with_suffix(".json.tmp")
    with temporary_path.open("w", encoding="utf-8") as file:
        json.dump(report, file, indent=2, sort_keys=True)
        file.write("\n")
    os.replace(temporary_path, report_path)

    print(f"Session integrity: {report['overall']}", flush=True)
    for check in report["checks"]:
        if check["level"] != "PASS":
            print(
                f"  {check['level']} {check['name']}: {check['detail']}",
                flush=True,
            )
    print(f"Integrity report: {report_path}", flush=True)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "session_dir",
        nargs="?",
        default=os.path.join(os.environ.get("ROS_BAG_DIR", "/bags"), "latest"),
    )
    args = parser.parse_args()
    report = generate_session_report(args.session_dir)
    return 0 if report["overall"] != "ERROR" else 1


if __name__ == "__main__":
    raise SystemExit(main())
