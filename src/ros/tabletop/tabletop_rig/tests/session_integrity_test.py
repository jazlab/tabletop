import json
import threading
import time

import yaml
from tabletop_rig.utils.session_integrity import (
    CAMERAS,
    REQUIRED_OPTITRACK_TOPICS,
    REQUIRED_ROBOT_TOPICS,
    REQUIRED_TASK_TOPICS,
    build_session_report,
    generate_session_report,
)


def _write_metadata(session_dir, counts):
    bag_dir = session_dir / "bag"
    bag_dir.mkdir()
    topics = [
        {
            "topic_metadata": {
                "name": topic,
                "type": "std_msgs/msg/String",
                "serialization_format": "cdr",
            },
            "message_count": count,
        }
        for topic, count in counts.items()
    ]
    with (bag_dir / "metadata.yaml").open("w") as file:
        yaml.safe_dump(
            {
                "rosbag2_bagfile_information": {
                    "topics_with_message_count": topics
                }
            },
            file,
        )


def _write_flir_preflight(session_dir):
    with (session_dir / "flir_preflight.yaml").open("w") as file:
        yaml.safe_dump(
            {
                "status": "PASS",
                "camera_info": {camera: {} for camera in CAMERAS},
            },
            file,
        )


def _complete_counts():
    counts = {
        topic: 100
        for topic in (
            *REQUIRED_TASK_TOPICS,
            *REQUIRED_ROBOT_TOPICS,
            *REQUIRED_OPTITRACK_TOPICS,
        )
    }
    for camera in CAMERAS:
        prefix = f"/cam_sync/{camera}"
        counts[f"{prefix}/image_raw/compressed"] = 1000
        counts[f"{prefix}/meta"] = 1000
    counts["/eyelink/sample_array"] = 25
    return counts


def test_complete_session_passes_and_is_written(tmp_path):
    _write_metadata(tmp_path, _complete_counts())
    _write_flir_preflight(tmp_path)
    (tmp_path / "eye.edf").write_bytes(b"edf")
    with (tmp_path / "eyelink_integrity.json").open("w") as file:
        json.dump(
            {
                "status": "complete",
                "edf_saved": True,
                "stats": {
                    "batches": 25,
                    "estimated_missing_samples": 0,
                },
            },
            file,
        )

    report = generate_session_report(tmp_path)

    assert report["overall"] == "PASS"
    assert (
        json.loads((tmp_path / "session_integrity.json").read_text())[
            "overall"
        ]
        == "PASS"
    )


def test_camera_metadata_and_eyelink_shortfalls_warn(tmp_path):
    counts = _complete_counts()
    counts["/cam_sync/left_bottom_cam/meta"] = 997
    counts["/eyelink/sample_array"] = 23
    _write_metadata(tmp_path, counts)
    _write_flir_preflight(tmp_path)
    with (tmp_path / "eyelink_integrity.json").open("w") as file:
        json.dump(
            {
                "status": "complete",
                "edf_saved": True,
                "stats": {
                    "batches": 25,
                    "estimated_missing_samples": 0,
                },
            },
            file,
        )

    report = build_session_report(tmp_path)

    assert report["overall"] == "WARN"
    details = {check["name"]: check for check in report["checks"]}
    assert details["left_bottom_cam"]["level"] == "WARN"
    assert "image-meta=3" in details["left_bottom_cam"]["detail"]
    assert details["eyelink"]["level"] == "WARN"
    assert "shortfall=2" in details["eyelink"]["detail"]


def test_archived_unmatched_camera_frames_are_reported(tmp_path):
    counts = _complete_counts()
    prefix = "/cam_sync/left_back_top_cam"
    counts[f"{prefix}/image_raw_unmatched/compressed"] = 7
    counts[f"{prefix}/meta_unmatched"] = 7
    _write_metadata(tmp_path, counts)
    _write_flir_preflight(tmp_path)
    with (tmp_path / "eyelink_integrity.json").open("w") as file:
        json.dump(
            {
                "status": "complete",
                "edf_saved": True,
                "stats": {"batches": 25, "estimated_missing_samples": 0},
            },
            file,
        )

    report = build_session_report(tmp_path)

    details = {check["name"]: check for check in report["checks"]}
    assert details["left_back_top_cam"]["level"] == "WARN"
    assert "archived_unmatched_images=7" in details[
        "left_back_top_cam"
    ]["detail"]
    assert "archived_unmatched_meta=7" in details["left_back_top_cam"][
        "detail"
    ]


def test_generate_waits_for_delayed_eyelink_source_report(tmp_path):
    _write_metadata(tmp_path, _complete_counts())
    _write_flir_preflight(tmp_path)

    def write_report():
        time.sleep(0.05)
        with (tmp_path / "eyelink_integrity.json").open("w") as file:
            json.dump(
                {
                    "status": "complete",
                    "edf_saved": True,
                    "stats": {
                        "batches": 25,
                        "estimated_missing_samples": 0,
                    },
                },
                file,
            )

    writer = threading.Thread(target=write_report)
    writer.start()
    report = generate_session_report(tmp_path, eyelink_report_timeout_sec=1.0)
    writer.join()

    details = {check["name"]: check for check in report["checks"]}
    assert details["eyelink"]["level"] == "PASS"
    assert "recorded_batches=25" in details["eyelink"]["detail"]
