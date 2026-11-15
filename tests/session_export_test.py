"""Offline launcher tests; Docker, GPU, and all hardware commands are mocked."""

import json
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

from tabletop_py.utils import session_export as export


@pytest.fixture
def bag(tmp_path):
    path = tmp_path / "session/bag"
    path.mkdir(parents=True)
    (path / "bag_0.mcap").write_bytes(b"test fixture")
    topics = [
        ("/cam/image_raw/compressed", "sensor_msgs/msg/CompressedImage", 2),
        (
            "/cam/image_raw_unmatched/compressed",
            "sensor_msgs/msg/CompressedImage",
            1,
        ),
        ("/cam/meta_unmatched", "flir_camera_msgs/msg/ImageMetaData", 1),
    ]
    info = {
        "storage_identifier": "mcap",
        "relative_file_paths": ["bag_0.mcap"],
        "message_count": 4,
        "topics_with_message_count": [
            {
                "topic_metadata": {"name": name, "type": kind},
                "message_count": count,
            }
            for name, kind, count in topics
        ],
    }
    (path / "metadata.yaml").write_text(
        yaml.safe_dump({"rosbag2_bagfile_information": info})
    )
    return path


@pytest.fixture
def plan(tmp_path, bag):
    selected, excluded, _ = export.read_metadata(bag, True)
    return export.ExportPlan(
        tmp_path,
        bag,
        tmp_path / "output",
        tmp_path / "scratch",
        selected,
        excluded,
        [{"id": "running-id", "name": "tabletop-flir"}],
        "sha256",
    )


def manifest(selected):
    return {
        "complete": True,
        "total_failed": 0,
        "total_expected": sum(selected.values()),
        "total_succeeded": sum(selected.values()),
        "topics": [
            {"topic": key, "succeeded": count, "failed": 0}
            for key, count in selected.items()
        ],
    }


def test_unmatched_omission_is_explicit_and_keeps_metadata(bag):
    with pytest.raises(RuntimeError, match="--skip-unmatched-images"):
        export.read_metadata(bag, False)
    selected, excluded, images = export.read_metadata(bag, True)
    assert images == 1
    assert sum(selected.values()) == 3
    assert sum(excluded.values()) == 1
    assert "/cam/meta_unmatched" in selected


def test_source_storage_path_cannot_escape(bag):
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text())
    other = bag.parent / "other.mcap"
    other.write_bytes(b"not a source storage file")
    metadata["rosbag2_bagfile_information"]["relative_file_paths"] = [
        "../other.mcap"
    ]
    (bag / "metadata.yaml").write_text(yaml.safe_dump(metadata))
    with pytest.raises(RuntimeError, match="inside"):
        export.read_metadata(bag, True)


def test_metadata_totals_must_reconcile(bag):
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text())
    metadata["rosbag2_bagfile_information"]["message_count"] = 100
    (bag / "metadata.yaml").write_text(yaml.safe_dump(metadata))
    with pytest.raises(RuntimeError, match="reconcile"):
        export.read_metadata(bag, True)


@pytest.mark.parametrize(
    "change",
    [
        {"complete": False},
        {"total_failed": 1},
        {"total_succeeded": 0},
        {"total_expected": 0},
    ],
)
def test_manifest_failure_is_not_success(plan, change):
    data = manifest(plan.selected)
    data.update(change)
    with pytest.raises(RuntimeError):
        export.validate_manifest(data, plan.selected)


def test_manifest_per_topic_and_duplicate_checks(plan):
    export.validate_manifest(manifest(plan.selected), plan.selected)
    data = manifest(plan.selected)
    data["topics"][0]["succeeded"] = 0
    with pytest.raises(RuntimeError, match="Per-topic"):
        export.validate_manifest(data, plan.selected)
    data = manifest(plan.selected)
    data["topics"].append(data["topics"][0])
    with pytest.raises(RuntimeError, match="Duplicate"):
        export.validate_manifest(data, plan.selected)


def test_docker_is_offline_with_read_only_inputs(plan):
    command = export.docker_command(plan)
    assert command[command.index("--network") + 1] == "none"
    assert f"type=bind,src={plan.bag},dst=/input/bag,readonly" in command
    assert f"type=bind,src={plan.workspace},dst=/tabletop,readonly" in command
    assert (
        "--exclude-topics /cam/image_raw_unmatched/compressed" in command[-1]
    )
    assert "--overwrite" not in command[-1]
    assert "tt-launch" not in command[-1]
    assert "--runtime" in command
    assert "--runtime" not in export.docker_command(
        replace(plan, codec="libx264")
    )


@pytest.mark.parametrize("mode", ["success", "failure", "interrupt"])
def test_only_original_services_restored_after_export_or_interrupt(
    plan, monkeypatch, mode
):
    calls = []

    class Process:
        code = None

        def wait(self, timeout=None):
            if mode == "interrupt" and self.code is None:
                raise KeyboardInterrupt
            if self.code is None:
                self.code = 1 if mode == "failure" else 0
                (plan.output / "export_manifest.json").write_text(
                    json.dumps(manifest(plan.selected))
                )
            return self.code

        def poll(self):
            return self.code

    process = Process()

    def run(command, **kwargs):
        calls.append(command)
        if command[-1] == export.CONTAINER:
            process.code = 130

    monkeypatch.setattr(export.subprocess, "run", run)
    monkeypatch.setattr(export.subprocess, "Popen", lambda *a, **kw: process)
    if mode == "failure":
        with pytest.raises(RuntimeError, match="exited 1"):
            export.run_export(plan)
    else:
        assert export.run_export(plan) == (130 if mode == "interrupt" else 0)
    assert calls[0] == ["docker", "stop", "--timeout", "30", "running-id"]
    assert calls[-1] == ["docker", "start", "running-id"]
    if mode == "interrupt":
        assert calls[-2] == [
            "docker",
            "stop",
            "--timeout",
            "120",
            export.CONTAINER,
        ]
    assert json.loads((plan.output / "export_provenance.json").read_text())[
        "originals_mounted_read_only"
    ]


def test_previously_stopped_services_are_not_started(plan, monkeypatch):
    plan = replace(plan, original_services=[])
    runner = Mock()
    monkeypatch.setattr(export.subprocess, "run", runner)

    class Process:
        def wait(self, **kwargs):
            (plan.output / "export_manifest.json").write_text(
                json.dumps(manifest(plan.selected))
            )
            return 0

        def poll(self):
            return 0

    monkeypatch.setattr(export.subprocess, "Popen", lambda *a, **kw: Process())
    assert export.run_export(plan) == 0
    runner.assert_not_called()


def test_check_creates_no_directories_and_changes_no_services(
    plan, monkeypatch
):
    monkeypatch.setattr(export, "preflight", lambda args: plan)
    monkeypatch.setattr(
        export, "run_export", lambda plan: pytest.fail("must not export")
    )
    assert export.main([str(plan.bag), "--check"]) == 0
    assert not plan.output.exists()
    assert not plan.scratch.exists()
    assert not (plan.workspace / ".cache").exists()


def test_nas_missing_or_wrong_share_cannot_silently_fall_back(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(
        export.subprocess,
        "check_output",
        lambda *a, **kw: json.dumps(
            {
                "filesystems": [
                    {
                        "target": str(tmp_path),
                        "source": "/dev/nvme0n1",
                        "fstype": "ext4",
                    }
                ],
            }
        ),
    )
    with pytest.raises(RuntimeError, match="local fallback"):
        export.validate_nas(tmp_path, "//nas/share")


def mock_preflight_commands(monkeypatch, names):
    monkeypatch.setenv("TABLETOP_RECORDING_BACKEND", "local")
    monkeypatch.setattr(export.subprocess, "run", Mock())

    def output(command, **kwargs):
        if command[:2] == ["docker", "ps"]:
            return "\n".join(names)
        if command[:2] == ["docker", "inspect"]:
            return json.dumps(
                [
                    {
                        "Id": name + "-id",
                        "Name": "/" + name,
                        "State": {"Running": True},
                    }
                    for name in command[2:]
                ]
            )
        raise AssertionError(command)

    monkeypatch.setattr(export.subprocess, "check_output", output)
    monkeypatch.setattr(
        export.shutil, "disk_usage", lambda path: SimpleNamespace(free=10**15)
    )


def test_pausing_acquisition_requires_explicit_opt_in(plan, monkeypatch):
    mock_preflight_commands(monkeypatch, ["tabletop-flir"])
    args = export.parser().parse_args(
        [
            str(plan.bag),
            "--workspace",
            str(plan.workspace),
            "--skip-unmatched-images",
        ]
    )
    with pytest.raises(RuntimeError, match="--pause-rig"):
        export.preflight(args)
    args.pause_rig = True
    result = export.preflight(args)
    assert result.original_services == [
        {"id": "tabletop-flir-id", "name": "tabletop-flir"}
    ]
    assert not result.output.exists()


def test_active_task_refused_even_with_pause_flag(plan, monkeypatch):
    mock_preflight_commands(monkeypatch, ["tabletop-commander"])
    args = export.parser().parse_args(
        [
            str(plan.bag),
            "--workspace",
            str(plan.workspace),
            "--skip-unmatched-images",
            "--pause-rig",
        ]
    )
    with pytest.raises(RuntimeError, match="active"):
        export.preflight(args)


def test_unrelated_nonempty_output_is_not_overwritten(plan, monkeypatch):
    mock_preflight_commands(monkeypatch, [])
    plan.output.mkdir()
    (plan.output / "keep.txt").write_text("user data")
    args = export.parser().parse_args(
        [
            str(plan.bag),
            "--workspace",
            str(plan.workspace),
            "--output",
            str(plan.output),
            "--skip-unmatched-images",
        ]
    )
    with pytest.raises(RuntimeError, match="not empty"):
        export.preflight(args)
    assert (plan.output / "keep.txt").read_text() == "user data"
