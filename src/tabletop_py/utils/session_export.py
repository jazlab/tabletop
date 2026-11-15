"""Offline Docker export with read-only inputs and explicit acquisition pausing."""

import argparse
import fcntl
import hashlib
import json
import os
import shlex
import shutil
import signal
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import yaml
from dotenv import dotenv_values

LIVE_SERVICES = {
    "tabletop-ur",
    "tabletop-flir",
    "tabletop-teensy",
    "tabletop-optitrack",
    "tabletop-eyelink",
}
IMAGE_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}
CONTAINER = "tabletop-offline-export"


@dataclass
class ExportPlan:
    workspace: Path
    bag: Path
    output: Path
    scratch: Path
    selected: dict
    excluded: dict
    original_services: list
    metadata_sha256: str
    image: str = "jazlabtabletop/ros-base"
    codec: str = "h264_nvenc"
    quality: int = 28
    jobs: int = 6


def read_metadata(bag: Path, skip_unmatched: bool):
    metadata_path = bag / "metadata.yaml"
    info = yaml.safe_load(metadata_path.read_text())[
        "rosbag2_bagfile_information"
    ]
    if info["storage_identifier"] != "mcap":
        raise RuntimeError("This launcher requires a finalized MCAP bag.")
    files = info.get("relative_file_paths", [])
    if not files:
        raise RuntimeError("Bag metadata contains no storage files.")
    for name in files:
        file = (bag / name).resolve(strict=True)
        if not file.is_relative_to(bag) or not file.is_file():
            raise RuntimeError(
                "Storage files must remain inside the source bag."
            )
    selected, excluded = {}, {}
    active_images = 0
    for item in info["topics_with_message_count"]:
        name = item["topic_metadata"]["name"]
        kind = item["topic_metadata"]["type"]
        count = item["message_count"]
        unmatched = "/image_raw_unmatched" in name and kind in IMAGE_TYPES
        if unmatched and count and not skip_unmatched:
            raise RuntimeError(
                "Unmatched images are present. To intentionally omit only these "
                "images, use --skip-unmatched-images. Metadata and originals remain."
            )
        target = excluded if unmatched and skip_unmatched else selected
        target[name] = count
        if target is selected and kind in IMAGE_TYPES and count:
            active_images += 1
    if (
        sum(selected.values()) + sum(excluded.values())
        != info["message_count"]
    ):
        raise RuntimeError("Source metadata topic counts do not reconcile.")
    return selected, excluded, active_images


def validate_manifest(manifest: dict, selected: dict) -> None:
    if not manifest.get("complete") or manifest.get("total_failed", 1):
        raise RuntimeError(
            "Export is incomplete or reports conversion failures."
        )
    total = sum(selected.values())
    if (
        manifest.get("total_expected") != total
        or manifest.get("total_succeeded") != total
    ):
        raise RuntimeError(
            "Export total differs from the selected source count."
        )
    topics = manifest["topics"]
    if len({item["topic"] for item in topics}) != len(topics):
        raise RuntimeError("Duplicate topic in export manifest.")
    actual = {item["topic"]: item["succeeded"] for item in topics}
    if (
        set(actual) - set(selected)
        or any(
            actual.get(name, 0) != count for name, count in selected.items()
        )
        or any(item.get("failed", 0) for item in topics)
    ):
        raise RuntimeError(
            "Per-topic export counts differ from source metadata."
        )


def existing_parent(path: Path) -> Path:
    while not path.exists():
        path = path.parent
    return path


def validate_nas(path: Path, expected_share: str) -> None:
    info = json.loads(
        subprocess.check_output(
            [
                "findmnt",
                "-J",
                "-T",
                str(existing_parent(path)),
                "-o",
                "TARGET,SOURCE,FSTYPE",
            ],
            text=True,
        )
    )["filesystems"][0]
    actual = info["source"].split("[", 1)[0].rstrip("/")
    if info["fstype"] != "cifs" or actual != expected_share.rstrip("/"):
        raise RuntimeError(
            "Expected NAS is not mounted; refusing a local fallback."
        )


def provenance(plan: ExportPlan) -> dict:
    return {
        "source_bag": str(plan.bag),
        "metadata_sha256": plan.metadata_sha256,
        "selected_topic_counts": plan.selected,
        "intentionally_omitted_image_topic_counts": plan.excluded,
        "originals_mounted_read_only": True,
        "metadata_for_omitted_images_is_still_selected": True,
    }


def preflight(args) -> ExportPlan:
    workspace = args.workspace.resolve(strict=True)
    session = args.session.resolve(strict=True)
    bag = (
        session / "bag"
        if (session / "bag/metadata.yaml").is_file()
        else session
    ).resolve(strict=True)
    if not (bag / "metadata.yaml").is_file():
        raise RuntimeError(
            "Provide a finalized session or bag directory with metadata.yaml."
        )
    session_root = bag.parent
    output = (args.output or session_root / "export").resolve()
    scratch_root = (
        args.scratch_root or workspace / ".cache/export-scratch"
    ).resolve()
    identity = hashlib.sha256(str(bag).encode()).hexdigest()[:10]
    scratch = scratch_root / f"{session_root.name}-{identity}"
    if output.is_relative_to(bag) or scratch.is_relative_to(bag):
        raise RuntimeError(
            "Output and scratch cannot be inside the source bag."
        )
    if output.is_relative_to(scratch) or scratch.is_relative_to(output):
        raise RuntimeError("Output and scratch directories cannot overlap.")
    settings = {**dotenv_values(workspace / ".env"), **os.environ}
    expected = args.nas_source or (
        settings.get("TABLETOP_NAS_SOURCE")
        if settings.get("TABLETOP_RECORDING_BACKEND") == "nas"
        else None
    )
    if settings.get("TABLETOP_RECORDING_BACKEND") == "nas" and not expected:
        raise RuntimeError(
            "NAS configuration requires an explicit expected share."
        )
    if expected:
        validate_nas(bag, expected)
        validate_nas(output, expected)
    for path, minimum, label in [
        (scratch, args.minimum_scratch_gb, "scratch"),
        (output, args.minimum_output_gb, "output"),
    ]:
        if (
            minimum < 0
            or shutil.disk_usage(existing_parent(path)).free < minimum * 10**9
        ):
            raise RuntimeError(
                f"Insufficient {label} space; reserve {minimum} GB."
            )
    selected, excluded, image_topics = read_metadata(
        bag, args.skip_unmatched_images
    )
    if args.video_codec == "h264_nvenc":
        if image_topics > 6:
            raise RuntimeError(
                "More than six active video topics: split the export or use libx264."
            )
        subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=name,driver_version",
                "--format=csv,noheader",
            ],
            check=True,
        )
    if not 0 <= args.video_quality <= 51 or args.jobs < 1:
        raise RuntimeError(
            "Video quality must be 0..51 and jobs must be positive."
        )
    subprocess.run(
        ["docker", "image", "inspect", args.image],
        check=True,
        stdout=subprocess.DEVNULL,
    )
    names = subprocess.check_output(
        ["docker", "ps", "--format", "{{.Names}}"], text=True
    ).splitlines()
    if any(
        name.startswith("tabletop-")
        and any(word in name for word in ("commander", "recorder", "export"))
        for name in names
    ):
        raise RuntimeError(
            "A task, recorder, or export is active; finish it first."
        )
    live = [name for name in names if name in LIVE_SERVICES]
    if live and not args.pause_rig:
        raise RuntimeError(
            "Live acquisition services exist. Use --pause-rig only while the rig is idle."
        )
    state = (
        json.loads(
            subprocess.check_output(["docker", "inspect", *live], text=True)
        )
        if live
        else []
    )
    original = [
        {"id": x["Id"], "name": x["Name"].lstrip("/")}
        for x in state
        if x["State"]["Running"]
    ]
    plan = ExportPlan(
        workspace,
        bag,
        output,
        scratch,
        selected,
        excluded,
        original,
        hashlib.sha256((bag / "metadata.yaml").read_bytes()).hexdigest(),
        args.image,
        args.video_codec,
        args.video_quality,
        args.jobs,
    )
    if output.exists() and any(output.iterdir()):
        marker = output / "export_provenance.json"
        if not marker.is_file() or json.loads(
            marker.read_text()
        ) != provenance(plan):
            raise RuntimeError(
                "Output is not empty or has different provenance; choose another --output."
            )
        manifest = output / "export_manifest.json"
        if manifest.exists():
            previous = json.loads(manifest.read_text())
            if previous.get("complete") and previous.get("total_failed") == 0:
                validate_manifest(previous, selected)
                raise RuntimeError(
                    "Export already complete; validate/reuse it rather than re-exporting."
                )
    return plan


def docker_command(plan: ExportPlan) -> list[str]:
    unbag = [
        "/tabletop/install/tabletop_unbag/lib/tabletop_unbag/unbag",
        "/input/bag",
        "-o",
        "/output",
        "--handlers",
        "csv",
        "video",
        "--video-codec",
        plan.codec,
        "--video-quality",
        str(plan.quality),
        "--csv-spool-dir",
        "/scratch",
        "--jobs",
        str(plan.jobs),
        "--minimum-free-gb",
        "20",
    ]
    if plan.excluded:
        unbag += ["--exclude-topics", *sorted(plan.excluded)]
    command = [
        "docker",
        "run",
        "--rm",
        "--init",
        "--name",
        CONTAINER,
        "--stop-signal",
        "SIGINT",
        "--user",
        f"{os.getuid()}:{os.getgid()}",
        "--network",
        "none",
    ]
    if plan.codec == "h264_nvenc":
        command += [
            "--runtime",
            "nvidia",
            "-e",
            "NVIDIA_VISIBLE_DEVICES=all",
            "-e",
            "NVIDIA_DRIVER_CAPABILITIES=video,compute,utility",
        ]
    for source, target, readonly in [
        (plan.workspace, "/tabletop", True),
        (plan.bag, "/input/bag", True),
        (plan.output, "/output", False),
        (plan.scratch, "/scratch", False),
    ]:
        command += [
            "--mount",
            f"type=bind,src={source},dst={target}"
            + (",readonly" if readonly else ""),
        ]
    command += [
        "--entrypoint",
        "bash",
        plan.image,
        "-lc",
        "source /opt/ros/jazzy/setup.bash && source /tabletop/install/setup.bash && exec "
        + shlex.join(unbag),
    ]
    return command


def run_export(plan: ExportPlan) -> int:
    plan.output.mkdir(parents=True, exist_ok=True)
    plan.scratch.mkdir(parents=True, exist_ok=True)
    (plan.output / "export_provenance.json").write_text(
        json.dumps(provenance(plan), indent=2)
    )
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    log_path = plan.output / f"export_{stamp}.log"
    (plan.output / f"export_services_{stamp}.json").write_text(
        json.dumps(plan.original_services, indent=2)
    )
    proc = None
    paused = False
    old_handler = signal.getsignal(signal.SIGINT)
    try:
        paused = True
        if plan.original_services:
            subprocess.run(
                [
                    "docker",
                    "stop",
                    "--timeout",
                    "30",
                    *[x["id"] for x in plan.original_services],
                ],
                check=True,
            )
        print(
            "Export starting. Keep this terminal open. Log:",
            log_path,
            flush=True,
        )
        with log_path.open("w") as log:
            proc = subprocess.Popen(
                docker_command(plan), stdout=log, stderr=subprocess.STDOUT
            )
            while True:
                try:
                    code = proc.wait(timeout=60)
                    break
                except subprocess.TimeoutExpired:
                    progress = [
                        line
                        for line in log_path.read_text(
                            errors="replace"
                        ).splitlines()
                        if line.startswith("Exporting:")
                    ]
                    print(
                        datetime.now().strftime("%H:%M:%S"),
                        progress[-1] if progress else "Preparing export",
                        flush=True,
                    )
        if code:
            raise RuntimeError(
                f"Exporter exited {code}; retain partial outputs for resume. See {log_path}"
            )
        validate_manifest(
            json.loads((plan.output / "export_manifest.json").read_text()),
            plan.selected,
        )
        print(
            "Export counts verified. Original recordings remain; completed CSV scratch files are removed.",
            flush=True,
        )
        return 0
    except KeyboardInterrupt:
        print(
            "Stopping export before restoring services; original recordings remain read-only.",
            flush=True,
        )
        return 130
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if proc is not None and proc.poll() is None:
                subprocess.run(
                    ["docker", "stop", "--timeout", "120", CONTAINER],
                    check=False,
                )
                proc.wait(timeout=30)
        finally:
            try:
                if paused and plan.original_services:
                    subprocess.run(
                        [
                            "docker",
                            "start",
                            *[x["id"] for x in plan.original_services],
                        ],
                        check=True,
                    )
                    print(
                        "Previously running services restored; no task was launched.",
                        flush=True,
                    )
            finally:
                signal.signal(signal.SIGINT, old_handler)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument(
        "session", type=Path, help="Finalized session or bag directory"
    )
    result.add_argument(
        "--workspace",
        type=Path,
        default=Path(
            os.environ.get("TABLETOP_DIR", Path(__file__).resolve().parents[3])
        ),
    )
    result.add_argument("--output", type=Path)
    result.add_argument("--scratch-root", type=Path)
    result.add_argument(
        "--nas-source",
        help="Expected //server/share; otherwise uses opted-in .env NAS configuration",
    )
    result.add_argument(
        "--pause-rig",
        action="store_true",
        help="Explicitly pause/restore only previously running acquisition services",
    )
    result.add_argument(
        "--skip-unmatched-images",
        action="store_true",
        help="Intentionally omit unmatched images, retaining metadata and originals",
    )
    result.add_argument(
        "--check",
        action="store_true",
        help="Read-only preflight; no directories, services, or exports changed",
    )
    result.add_argument(
        "--video-codec",
        choices=["h264_nvenc", "libx264"],
        default="h264_nvenc",
    )
    result.add_argument("--video-quality", type=int, default=28)
    result.add_argument("--jobs", type=int, default=6)
    result.add_argument("--minimum-scratch-gb", type=int, default=150)
    result.add_argument("--minimum-output-gb", type=int, default=250)
    result.add_argument("--image", default="jazlabtabletop/ros-base")
    return result


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    try:
        plan = preflight(args)
        print("Input (read-only):", plan.bag)
        print("Output:", plan.output)
        print("Scratch:", plan.scratch)
        print(
            f"Selected: {sum(plan.selected.values()):,}; intentionally omitted images: {sum(plan.excluded.values()):,}"
        )
        if args.check:
            print("Preflight passed. Nothing was paused or exported.")
            return 0
        cache = plan.workspace / ".cache"
        cache.mkdir(exist_ok=True)
        with (cache / "offline-export.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            # Recheck under the cross-session lock, before changing services.
            plan = preflight(args)
            return run_export(plan)
    except (
        OSError,
        ValueError,
        KeyError,
        RuntimeError,
        subprocess.SubprocessError,
    ) as error:
        print(f"Export launcher stopped: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
