"""NAS recording safeguards shared by host tools and the recorder (stdlib only)."""

import json
import os
import re
import subprocess
import time
from copy import deepcopy
from pathlib import Path


def mount_info(path: str, *, exact: bool = True) -> dict:
    result = subprocess.run(
        [
            "findmnt",
            "--json",
            "--mountpoint" if exact else "--target",
            str(Path(path).resolve()),
            "-o",
            "TARGET,SOURCE,FSTYPE,OPTIONS",
        ],
        capture_output=True,
        text=True,
        timeout=10,
    )
    if result.returncode:
        raise RuntimeError(f"Recording storage is not mounted: {path}")
    return json.loads(result.stdout)["filesystems"][0]


def validate_nas_storage(directory: str, mountpoint: str, source: str) -> dict:
    """Refuse local-directory fallback, wrong shares, and unvalidated caching."""
    if not mountpoint or not source:
        raise RuntimeError(
            "NAS recording requires its mountpoint and expected //server/share"
        )
    info = mount_info(mountpoint)
    actual_source = info["source"].split("[", 1)[0].rstrip("/")
    if info["fstype"] != "cifs" or actual_source != source.rstrip("/"):
        raise RuntimeError(
            f"Wrong recording filesystem: {info['fstype']} {info['source']}; expected {source}"
        )
    options = set(info["options"].split(","))
    missing = {"cache=none", "mfsymlinks"} - options
    if missing:
        raise RuntimeError(f"NAS mount requires: {', '.join(sorted(missing))}")
    if "ro" in options:
        raise RuntimeError("NAS recording mount is read-only")
    root = Path(mountpoint).resolve(strict=True)
    directory_path = Path(directory).resolve(strict=True)
    if not directory_path.is_relative_to(root) or not directory_path.is_dir():
        raise RuntimeError(
            f"Recording directory is outside the expected NAS mount: {directory}"
        )
    target_info = mount_info(str(directory_path), exact=False)
    if Path(target_info["target"]).resolve() != root:
        raise RuntimeError(
            "Recording directory crosses into another filesystem"
        )
    free = os.statvfs(directory_path)
    info["available_bytes"] = free.f_bavail * free.f_frsize
    return info


def validate_recording_environment(directory: str) -> dict | None:
    backend = os.environ.get("TABLETOP_RECORDING_BACKEND") or "local"
    if backend not in ("local", "nas"):
        raise RuntimeError("TABLETOP_RECORDING_BACKEND must be local or nas")
    if backend != "nas":
        return None
    return validate_nas_storage(
        directory,
        os.environ.get("TABLETOP_NAS_MOUNT", "/bags"),
        os.environ.get("TABLETOP_NAS_SOURCE", ""),
    )


def recorder_qos_profiles(profiles: dict, *, nas: bool) -> dict:
    profiles = deepcopy(profiles)
    if nas:
        # Reader-local, bounded queues: no publisher backpressure and no raw
        # acquisition-buffer or robot-controller changes. Four seconds at 120 Hz.
        for topic, profile in profiles.items():
            if "/image_raw" in topic:
                profile.update(
                    history="keep_last", depth=512, reliability="best_effort"
                )
    return profiles


def cifs_read_bytes(text: str, source: str) -> int:
    """Aggregate kernel counters for the selected SMB share, not local disks."""
    wanted = source.rstrip("/").replace("\\", "/").lower()
    selected = False
    values = []
    for line in text.splitlines():
        match = re.match(r"\s*\d+\)\s+(\\\\\S+|//\S+)", line)
        if match:
            selected = (
                match[1].rstrip("/").replace("\\", "/").lower() == wanted
            )
        elif selected:
            match = re.search(r"Bytes read:\s*(\d+)", line)
            if match:
                values.append(int(match[1]))
    if not values:
        raise RuntimeError(
            f"No readable CIFS I/O counters for {source}; cannot verify recording storage"
        )
    return sum(values)


def check_nas_idle(
    directory: str,
    mountpoint: str,
    source: str,
    seconds: int = 5,
    max_read_mib_s: int = 5,
) -> dict:
    if seconds < 1 or max_read_mib_s < 0:
        raise ValueError(
            "Storage sampling settings must be nonnegative whole numbers (seconds >= 1)"
        )
    info = validate_nas_storage(directory, mountpoint, source)
    stats = Path("/proc/fs/cifs/Stats")
    before = cifs_read_bytes(stats.read_text(), source)
    started = time.monotonic()
    time.sleep(seconds)
    after = cifs_read_bytes(stats.read_text(), source)
    validate_nas_storage(directory, mountpoint, source)
    elapsed = time.monotonic() - started
    if after < before:
        raise RuntimeError(
            "NAS I/O counters reset during preflight; retry after the connection is stable"
        )
    rate = (after - before) / elapsed / 1024**2
    if rate > max_read_mib_s:
        raise RuntimeError(
            f"NAS has {rate:.1f} MiB/s of competing reads; stop analysis/copying before recording"
        )
    info["read_mib_s"] = rate
    return info
