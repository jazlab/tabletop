"""Pipeline stages [0]–[3]: inventory → extract → sync → write."""

from __future__ import annotations

import json
import time
from pathlib import Path

from .bag import summarize
from .extract import CAMERAS, extract
from .session import inventory
from .sync import run_sync
from .timeline import build_timeline
from .write_task import write_task
from .write_video import write_video

FILES = ("task.nwb", "video.nwb", "ephys.nwb", "spike_sorting.nwb")


def convert(
    session_dir: Path,
    out_dir: Path | None = None,
    profile: str | None = None,
    log=print,
) -> dict:
    session_dir = Path(session_dir).resolve()
    out_dir = Path(out_dir) if out_dir else session_dir / "nwb"
    out_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.time()

    log(f"[0] inventory {session_dir.name}")
    inv = inventory(session_dir, profile)
    log(
        f"    profile: {inv.profile} ({inv.profile_source}): {'; '.join(inv.profile_reasons)}"
    )
    (out_dir / "inventory.json").write_text(
        json.dumps(inv.to_json(), indent=1, default=str)
    )

    log("[1] extract")
    summary = summarize(Path(inv.bag_dir))
    ex = extract(Path(inv.bag_dir), summary)

    log("[2] sync")
    sync = run_sync(ex, inv.bag_start_ns, inv.bag_end_ns)
    for name, f in sync.fits.items():
        extra = (
            (
                f" matched {f.n_matched}/{f.n_stream_pulses}, rms {f.residual_rms_ms:.2f} ms, "
                f"drift {f.drift_ppm:.1f} ppm"
            )
            if f.status == "ok"
            else (f" ({f.note})" if f.note else "")
        )
        log(f"    {name:10s} {f.method:10s} {f.status}{extra}")
    (out_dir / "sync_report.json").write_text(
        json.dumps(sync.report(), indent=1, default=str)
    )
    tl = build_timeline(ex, sync)

    log("[3] write")
    manifest: dict = {
        "session_id": inv.session_id,
        "profile": inv.profile,
        "session_start_ns": tl.start_ns,
        "files": {},
    }
    counts = write_task(inv, ex, sync, tl, out_dir / "task.nwb")
    manifest["files"]["task.nwb"] = {"status": "written", "counts": counts}
    log("    task.nwb written")

    has_video = bool(inv.video_dirs) or any(
        ex.has(f"cam_meta_{c}") for c in CAMERAS
    )
    if has_video:
        vcounts = write_video(inv, ex, sync, tl, out_dir / "video.nwb")
        manifest["files"]["video.nwb"] = {
            "status": "written",
            "counts": vcounts,
            "note": ""
            if inv.video_dirs
            else "camera metadata only: no unbag video segments found",
        }
        log("    video.nwb written")
    else:
        manifest["files"]["video.nwb"] = {
            "status": "absent",
            "reason": "no camera frames or metadata recorded",
        }

    if inv.spikeglx_meta:
        manifest["files"]["ephys.nwb"] = {
            "status": "not_implemented",
            "reason": "SpikeGLX data present but the ephys writer is not implemented "
            "in this version (see PLAN.md To-do)",
        }
        log("    ephys.nwb NOT written: writer not implemented yet")
    else:
        manifest["files"]["ephys.nwb"] = {
            "status": "absent",
            "reason": f"profile {inv.profile}: no SpikeGLX data",
        }
    manifest["files"]["spike_sorting.nwb"] = {
        "status": "absent",
        "reason": "spike sorting is a later stage",
    }
    manifest["elapsed_s"] = round(time.time() - t0, 1)
    (out_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=1, default=str)
    )
    log(f"    done in {manifest['elapsed_s']} s → {out_dir}")
    return manifest
