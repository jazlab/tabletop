"""Write ``video.nwb``: FLIR frames as external H.264 segments + per-frame metadata.

Frames come from ``tabletop_unbag --handlers video`` output
(``<session>/unbag/cam_sync_<cam>_image_raw_video/segment_NNNNNN.mkv`` with a
``segment_NNNNNN_timestamps.csv`` sidecar). The NWB file references the
segments by relative path; it does not copy pixels (D4 is still open).
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import pandas as pd
from pynwb import TimeSeries
from pynwb.image import ImageSeries

from .extract import CAMERAS, Extracted
from .nwb_common import compressed, new_nwbfile, provenance_table, write
from .session import Inventory
from .sync import SyncResult
from .timeline import Timeline


def read_sidecars(
    session_dir: Path, segments: list[str]
) -> tuple[pd.DataFrame, list[int]]:
    frames, starting = [], []
    n = 0
    for seg in segments:
        sc = session_dir / seg.replace(".mkv", "_timestamps.csv")
        df = pd.read_csv(sc)
        starting.append(n)
        n += len(df)
        df["segment"] = seg
        frames.append(df)
    return (
        pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()
    ), starting


def dropped_frames(frame_ids: np.ndarray) -> list[tuple[int, int, int]]:
    """(index_before_gap, missing_count, first_missing_id) for gaps in camera_frame_id."""
    d = np.diff(frame_ids.astype(np.int64))
    return [
        (int(i), int(d[i] - 1), int(frame_ids[i] + 1))
        for i in np.flatnonzero(d > 1)
    ]


def write_video(
    inv: Inventory, ex: Extracted, sync: SyncResult, tl: Timeline, out: Path
) -> dict:
    session_dir = Path(inv.session_dir)
    nwb = new_nwbfile(
        inv,
        tl,
        "video",
        "TableTop FLIR cameras (external video segments) and per-frame metadata.",
    )
    counts: dict[str, int] = {}
    cam_fit = sync.fits.get("cameras")
    use_fit = cam_fit is not None and cam_fit.status == "ok"

    def header_to_rel(ns: np.ndarray) -> np.ndarray:
        x = tl.host_ns_to_origin_s(ns)
        return (
            cam_fit.to_rel_s(x, tl.start_ns)
            if use_fit
            else (x - (tl.start_ns - tl.origin_ns) / 1e9)
        )

    meta_mod = nwb.create_processing_module(
        "camera_metadata", "Per-frame FLIR metadata from /cam_sync/<cam>/meta."
    )
    drops = []
    for cam in CAMERAS:
        segs = inv.video_dirs.get(cam, [])
        if segs:
            frames, starting = read_sidecars(session_dir, segs)
            rel_paths = [
                os.path.relpath(session_dir / s, out.parent) for s in segs
            ]
            nwb.add_acquisition(
                ImageSeries(
                    name=cam,
                    external_file=rel_paths,
                    starting_frame=starting,
                    format="external",
                    timestamps=compressed(
                        header_to_rel(frames["header_stamp_ns"].to_numpy())
                    ),
                    unit="n/a",
                    description=f"{cam}: H.264 proxy segments from tabletop_unbag; per-frame timestamps from the "
                    f"segment sidecars' header_stamp_ns via "
                    f"{'pulse_fit (Line3 on ' + cam_fit.stream + ')' if use_fit else 'host clock'}.",
                )
            )
            counts[f"frames_{cam}"] = len(frames)
        meta = ex.tables.get(f"cam_meta_{cam}")
        if meta is not None and len(meta):
            t = TimeSeries(
                name=f"{cam}_exposure_time",
                data=compressed(meta.exposure_time.to_numpy()),
                timestamps=compressed(
                    header_to_rel(meta.header_ns.to_numpy())
                ),
                unit="microseconds",
                description="exposure time",
            )
            meta_mod.add(t)
            for col, unit in [
                ("gain", "dB"),
                ("line_status", "n/a (bit 3 = Line3 sync)"),
                ("camera_frame_id", "n/a"),
                ("brightness", "n/a"),
                ("synchronized_timestamp_valid", "n/a"),
            ]:
                meta_mod.add(
                    TimeSeries(
                        name=f"{cam}_{col}",
                        data=compressed(
                            meta[col].to_numpy().astype(np.float64)
                            if col == "gain"
                            else meta[col].to_numpy().astype(np.int64)
                        ),
                        timestamps=t,
                        unit=unit,
                        description=col,
                    )
                )
            ts_rel = header_to_rel(meta.header_ns.to_numpy())
            for i, n_missing, first_id in dropped_frames(
                meta.camera_frame_id.to_numpy()
            ):
                drops.append(
                    (ts_rel[i], ts_rel[i + 1], cam, n_missing, first_id)
                )
            counts[f"meta_{cam}"] = len(meta)
        um = ex.tables.get(f"cam_meta_unmatched_{cam}")
        counts[f"unmatched_{cam}"] = 0 if um is None else len(um)
    if drops:
        nwb.add_time_intervals(_drops_table(drops))
    prov = nwb.create_processing_module(
        "provenance", "How this file was made."
    )
    prov.add(
        provenance_table(
            inv,
            tl,
            {
                "sync_report": json.dumps(sync.report(), default=str),
                "written_counts": json.dumps(counts),
            },
        )
    )
    write(nwb, out)
    return counts


def _drops_table(drops):
    from pynwb.epoch import TimeIntervals

    ti = TimeIntervals(
        name="dropped_frames",
        description="Gaps in camera_frame_id (frames the camera produced but the "
        "recording does not contain).",
    )
    ti.add_column("camera", "camera name")
    ti.add_column("n_missing", "number of missing frame ids")
    ti.add_column("first_missing_frame_id", "first missing camera_frame_id")
    for start, stop, cam, n, fid in drops:
        ti.add_interval(
            start_time=float(start),
            stop_time=float(stop),
            camera=cam,
            n_missing=n,
            first_missing_frame_id=fid,
        )
    return ti
