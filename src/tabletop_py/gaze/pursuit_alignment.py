"""Create a pupil-corrected model input from a pursuit + 27-hold CSV export."""

import argparse
import re
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.ndimage import uniform_filter1d

from tabletop_py.gaze.preprocess import (
    EYELINK_DATA_COLS,
    eyelink_array_to_samples,
)
from tabletop_py.gaze.target_marker import (
    _OPTITRACK_IN_WORLD,
    _RIGHT_BASE_IN_WORLD,
    _WORLD_FROM_OPTITRACK,
    _WORLD_FROM_RIGHT_BASE,
    _stamp_ns,
    select_target_marker,
)

_STAMP = ["header.stamp.sec", "header.stamp.nanosec"]
_HOLD = re.compile(r"^STATIC_HOLD_(START|END) point=(\d+)\b")
DEFAULT_OFFSET = (-0.002, -0.072, 0.0)


def timeline(log):
    """Require one complete, ordered static block from a single run."""
    holds, waits = {}, []
    for sec, ns, msg in log[["stamp.sec", "stamp.nanosec", "msg"]].itertuples(
        index=False, name=None
    ):
        stamp = int(sec) * 10**9 + int(ns)
        match = _HOLD.match(str(msg))
        if match:
            phase, index = match.groups()
            hold = holds.setdefault(int(index), {})
            if phase in hold:
                raise ValueError(f"duplicate hold {index} {phase}")
            hold[phase] = stamp
        if "Waiting 5 seconds before moving to get baseline" in str(msg):
            waits.append(stamp)
    if len(waits) != 1 or sorted(holds) != list(range(1, 28)):
        raise ValueError(
            "expected one pursuit baseline and all 27 hold events"
        )
    previous = waits[0]
    for i in range(1, 28):
        h = holds[i]
        if set(h) != {"START", "END"} or not previous < h["START"] < h["END"]:
            raise ValueError(
                f"incomplete, overlapping or nonmonotonic hold {i}"
            )
        previous = h["END"]
    return waits[0], holds


def motion_bounds(tcp, wait, first_hold):
    """Infer the executed lap from the wrist, without using static gaze/labels."""
    t = (_stamp_ns(tcp) - wait) * 1e-9
    cols = [f"pose.position.{axis}" for axis in "xyz"]
    xyz = (
        _WORLD_FROM_RIGHT_BASE.apply(tcp[cols].to_numpy(float, copy=True))
        + _RIGHT_BASE_IN_WORLD
    )
    sample = np.arange(3.0, (first_hold - wait) * 1e-9, 0.01)
    if (
        len(sample) < 200
        or not len(t)
        or t[0] > sample[0]
        or t[-1] < sample[-1]
    ):
        raise ValueError(
            "wrist data do not cover the pursuit baseline and lap"
        )
    pts = np.column_stack([np.interp(sample, t, axis) for axis in xyz.T])
    smooth = uniform_filter1d(pts, size=9, axis=0, mode="nearest")
    speed = np.linalg.norm(np.gradient(smooth, 0.01, axis=0), axis=1)
    sustained = uniform_filter1d((speed > 0.012).astype(float), size=13) > 0.85
    starts = np.flatnonzero((sample >= 4.95) & (sample <= 8.5) & sustained)
    home = np.median(pts[(sample >= 3.5) & (sample < 4.75)], axis=0)
    stops = np.flatnonzero(
        (sample > sample[-1] - 8.0)
        & (sample < sample[-1] - 0.12)
        & (np.linalg.norm(smooth - home, axis=1) < 0.005)
        & (speed < 0.016)
    )
    if not len(starts) or not len(stops):
        raise ValueError(
            "no sustained pursuit onset or stationary return to center"
        )
    start, end = (wait + round(sample[i] * 1e9) for i in (starts[0], stops[0]))
    if not 90 < (end - start) * 1e-9 < 450 or end >= first_hold:
        raise ValueError(
            "unexpected moving block duration or overlap with static block"
        )
    return start, end


def previous(t, x, grid):
    """Last observed binocular sample; never use future eyes or bridge dropouts."""
    i = np.searchsorted(t, grid, side="right") - 1
    z = x[i.clip(0)].copy()
    z[(i < 0) | (grid - t[i.clip(0)] > 0.003)] = np.nan
    return z


def interpolate(t, x, grid, limit):
    """Interpolate valid adjacent observations only within a short interval."""
    r = np.searchsorted(t, grid).clip(0, len(t) - 1)
    left = (r - 1).clip(0)
    dt = t[r] - t[left]
    good = np.isfinite(x).all(axis=1)
    ok = (
        (t[left] < grid) & (grid < t[r]) & (dt <= limit) & good[left] & good[r]
    )
    exact = (t[r] == grid) & good[r]
    result = np.full((len(grid), x.shape[1]), np.nan)
    w = ((grid[ok] - t[left[ok]]) / dt[ok])[:, None]
    result[ok] = (1 - w) * x[left[ok]] + w * x[r[ok]]
    result[exact] = x[r[exact]]
    return result


def prepare(directory, offset=DEFAULT_OFFSET):
    directory = Path(directory)
    offset = np.asarray(offset, dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError("marker-to-LED offset must be three finite meters")

    def sorted_csv(file):
        return pd.read_csv(directory / file).sort_values(
            _STAMP, kind="stable", ignore_index=True
        )

    wait, holds = timeline(
        pd.read_csv(
            directory / "rosout.csv",
            usecols=["stamp.sec", "stamp.nanosec", "msg"],
        )
    )
    tcp = sorted_csv("right_tcp_pose_broadcaster_pose.csv")
    start, end = motion_bounds(tcp, wait, holds[1]["START"])
    duration = (end - start) * 1e-9
    marks = sorted_csv("markers.csv")
    eye = eyelink_array_to_samples(
        pd.read_csv(directory / "eyelink_sample_array.csv")
    )
    eye.sort_values(_STAMP, kind="stable", ignore_index=True, inplace=True)
    if eye.empty or marks.empty:
        raise ValueError("empty marker or EyeLink stream")
    raw = eye[[*EYELINK_DATA_COLS, "left_pupil", "right_pupil"]].to_numpy(
        float, copy=True
    )
    ok = (
        np.isfinite(raw).all(1)
        & (np.abs(raw[:, :4]) <= 20000).all(1)
        & (raw[:, 4:] > 0).all(1)
    )
    raw[~ok] = np.nan
    # The marker labels are measured, not the commanded static positions.
    measured = (
        select_target_marker(marks, tcp, fit_window_ns=(start, end)) + offset
    )
    target = (
        _WORLD_FROM_OPTITRACK.apply(measured) + _OPTITRACK_IN_WORLD
    ) * 1000
    grid = np.arange(0, (holds[27]["END"] - start) * 1e-9 + 0.005, 0.001)
    et = (_stamp_ns(eye) - start) * 1e-9
    x = previous(et, raw, grid)
    xy = interpolate(et, raw[:, :4], grid, 0.003)
    sm = (
        pd.DataFrame(xy)
        .rolling(20, center=True, min_periods=20)
        .mean()
        .to_numpy()
    )
    y = interpolate((_stamp_ns(marks) - start) * 1e-9, target, grid, 0.020)
    v = np.linalg.norm(
        np.gradient(sm, 0.001, axis=0).reshape(-1, 2, 2), axis=2
    )
    valid = (
        np.isfinite(sm).all(1) & np.isfinite(x).all(1) & np.isfinite(y).all(1)
    )
    poor = ~np.isfinite(v).all(1) | ((v < 100) | (v > 30000)).any(1)
    bad = (
        pd.Series(poor.astype(int)).rolling(100).max().fillna(1).to_numpy(bool)
    )
    ids = np.zeros(len(grid), dtype="i2")
    for i, h in holds.items():
        a, b = (h["START"] - start) * 1e-9, (h["END"] - start) * 1e-9
        ids[(grid >= a + 0.200) & (grid < min(a + 0.475, b - 0.025))] = i
    return dict(
        x=x.astype("f4"),
        y=y.astype("f4"),
        g=grid,
        valid=valid,
        ids=ids,
        bad=bad,
        duration=duration,
        marker_to_led_m=offset,
        motion_start_ns=start,
        motion_end_ns=end,
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "csv_dir", type=Path, help="directory of per-topic exported CSV files"
    )
    parser.add_argument(
        "output", type=Path, help="new aligned NPZ; will not overwrite"
    )
    parser.add_argument(
        "--marker-to-led-m",
        type=float,
        nargs=3,
        default=DEFAULT_OFFSET,
        metavar=("DX", "DY", "DZ"),
        help="fixed OptiTrack-frame offset in meters (default: -0.002 -0.072 0)",
    )
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"output exists: {args.output}")
    data = prepare(args.csv_dir, args.marker_to_led_m)
    with args.output.open("xb") as f:
        try:
            np.savez_compressed(f, **data)
        except BaseException:
            args.output.unlink(missing_ok=True)
            raise
    valid, ids = data["valid"], data["ids"]
    print(
        f"Saved {args.output}: {data['duration']:.2f} s moving; "
        f"{len(np.unique(ids[valid & (ids > 0)]))}/27 static holds have usable samples."
    )


if __name__ == "__main__":
    main()
