"""Trials and epochs from TaskEvent (D6).

A trial starts at TRIAL_PRESENTED and stops at the matching
TRIAL_DEPRESENTING (same trial_number). A trial with no DEPRESENTING is kept,
marked ``completed=False``, and stops at the next trial's start or the task
end. Events carrying a trial_number ≥ 0 are assigned by number; untagged
events (rewards, Flic presses, …) are assigned by time window.
"""

from __future__ import annotations

import numpy as np
import pandas as pd

PRESENTED, DEPRESENTING, TASK_STARTED, TASK_ENDED = 3, 4, 1, 2
TRIAL_EVENT_COLUMNS = {
    5: "object_miss_time",
    6: "stimulus_revealed_time",
    7: "motion_started_time",
    8: "go_cue_time",
}


def epochs(task_events: pd.DataFrame, end_ns: int) -> pd.DataFrame:
    """Task runs: TASK_STARTED → next TASK_ENDED (or end of recording)."""
    te = task_events.sort_values("header_ns")
    rows = []
    start = None
    for r in te.itertuples():
        if r.event_type == TASK_STARTED:
            if start is not None:
                rows.append((start, r.header_ns, False))
            start = r.header_ns
        elif r.event_type == TASK_ENDED and start is not None:
            rows.append((start, r.header_ns, True))
            start = None
    if start is not None:
        rows.append((start, end_ns, False))
    return pd.DataFrame(rows, columns=["start_ns", "stop_ns", "ended_cleanly"])


def build_trials(task_events: pd.DataFrame, end_ns: int) -> pd.DataFrame:
    te = task_events.sort_values("header_ns").reset_index(drop=True)
    pres = te[te.event_type == PRESENTED]
    dep = te[te.event_type == DEPRESENTING]
    ends = te[te.event_type == TASK_ENDED].header_ns.to_numpy()
    rows = []
    starts = pres.header_ns.to_numpy()
    for k, r in enumerate(pres.itertuples()):
        d = dep[
            (dep.trial_number == r.trial_number)
            & (dep.header_ns >= r.header_ns)
        ]
        nxt = starts[k + 1] if k + 1 < len(starts) else end_ns
        later_end = ends[ends >= r.header_ns]
        limit = min(nxt, later_end[0]) if len(later_end) else nxt
        if len(d) and d.header_ns.iloc[0] <= limit:
            stop, completed, reason = (
                int(d.header_ns.iloc[0]),
                True,
                "depresented",
            )
        else:
            stop, completed = int(limit), False
            reason = (
                "next_trial"
                if limit == nxt and k + 1 < len(starts)
                else "task_end"
            )
        rows.append(
            {
                "start_ns": int(r.header_ns),
                "stop_ns": stop,
                "trial_number": int(r.trial_number),
                "group_name": r.group_name,
                "object_id": r.object_id,
                "completed": completed,
                "stop_reason": reason,
            }
        )
    trials = pd.DataFrame(
        rows,
        columns=[
            "start_ns",
            "stop_ns",
            "trial_number",
            "group_name",
            "object_id",
            "completed",
            "stop_reason",
        ],
    )
    for etype, col in TRIAL_EVENT_COLUMNS.items():
        trials[col + "_ns"] = np.nan
        ev = te[te.event_type == etype]
        for i, t in trials.iterrows():
            e = ev[
                (ev.trial_number == t.trial_number)
                & (ev.header_ns >= t.start_ns)
                & (ev.header_ns <= t.stop_ns)
            ]
            if len(e):
                trials.at[i, col + "_ns"] = float(e.header_ns.iloc[0])
    return trials


def assign_by_number(
    task_events: pd.DataFrame, trials: pd.DataFrame
) -> np.ndarray:
    """Row index into ``trials`` for each task event (-1 if none)."""
    out = np.full(len(task_events), -1, dtype=np.int64)
    for i, r in enumerate(task_events.itertuples()):
        if r.trial_number < 0:
            continue
        m = np.flatnonzero(
            (trials.trial_number.to_numpy() == r.trial_number)
            & (trials.start_ns.to_numpy() <= r.header_ns)
            & (trials.stop_ns.to_numpy() >= r.header_ns)
        )
        if len(m):
            out[i] = m[0]
    return out


def assign_by_window(times_ns: np.ndarray, trials: pd.DataFrame) -> np.ndarray:
    """Row index of the trial whose [start, stop] contains each time (-1 if none).

    Trials do not overlap (``_trial_lock``), so the containing trial is unique.
    """
    t = np.asarray(times_ns, dtype=np.float64)
    if len(trials) == 0 or len(t) == 0:
        return np.full(len(t), -1, dtype=np.int64)
    starts = trials.start_ns.to_numpy().astype(np.float64)
    stops = trials.stop_ns.to_numpy().astype(np.float64)
    order = np.argsort(starts)
    k = np.searchsorted(starts[order], t, side="right") - 1
    out = np.full(len(t), -1, dtype=np.int64)
    ok = k >= 0
    cand = order[np.clip(k, 0, None)]
    ok &= t <= stops[cand]
    out[ok] = cand[ok]
    return out
