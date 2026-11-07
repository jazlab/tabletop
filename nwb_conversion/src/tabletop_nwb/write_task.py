"""Write ``task.nwb``: everything except raw ephys and raw video."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd
from pynwb import TimeSeries
from pynwb.behavior import EyeTracking, PupilTracking, SpatialSeries

from .extract import ARMS, SERVICES, Extracted, service_calls
from .nwb_common import (
    compressed,
    dynamic_table,
    events_table,
    new_nwbfile,
    provenance_table,
    write,
)
from .session import Inventory
from .sync import SyncResult
from .timeline import Timeline
from .trials import (
    TRIAL_EVENT_COLUMNS,
    assign_by_number,
    assign_by_window,
    build_trials,
    epochs,
)

EYELINK_MISSING = -32768
TEENSY_DIGITAL = [
    "is_safety_laser_broken",
    "is_left_arm_locked",
    "is_right_arm_locked",
    "is_reward_active",
    "is_button_pressed",
    "is_smartglass_revealed",
    "sync_pulse_state",
]
GLOVE = [f"left_glove_{i}" for i in range(5)] + [
    f"right_glove_{i}" for i in range(5)
]


def _clean(s: pd.Series):
    """Column values safe for HDF5 (no mixed None/NaN/str)."""
    if s.dtype == object or str(s.dtype) in ("string", "str"):
        non_null = s.dropna()
        if len(non_null) and all(
            isinstance(v, (bool, np.bool_)) for v in non_null
        ):
            return s.map(
                lambda v: np.nan if pd.isna(v) else float(v)
            ).to_numpy(dtype=float)
        if len(non_null) and all(
            isinstance(v, (int, float, np.number)) and not isinstance(v, bool)
            for v in non_null
        ):
            return s.astype(float).to_numpy()
        return [("" if pd.isna(v) else str(v)) for v in s]
    if s.dtype == bool:
        return s.to_numpy()
    return s.to_numpy(dtype=float) if s.isna().any() else s.to_numpy()


def write_task(
    inv: Inventory, ex: Extracted, sync: SyncResult, tl: Timeline, out: Path
) -> dict:
    nwb = new_nwbfile(
        inv,
        tl,
        "task",
        "TableTop behavior: trials, task events, I/O, eye tracking, robot state, sync.",
    )
    counts: dict[str, int] = {}
    end_ns = int(inv.bag_end_ns)

    # ---- epochs and trials -------------------------------------------------
    te = (
        ex.tables["task_event"].sort_values("header_ns").reset_index(drop=True)
    )
    ep = epochs(te, end_ns)
    if len(ep):
        nwb.add_epoch_column(
            "ended_cleanly", "True if the run ended with TASK_ENDED"
        )
        for r in ep.itertuples():
            nwb.add_epoch(
                start_time=float(tl.rel_host_ns(r.start_ns)),
                stop_time=float(tl.rel_host_ns(r.stop_ns)),
                tags=["task"],
                ended_cleanly=bool(r.ended_cleanly),
            )
    counts["epochs"] = len(ep)

    trials = build_trials(te, end_ns)
    reward = service_calls(ex.tables.get("srv_set_reward"))
    if len(reward) and "req_activate" in reward:
        reward_on = reward[reward["req_activate"].astype(bool)]
    else:
        reward_on = reward
    flic = ex.tables["flic"]
    if len(trials):
        r_idx = assign_by_window(
            reward_on["time_ns"].to_numpy() if len(reward_on) else [], trials
        )
        f_idx = assign_by_window(flic["header_ns"].to_numpy(), trials)
        cols = {
            "trial_number": "TaskEvent.trial_number (task's own numbering; may be out of start-time order)",
            "group_name": "manipulator / group that presented the object",
            "presented_object": "TaskEvent.object_id of the presented object",
            "completed": "True if TRIAL_DEPRESENTING was recorded for this trial",
            "stop_reason": "depresented | next_trial | task_end",
            **{
                c: f"first {c[:-5].upper()} event of this trial (s; NaN if none)"
                for c in TRIAL_EVENT_COLUMNS.values()
            },
            "n_rewards": "reward activations (set_reward activate=True) inside the trial window",
            "first_reward_time": "first reward activation in the trial window (s; NaN if none)",
            "n_flic_presses": "Flic button presses inside the trial window",
            "first_flic_press_time": "first Flic press in the trial window (s; NaN if none)",
        }
        for c, d in cols.items():
            nwb.add_trial_column(c, d)
        for i, t in trials.iterrows():
            rt = (
                reward_on["time_ns"].to_numpy()[r_idx == i]
                if len(reward_on)
                else np.array([])
            )
            ft = flic["header_ns"].to_numpy()[f_idx == i]
            row = {
                "start_time": float(tl.rel_host_ns(t.start_ns)),
                "stop_time": float(tl.rel_host_ns(t.stop_ns)),
                "trial_number": int(t.trial_number),
                "group_name": t.group_name,
                "presented_object": t.object_id,
                "completed": bool(t.completed),
                "stop_reason": t.stop_reason,
                "n_rewards": len(rt),
                "first_reward_time": float(tl.rel_host_ns(rt.min()))
                if len(rt)
                else np.nan,
                "n_flic_presses": len(ft),
                "first_flic_press_time": float(tl.rel_host_ns(ft.min()))
                if len(ft)
                else np.nan,
            }
            for c in TRIAL_EVENT_COLUMNS.values():
                v = t[c + "_ns"]
                row[c] = (
                    float(tl.rel_host_ns(int(v))) if not pd.isna(v) else np.nan
                )
            nwb.add_trial(**row)
    counts["trials"] = len(trials)

    # ---- event tables ------------------------------------------------------
    if len(te):
        tidx = (
            assign_by_number(te, trials)
            if len(trials)
            else np.full(len(te), -1)
        )
        nwb.add_events_table(
            events_table(
                "task_events",
                "Every tabletop_interfaces/TaskEvent, in publish order.",
                tl.rel_host_ns(te.header_ns),
                source="/tabletop/task_event (header stamp, host ROS time)",
                columns={
                    "event_type": (
                        te.event_type.to_numpy(dtype=np.int16),
                        "TaskEvent constant",
                    ),
                    "event_name": (
                        list(te.event_name),
                        "TaskEvent constant name",
                    ),
                    "trial_number": (
                        te.trial_number.to_numpy(dtype=np.int32),
                        "-1 for task-level events",
                    ),
                    "group_name": (list(te.group_name), ""),
                    "presented_object": (
                        list(te.object_id),
                        "TaskEvent.object_id",
                    ),
                    "trial_index": (
                        tidx,
                        "row in the trials table (-1 if none)",
                    ),
                    "bag_receive_time": (
                        tl.rel_host_ns(te.log_ns),
                        "bag receive time (s)",
                    ),
                },
            )
        )
    counts["task_events"] = len(te)

    for s in SERVICES:
        calls = service_calls(ex.tables.get(f"srv_{s}"))
        counts[f"srv_{s}"] = len(calls)
        if not len(calls):
            continue
        name = {
            "set_reward": "reward",
            "set_arm_lock": "arm_lock",
            "set_buzzer": "buzzer",
            "set_smartglass": "smartglass",
            "set_solenoid": "solenoid",
            "set_safety_laser": "safety_laser",
            "set_sync_pulses": "sync_pulse_control",
            "ping": "teensy_ping",
        }[s]
        dur = (
            calls["req_duration"].to_numpy(dtype=float)
            if "req_duration" in calls
            else None
        )
        if s == "set_reward" and dur is not None and "req_activate" in calls:
            dur = np.where(calls["req_activate"].astype(bool), dur, np.nan)
        extra = {
            c: (_clean(calls[c]), f"request/response field {c}")
            for c in calls.columns
            if c.startswith(("req_", "resp_")) and c != "req_duration"
        }
        extra["time_source"] = (
            list(calls["time_source"]),
            "request_received (server) or request_sent (client)",
        )
        extra["response_time"] = (
            np.array(
                [
                    tl.rel_host_ns(int(v)) if not pd.isna(v) else np.nan
                    for v in calls["response_ns"]
                ]
            ),
            "response stamp (s)",
        )
        extra["trial_index"] = (
            assign_by_window(calls["time_ns"].to_numpy(), trials),
            "trial window (-1 if none)",
        )
        nwb.add_events_table(
            events_table(
                name,
                f"Teensy service /teensy/{s} calls (one row per call, from service introspection).",
                tl.rel_host_ns(calls["time_ns"].to_numpy().astype(np.int64)),
                durations=dur,
                source=f"/teensy/{s}/_service_event",
                columns=extra,
            )
        )

    if len(flic):
        nwb.add_events_table(
            events_table(
                "flic_press",
                "Flic button presses.",
                tl.rel("flic"),
                source="/flic/button_pressed_time header stamp",
                columns={
                    "trial_index": (
                        assign_by_window(flic.header_ns.to_numpy(), trials),
                        "trial window (-1 if none)",
                    )
                },
            )
        )
    counts["flic_press"] = len(flic)

    # ---- acquisition: eye tracking ----------------------------------------
    if ex.has("eyelink"):
        e = ex.arrays["eyelink"]
        t = tl.rel("eyelink")
        method = tl.streams["eyelink"].method

        def gaze(x, y):
            g = np.column_stack([x, y]).astype(float)
            g[g == EYELINK_MISSING] = np.nan
            return g

        desc = f"EyeLink samples; timestamps via {method} ({tl.streams['eyelink'].source})."
        left = SpatialSeries(
            name="left_eye",
            description=desc,
            data=compressed(gaze(e["left_x"], e["left_y"])),
            timestamps=compressed(t),
            reference_frame="EyeLink link sample coordinates (gaze/raw, tracker units)",
            unit="n.a.",
        )
        right = SpatialSeries(
            name="right_eye",
            description=desc,
            data=compressed(gaze(e["right_x"], e["right_y"])),
            timestamps=left,
            reference_frame="EyeLink link sample coordinates (gaze/raw, tracker units)",
            unit="n.a.",
        )
        nwb.add_acquisition(EyeTracking(spatial_series=[left, right]))
        lp = e["left_pupil"].astype(float)
        rp = e["right_pupil"].astype(float)
        lp[lp == EYELINK_MISSING] = np.nan
        rp[rp == EYELINK_MISSING] = np.nan
        nwb.add_acquisition(
            PupilTracking(
                time_series=[
                    TimeSeries(
                        name="left_pupil",
                        data=compressed(lp),
                        timestamps=left,
                        unit="EyeLink area units",
                        description=desc,
                    ),
                    TimeSeries(
                        name="right_pupil",
                        data=compressed(rp),
                        timestamps=left,
                        unit="EyeLink area units",
                        description=desc,
                    ),
                ]
            )
        )
        nwb.add_acquisition(
            TimeSeries(
                name="eyelink_input",
                data=compressed(e["input"].astype(np.int16)),
                timestamps=left,
                unit="n/a",
                description="EyeLink digital input word; sync pulse = bit 0x20, active-low.",
            )
        )
        nwb.add_acquisition(
            TimeSeries(
                name="eyelink_tracker_time",
                data=compressed(e["eyelink_time_ms"].astype(np.int64)),
                timestamps=left,
                unit="ms",
                description="EyeLink tracker clock per sample (raw, for re-synchronization).",
            )
        )
    counts["eyelink"] = len(ex.arrays.get("eyelink", {}).get("input", []))

    # ---- acquisition: robot -----------------------------------------------
    if ex.has("joint_states"):
        js = ex.arrays["joint_states"]
        names = ex.meta["joint_names"]
        d = f"Columns are joints in this order: {', '.join(names)}."
        pos = TimeSeries(
            name="robot_joint_position",
            data=compressed(js["position"]),
            timestamps=compressed(tl.rel("joint_states")),
            unit="radians",
            description=d,
        )
        nwb.add_acquisition(pos)
        nwb.add_acquisition(
            TimeSeries(
                name="robot_joint_velocity",
                data=compressed(js["velocity"]),
                timestamps=pos,
                unit="radians/s",
                description=d,
            )
        )
        nwb.add_acquisition(
            TimeSeries(
                name="robot_joint_effort",
                data=compressed(js["effort"]),
                timestamps=pos,
                unit="unknown (ur_robot_driver effort; see U8)",
                description=d,
            )
        )
    counts["joint_states"] = len(
        ex.arrays.get("joint_states", {}).get("header_ns", [])
    )
    for a in ARMS:
        w = ex.tables[f"wrench_{a}"]
        if len(w):
            f = TimeSeries(
                name=f"robot_{a}_force",
                data=compressed(w[["fx", "fy", "fz"]].to_numpy()),
                timestamps=compressed(tl.rel(f"wrench_{a}")),
                unit="newtons",
                description=f"{a} UR force (x, y, z) from the F/T sensor broadcaster",
            )
            nwb.add_acquisition(f)
            nwb.add_acquisition(
                TimeSeries(
                    name=f"robot_{a}_torque",
                    data=compressed(w[["tx", "ty", "tz"]].to_numpy()),
                    timestamps=f,
                    unit="newton-meters",
                    description=f"{a} UR torque (x, y, z)",
                )
            )
        counts[f"wrench_{a}"] = len(w)
        p = ex.tables[f"tcp_pose_{a}"]
        if len(p):
            pp = TimeSeries(
                name=f"robot_{a}_tcp_position",
                data=compressed(p[["x", "y", "z"]].to_numpy()),
                timestamps=compressed(tl.rel(f"tcp_pose_{a}")),
                unit="meters",
                description=f"{a} TCP position",
            )
            nwb.add_acquisition(pp)
            nwb.add_acquisition(
                TimeSeries(
                    name=f"robot_{a}_tcp_orientation",
                    data=compressed(p[["qx", "qy", "qz", "qw"]].to_numpy()),
                    timestamps=pp,
                    unit="quaternion (x, y, z, w)",
                    description=f"{a} TCP orientation",
                )
            )
        counts[f"tcp_pose_{a}"] = len(p)
        if ex.has(f"io_{a}"):
            io = ex.arrays[f"io_{a}"]
            st = tl.streams[f"io_{a}"]
            di = TimeSeries(
                name=f"ur_{a}_digital_in",
                data=compressed(io["digital_in"].astype(np.uint8)),
                timestamps=compressed(tl.rel(f"io_{a}")),
                unit="n/a",
                description=f"UR standard digital inputs 0–17 (DI3 = sync pulse, active-low). "
                f"Timestamps via {st.method} ({st.source}).",
            )
            nwb.add_acquisition(di)
            nwb.add_acquisition(
                TimeSeries(
                    name=f"ur_{a}_digital_out",
                    data=compressed(io["digital_out"].astype(np.uint8)),
                    timestamps=di,
                    unit="n/a",
                    description="UR standard digital outputs 0–17",
                )
            )
        counts[f"io_{a}"] = len(ex.arrays.get(f"io_{a}", {}).get("log_ns", []))

    # ---- acquisition: Teensy ----------------------------------------------
    ts = ex.tables["teensy_sensor"]
    if len(ts):
        dig = TimeSeries(
            name="teensy_digital_states",
            data=compressed(ts[TEENSY_DIGITAL].to_numpy().astype(np.uint8)),
            timestamps=compressed(tl.rel("teensy_sensor")),
            unit="n/a",
            description=f"Teensy state flags, columns: {', '.join(TEENSY_DIGITAL)}.",
        )
        nwb.add_acquisition(dig)
        nwb.add_acquisition(
            TimeSeries(
                name="teensy_glove_adc",
                data=compressed(ts[GLOVE].to_numpy().astype(np.uint16)),
                timestamps=dig,
                unit="ADC counts (10-bit)",
                description=f"Tactile glove sensors, columns: {', '.join(GLOVE)}.",
            )
        )
    counts["teensy_sensor"] = len(ts)

    mk = ex.tables["markers"]
    if len(mk):
        nwb.add_acquisition(
            dynamic_table(
                "optitrack_markers",
                "OptiTrack markers, one row per marker per frame.",
                {
                    "timestamp": (
                        tl.rel("markers"),
                        "seconds since session start (header stamp)",
                    ),
                    "frame_number": (
                        mk.frame_number.to_numpy(),
                        "mocap4r2 frame counter",
                    ),
                    "marker_index": (mk.marker_index.to_numpy(), ""),
                    "marker_name": (list(mk.marker_name), ""),
                    "x": (mk.x.to_numpy(), "meters"),
                    "y": (mk.y.to_numpy(), "meters"),
                    "z": (mk.z.to_numpy(), "meters"),
                },
            )
        )
    counts["markers"] = len(mk)

    # ---- processing: sync --------------------------------------------------
    sync_mod = nwb.create_processing_module(
        "sync",
        "Synchronization of every stream to the Teensy pulse train (host ROS time).",
    )
    fits = [f.summary() for f in sync.fits.values()]
    keys = list(fits[0].keys())
    sync_mod.add(
        dynamic_table(
            "sync_fits",
            "One row per stream: how its timestamps were mapped to host ROS time. "
            "host_s_since_origin = a * (stream_clock_s - x0_s) + b_s.",
            {k: (_clean(pd.Series([f[k] for f in fits])), k) for k in keys},
        )
    )
    ref = sync.reference
    if len(ref.onsets):
        sync_mod.add(
            events_table(
                "teensy_sync_pulses",
                "Teensy pin-31 sync pulse rising edges.",
                ref.onsets - (tl.start_ns - sync.origin_ns) / 1e9,
                durations=ref.widths,
                source="TeensySensor.sync_pulse_last_time_on/off",
            )
        )
    for name, p in sync.stream_pulses.items():
        if len(p.onsets) == 0:
            continue
        f = sync.fits[name]
        aligned = (
            f.to_rel_s(p.onsets, tl.start_ns)
            if f.status == "ok"
            else np.full(len(p.onsets), np.nan)
        )
        sync_mod.add(
            events_table(
                f"{name}_sync_pulses",
                f"Sync pulses detected in {name} ({f.clock}); timestamp = aligned (NaN if no fit).",
                aligned,
                durations=p.widths,
                columns={
                    "stream_clock_s": (
                        p.onsets,
                        f"onset in the stream clock: {f.clock}",
                    )
                },
            )
        )
    counts["teensy_sync_pulses"] = len(ref.onsets)

    # ---- processing: logs and provenance ----------------------------------
    logs = nwb.create_processing_module("logs", "ROS and Teensy logs.")
    ro = ex.tables["rosout"]
    if len(ro):
        logs.add(
            events_table(
                "rosout",
                "/rosout",
                tl.rel("rosout"),
                columns={
                    "level": (
                        ro.level.to_numpy(dtype=np.int16),
                        "rcl log level",
                    ),
                    "logger": (list(ro.name), ""),
                    "message": (list(ro.msg), ""),
                },
            )
        )
    tlg = ex.tables["teensy_log"]
    if len(tlg):
        logs.add(
            events_table(
                "teensy_log",
                "/teensy/log",
                tl.rel("teensy_log"),
                columns={"message": (list(tlg.data), "")},
            )
        )
    counts["rosout"], counts["teensy_log"] = len(ro), len(tlg)

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
                "joint_name_changes_skipped": ex.meta.get(
                    "joint_name_changes", 0
                ),
            },
        )
    )
    write(nwb, out)
    return counts
