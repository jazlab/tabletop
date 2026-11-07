"""Stage [1]: one pass over the MCAP → per-stream tables (host ROS time, int ns).

Every table keeps the raw clock columns it has (``header_ns``: message header
stamp; ``log_ns``: bag receive time; device clocks such as ``eyelink_time_ms``)
so that the sync stage can choose which clock to align.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

from .bag import BagSummary, iter_messages, stamp_ns

CAMERAS = [
    "left_front_top_cam",
    "left_back_top_cam",
    "left_bottom_cam",
    "right_front_top_cam",
    "right_back_top_cam",
    "right_bottom_cam",
]
SERVICES = [
    "set_reward",
    "set_arm_lock",
    "set_buzzer",
    "set_smartglass",
    "set_solenoid",
    "set_safety_laser",
    "set_sync_pulses",
    "ping",
]
ARMS = ["left", "right"]

TOPICS = {
    "teensy_sensor": "/teensy/sensor",
    "teensy_log": "/teensy/log",
    "task_event": "/tabletop/task_event",
    "flic": "/flic/button_pressed_time",
    "eyelink": "/eyelink/sample_array",
    "joint_states": "/joint_states",
    "markers": "/markers",
    "rosout": "/rosout",
    **{
        f"wrench_{a}": f"/{a}_force_torque_sensor_broadcaster/wrench"
        for a in ARMS
    },
    **{f"tcp_pose_{a}": f"/{a}_tcp_pose_broadcaster/pose" for a in ARMS},
    **{f"io_{a}": f"/{a}_io_and_status_controller/io_states" for a in ARMS},
    **{f"cam_meta_{c}": f"/cam_sync/{c}/meta" for c in CAMERAS},
    **{
        f"cam_meta_unmatched_{c}": f"/cam_sync/{c}/meta_unmatched"
        for c in CAMERAS
    },
    **{f"srv_{s}": f"/teensy/{s}/_service_event" for s in SERVICES},
}
TOPIC_TO_KEY = {v: k for k, v in TOPICS.items()}

TASK_EVENT_NAMES = {
    1: "TASK_STARTED",
    2: "TASK_ENDED",
    3: "TRIAL_PRESENTED",
    4: "TRIAL_DEPRESENTING",
    5: "OBJECT_MISS",
    6: "STIMULUS_REVEALED",
    7: "MOTION_STARTED",
    8: "GO_CUE",
}


@dataclass
class Extracted:
    tables: dict[str, pd.DataFrame] = field(default_factory=dict)
    arrays: dict[str, dict[str, np.ndarray]] = field(default_factory=dict)
    meta: dict[str, object] = field(default_factory=dict)

    def has(self, key: str) -> bool:
        if key in self.tables:
            return len(self.tables[key]) > 0
        if key in self.arrays:
            a = self.arrays[key]
            return bool(a) and len(next(iter(a.values()))) > 0
        return False


def _fields(obj) -> dict:
    names = getattr(obj, "__slots__", None) or [
        k for k in vars(obj) if not k.startswith("_")
    ]
    out = {}
    for k in names:
        v = getattr(obj, k)
        if hasattr(v, "sec") and hasattr(v, "nanosec"):
            out[k] = stamp_ns(v) / 1e9
        elif isinstance(v, (bool, int, float, str)):
            out[k] = v
    return out


def extract(bag_dir: Path, summary: BagSummary) -> Extracted:
    """Decode every topic this pipeline uses (skips images and visualization topics)."""
    wanted = [t for t in TOPICS.values() if summary.count(t) > 0]
    rows: dict[str, list] = defaultdict(list)
    eye: dict[str, list] = defaultdict(list)
    joint_names: list[str] | None = None
    joint_name_changes = 0
    srv_rows: dict[str, list] = defaultdict(list)

    for m in iter_messages(bag_dir, wanted):
        key = TOPIC_TO_KEY[m.topic]
        x = m.msg
        if key == "teensy_sensor":
            rows[key].append(
                (
                    stamp_ns(x.header.stamp),
                    m.log_time_ns,
                    x.is_safety_laser_broken,
                    x.is_left_arm_locked,
                    x.is_right_arm_locked,
                    x.is_reward_active,
                    x.is_button_pressed,
                    x.is_smartglass_revealed,
                    *x.left_tactile_glove_states,
                    *x.right_tactile_glove_states,
                    x.sync_pulse_state,
                    stamp_ns(x.sync_pulse_last_time_on),
                    stamp_ns(x.sync_pulse_last_time_off),
                    stamp_ns(x.safety_laser_last_time_broken),
                    stamp_ns(x.button_last_time_pressed),
                )
            )
        elif key == "eyelink":
            for s in x.samples:
                eye["header_ns"].append(stamp_ns(s.header.stamp))
                eye["log_ns"].append(m.log_time_ns)
                eye["eyelink_time_ms"].append(s.eyelink_time_ms)
                eye["left_x"].append(s.left_x)
                eye["left_y"].append(s.left_y)
                eye["left_pupil"].append(s.left_pupil)
                eye["right_x"].append(s.right_x)
                eye["right_y"].append(s.right_y)
                eye["right_pupil"].append(s.right_pupil)
                eye["input"].append(s.input)
        elif key == "task_event":
            rows[key].append(
                (
                    stamp_ns(x.header.stamp),
                    m.log_time_ns,
                    x.event_type,
                    TASK_EVENT_NAMES.get(
                        x.event_type, f"UNKNOWN_{x.event_type}"
                    ),
                    x.session_id,
                    x.trial_number,
                    x.group_name,
                    x.object_id,
                )
            )
        elif key == "flic":
            rows[key].append((stamp_ns(x.stamp), m.log_time_ns, x.frame_id))
        elif key == "joint_states":
            if joint_names is None:
                joint_names = list(x.name)
            elif list(x.name) != joint_names:
                joint_name_changes += 1
                continue
            rows[key].append(
                (
                    stamp_ns(x.header.stamp),
                    m.log_time_ns,
                    list(x.position),
                    list(x.velocity),
                    list(x.effort),
                )
            )
        elif key.startswith("wrench_"):
            f, t = x.wrench.force, x.wrench.torque
            rows[key].append(
                (
                    stamp_ns(x.header.stamp),
                    m.log_time_ns,
                    f.x,
                    f.y,
                    f.z,
                    t.x,
                    t.y,
                    t.z,
                )
            )
        elif key.startswith("tcp_pose_"):
            p, q = x.pose.position, x.pose.orientation
            rows[key].append(
                (
                    stamp_ns(x.header.stamp),
                    m.log_time_ns,
                    p.x,
                    p.y,
                    p.z,
                    q.x,
                    q.y,
                    q.z,
                    q.w,
                )
            )
        elif key.startswith("io_"):
            din = [False] * 18
            for d in x.digital_in_states:
                if d.pin < 18:
                    din[d.pin] = bool(d.state)
            dout = [False] * 18
            for d in x.digital_out_states:
                if d.pin < 18:
                    dout[d.pin] = bool(d.state)
            rows[key].append((m.log_time_ns, din, dout))
        elif key.startswith("cam_meta"):
            rows[key].append(
                (
                    stamp_ns(x.header.stamp),
                    m.log_time_ns,
                    x.camera_time,
                    x.camera_frame_id,
                    x.synchronized_timestamp_valid,
                    x.brightness,
                    x.exposure_time,
                    x.max_exposure_time,
                    x.gain,
                    x.line_status,
                )
            )
        elif key == "markers":
            h = stamp_ns(x.header.stamp)
            for mk in x.markers:
                rows[key].append(
                    (
                        h,
                        m.log_time_ns,
                        x.frame_number,
                        x.mid_exposure_time,
                        x.data_received_time,
                        mk.marker_index,
                        mk.marker_name,
                        mk.translation.x,
                        mk.translation.y,
                        mk.translation.z,
                    )
                )
            if not x.markers:
                rows["markers_empty_frames"].append((h, x.frame_number))
        elif key == "rosout":
            rows[key].append(
                (stamp_ns(x.stamp), m.log_time_ns, x.level, x.name, x.msg)
            )
        elif key == "teensy_log":
            rows[key].append((m.log_time_ns, x.data))
        elif key.startswith("srv_"):
            info = x.info
            rec = {
                "log_ns": m.log_time_ns,
                "info_event_type": info.event_type,
                "info_ns": stamp_ns(info.stamp),
                "sequence_number": info.sequence_number,
                "client_gid": bytes(info.client_gid).hex(),
            }
            if x.request:
                rec.update(
                    {f"req_{k}": v for k, v in _fields(x.request[0]).items()}
                )
            if x.response:
                rec.update(
                    {f"resp_{k}": v for k, v in _fields(x.response[0]).items()}
                )
            srv_rows[key].append(rec)

    out = Extracted()
    out.meta["joint_names"] = joint_names or []
    out.meta["joint_name_changes"] = joint_name_changes

    glove = [f"left_glove_{i}" for i in range(5)] + [
        f"right_glove_{i}" for i in range(5)
    ]
    out.tables["teensy_sensor"] = pd.DataFrame(
        rows["teensy_sensor"],
        columns=[
            "header_ns",
            "log_ns",
            "is_safety_laser_broken",
            "is_left_arm_locked",
            "is_right_arm_locked",
            "is_reward_active",
            "is_button_pressed",
            "is_smartglass_revealed",
            *glove,
            "sync_pulse_state",
            "sync_on_ns",
            "sync_off_ns",
            "safety_laser_last_broken_ns",
            "button_last_pressed_ns",
        ],
    )
    out.tables["task_event"] = pd.DataFrame(
        rows["task_event"],
        columns=[
            "header_ns",
            "log_ns",
            "event_type",
            "event_name",
            "session_id",
            "trial_number",
            "group_name",
            "object_id",
        ],
    )
    out.tables["flic"] = pd.DataFrame(
        rows["flic"], columns=["header_ns", "log_ns", "frame_id"]
    )
    out.arrays["eyelink"] = (
        {k: np.asarray(v) for k, v in eye.items()} if eye else {}
    )
    js = rows["joint_states"]
    out.arrays["joint_states"] = (
        {
            "header_ns": np.array([r[0] for r in js], dtype=np.int64),
            "log_ns": np.array([r[1] for r in js], dtype=np.int64),
            "position": np.array([r[2] for r in js], dtype=float).reshape(
                len(js), -1
            ),
            "velocity": np.array([r[3] for r in js], dtype=float).reshape(
                len(js), -1
            ),
            "effort": np.array([r[4] for r in js], dtype=float).reshape(
                len(js), -1
            ),
        }
        if js
        else {}
    )
    for a in ARMS:
        out.tables[f"wrench_{a}"] = pd.DataFrame(
            rows[f"wrench_{a}"],
            columns=[
                "header_ns",
                "log_ns",
                "fx",
                "fy",
                "fz",
                "tx",
                "ty",
                "tz",
            ],
        )
        out.tables[f"tcp_pose_{a}"] = pd.DataFrame(
            rows[f"tcp_pose_{a}"],
            columns=[
                "header_ns",
                "log_ns",
                "x",
                "y",
                "z",
                "qx",
                "qy",
                "qz",
                "qw",
            ],
        )
        io = rows[f"io_{a}"]
        out.arrays[f"io_{a}"] = (
            {
                "log_ns": np.array([r[0] for r in io], dtype=np.int64),
                "digital_in": np.array([r[1] for r in io], dtype=bool).reshape(
                    len(io), 18
                ),
                "digital_out": np.array(
                    [r[2] for r in io], dtype=bool
                ).reshape(len(io), 18),
            }
            if io
            else {}
        )
    cam_cols = [
        "header_ns",
        "log_ns",
        "camera_time",
        "camera_frame_id",
        "synchronized_timestamp_valid",
        "brightness",
        "exposure_time",
        "max_exposure_time",
        "gain",
        "line_status",
    ]
    for c in CAMERAS:
        out.tables[f"cam_meta_{c}"] = pd.DataFrame(
            rows[f"cam_meta_{c}"], columns=cam_cols
        )
        out.tables[f"cam_meta_unmatched_{c}"] = pd.DataFrame(
            rows[f"cam_meta_unmatched_{c}"], columns=cam_cols
        )
    out.tables["markers"] = pd.DataFrame(
        rows["markers"],
        columns=[
            "header_ns",
            "log_ns",
            "frame_number",
            "mid_exposure_time",
            "data_received_time",
            "marker_index",
            "marker_name",
            "x",
            "y",
            "z",
        ],
    )
    out.tables["rosout"] = pd.DataFrame(
        rows["rosout"], columns=["header_ns", "log_ns", "level", "name", "msg"]
    )
    out.tables["teensy_log"] = pd.DataFrame(
        rows["teensy_log"], columns=["log_ns", "data"]
    )
    for s in SERVICES:
        out.tables[f"srv_{s}"] = pd.DataFrame(srv_rows[f"srv_{s}"])
    return out


def service_calls(df: pd.DataFrame) -> pd.DataFrame:
    """Collapse the introspection events of each service call into one row.

    Server and client publish their own events (REQUEST_RECEIVED/RESPONSE_SENT
    and REQUEST_SENT/RESPONSE_RECEIVED) under different GIDs. Calls are built
    from the server side when it is introspected (the Teensy/mock acting on
    the request), otherwise from the client side.
    """
    cols = [
        "time_ns",
        "time_source",
        "sequence_number",
        "client_gid",
        "response_ns",
    ]
    if df is None or len(df) == 0:
        return pd.DataFrame(columns=cols)
    server = df[df["info_event_type"].isin([1, 2])]
    side, req_type, resp_type, label = (
        (server, 1, 2, "request_received")
        if len(server)
        else (df[df["info_event_type"].isin([0, 3])], 0, 3, "request_sent")
    )
    calls = []
    for (gid, seq), g in side.groupby(
        ["client_gid", "sequence_number"], sort=False
    ):
        by_type = {
            int(t): r
            for t, r in zip(g["info_event_type"], g.to_dict("records"))
        }
        if req_type not in by_type:
            continue
        rec = {
            "time_ns": int(by_type[req_type]["info_ns"]),
            "time_source": label,
            "sequence_number": int(seq),
            "client_gid": gid,
            "response_ns": int(by_type[resp_type]["info_ns"])
            if resp_type in by_type
            else None,
        }
        for r in by_type.values():
            for k, v in r.items():
                if (
                    (k.startswith("req_") or k.startswith("resp_"))
                    and k not in rec
                    and not pd.isna(v)
                ):
                    rec[k] = v
        calls.append(rec)
    out = pd.DataFrame(calls, columns=None if calls else cols)
    return (
        out.sort_values("time_ns").reset_index(drop=True) if len(out) else out
    )
