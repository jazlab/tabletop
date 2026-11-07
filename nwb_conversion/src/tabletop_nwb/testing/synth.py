"""Synthetic TableTop sessions with known ground truth (PLAN §10.6).

Writes a real rosbag2-style MCAP using the rig's own message schemas
(``schemas.json``, copied from a recorded bag), plus — for the ``rig``
profile — camera metadata, OptiTrack markers and fake ``unbag`` video
segments with timestamp sidecars. Every stream carries the Teensy pulse train
through its own clock with injected offset/drift/jitter, and the generator
writes ``ground_truth.json`` so tests can check the converter exactly.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
from mcap.writer import Writer

from ..bag import encoder_for
from ..extract import CAMERAS

SCHEMAS = json.loads((Path(__file__).parent / "schemas.json").read_text())
SCHEMAS.setdefault(
    "/right_io_and_status_controller/io_states",
    SCHEMAS["/left_io_and_status_controller/io_states"],
)
for _c in CAMERAS:
    SCHEMAS.setdefault(
        f"/cam_sync/{_c}/meta", SCHEMAS["/cam_sync/left_back_top_cam/meta"]
    )

T0_NS = 1_700_000_000_000_000_000  # 2023-11-14, in the past (nwbinspector rejects future dates)
JOINTS = [
    f"{a}_{j}"
    for a in ("left", "right")
    for j in (
        "elbow_joint",
        "shoulder_lift_joint",
        "shoulder_pan_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    )
]


@dataclass
class Truth:
    profile: str
    duration_s: float = 30.0
    seed: int = 0
    eyelink_offset_s: float = (
        123456.789  # tracker clock = (host - T0) * (1 + drift) + offset
    )
    eyelink_drift_ppm: float = 40.0
    camera_offset_ms: float = 3.0  # FLIR header stamp error vs true host time
    camera_drift_ppm: float = -25.0
    teensy_jitter_ms: float = 0.2
    ur_latency_ms: tuple = (1.0, 4.0)
    dropped_frames: dict = field(
        default_factory=lambda: {"right_bottom_cam": [500, 501, 502]}
    )
    eyelink_random_input: bool = False  # simulator-style input word even on the rig profile (mutation test)
    n_trials: int = 0
    n_aborted: int = 0
    n_pulses: int = 0
    counts: dict = field(default_factory=dict)
    pulse_onsets_ns: list = field(default_factory=list, repr=False)


def _t(ns: int) -> dict:
    return {
        "sec": int(ns // 1_000_000_000),
        "nanosec": int(ns % 1_000_000_000),
    }


def _hdr(ns: int, frame: str = "") -> dict:
    return {"stamp": _t(ns), "frame_id": frame}


class _Bag:
    def __init__(self):
        self.msgs: list[tuple[int, str, dict]] = []

    def add(self, topic: str, log_ns: int, msg: dict) -> None:
        self.msgs.append((int(log_ns), topic, msg))

    def write(self, path: Path) -> dict[str, int]:
        path.parent.mkdir(parents=True, exist_ok=True)
        counts: dict[str, int] = {}
        with open(path, "wb") as f:
            w = Writer(f)
            w.start(profile="ros2", library="tabletop_nwb synth")
            chans, encs = {}, {}
            for topic, (sname, stext) in SCHEMAS.items():
                sid = w.register_schema(sname, "ros2msg", stext.encode())
                chans[topic] = w.register_channel(topic, "cdr", sid)
                encs[topic] = encoder_for(sname, stext)
            for i, (log_ns, topic, msg) in enumerate(
                sorted(self.msgs, key=lambda m: m[0])
            ):
                w.add_message(
                    chans[topic],
                    log_time=log_ns,
                    publish_time=log_ns,
                    data=encs[topic](msg),
                    sequence=i,
                )
                counts[topic] = counts.get(topic, 0) + 1
            w.finish()
        return counts


def make_session(root: Path, profile: str = "rig", **kw) -> tuple[Path, Truth]:
    """Create ``root/<session_id>/`` and return (session_dir, truth)."""
    truth = Truth(profile=profile, **kw)
    rng = np.random.default_rng(truth.seed)
    sid = f"session_synth_{profile.replace('+', '_')}_{truth.seed}"
    sdir = Path(root) / sid
    bag = _Bag()
    D = int(truth.duration_s * 1e9)
    task_start, task_end = T0_NS + int(1e9), T0_NS + D - int(1e9)

    # --- Teensy pulse train (true physical times), only during the task -----
    onsets = []
    k = 1
    while True:
        on = task_start + int(k * 1e9 + rng.uniform(0.05, 0.2) * 1e9)
        if on + int(0.1e9) >= task_end:
            break
        onsets.append(on)
        k += 1
    onsets = np.array(onsets, dtype=np.int64)
    offs = onsets + int(0.1e9)
    truth.n_pulses = len(onsets)
    truth.pulse_onsets_ns = onsets.tolist()

    def pulse_active(t_ns: np.ndarray) -> np.ndarray:
        i = np.searchsorted(onsets, t_ns, side="right") - 1
        ok = i >= 0
        return ok & (t_ns < offs[np.clip(i, 0, None)])

    rep_on = onsets + (
        rng.normal(0, truth.teensy_jitter_ms, len(onsets)) * 1e6
    ).astype(np.int64)
    rep_off = offs + (
        rng.normal(0, truth.teensy_jitter_ms, len(onsets)) * 1e6
    ).astype(np.int64)
    stale = T0_NS - int(
        3600e9
    )  # last_time_on persists from a previous session
    for t in range(T0_NS, T0_NS + D, int(10e6)):
        i = np.searchsorted(onsets, t, side="right") - 1
        last_on = int(rep_on[i]) if i >= 0 else stale
        last_off = (
            int(rep_off[i])
            if i >= 0 and t >= offs[i]
            else (int(rep_off[i - 1]) if i >= 1 else stale)
        )
        bag.add(
            "/teensy/sensor",
            t + 300_000,
            {
                "header": _hdr(t),
                "is_safety_laser_broken": False,
                "is_left_arm_locked": True,
                "is_right_arm_locked": True,
                "is_reward_active": False,
                "is_button_pressed": False,
                "is_smartglass_revealed": False,
                "left_tactile_glove_states": rng.integers(0, 1024, 5).tolist(),
                "right_tactile_glove_states": rng.integers(
                    0, 1024, 5
                ).tolist(),
                "sync_pulse_state": bool(pulse_active(np.array([t]))[0]),
                "sync_pulse_last_time_on": _t(last_on),
                "sync_pulse_last_time_off": _t(last_off),
                "safety_laser_last_time_broken": _t(0),
                "button_last_time_pressed": _t(0),
            },
        )

    # --- task events, trials, rewards, Flic ---------------------------------
    def task_event(t, etype, trial=-1, group="", obj=""):
        bag.add(
            "/tabletop/task_event",
            t + 500_000,
            {
                "header": _hdr(t),
                "event_type": etype,
                "session_id": sid,
                "trial_number": trial,
                "group_name": group,
                "object_id": obj,
            },
        )

    seq = {"reward": 0}

    def service(topic, t, req, resp):
        seq["reward"] += 1
        client, server = [1] * 16, [2] * 16
        for et, dt, gid, r, s in [
            (0, 0, client, [req], []),
            (1, 400_000, server, [req], []),
            (2, 1_200_000, server, [], [resp]),
            (3, 1_600_000, client, [], [resp]),
        ]:
            bag.add(
                topic,
                t + dt + 100_000,
                {
                    "info": {
                        "event_type": et,
                        "stamp": _t(t + dt),
                        "client_gid": gid,
                        "sequence_number": seq["reward"],
                    },
                    "request": r,
                    "response": s,
                },
            )

    task_event(task_start, 1)
    t = task_start + int(0.5e9)
    trial = 0
    while t + int(5e9) < task_end:
        group = "left_manipulator" if trial % 2 else "right_manipulator"
        obj = f"obj_{trial}"
        task_event(t, 3, trial, group, obj)
        task_event(t + int(0.8e9), 8, trial, group, obj)
        aborted = trial == 2
        if not aborted:
            press = t + int(rng.uniform(1.5, 3.0) * 1e9)
            bag.add(
                "/flic/button_pressed_time",
                press + 3_000_000,
                _hdr(press, "flic"),
            )
            service(
                "/teensy/set_reward/_service_event",
                press + int(20e6),
                {
                    "activate": True,
                    "duration": {"sec": 0, "nanosec": 250_000_000},
                },
                {"success": True, "message": "Reward started for 0.25 s"},
            )
            task_event(t + int(3.5e9), 4, trial, group, obj)
        else:
            truth.n_aborted += 1
        trial += 1
        t += int(5e9)
    truth.n_trials = trial
    task_event(task_end, 2)

    # --- EyeLink (1 kHz tracker clock, batches of 25) ------------------------
    host = np.arange(T0_NS, T0_NS + D, int(1e6), dtype=np.int64)
    trk_ms = np.floor(
        (
            (host - T0_NS) / 1e9 * (1 + truth.eyelink_drift_ppm * 1e-6)
            + truth.eyelink_offset_s
        )
        * 1e3
    )
    if profile == "sim" or truth.eyelink_random_input:
        inp = rng.choice([127, 95], len(host))
    else:
        inp = np.where(pulse_active(host), 95, 127)
    gx = 500 + 200 * np.sin(np.arange(len(host)) / 300)
    for b in range(0, len(host), 25):
        sl = slice(b, b + 25)
        samples = [
            {
                "header": _hdr(int(h) + 2_000_000),
                "eyelink_time_ms": int(tm),
                "left_x": float(x),
                "left_y": 400.0,
                "left_pupil": 5000.0,
                "right_x": float(x),
                "right_y": 400.0,
                "right_pupil": 5000.0,
                "input": int(i),
            }
            for h, tm, x, i in zip(host[sl], trk_ms[sl], gx[sl], inp[sl])
        ]
        bag.add(
            "/eyelink/sample_array",
            int(host[sl][-1]) + 4_000_000,
            {"samples": samples},
        )

    # --- robot: joint states, wrench, IO (125 Hz) ----------------------------
    for t in range(T0_NS, T0_NS + D, int(8e6)):
        bag.add(
            "/joint_states",
            t + 400_000,
            {
                "header": _hdr(t, "base_link"),
                "name": JOINTS,
                "position": [0.1 * np.sin(t / 1e9)] * 12,
                "velocity": [0.0] * 12,
                "effort": [0.0] * 12,
            },
        )
        for a in ("left", "right"):
            bag.add(
                f"/{a}_force_torque_sensor_broadcaster/wrench",
                t + 500_000,
                {
                    "header": _hdr(t, f"{a}_tool0_controller"),
                    "wrench": {
                        "force": {"x": 1.0, "y": 0.0, "z": -2.0},
                        "torque": {"x": 0.0, "y": 0.1, "z": 0.0},
                    },
                },
            )
        lat = int(rng.uniform(*truth.ur_latency_ms) * 1e6)
        di3 = not (profile != "sim" and bool(pulse_active(np.array([t]))[0]))
        din = [
            {"pin": p, "state": (di3 if p == 3 else True)} for p in range(18)
        ]
        bag.add(
            "/left_io_and_status_controller/io_states",
            t + lat,
            {
                "digital_in_states": din,
                "digital_out_states": [
                    {"pin": p, "state": False} for p in range(18)
                ],
                "flag_states": [],
                "analog_in_states": [],
                "analog_out_states": [],
            },
        )

    bag.add(
        "/rosout",
        T0_NS + int(0.2e9),
        {
            "stamp": _t(T0_NS + int(0.2e9)),
            "level": 20,
            "name": "synth",
            "msg": "synthetic session",
            "file": "",
            "function": "",
            "line": 0,
        },
    )

    # --- rig only: cameras (120 Hz), OptiTrack, fake unbag video ------------
    if profile != "sim":
        host_f = np.arange(
            T0_NS + int(0.5e9),
            T0_NS + D - int(0.5e9),
            int(1e9 / 120),
            dtype=np.int64,
        )
        hdr_f = (
            host_f
            + ((host_f - T0_NS) * truth.camera_drift_ppm * 1e-6).astype(
                np.int64
            )
            + int(truth.camera_offset_ms * 1e6)
        )
        line3 = pulse_active(host_f)
        for cam in CAMERAS:
            drop = set(truth.dropped_frames.get(cam, []))
            keep = np.array([i not in drop for i in range(len(host_f))])
            for i in np.flatnonzero(keep):
                ls = (
                    (1 << 3)
                    if (cam == "left_back_top_cam" and line3[i])
                    else 0
                )
                bag.add(
                    f"/cam_sync/{cam}/meta",
                    int(host_f[i]) + 6_000_000,
                    {
                        "header": _hdr(int(hdr_f[i]), cam),
                        "camera_time": int(i * 8_333_333),
                        "camera_frame_id": int(i),
                        "synchronized_timestamp_valid": True,
                        "brightness": 100,
                        "exposure_time": 2000,
                        "max_exposure_time": 8000,
                        "gain": 1.5,
                        "line_status": ls | 1,
                    },
                )
            vdir = sdir / "unbag" / f"cam_sync_{cam}_image_raw_video"
            vdir.mkdir(parents=True, exist_ok=True)
            idx = np.flatnonzero(keep)
            seg_len = 120 * 10
            for s, b in enumerate(range(0, len(idx), seg_len)):
                part = idx[b : b + seg_len]
                (vdir / f"segment_{s:06d}.mkv").write_bytes(
                    b"\x1aE\xdf\xa3 synthetic placeholder"
                )
                rows = [
                    "segment_frame_index,header_stamp_ns,bag_time_ns,video_pts_us,pts_adjusted"
                ]
                rows += [
                    f"{j},{hdr_f[i]},{host_f[i] + 6_000_000},{(hdr_f[i] - hdr_f[part[0]]) // 1000},0"
                    for j, i in enumerate(part)
                ]
                (vdir / f"segment_{s:06d}_timestamps.csv").write_text(
                    "\n".join(rows) + "\n"
                )
        for fi, t in enumerate(range(T0_NS, T0_NS + D, int(10e6))):
            bag.add(
                "/markers",
                t + 1_000_000,
                {
                    "header": _hdr(t, "optitrack"),
                    "header_original": _hdr(t, "optitrack"),
                    "frame_number": fi,
                    "mid_exposure_time": 0,
                    "data_received_time": 0,
                    "markers": [
                        {
                            "id_type": 1,
                            "marker_index": m,
                            "marker_name": "",
                            "translation": {"x": 0.1 * m, "y": 0.0, "z": 0.5},
                        }
                        for m in range(3)
                    ],
                },
            )
        (sdir / "eyelink_integrity.json").write_text(
            json.dumps({"synthetic": True})
        )
        (sdir / "session_metadata.yaml").write_text(
            "experimenter: [Synthetic, Data]\nlab: synthetic\ninstitution: synthetic\n"
            "experiment_description: synthetic TableTop session for converter tests\n"
            "subject: {subject_id: synth01, species: Macaca mulatta, sex: U, age: P5Y}\n"
        )

    truth.counts = bag.write(sdir / "bag" / "bag_0.mcap")
    (sdir / "ground_truth.json").write_text(
        json.dumps(asdict(truth), indent=1)
    )
    return sdir, truth
