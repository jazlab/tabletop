"""Post-conversion checks (PLAN §10). Each check yields ``Result`` rows.

Categories: A files · B cross-file · C presence & conservation · D timestamps ·
E sync · F trials & events · G ephys · H spike sorting.

Raw-source counts are recomputed here from the MCAP independently of the
converter's extraction code paths (only the low-level decoder is shared).
"""

from __future__ import annotations

import json
from collections import Counter
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path

import numpy as np
import yaml
from pynwb import NWBHDF5IO, TimeSeries

from ..bag import iter_messages, summarize
from ..sync import (
    EYELINK_SYNC_MASK,
    FLIR_SYNC_BIT,
    FLIR_SYNC_CAMERA,
    UR_SYNC_DI,
    pulses_from_levels,
)

PASS, WARN, FAIL, INFO, SKIP = "PASS", "WARN", "FAIL", "INFO", "SKIP"
CATEGORIES = {
    "A": "Files",
    "B": "Cross-file",
    "C": "Stream presence & conservation",
    "D": "Timestamps",
    "E": "Synchronization",
    "F": "Trials & events",
    "G": "Ephys",
    "H": "Spike sorting",
}


@dataclass
class Result:
    id: str
    status: str
    message: str
    measured: object = None
    threshold: object = None

    @property
    def category(self) -> str:
        return self.id[0]


def _merge(a: dict, b: dict) -> dict:
    out = dict(a)
    for k, v in b.items():
        out[k] = (
            _merge(out[k], v)
            if isinstance(v, dict) and isinstance(out.get(k), dict)
            else v
        )
    return out


@dataclass
class Ctx:
    nwb_dir: Path
    _open: dict = field(default_factory=dict)

    @cached_property
    def manifest(self) -> dict:
        return json.loads((self.nwb_dir / "manifest.json").read_text())

    @cached_property
    def inventory(self) -> dict:
        return json.loads((self.nwb_dir / "inventory.json").read_text())

    @property
    def profile(self) -> str:
        return self.manifest["profile"]

    @cached_property
    def exp(self) -> dict:
        base = yaml.safe_load(
            (Path(__file__).parent / "expectations.yaml").read_text()
        )
        ov = self.nwb_dir / "expectations.override.yaml"
        return (
            _merge(base, yaml.safe_load(ov.read_text()) or {})
            if ov.is_file()
            else base
        )

    def level(self, section: str, name: str) -> str:
        return self.exp[section][name][self.profile]

    def nwb(self, name: str):
        if name not in self._open:
            p = self.nwb_dir / name
            if not p.is_file():
                self._open[name] = None
            else:
                io = NWBHDF5IO(str(p), "r")
                self._open[name] = (io, io.read())
        v = self._open[name]
        return v[1] if v else None

    def close(self) -> None:
        for v in self._open.values():
            if v:
                v[0].close()

    def get(self, file: str, path: str):
        n = self.nwb(file)
        if n is None:
            return None
        kind, _, rest = path.partition("/")
        try:
            if kind == "acquisition":
                return n.acquisition.get(rest)
            if kind == "events":
                return n.events.get(rest) if n.events else None
            if kind == "processing":
                mod, _, obj = rest.partition("/")
                return (
                    n.processing[mod].data_interfaces.get(obj)
                    if mod in n.processing
                    else None
                )
        except (KeyError, AttributeError):
            return None
        return None

    @cached_property
    def provenance(self) -> dict:
        n = self.nwb("task.nwb")
        t = n.processing["provenance"]["provenance"]
        return dict(zip(t["key"][:], t["value"][:]))

    @cached_property
    def sync_report(self) -> dict:
        return json.loads((self.nwb_dir / "sync_report.json").read_text())

    @cached_property
    def duration_s(self) -> float:
        return (
            self.inventory["bag_end_ns"]
            - int(self.manifest["session_start_ns"])
        ) / 1e9

    # ---- independent raw recount -------------------------------------------
    @cached_property
    def raw(self) -> dict:
        bag_dir = Path(self.inventory["bag_dir"])
        s = summarize(bag_dir)
        out = {
            "messages": {t: i.count for t, i in s.topics.items()},
            "eyelink_samples": 0,
            "markers": 0,
            "task_event_types": Counter(),
            "calls": {},
        }
        decode = [
            t
            for t in (
                "/eyelink/sample_array",
                "/markers",
                "/tabletop/task_event",
            )
            if s.count(t)
        ]
        decode += [
            t for t in s.topics if t.endswith("/_service_event") and s.count(t)
        ]
        ev_types: dict[str, Counter] = {}
        for m in iter_messages(bag_dir, decode):
            if m.topic == "/eyelink/sample_array":
                out["eyelink_samples"] += len(m.msg.samples)
            elif m.topic == "/markers":
                out["markers"] += len(m.msg.markers)
            elif m.topic == "/tabletop/task_event":
                out["task_event_types"][int(m.msg.event_type)] += 1
            else:
                ev_types.setdefault(m.topic, Counter())[
                    int(m.msg.info.event_type)
                ] += 1
        for t, c in ev_types.items():
            out["calls"][t] = c[1] if c[1] else c[0]
        return out

    def raw_count(self, stream: dict) -> int:
        unit = stream.get("unit")
        if unit == "samples":
            return self.raw["eyelink_samples"]
        if unit == "markers":
            return self.raw["markers"]
        if unit == "calls":
            return self.raw["calls"].get(stream["topic"], 0)
        return self.raw["messages"].get(stream["topic"], 0)


CHECKS: list[tuple[str, Callable[[Ctx], Iterable[Result]]]] = []


def check(cid: str):
    def deco(fn):
        CHECKS.append((cid, fn))
        return fn

    return deco


def _rows(obj) -> int:
    if obj is None:
        return 0
    if isinstance(obj, TimeSeries):
        return int(obj.data.shape[0])
    return len(obj)


def _timestamps(obj) -> np.ndarray | None:
    if isinstance(obj, TimeSeries):
        return (
            np.asarray(obj.timestamps[:])
            if obj.timestamps is not None
            else None
        )
    if obj is not None and "timestamp" in getattr(obj, "colnames", ()):
        return np.asarray(obj["timestamp"][:])
    return None


def _written(ctx: Ctx) -> list[str]:
    return [
        f
        for f, v in ctx.manifest["files"].items()
        if v.get("status") == "written"
    ]


# =============================== A. files ====================================
@check("A1")
def files_vs_profile(ctx: Ctx):
    for f, levels in ctx.exp["files"].items():
        lvl = levels[ctx.profile]
        entry = ctx.manifest["files"].get(
            f, {"status": "missing_from_manifest"}
        )
        st = entry.get("status")
        on_disk = (ctx.nwb_dir / f).is_file()
        if st == "not_implemented":
            yield Result(
                "A1",
                FAIL,
                f"{f}: source data present but not converted: {entry.get('reason')}",
            )
        elif lvl == "required" and not (st == "written" and on_disk):
            yield Result(
                "A1",
                FAIL,
                f"{f}: required for profile {ctx.profile} but {st}"
                f"{'' if on_disk else ' (not on disk)'}: {entry.get('reason', '')}",
            )
        elif lvl == "absent" and on_disk:
            yield Result(
                "A1",
                WARN,
                f"{f}: written although profile {ctx.profile} does not expect it",
            )
        elif st == "written" and not on_disk:
            yield Result(
                "A1",
                FAIL,
                f"{f}: manifest says written but the file is missing",
            )
        else:
            yield Result("A1", PASS, f"{f}: {st} ({lvl} for {ctx.profile})")


@check("A2")
def files_open(ctx: Ctx):
    for f in _written(ctx):
        try:
            n = ctx.nwb(f)
            yield Result(
                "A2", PASS if n is not None else FAIL, f"{f} opens with pynwb"
            )
        except Exception as e:  # noqa: BLE001
            yield Result("A2", FAIL, f"{f} does not open: {e!r}")


@check("A3")
def inspector(ctx: Ctx):
    try:
        from nwbinspector import inspect_nwbfile
    except ImportError:
        yield Result("A3", SKIP, "nwbinspector not installed")
        return
    cfg = ctx.exp.get("inspector", {})
    ignore = set(cfg.get("ignore", []))
    sim_relaxed = set(cfg.get("sim_warn_only", []))
    for f in _written(ctx):
        msgs = list(inspect_nwbfile(nwbfile_path=str(ctx.nwb_dir / f)))
        by = Counter(m.importance.name for m in msgs)
        kept = [m for m in msgs if m.check_function_name not in ignore]
        fails, warns = [], []
        for m in kept:
            imp = m.importance.name
            if imp in ("CRITICAL", "PYNWB_VALIDATION"):
                (
                    warns
                    if ctx.profile == "sim"
                    and m.check_function_name in sim_relaxed
                    else fails
                ).append(m)
            elif imp in (
                "BEST_PRACTICE_VIOLATION",
                "ERROR",
            ):  # ERROR = the inspector itself crashed on a check
                warns.append(m)
        names = sorted({m.check_function_name for m in fails or warns})
        detail = f"nwbinspector {dict(by)}; ignored {len(msgs) - len(kept)} ({', '.join(sorted(ignore))})"
        if fails:
            yield Result(
                "A3",
                FAIL,
                f"{f}: {len(fails)} critical: {', '.join(names)} | first: {fails[0].message[:160]} "
                f"@ {fails[0].location} | {detail}",
                dict(by),
            )
        elif warns:
            yield Result(
                "A3",
                WARN,
                f"{f}: {len(warns)} warnings: {', '.join(names)} | {detail}",
                dict(by),
            )
        else:
            yield Result(
                "A3",
                PASS,
                f"{f}: no critical issues or violations | {detail}",
                dict(by),
            )


@check("A4")
def external_video(ctx: Ctx):
    n = ctx.nwb("video.nwb")
    if n is None:
        yield Result("A4", SKIP, "no video.nwb")
        return
    series = [
        s
        for s in n.acquisition.values()
        if getattr(s, "external_file", None) is not None
    ]
    if not series:
        yield Result(
            "A4",
            WARN,
            "video.nwb has camera metadata but no video: run `unbag --handlers video` first",
        )
        return
    for s in series:
        missing = [
            p for p in s.external_file[:] if not (ctx.nwb_dir / p).is_file()
        ]
        yield Result(
            "A4",
            FAIL if missing else PASS,
            f"{s.name}: {len(s.external_file)} segments"
            + (f", missing {missing[:3]}" if missing else " exist"),
        )


@check("A5")
def sim_labelled(ctx: Ctx):
    for f in _written(ctx):
        n = ctx.nwb(f)
        kw = list(n.keywords[:]) if n.keywords is not None else []
        ok = f"profile:{ctx.profile}" in kw and (
            ctx.profile != "sim" or "SIMULATED" in n.session_description
        )
        yield Result(
            "A5",
            PASS if ok else FAIL,
            f"{f}: profile {ctx.profile} recorded in keywords/description",
        )


# ============================== B. cross-file ================================
@check("B1")
def shared_fields(ctx: Ctx):
    files = [f for f in _written(ctx) if ctx.nwb(f) is not None]
    if len(files) < 2:
        yield Result("B1", SKIP, "fewer than two NWB files")
        return
    for attr in (
        "session_start_time",
        "session_id",
        "timestamps_reference_time",
    ):
        vals = {f: str(getattr(ctx.nwb(f), attr)) for f in files}
        yield Result(
            "B1",
            PASS if len(set(vals.values())) == 1 else FAIL,
            f"{attr} identical across files",
            vals,
        )
    subj = {
        f: (ctx.nwb(f).subject.subject_id if ctx.nwb(f).subject else None)
        for f in files
    }
    yield Result(
        "B1",
        PASS if len(set(subj.values())) == 1 else FAIL,
        "subject_id identical across files",
        subj,
    )


@check("B2")
def shared_provenance(ctx: Ctx):
    files = [f for f in _written(ctx) if ctx.nwb(f) is not None]
    vals = {}
    for f in files:
        t = ctx.nwb(f).processing["provenance"]["provenance"]
        d = dict(zip(t["key"][:], t["value"][:]))
        vals[f] = (
            d.get("session_start_ns"),
            d.get("profile"),
            d.get("sync_report"),
        )
    same = len(set(vals.values())) == 1
    yield Result(
        "B2",
        PASS if same else FAIL,
        "session_start_ns, profile and sync report identical across files",
    )
    start = vals[files[0]][0] if files else None
    yield Result(
        "B2",
        PASS if str(ctx.manifest["session_start_ns"]) == str(start) else FAIL,
        "manifest session_start_ns matches the files",
    )


# ===================== C. stream presence & conservation =====================
@check("C1")
def presence_and_counts(ctx: Ctx):
    skipped_js = int(ctx.provenance.get("joint_name_changes_skipped", 0))
    for name, st in ctx.exp["streams"].items():
        lvl = st[ctx.profile]
        obj = ctx.get("task.nwb", st["nwb"])
        raw = ctx.raw_count(st)
        got = _rows(obj)
        if name == "joint_states":
            raw -= skipped_js
        if raw == 0 and obj is None:
            sev = FAIL if lvl == "required" else INFO
            yield Result(
                "C1", sev, f"{name}: not recorded ({lvl} for {ctx.profile})"
            )
            continue
        if raw > 0 and obj is None:
            yield Result(
                "C1",
                FAIL,
                f"{name}: {raw} in raw source but missing from task.nwb (lost in conversion)",
            )
            continue
        if lvl == "absent":
            yield Result(
                "C1",
                WARN,
                f"{name}: present ({got}) although profile {ctx.profile} does not expect it",
            )
        yield Result(
            "C1",
            PASS if got == raw else FAIL,
            f"{name}: {got} written vs {raw} in raw source",
            got,
            raw,
        )


@check("C3")
def cameras_complete(ctx: Ctx):
    """Every camera with video segments / metadata in the raw session is in video.nwb, frame for frame."""
    cams_video = ctx.inventory.get("video_dirs", {})
    cams_meta = {
        t.split("/")[2]: c
        for t, c in ctx.raw["messages"].items()
        if t.startswith("/cam_sync/") and t.endswith("/meta") and c > 0
    }
    if not cams_video and not cams_meta:
        yield Result("C3", SKIP, "no cameras recorded")
        return
    n = ctx.nwb("video.nwb")
    if n is None:
        yield Result(
            "C3",
            FAIL,
            f"cameras recorded ({sorted(set(cams_video) | set(cams_meta))}) but no video.nwb",
        )
        return
    session_dir = Path(ctx.inventory["session_dir"])
    for cam, segs in sorted(cams_video.items()):
        s = n.acquisition.get(cam)
        if s is None:
            yield Result(
                "C3",
                FAIL,
                f"{cam}: {len(segs)} video segments on disk but no ImageSeries in video.nwb",
            )
            continue
        raw_frames = sum(
            sum(
                1
                for _ in open(
                    session_dir / p.replace(".mkv", "_timestamps.csv")
                )
            )
            - 1
            for p in segs
        )
        got = len(s.timestamps)
        yield Result(
            "C3",
            PASS if got == raw_frames else FAIL,
            f"{cam}: {got} frame timestamps vs {raw_frames} in "
            f"segment sidecars",
            got,
            raw_frames,
        )
    meta = (
        n.processing["camera_metadata"].data_interfaces
        if "camera_metadata" in n.processing
        else {}
    )
    for cam, c in sorted(cams_meta.items()):
        s = meta.get(f"{cam}_camera_frame_id")
        got = 0 if s is None else len(s.data)
        yield Result(
            "C3",
            PASS if got == c else FAIL,
            f"{cam}: {got} metadata rows vs {c} in bag",
            got,
            c,
        )


@check("C2")
def value_sanity(ctx: Ctx):
    g = ctx.get("task.nwb", "acquisition/teensy_glove_adc")
    if g is not None:
        d = g.data[:]
        ok = d.min() >= 0 and d.max() <= 1023
        yield Result(
            "C2",
            PASS if ok else FAIL,
            f"glove ADC within 0–1023 (min {d.min()}, max {d.max()})",
        )
    j = ctx.get("task.nwb", "acquisition/robot_joint_position")
    if j is not None:
        d = j.data[:]
        ok = np.all(np.isfinite(d)) and np.abs(d).max() <= 2 * np.pi + 0.1
        yield Result(
            "C2",
            PASS if ok else FAIL,
            f"joint positions finite and within ±2π (max |q| {np.abs(d).max():.2f})",
        )
    for side in ("left", "right"):
        f = ctx.get("task.nwb", f"acquisition/robot_{side}_force")
        if f is not None:
            frac = float(np.mean(~np.isfinite(f.data[:])))
            yield Result(
                "C2",
                PASS if frac == 0 else WARN if frac < 1 else INFO,
                f"{side} wrench: {frac:.0%} non-finite"
                + (" (mock robot publishes NaN)" if frac == 1 else ""),
            )
    e = ctx.get("task.nwb", "acquisition/eyelink_input")
    if e is not None:
        d = e.data[:]
        yield Result(
            "C2",
            PASS if d.min() >= 0 and d.max() <= 255 else FAIL,
            "EyeLink input word within 0–255",
        )
    for name in ("left_eye", "right_eye"):
        et = ctx.get("task.nwb", "acquisition/EyeTracking")
        if et is not None and name in et.spatial_series:
            d = et.spatial_series[name].data[:]
            yield Result(
                "C2",
                PASS if np.any(np.isfinite(d)) else WARN,
                f"{name}: {np.mean(np.isfinite(d[:, 0])):.1%} valid samples (missing sentinel → NaN)",
            )


# =============================== D. timestamps ================================
def _all_series(ctx: Ctx):
    for f in _written(ctx):
        n = ctx.nwb(f)
        objs = list(n.acquisition.values())
        for mod in n.processing.values():
            objs += list(mod.data_interfaces.values())
        if n.events:
            objs += list(n.events.values())
        for o in objs:
            if hasattr(o, "spatial_series"):
                yield from ((f, s) for s in o.spatial_series.values())
            elif hasattr(o, "time_series") and not isinstance(o, TimeSeries):
                yield from ((f, s) for s in o.time_series.values())
            else:
                yield f, o


@check("D1")
def timestamps_valid(ctx: Ctx):
    margin = ctx.exp["thresholds"]["timestamp_margin_s"]
    lo, hi = -margin, ctx.duration_s + margin
    seen = set()
    for f, o in _all_series(ctx):
        ts = _timestamps(o)
        if ts is None or len(ts) == 0:
            continue
        key = (
            f,
            id(o.timestamps)
            if isinstance(o, TimeSeries)
            and isinstance(o.timestamps, TimeSeries)
            else o.name,
        )
        if key in seen:
            continue
        seen.add(key)
        fitted_nan = o.name.endswith(
            "_sync_pulses"
        )  # unaligned detected pulses are NaN by design
        finite = np.isfinite(ts)
        if not finite.all() and not fitted_nan:
            yield Result(
                "D1",
                FAIL,
                f"{f}:{o.name}: {np.sum(~finite)} non-finite timestamps",
            )
            continue
        t = ts[finite]
        if not len(t):
            continue
        d = np.diff(t)
        out = np.sum((t < lo) | (t > hi))
        problems = []
        if np.any(d < 0):
            problems.append(
                f"{np.sum(d < 0)} decreasing steps (min {d.min() * 1e3:.3f} ms)"
            )
        if out:
            problems.append(f"{out} outside [{lo:.1f}, {hi:.1f}] s")
        # long-format tables (one row per marker) legitimately repeat timestamps
        dup = (
            float(np.mean(d == 0))
            if len(d) and isinstance(o, TimeSeries)
            else 0.0
        )
        if problems:
            sev = FAIL
        elif dup > 0.01:
            sev, problems = WARN, [f"{dup:.1%} duplicate timestamps"]
        else:
            sev = PASS
        yield Result(
            "D1",
            sev,
            f"{f}:{o.name}: {len(t)} timestamps "
            + ("; ".join(problems) or "finite, ordered, in range"),
        )


@check("D2")
def rates_and_gaps(ctx: Ctx):
    th = ctx.exp["thresholds"]
    for name, st in ctx.exp["streams"].items():
        obj = ctx.get("task.nwb", st["nwb"])
        ts = _timestamps(obj)
        if ts is None or len(ts) < 3:
            continue
        d = np.diff(ts)
        med = float(np.median(d[d > 0])) if np.any(d > 0) else float("nan")
        if "rate_hz" in st and np.isfinite(med):
            rate = 1 / med
            ok = abs(rate / st["rate_hz"] - 1) <= th["rate_tolerance"]
            # the simulator does not run in real time, so its rates are informational
            sev = PASS if ok else INFO if ctx.profile == "sim" else WARN
            yield Result(
                "D2",
                sev,
                f"{name}: median rate {rate:.1f} Hz (expected {st['rate_hz']})",
                rate,
                st["rate_hz"],
            )
        if name in (
            "task_events",
            "flic_press",
            "reward",
            "arm_lock",
            "smartglass",
            "solenoid",
            "buzzer",
            "rosout",
        ):
            continue
        big = d > th["gap_factor"] * med
        yield Result(
            "D2",
            WARN if big.any() else PASS,
            f"{name}: {int(big.sum())} gaps > {th['gap_factor']}× median interval"
            + (f" (largest {d.max():.3f} s)" if big.any() else ""),
            int(big.sum()),
        )


@check("D4")
def reordered(ctx: Ctx):
    """Samples the converter had to reorder because their stamps were out of receive order."""
    r = json.loads(ctx.provenance.get("reordered_samples", "{}"))
    noisy = {
        "rosout",
        "teensy_log",
    }  # many publishers / clocks in one topic: disorder is expected
    for name, n in sorted(r.items()):
        yield Result(
            "D4",
            INFO if name in noisy else WARN,
            f"{name}: {n} samples reordered by timestamp (header stamps out of receive order)",
            n,
        )
    if not r:
        yield Result("D4", PASS, "no stream needed reordering")


@check("D3")
def cameras_agree(ctx: Ctx):
    n = ctx.nwb("video.nwb")
    if n is None or "camera_metadata" not in n.processing:
        yield Result("D3", SKIP, "no camera metadata")
        return
    m = n.processing["camera_metadata"].data_interfaces
    cams = sorted(
        {
            k.rsplit("_camera_frame_id", 1)[0]
            for k in m
            if k.endswith("_camera_frame_id")
        }
    )
    ref = None
    for c in cams:
        fid = m[f"{c}_camera_frame_id"].data[:]
        t = m[f"{c}_camera_frame_id"].timestamps[:]
        if ref is None:
            ref = (c, dict(zip(fid.tolist(), t.tolist())))
            continue
        common = [
            (t_, ref[1][f])
            for f, t_ in zip(fid.tolist(), t.tolist())
            if f in ref[1]
        ]
        if not common:
            yield Result(
                "D3", WARN, f"{c}: no frame ids in common with {ref[0]}"
            )
            continue
        diff = np.abs(np.array([a - b for a, b in common])) * 1e3
        yield Result(
            "D3",
            PASS if diff.max() <= 1.0 else WARN,
            f"{c} vs {ref[0]}: same frame id → max time difference {diff.max():.3f} ms over {len(common)} frames",
            float(diff.max()),
            1.0,
        )


# ============================ E. synchronization ==============================
def _fit(ctx: Ctx, stream: str) -> dict | None:
    return ctx.sync_report["fits"].get(stream)


@check("E1")
def sync_methods(ctx: Ctx):
    task_ran = ctx.raw["task_event_types"].get(1, 0) > 0
    for stream, levels in ctx.exp["sync"].items():
        want = levels[ctx.profile]
        f = _fit(ctx, stream)
        status = f["status"] if f else "absent"
        if want in ("reference", "reference_if_task"):
            n = f["n_stream_pulses"] if f else 0
            if n > 0:
                yield Result("E1", PASS, f"teensy reference: {n} pulses")
            elif want == "reference_if_task" and not task_ran:
                yield Result(
                    "E1",
                    INFO,
                    "teensy reference: no pulses (no task ran, so pulses were never enabled)",
                )
            else:
                yield Result(
                    "E1",
                    FAIL,
                    "teensy reference: no sync pulses recorded"
                    + (" although a task ran" if task_ran else ""),
                )
        elif want == "absent":
            yield Result(
                "E1",
                PASS if f is None else WARN,
                f"{stream}: {'absent' if f is None else status} "
                f"(not expected for {ctx.profile})",
            )
        elif want == "host_clock":
            if f is not None and status == "ok":
                yield Result(
                    "E1",
                    FAIL,
                    f"{stream}: pulse fit succeeded on a stream expected to carry no real "
                    f"pulses (spurious fit)",
                )
            else:
                yield Result(
                    "E1",
                    PASS,
                    f"{stream}: no pulse fit ({status}); host clock used as expected",
                )
        elif want == "any":
            yield Result("E1", INFO, f"{stream}: {status}")
        elif want in ("pulse_fit", "pulse_fit_warn"):
            if f is None:
                yield Result(
                    "E1",
                    FAIL if want == "pulse_fit" else WARN,
                    f"{stream}: stream missing, cannot sync",
                )
            else:
                sev = (
                    PASS
                    if status == "ok"
                    else FAIL
                    if want == "pulse_fit"
                    else WARN
                )
                yield Result(
                    "E1",
                    sev,
                    f"{stream}: {status}"
                    + (f" ({f['note']})" if f.get("note") else ""),
                )


@check("E2")
def sync_quality(ctx: Ctx):
    th = ctx.exp["thresholds"]
    for stream, cls in ctx.exp["sync_class"].items():
        f = _fit(ctx, stream)
        if not f or f["status"] != "ok":
            continue
        mf = f["matched_fraction"]
        yield Result(
            "E2",
            FAIL
            if mf < th["matched_fraction"]["fail"]
            else WARN
            if mf < th["matched_fraction"]["warn"]
            else PASS,
            f"{stream}: matched {f['n_matched']}/{f['n_stream_pulses']} pulses ({mf:.1%})",
            mf,
        )
        r = th["residual_ms"][cls]
        rms, mx = f["residual_rms_ms"], f["residual_max_ms"]
        sev = (
            FAIL
            if rms > r["rms_fail"] or mx > r["max_fail"]
            else WARN
            if rms > r["rms_warn"]
            else PASS
        )
        yield Result(
            "E2",
            sev,
            f"{stream}: residual rms {rms:.3f} ms, max {mx:.3f} ms ({cls})",
            (rms, mx),
            r,
        )
        dp = abs(f["drift_ppm"])
        yield Result(
            "E2",
            FAIL
            if dp > th["drift_ppm_abs"]["fail"]
            else WARN
            if dp > th["drift_ppm_abs"]["warn"]
            else PASS,
            f"{stream}: drift {f['drift_ppm']:.1f} ppm",
            f["drift_ppm"],
        )
        yield Result(
            "E2",
            INFO,
            f"{stream}: fit covers {f['coverage_s']:.0f} s",
            f["coverage_s"],
        )


@check("E3")
def teensy_train(ctx: Ctx):
    p = ctx.get("task.nwb", "processing/sync/teensy_sync_pulses")
    if p is None or len(p) == 0:
        yield Result("E3", SKIP, "no Teensy pulses")
        return
    th = ctx.exp["thresholds"]
    t = p["timestamp"][:]
    ipi = np.diff(t)
    lo, hi = th["teensy_ipi_s"]
    bad = (ipi < lo) | (ipi > hi)
    yield Result(
        "E3",
        PASS if bad.mean() < 0.01 else WARN,
        f"Teensy inter-pulse intervals: {bad.sum()}/{len(ipi)} outside [{lo}, {hi}] s "
        f"(range {ipi.min():.3f}–{ipi.max():.3f} s)",
    )
    w = p["duration"][:]
    w = w[np.isfinite(w)]
    lo, hi = th["teensy_width_s"]
    badw = (w < lo) | (w > hi)
    yield Result(
        "E3",
        PASS if badw.mean() < 0.01 else WARN,
        f"Teensy pulse widths: {badw.sum()}/{len(w)} outside "
        f"[{lo}, {hi}] s (median {np.median(w) * 1e3:.1f} ms)",
    )


def _nwb_pulses(ctx: Ctx, stream: str) -> np.ndarray | None:
    """Re-detect pulse onsets from data as written to NWB, on the NWB timeline."""
    if stream == "eyelink":
        s = ctx.get("task.nwb", "acquisition/eyelink_input")
        if s is None:
            return None
        return pulses_from_levels(
            s.timestamps[:],
            (s.data[:].astype(np.int64) & EYELINK_SYNC_MASK) == 0,
        ).onsets
    if stream.startswith("ur_"):
        s = ctx.get("task.nwb", f"acquisition/{stream}_digital_in")
        if s is None:
            return None
        return pulses_from_levels(
            s.timestamps[:], s.data[:, UR_SYNC_DI] == 0
        ).onsets
    if stream == "cameras":
        n = ctx.nwb("video.nwb")
        if n is None or "camera_metadata" not in n.processing:
            return None
        s = n.processing["camera_metadata"].data_interfaces.get(
            f"{FLIR_SYNC_CAMERA}_line_status"
        )
        if s is None:
            return None
        return pulses_from_levels(
            s.timestamps[:],
            ((s.data[:].astype(np.int64) >> FLIR_SYNC_BIT) & 1) == 1,
        ).onsets
    return None


def _nearest_ms(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    j = np.clip(np.searchsorted(b, a), 1, len(b) - 1)
    return np.minimum(np.abs(a - b[j - 1]), np.abs(a - b[j])) * 1e3


@check("E4")
def closure(ctx: Ctx):
    """Pulses re-detected from the written NWB data must land on the Teensy pulses."""
    ref = ctx.get("task.nwb", "processing/sync/teensy_sync_pulses")
    if ref is None or len(ref) < 2:
        yield Result("E4", SKIP, "no Teensy reference pulses")
        return
    r = np.sort(ref["timestamp"][:])
    th = ctx.exp["thresholds"]["closure_median_ms"]
    for stream, cls in ctx.exp["sync_class"].items():
        f = _fit(ctx, stream)
        if not f or f["status"] != "ok":
            yield Result(
                "E4", SKIP, f"{stream}: no pulse fit, nothing to close"
            )
            continue
        on = _nwb_pulses(ctx, stream)
        if on is None or len(on) == 0:
            yield Result(
                "E4",
                FAIL,
                f"{stream}: fit reported ok but no pulses found in the written NWB data",
            )
            continue
        on = on[(on >= r[0] - 0.5) & (on <= r[-1] + 0.5)]
        e = _nearest_ms(on, r)
        med = float(np.median(e))
        yield Result(
            "E4",
            PASS if med <= th[cls] else FAIL,
            f"{stream}: {len(on)} pulses re-detected in NWB; median |Δ| to Teensy {med:.3f} ms, "
            f"p95 {np.percentile(e, 95):.3f} ms",
            med,
            th[cls],
        )


@check("E5")
def triangle(ctx: Ctx):
    """Hardware streams aligned independently must agree with each other."""
    a, b = "eyelink", "cameras"
    fa, fb = _fit(ctx, a), _fit(ctx, b)
    if not (fa and fb and fa["status"] == "ok" and fb["status"] == "ok"):
        yield Result("E5", SKIP, f"needs pulse fits for both {a} and {b}")
        return
    pa, pb = _nwb_pulses(ctx, a), _nwb_pulses(ctx, b)
    e = _nearest_ms(pa, np.sort(pb))
    e = e[e < 100]
    th = ctx.exp["thresholds"]["triangle_median_ms"]
    med = float(np.median(e)) if len(e) else float("inf")
    yield Result(
        "E5",
        PASS if med <= th else FAIL,
        f"{a} ↔ {b}: median |Δ| {med:.3f} ms over {len(e)} pulses",
        med,
        th,
    )


# =========================== F. trials & events ===============================
@check("F1")
def trials_conservation(ctx: Ctx):
    n = ctx.nwb("task.nwb")
    raw = ctx.raw["task_event_types"]
    nt = len(n.trials) if n.trials is not None else 0
    yield Result(
        "F1",
        PASS if nt == raw.get(3, 0) else FAIL,
        f"{nt} trials vs {raw.get(3, 0)} TRIAL_PRESENTED in raw",
    )
    te = ctx.get("task.nwb", "events/task_events")
    if te is not None:
        got = Counter(te["event_type"][:].tolist())
        for et in sorted(set(raw) | set(got)):
            yield Result(
                "F1",
                PASS if got.get(et, 0) == raw.get(et, 0) else FAIL,
                f"task event type {et}: {got.get(et, 0)} written vs {raw.get(et, 0)} raw",
            )


@check("F2")
def trials_consistency(ctx: Ctx):
    n = ctx.nwb("task.nwb")
    if n.trials is None or len(n.trials) == 0:
        yield Result("F2", SKIP, "no trials")
        return
    df = n.trials.to_dataframe()
    s, e = df.start_time.to_numpy(), df.stop_time.to_numpy()
    yield Result(
        "F2",
        PASS if np.all(e > s) else FAIL,
        "every trial has stop_time > start_time",
    )
    yield Result(
        "F2",
        PASS if np.all(np.diff(s) > 0) else FAIL,
        "trials sorted by start_time",
    )
    ov = np.sum(s[1:] < e[:-1])
    yield Result(
        "F2",
        PASS if ov == 0 else FAIL,
        f"trials do not overlap ({ov} overlaps)",
    )
    for c in [
        c
        for c in df.columns
        if c.endswith("_time") and c not in ("start_time", "stop_time")
    ]:
        v = df[c].to_numpy(dtype=float)
        ok = np.isnan(v) | ((v >= s - 1e-9) & (v <= e + 1e-9))
        yield Result(
            "F2",
            PASS if ok.all() else FAIL,
            f"trials.{c} inside its trial window ({np.sum(~ok)} outside)",
        )
    inc = int((~df.completed.astype(bool)).sum())
    yield Result(
        "F2",
        INFO,
        f"{inc} of {len(df)} trials have no TRIAL_DEPRESENTING (completed=False)",
        inc,
    )


@check("F3")
def event_assignment(ctx: Ctx):
    n = ctx.nwb("task.nwb")
    te = ctx.get("task.nwb", "events/task_events")
    if te is None or n.trials is None or len(n.trials) == 0:
        yield Result("F3", SKIP, "no task events or trials")
        return
    tn = te["trial_number"][:]
    ti = te["trial_index"][:]
    tagged = tn >= 0
    yield Result(
        "F3",
        PASS if np.all(ti[tagged] >= 0) else FAIL,
        f"every trial-tagged task event is assigned to a trial ({np.sum(ti[tagged] < 0)} unassigned)",
    )
    tr = n.trials.to_dataframe()
    ts = te["timestamp"][:]
    ok = [
        tr.start_time.iloc[i] - 1e-9 <= t <= tr.stop_time.iloc[i] + 1e-9
        for t, i in zip(ts[tagged], ti[tagged])
        if i >= 0
    ]
    yield Result(
        "F3",
        PASS if all(ok) else FAIL,
        "assigned task events lie inside their trial window",
    )
    num_ok = all(
        tr.trial_number.iloc[i] == k
        for k, i in zip(tn[tagged], ti[tagged])
        if i >= 0
    )
    yield Result(
        "F3",
        PASS if num_ok else FAIL,
        "assigned trial has the event's trial_number",
    )
    for name in ("reward", "flic_press"):
        ev = ctx.get("task.nwb", f"events/{name}")
        if ev is None:
            continue
        inside = int(np.sum(ev["trial_index"][:] >= 0))
        yield Result(
            "F3",
            INFO,
            f"{name}: {inside}/{len(ev)} inside a trial window",
            inside,
        )
    if (
        "n_rewards" in tr
        and (rw := ctx.get("task.nwb", "events/reward")) is not None
    ):
        act = (
            rw["req_activate"][:]
            if "req_activate" in rw.colnames
            else np.ones(len(rw))
        )
        inside = np.sum((rw["trial_index"][:] >= 0) & (np.asarray(act) > 0))
        yield Result(
            "F3",
            PASS if tr.n_rewards.sum() == inside else FAIL,
            f"trials.n_rewards sums to the reward activations inside trials ({tr.n_rewards.sum()} vs {inside})",
        )


# ============================ G/H. ephys & sorting ============================
@check("G1")
def ephys(ctx: Ctx):
    if ctx.profile != "rig+ephys":
        yield Result("G1", SKIP, f"profile {ctx.profile}: no ephys")
    else:
        yield Result(
            "G1",
            FAIL,
            "ephys checks not implemented yet (ephys.nwb writer pending)",
        )


@check("H1")
def sorting(ctx: Ctx):
    yield Result("H1", SKIP, "spike_sorting.nwb not produced yet")


def run_checks(
    nwb_dir: Path, only: Iterable[str] | None = None
) -> list[Result]:
    ctx = Ctx(Path(nwb_dir))
    results: list[Result] = []
    try:
        for cid, fn in CHECKS:
            if only and cid[0] not in only:
                continue
            try:
                results.extend(fn(ctx))
            except Exception as e:  # noqa: BLE001 — a crashing check is a FAIL, not a crash
                results.append(
                    Result(
                        cid, FAIL, f"check crashed: {type(e).__name__}: {e}"
                    )
                )
    finally:
        ctx.close()
    return results
