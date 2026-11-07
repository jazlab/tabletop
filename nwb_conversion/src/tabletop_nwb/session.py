"""Stage [0]: inventory a session directory and choose its recording profile."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import yaml

from .bag import BagSummary, iter_messages, summarize
from .extract import CAMERAS

PROFILES = ("sim", "rig", "rig+ephys")


class ProfileError(RuntimeError):
    """The requested profile contradicts what the session directory contains."""


@dataclass
class Inventory:
    session_dir: str
    session_id: str
    bag_dir: str
    bag_files: list[str]
    bag_start_ns: int | None
    bag_end_ns: int | None
    topic_counts: dict[str, int]
    topic_schemas: dict[str, str]
    edf_path: str | None
    eyelink_integrity: dict | None
    session_integrity: dict | None
    session_metadata: dict | None
    video_dirs: dict[
        str, list[str]
    ]  # camera → segment .mkv paths (relative to session dir)
    spikeglx_meta: list[str]
    eyelink_input_change_fraction: float | None
    profile: str = ""
    profile_source: str = ""
    profile_reasons: list[str] = field(default_factory=list)

    def to_json(self) -> dict:
        return asdict(self)

    def count(self, topic: str) -> int:
        return self.topic_counts.get(topic, 0)


def _eyelink_input_change_fraction(
    bag_dir: Path, summary: BagSummary, max_msgs: int = 300
) -> float | None:
    """Fraction of consecutive EyeLink samples whose input word changes.

    Real sync pulses change it ~2× per second (≈0.002 at 1 kHz); the
    simulator draws it at random every sample (≈0.5).
    """
    if summary.count("/eyelink/sample_array") == 0:
        return None
    vals = []
    for i, m in enumerate(iter_messages(bag_dir, ["/eyelink/sample_array"])):
        vals.extend(s.input for s in m.msg.samples)
        if i >= max_msgs:
            break
    if len(vals) < 2:
        return None
    return float(np.mean(np.diff(np.asarray(vals)) != 0))


def _find_video(session_dir: Path) -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    unbag = session_dir / "unbag"
    for cam in CAMERAS:
        d = unbag / f"cam_sync_{cam}_image_raw_video"
        if d.is_dir():
            segs = sorted(d.glob("segment_*.mkv"))
            if segs:
                out[cam] = [str(p.relative_to(session_dir)) for p in segs]
    return out


def _load_json(p: Path) -> dict | None:
    if p.is_file():
        try:
            return json.loads(p.read_text())
        except json.JSONDecodeError:
            return {"_error": f"could not parse {p.name}"}
    return None


def inventory(session_dir: Path, profile: str | None = None) -> Inventory:
    session_dir = Path(session_dir).resolve()
    bag_dir = session_dir / "bag"
    if not bag_dir.is_dir():
        raise FileNotFoundError(f"{session_dir} has no bag/ directory")
    summary = summarize(bag_dir)
    edf = session_dir / "eyelink_received" / "last.edf"
    meta_path = session_dir / "session_metadata.yaml"
    inv = Inventory(
        session_dir=str(session_dir),
        session_id=session_dir.name,
        bag_dir=str(bag_dir),
        bag_files=[str(f) for f in summary.files],
        bag_start_ns=summary.start_ns,
        bag_end_ns=summary.end_ns,
        topic_counts={t: i.count for t, i in sorted(summary.topics.items())},
        topic_schemas={t: i.schema for t, i in sorted(summary.topics.items())},
        edf_path=str(edf) if edf.is_file() else None,
        eyelink_integrity=_load_json(session_dir / "eyelink_integrity.json"),
        session_integrity=_load_json(session_dir / "session_integrity.json"),
        session_metadata=yaml.safe_load(meta_path.read_text())
        if meta_path.is_file()
        else None,
        video_dirs=_find_video(session_dir),
        spikeglx_meta=sorted(
            str(p.relative_to(session_dir))
            for p in session_dir.rglob("*.ap.meta")
        ),
        eyelink_input_change_fraction=_eyelink_input_change_fraction(
            bag_dir, summary
        ),
    )
    choose_profile(inv, profile)
    return inv


def evidence(inv: Inventory) -> tuple[list[str], list[str], list[str]]:
    """(ephys, rig, sim) evidence strings."""
    ephys = [f"SpikeGLX metadata: {m}" for m in inv.spikeglx_meta]
    rig = []
    if inv.edf_path:
        rig.append("EyeLink EDF present")
    if inv.eyelink_integrity:
        rig.append("eyelink_integrity.json present")
    if inv.video_dirs:
        rig.append(f"video segments for {len(inv.video_dirs)} cameras")
    n_meta = sum(inv.count(f"/cam_sync/{c}/meta") for c in CAMERAS)
    n_img = sum(inv.count(f"/cam_sync/{c}/image_raw") for c in CAMERAS)
    if n_meta or n_img:
        rig.append(f"{n_meta} camera metadata / {n_img} image messages in bag")
    if inv.count("/markers"):
        rig.append(f"{inv.count('/markers')} OptiTrack marker messages")
    sim = []
    f = inv.eyelink_input_change_fraction
    if f is not None and f > 0.2:
        sim.append(
            f"EyeLink input word changes on {f:.0%} of samples (simulator draws it at random)"
        )
    if not rig:
        sim.append("no rig-only sources (EDF, cameras, OptiTrack)")
    return ephys, rig, sim


def choose_profile(inv: Inventory, requested: str | None) -> None:
    ephys, rig, sim = evidence(inv)
    inferred = "rig+ephys" if ephys else "rig" if rig else "sim"
    reasons = ephys or rig or sim
    if requested is None:
        inv.profile, inv.profile_source, inv.profile_reasons = (
            inferred,
            "inferred",
            reasons,
        )
        return
    if requested not in PROFILES:
        raise ProfileError(
            f"unknown profile {requested!r}; choose from {PROFILES}"
        )
    problems = []
    if requested == "rig+ephys" and not ephys:
        problems.append(
            "profile rig+ephys but no SpikeGLX *.ap.meta found in the session directory"
        )
    if requested in ("sim", "rig") and ephys:
        problems.append(
            f"profile {requested} but SpikeGLX data is present ({ephys[0]}); it would be silently dropped"
        )
    if requested == "sim" and rig:
        problems.append(
            f"profile sim but rig-only sources are present: {'; '.join(rig)}"
        )
    if (
        requested in ("rig", "rig+ephys")
        and not rig
        and any("random" in s for s in sim)
    ):
        problems.append(
            f"profile {requested} but the session looks simulated: {'; '.join(sim)}"
        )
    if problems:
        raise ProfileError(" | ".join(problems))
    inv.profile, inv.profile_source = requested, "requested"
    inv.profile_reasons = reasons + (
        [f"inferred profile would be {inferred}"]
        if inferred != requested
        else []
    )
