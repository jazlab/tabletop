"""Robot motion clips cut from recorded sessions, replacing MoveIt.

The rig plans every robot motion with MoveIt. The MuJoCo task plays back
motions recorded from real executions instead, cut at points the rig's
manipulation state machine makes unambiguous:

- ``fetch[object]``: from the arm's previous release (or its first motion of
  the session), through the grasp, to arrival at the SRDF ``fetched`` pose;
- ``present``: leaving ``fetched`` until ``TRIAL_PRESENTED`` (the SRDF
  ``present`` pose is the first presentation pose);
- ``unpresent``: ``TRIAL_DEPRESENTING`` until arrival back at ``fetched``;
- ``return[object]``: leaving ``fetched`` until the release at the mount
  (a session's final return continues to the next still period).

Clips keep any still periods inside them (on the Mac those include slow,
emulated planning). Clips are joined with minimum-jerk joint-space bridges
when their endpoints don't match.
"""

from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from virtual_tabletop import paths
from virtual_tabletop.assets.build import ROBOT_KP
from virtual_tabletop.assets.named_poses import load_named_poses
from virtual_tabletop.bags.topics import TaskEventType
from virtual_tabletop.replay.timeline import Timeline, build_timeline

#: Joints of one arm, in actuator order (without the side prefix).
ARM_JOINTS = tuple(ROBOT_KP)

#: Group name of each arm side.
GROUPS = {"left": "left_manipulator", "right": "right_manipulator"}

STILL_SPEED = 0.02  # rad/s: below this the arm counts as still
STILL_MIN = 0.4  # s: a still period at least this long ends/starts a clip
AT_POSE_TOL = 5e-3  # rad: max joint error to count as "at" a named pose


@dataclass(frozen=True)
class MotionClip:
    """A recorded joint trajectory of one arm.

    Attributes:
        arm: ``"left"`` or ``"right"``.
        kind: ``fetch``, ``present``, ``unpresent`` or ``return``.
        object_id: Object carried or fetched, ``""`` for generic clips.
        t: Times from clip start (s), shape ``(N,)``, starting at 0.
        q: Joint positions in ``ARM_JOINTS`` order, shape ``(N, 6)``.
        source: Where the clip came from (bag path and time).
    """

    arm: str
    kind: str
    object_id: str
    t: np.ndarray
    q: np.ndarray
    source: str = ""
    #: Time within the clip where recorded motion starts (after any
    #: synthetic lead-in bridge added at playback).
    recorded_from: float = 0.0
    #: Session time the clip started at (s), for ordering.
    t_start: float = 0.0

    @property
    def duration(self) -> float:
        return float(self.t[-1])

    def at(self, time: float) -> np.ndarray:
        """Joint positions at ``time`` (clamped to the clip)."""
        return np.array(
            [np.interp(time, self.t, self.q[:, j]) for j in range(6)]
        )


def bridge(
    q0: np.ndarray,
    q1: np.ndarray,
    *,
    speed: float = 1.0,
    min_duration: float = 0.2,
    rate: float = 50.0,
) -> MotionClip:
    """Minimum-jerk joint-space move from ``q0`` to ``q1``.

    Duration scales with the largest joint change (``speed`` rad/s average).
    """
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    duration = max(min_duration, float(np.abs(q1 - q0).max()) / speed)
    t = np.linspace(0.0, duration, max(2, int(duration * rate) + 1))
    s = t / duration
    blend = 10 * s**3 - 15 * s**4 + 6 * s**5
    q = q0 + blend[:, None] * (q1 - q0)
    return MotionClip("", "bridge", "", t, q)


def named_arm_pose(arm: str, name: str) -> np.ndarray:
    """SRDF named pose of ``arm`` in ``ARM_JOINTS`` order."""
    pose = load_named_poses()[arm][name]
    return np.array([pose[f"{arm}_{j}"] for j in ARM_JOINTS])


class MotionLibrary:
    """Clips indexed by ``(arm, kind, object_id)``."""

    def __init__(self, clips: list[MotionClip] | None = None):
        self._clips: dict[tuple[str, str, str], list[MotionClip]] = (
            defaultdict(list)
        )
        for clip in clips or []:
            self.add(clip)

    def add(self, clip: MotionClip) -> None:
        self._clips[(clip.arm, clip.kind, clip.object_id)].append(clip)

    def get(
        self,
        arm: str,
        kind: str,
        object_id: str = "",
        near: np.ndarray | None = None,
    ) -> MotionClip:
        """A recorded clip for the key.

        Args:
            near: Current joint positions; picks the clip starting closest
                to them (shortest bridge). Without it, the first clip.
        """
        clips = self._clips.get((arm, kind, object_id))
        if not clips:
            raise KeyError(f"No {kind} clip for {arm} {object_id}".rstrip())
        if near is None:
            return clips[0]
        return min(clips, key=lambda c: float(np.abs(c.q[0] - near).max()))

    def has(self, arm: str, kind: str, object_id: str = "") -> bool:
        return bool(self._clips.get((arm, kind, object_id)))

    def objects(self, arm: str) -> set[str]:
        """Objects with both a fetch and a return clip for ``arm``."""
        return {
            o
            for (a, k, o) in self._clips
            if a == arm and k == "fetch" and self.has(arm, "return", o)
        }

    def __len__(self) -> int:
        return sum(len(v) for v in self._clips.values())

    def clips(self) -> list[MotionClip]:
        return [c for v in self._clips.values() for c in v]

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        arrays = {}
        for i, c in enumerate(self.clips()):
            meta = np.array(
                [c.arm, c.kind, c.object_id, c.source, repr(c.t_start)]
            )
            arrays[f"{i}_meta"] = meta
            arrays[f"{i}_t"] = c.t
            arrays[f"{i}_q"] = c.q
        np.savez_compressed(path, **arrays)

    @classmethod
    def load(cls, path: Path) -> "MotionLibrary":
        data = np.load(path)
        n = len({k.split("_")[0] for k in data.files})
        clips = []
        for i in range(n):
            arm, kind, object_id, source, t_start = (
                str(x) for x in data[f"{i}_meta"]
            )
            clips.append(
                MotionClip(
                    arm,
                    kind,
                    object_id,
                    data[f"{i}_t"],
                    data[f"{i}_q"],
                    source,
                    t_start=float(t_start),
                )
            )
        return cls(clips)


class SequenceLibrary(MotionLibrary):
    """Hands out one session's clips in recorded order, per arm and kind.

    For parity checks: if the task logic matches the rig's, it asks for the
    same objects in the same order, and the robots keep their recorded
    timing. Asking for a different object raises ``ClipOrderError``.
    """

    def __init__(self, clips: list[MotionClip]):
        super().__init__(clips)
        self._queues: dict[tuple[str, str], list[MotionClip]] = defaultdict(
            list
        )
        for clip in sorted(clips, key=lambda c: c.t_start):
            self._queues[(clip.arm, clip.kind)].append(clip)

    def get(self, arm, kind, object_id="", near=None):
        queue = self._queues[(arm, kind)]
        if not queue:
            raise ClipOrderError(f"No more recorded {kind} clips for {arm}")
        clip = queue.pop(0)
        if clip.object_id != object_id:
            raise ClipOrderError(
                f"{arm} {kind}: sim wants {object_id or '-'}, the session "
                f"did {clip.object_id or '-'} next ({clip.source})"
            )
        return clip


class ClipOrderError(RuntimeError):
    """The sim requested clips in a different order than the session."""


def _still_runs(speed: np.ndarray, rate: float) -> list[tuple[int, int]]:
    """``[start, end)`` frame ranges where the arm stays still long enough."""
    still = speed < STILL_SPEED
    runs, start = [], None
    for k, s in enumerate(np.r_[still, False]):
        if s and start is None:
            start = k
        elif not s and start is not None:
            if (k - start) / rate >= STILL_MIN:
                runs.append((start, k))
            start = None
    return runs


def clips_from_timeline(
    timeline: Timeline, model: mujoco.MjModel, source: str = ""
) -> list[MotionClip]:
    """Cut fetch/present/unpresent/return clips from one session."""
    rate = 1.0 / float(np.median(np.diff(timeline.t)))
    mocap_names = {
        model.body_mocapid[b]: model.body(b).name
        for b in range(model.nbody)
        if model.body_mocapid[b] >= 0
    }

    clips = []
    for arm, group in GROUPS.items():
        cols = [timeline.joint_names.index(f"{arm}_{j}") for j in ARM_JOINTS]
        q = timeline.robot_qpos[:, cols]
        at_fetched = (
            np.abs(q - named_arm_pose(arm, "fetched")).max(axis=1)
            < AT_POSE_TOL
        )
        speed = np.r_[0.0, np.abs(np.diff(q, axis=0)).max(axis=1) * rate]
        still = _still_runs(speed, rate)
        held = timeline.held[arm]

        def clip(kind, object_id, a, b):
            b = min(b, len(q) - 1)
            if b - a < 2:
                return None
            return MotionClip(
                arm,
                kind,
                object_id,
                timeline.t[a : b + 1] - timeline.t[a],
                q[a : b + 1].copy(),
                f"{source}@{timeline.t[a]:.2f}s",
                t_start=float(timeline.t[a]),
            )

        # Grasps and releases of this arm. Return and fetch split exactly at
        # the release: a return ends as the object is set down, and the
        # arm's next fetch starts there (or at its first motion of the
        # session). Only a final return keeps its retreat to rest.
        change = np.flatnonzero(np.diff(held)) + 1
        grasps = [k for k in change if held[k] >= 0]
        releases = [k for k in change if held[k] < 0]
        first_motion = still[0][1] if still and still[0][0] == 0 else 0
        for k in grasps:
            object_id = mocap_names[int(held[k])]
            start = max([r for r in releases if r <= k], default=first_motion)
            arrive = np.flatnonzero(at_fetched[k:])
            if len(arrive):
                c = clip("fetch", object_id, start, k + arrive[0])
                clips.append(c) if c else None
        for k in releases:
            object_id = mocap_names[int(held[k - 1])]
            leave = np.flatnonzero(at_fetched[:k])
            if not len(leave):
                continue
            end = k
            if not any(g > k for g in grasps):
                end = min([s for s, e in still if s >= k], default=len(q) - 1)
            c = clip("return", object_id, leave[-1], end)
            clips.append(c) if c else None

        for event in timeline.events:
            if event.group_name != group:
                continue
            k = timeline.frame_at(event.t)
            if event.type == TaskEventType.TRIAL_PRESENTED:
                leave = np.flatnonzero(at_fetched[:k])
                if len(leave):
                    c = clip("present", "", leave[-1], k)
                    clips.append(c) if c else None
            elif event.type == TaskEventType.TRIAL_DEPRESENTING:
                arrive = np.flatnonzero(at_fetched[k:])
                if len(arrive):
                    c = clip("unpresent", "", k, k + arrive[0])
                    clips.append(c) if c else None
    return clips


def build_library(
    bags: list[str | Path], model: mujoco.MjModel, rate: float = 50.0
) -> MotionLibrary:
    """Cut clips from several sessions into one library."""
    library = MotionLibrary()
    for bag in bags:
        timeline = build_timeline(bag, model, rate=rate)
        for c in clips_from_timeline(timeline, model, source=str(bag)):
            library.add(c)
    return library


#: Default location of the motion library (git-ignored cache).
DEFAULT_LIBRARY = paths.CACHE_DIR / "motion" / "library.npz"


def load_library(path: Path = DEFAULT_LIBRARY) -> MotionLibrary:
    """Load the saved motion library (build it with ``vtt-clips``)."""
    if not path.exists():
        raise FileNotFoundError(
            f"No motion library at {path}; build one with "
            "`uv run vtt-clips BAG [BAG ...]`"
        )
    return MotionLibrary.load(path)


def main() -> None:
    import argparse

    from virtual_tabletop.assets.build import load_model

    parser = argparse.ArgumentParser(
        description="Cut robot motion clips from recorded sessions."
    )
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("-o", "--output", type=Path, default=DEFAULT_LIBRARY)
    args = parser.parse_args()
    library = build_library(args.bags, load_model())
    library.save(args.output)
    print(f"{len(library)} clips -> {args.output}")
    for arm in GROUPS:
        print(f"  {arm}: {len(library.objects(arm))} objects")
