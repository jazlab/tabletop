"""Turn a rig bag into a fixed-rate timeline of model states.

The timeline holds everything a viewer or analysis needs per frame, already
expressed in the generated model's terms (joint order, mocap order), so
playback is just indexing. Building it needs no viewer, which keeps it
testable and reusable (e.g. for task-event parity checks).

Sources:

- robot joints: ``/joint_states`` (linear interpolation onto the frame grid)
- task objects: inferred from end-effector motion
  (:class:`virtual_tabletop.replay.objects.ObjectTracker`)
- smartglass, reward, arm locks: ``/teensy/sensor`` (held from the latest
  message at or before each frame)
- trials: ``/tabletop/task_event``; button presses: ``/flic/...``

The monkey is not driven yet: OptiTrack topics are absent from sim bags, and
hand replay (OptiTrack -> world -> arm IK) is a later step.
"""

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from virtual_tabletop.assets.robots import ARM_SIDES
from virtual_tabletop.bags import topics
from virtual_tabletop.bags.reader import read_joint_states
from virtual_tabletop.replay.objects import ObjectTracker


@dataclass(frozen=True)
class Trial:
    """One trial's span, from its first to its last task event.

    Attributes:
        number: Trial number from the task events.
        start: First event time (timeline seconds).
        end: Last event time (timeline seconds).
        object_id: Trial object (from the events), possibly empty.
        group_name: Trial group, possibly empty.
    """

    number: int
    start: float
    end: float
    object_id: str
    group_name: str


@dataclass(frozen=True)
class Timeline:
    """Per-frame model state for a bag.

    Attributes:
        t: Frame times in seconds from ``t0``, shape ``(N,)``.
        t0: Epoch time of frame 0 (seconds).
        joint_names: Robot joints, in ``qpos_adr`` order.
        qpos_adr: Model ``qpos`` address of each robot joint.
        robot_qpos: Robot joint positions, ``(N, n_joints)``.
        mocap_pos: Task-object positions, ``(N, nmocap, 3)``.
        mocap_quat: Task-object orientations (wxyz), ``(N, nmocap, 4)``.
        held: Index of the object each arm holds (-1 if none),
            ``{arm: (N,)}``.
        glass_revealed: ``(N,)`` bool; ``None`` without Teensy data.
        reward_active: ``(N,)`` bool; ``None`` without Teensy data.
        trials: Trials in time order.
        events: Task events, with ``t`` converted to timeline seconds.
        presses: Button press times (timeline seconds).
    """

    t: np.ndarray
    t0: float
    joint_names: list[str]
    qpos_adr: np.ndarray
    robot_qpos: np.ndarray
    mocap_pos: np.ndarray
    mocap_quat: np.ndarray
    held: dict[str, np.ndarray]
    glass_revealed: np.ndarray | None
    reward_active: np.ndarray | None
    trials: list[Trial]
    events: list[topics.TaskEvent]
    presses: np.ndarray

    def frame_at(self, time: float) -> int:
        """Index of the last frame at or before ``time``."""
        return int(
            np.clip(
                np.searchsorted(self.t, time, "right") - 1, 0, len(self.t) - 1
            )
        )

    def trial_at(self, time: float) -> Trial | None:
        """The trial spanning ``time``, if any."""
        for trial in self.trials:
            if trial.start <= time <= trial.end:
                return trial
        return None


def _hold(t_src: np.ndarray, values: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Zero-order hold of ``values`` (sampled at ``t_src``) onto ``t``."""
    idx = np.searchsorted(t_src, t, "right") - 1
    # Frames before the first sample take the first value.
    return values[np.clip(idx, 0, len(values) - 1)]


def _trials(events: list[topics.TaskEvent]) -> list[Trial]:
    by_number: dict[int, list[topics.TaskEvent]] = {}
    for event in events:
        if event.trial_number >= 0:
            by_number.setdefault(event.trial_number, []).append(event)
    trials = []
    for number, trial_events in by_number.items():
        object_id = next(
            (e.object_id for e in trial_events if e.object_id), ""
        )
        group = next((e.group_name for e in trial_events if e.group_name), "")
        trials.append(
            Trial(
                number,
                trial_events[0].t,
                trial_events[-1].t,
                object_id,
                group,
            )
        )
    return sorted(trials, key=lambda tr: tr.start)


def build_timeline(
    bag: str | Path, model: mujoco.MjModel, rate: float = 30.0
) -> Timeline:
    """Build the replay timeline of ``bag`` for ``model``.

    Args:
        bag: Bag directory or MCAP file.
        model: The generated rig model (``virtual_tabletop.assets.build``).
        rate: Frame rate of the timeline (Hz).
    """
    joints = read_joint_states(bag)
    robot = [n for n in joints.names if n.startswith(ARM_SIDES)]
    if not robot:
        raise ValueError(f"No robot joints in /joint_states of {bag}")
    cols = [joints.names.index(n) for n in robot]
    t0 = float(joints.t[0])
    t = np.arange(0.0, joints.t[-1] - t0, 1.0 / rate)
    src_t = joints.t - t0
    robot_qpos = np.stack(
        [np.interp(t, src_t, joints.position[:, c]) for c in cols], axis=1
    )
    qpos_adr = np.array([model.jnt_qposadr[model.joint(n).id] for n in robot])

    data = mujoco.MjData(model)
    mocap_bodies = np.flatnonzero(model.body_mocapid >= 0)
    mocap_bodies = mocap_bodies[np.argsort(model.body_mocapid[mocap_bodies])]
    tracker = ObjectTracker(
        mount_pos=model.body_pos[mocap_bodies].copy(),
        mount_quat=model.body_quat[mocap_bodies].copy(),
    )
    eef_sites = {arm: model.site(f"{arm}_eef").id for arm in ARM_SIDES}
    mocap_pos = np.empty((len(t), model.nmocap, 3))
    mocap_quat = np.empty((len(t), model.nmocap, 4))
    held = {arm: np.full(len(t), -1) for arm in ARM_SIDES}
    quat = np.empty(4)
    for k in range(len(t)):
        data.qpos[qpos_adr] = robot_qpos[k]
        mujoco.mj_kinematics(model, data)
        eef = {}
        for arm, site in eef_sites.items():
            mujoco.mju_mat2Quat(quat, data.site_xmat[site])
            eef[arm] = (data.site_xpos[site].copy(), quat.copy())
        mocap_pos[k], mocap_quat[k] = tracker.update(eef)
        for arm in ARM_SIDES:
            index = tracker.holding(arm)
            held[arm][k] = -1 if index is None else index

    glass = reward = None
    teensy = topics.read_teensy(bag)
    if len(teensy.t):
        glass = _hold(teensy.t - t0, teensy.smartglass_revealed, t)
        reward = _hold(teensy.t - t0, teensy.reward_active, t)

    events = [
        topics.TaskEvent(
            e.t - t0,
            e.type,
            e.session_id,
            e.trial_number,
            e.group_name,
            e.object_id,
        )
        for e in topics.read_task_events(bag)
    ]
    presses = np.array([p.t - t0 for p in topics.read_button_presses(bag)])
    return Timeline(
        t=t,
        t0=t0,
        joint_names=robot,
        qpos_adr=qpos_adr,
        robot_qpos=robot_qpos,
        mocap_pos=mocap_pos,
        mocap_quat=mocap_quat,
        held=held,
        glass_revealed=glass,
        reward_active=reward,
        trials=_trials(events),
        events=events,
        presses=presses,
    )
