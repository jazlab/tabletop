"""The rig's foraging task, re-implemented on simulated time in MuJoCo.

Mirrors ``tabletop_tasks`` (``tasks/base.py`` ``BaseObjectInteractionTask``
and ``tasks/foraging.py`` ``ForagingTask``, plus ``run.py``):

- task: ``TASK_STARTED``; occlude + lock both arms; dispatch trial specs to
  idle arms (a busy arm makes the dispatcher wait for any trial to finish
  and feed its result back); wait for the rest; occlude + lock; each arm
  returns what it holds; ``TASK_ENDED``.
- trial (per arm): return any held object; fetch the trial object; take
  the presentation lock (one arm presents at a time); present (and move to
  the trial pose); ``TRIAL_PRESENTED``; reveal glass, wait
  ``stimulus_duration``; optionally occlude; wait ``delay_duration``;
  release the subject's arm(s); wait up to ``response_timeout`` for a
  press; on a press reward for ``reward_duration`` (and reveal for
  ``reveal_duration`` if > 0); occlude + lock; ``TRIAL_DEPRESENTING``;
  unpresent; release the lock.

Robot motion comes from recorded clips (:mod:`virtual_tabletop.task.motion`),
not planning; objects follow the end-effectors via the replay grasp
tracker. Not modelled: planning failures/retries (``OBJECT_MISS``), sounds,
service latencies, Ctrl-C cancellation.

Each trial and the dispatcher are Python generators yielding wait
commands; :class:`ForagingSim` advances simulated time in ``dt`` ticks and
resumes whatever is ready, so code reads like the rig's async task.
"""

import uuid
from collections.abc import Callable, Generator
from dataclasses import dataclass, field

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from virtual_tabletop import ik
from virtual_tabletop.assets.robots import ARM_SIDES
from virtual_tabletop.assets.sim_config import load_rig_config
from virtual_tabletop.bags.topics import TaskEvent, TaskEventType
from virtual_tabletop.replay.objects import ObjectTracker
from virtual_tabletop.task import responses
from virtual_tabletop.task.config import ForagingConfig, reachable_object_ids
from virtual_tabletop.task.generators import (
    TrialFeedback,
    TrialSpec,
    make_generator,
)
from virtual_tabletop.task.motion import (
    ARM_JOINTS,
    MotionClip,
    MotionLibrary,
    bridge,
    named_arm_pose,
)

# --- wait commands yielded by task processes --------------------------------


@dataclass
class Sleep:
    duration: float


@dataclass
class Play:
    side: str
    clip: MotionClip


@dataclass
class Lock:
    pass


@dataclass
class Until:
    predicate: Callable[[], bool]


@dataclass
class _Process:
    gen: Generator
    wait: object = None
    wake_at: float = 0.0
    done: bool = False
    value: object = None
    send: object = None


@dataclass
class Presentation:
    """The trial currently presented to the subject.

    Attributes:
        spec: Its trial spec.
        side: Robot arm presenting it.
        window_open: True while waiting for a response.
        window_start: Sim time the subject's arm(s) were released.
    """

    spec: TrialSpec
    side: str
    window_open: bool = False
    window_start: float = 0.0


@dataclass
class TrialResult:
    """Outcome of one trial (``None`` feedback if it never ran)."""

    spec: TrialSpec
    feedback: TrialFeedback | None
    presented_at: float | None = None
    depresented_at: float | None = None


@dataclass
class StepInfo:
    """What happened during one :meth:`ForagingSim.step`.

    Attributes:
        events: Task events emitted this step.
        reward_started: A reward began this step.
        reward_active: The reward solenoid is open at the end of the step.
    """

    events: list[TaskEvent] = field(default_factory=list)
    reward_started: bool = False
    reward_active: bool = False


class ForagingSim:
    """The foraging task running on the generated rig model.

    Args:
        model: The rig model (``assets.rig_model.load_rig_model``).
        config: Task config (``task.config.load_foraging_config``).
        library: Robot motion clips (``task.motion``).
        response: Source of button presses.
        dt: Control tick (s).
        seed: Seeds the random generator, if any.
        initial_q: Starting joint positions per arm (``ARM_JOINTS`` order);
            default the SRDF ``idle`` pose. The mock arms keep their pose
            between rig sessions, so parity runs start from the bag's.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        config: ForagingConfig,
        library: MotionLibrary,
        response: responses.ResponseModel | None = None,
        *,
        dt: float = 0.02,
        seed: int | None = None,
        initial_q: dict[str, np.ndarray] | None = None,
    ):
        self.model = model
        self.data = mujoco.MjData(model)
        self.config = config
        self.library = library
        self.response = response or responses.FlicSimResponse(seed=seed)
        self.dt = dt
        self.seed = seed
        self.initial_q = initial_q or {}
        self._glass = load_rig_config()["smartglass"]
        self._glass_geom = model.geom("smartglass").id
        self._mocap_index = {
            model.body(b).name: int(model.body_mocapid[b])
            for b in range(model.nbody)
            if model.body_mocapid[b] >= 0
        }
        self._arm_qadr = {
            side: np.array(
                [
                    model.jnt_qposadr[model.joint(f"{side}_{j}").id]
                    for j in ARM_JOINTS
                ]
            )
            for side in ARM_SIDES
        }
        self._arm_dadr = {
            side: np.array(
                [
                    model.jnt_dofadr[model.joint(f"{side}_{j}").id]
                    for j in ARM_JOINTS
                ]
            )
            for side in ARM_SIDES
        }
        self._eef_site = {s: model.site(f"{s}_eef").id for s in ARM_SIDES}
        self.reset()

    # --- public API -----------------------------------------------------

    def reset(self) -> None:
        """Start a new task session from the home keyframe."""
        mujoco.mj_resetDataKeyframe(
            self.model, self.data, self.model.key("home").id
        )
        self.time = 0.0
        self.session_id = uuid.uuid4().hex[:12]
        self.q = {
            s: np.array(self.initial_q.get(s, named_arm_pose(s, "idle")))
            for s in ARM_SIDES
        }
        self._motion: dict[str, tuple[MotionClip, float] | None] = {
            s: None for s in ARM_SIDES
        }
        self._grasp_allowed: dict[str, set[int]] = {
            s: set() for s in ARM_SIDES
        }
        self.tracker = ObjectTracker(
            mount_pos=self.data.mocap_pos.copy(),
            mount_quat=self.data.mocap_quat.copy(),
        )
        self.glass_revealed = False
        self.arm_locked = {s: True for s in ARM_SIDES}
        self.reward_until = -1.0
        self.presentation: Presentation | None = None
        self.events: list[TaskEvent] = []
        self.results: dict[int, TrialResult] = {}
        self._lock_owner: object | None = None
        self._lock_queue: list[_Process] = []
        self._procs: list[_Process] = []
        self._step_info = StepInfo()
        #: ``(time, glass_revealed, reward_active)`` at every change.
        self.signals: list[tuple[float, bool, bool]] = [(0.0, False, False)]
        self.generator = make_generator(
            self.config.generator_class,
            self._generator_kwargs(),
            reachable=reachable_object_ids(),
            seed=self.seed,
        )
        self._task = self._spawn(self._run_task())
        self._apply_state()
        self._run_ready()

    @property
    def done(self) -> bool:
        return self._task.done

    def step(self) -> StepInfo:
        """Advance one tick."""
        self._step_info = StepInfo()
        self.time += self.dt
        self._advance_motions()
        self._apply_state()
        self._run_ready()
        self._step_info.reward_active = self.reward_active
        self._record_signals()
        return self._step_info

    def _record_signals(self) -> None:
        state = (self.glass_revealed, self.reward_active)
        if state != self.signals[-1][1:]:
            self.signals.append((self.time, *state))

    def run(self, max_time: float = 36000.0) -> list[TaskEvent]:
        """Run the whole task with no monkey input; return its events."""
        while not self.done and self.time < max_time:
            self.step()
        return self.events

    @property
    def reward_active(self) -> bool:
        return self.time < self.reward_until

    # --- scheduler ------------------------------------------------------

    def _spawn(self, gen: Generator) -> _Process:
        proc = _Process(gen)
        self._procs.append(proc)
        return proc

    def _ready(self, proc: _Process) -> bool:
        wait = proc.wait
        if wait is None:
            return True
        if isinstance(wait, Sleep):
            return self.time >= proc.wake_at - 1e-9
        if isinstance(wait, Play):
            return self._motion[wait.side] is None
        if isinstance(wait, Lock):
            return self._lock_owner is proc
        if isinstance(wait, Until):
            return wait.predicate()
        raise TypeError(wait)

    def _resume(self, proc: _Process) -> None:
        try:
            wait = proc.gen.send(proc.send)
        except StopIteration as stop:
            proc.done, proc.value = True, stop.value
            self._procs.remove(proc)
            return
        proc.send = None
        proc.wait = wait
        if isinstance(wait, Sleep):
            proc.wake_at = self.time + wait.duration
        elif isinstance(wait, Play):
            self._start_motion(wait.side, wait.clip)
        elif isinstance(wait, Lock):
            if self._lock_owner is None:
                self._lock_owner = proc
            else:
                self._lock_queue.append(proc)

    def _run_ready(self) -> None:
        progressed = True
        while progressed:
            progressed = False
            for proc in list(self._procs):
                if not proc.done and self._ready(proc):
                    self._resume(proc)
                    progressed = True

    def _unlock(self) -> None:
        self._lock_owner = (
            self._lock_queue.pop(0) if self._lock_queue else None
        )

    # --- world state ----------------------------------------------------

    def _start_motion(self, side: str, clip: MotionClip) -> None:
        start = self.q[side]
        if np.abs(clip.q[0] - start).max() > 1e-3:
            lead = bridge(start, clip.q[0])
            clip = MotionClip(
                side,
                clip.kind,
                clip.object_id,
                np.r_[lead.t, lead.t[-1] + clip.t[1:]],
                np.vstack([lead.q, clip.q[1:]]),
                clip.source,
                recorded_from=lead.duration,
                t_start=clip.t_start,
            )
        self._motion[side] = (clip, self.time)

    def _advance_motions(self) -> None:
        for side, motion in self._motion.items():
            if motion is None:
                continue
            clip, start = motion
            tau = self.time - start
            self.q[side] = clip.at(tau)
            if tau >= clip.duration:
                self._motion[side] = None

    def _eef_pose(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, self.data.site_xmat[self._eef_site[side]])
        return self.data.site_xpos[self._eef_site[side]].copy(), quat

    def _apply_state(self) -> None:
        data = self.data
        for side in ARM_SIDES:
            data.qpos[self._arm_qadr[side]] = self.q[side]
            data.qvel[self._arm_dadr[side]] = 0.0
        mujoco.mj_kinematics(self.model, data)
        eef = {side: self._eef_pose(side) for side in ARM_SIDES}
        # Grasps only count along recorded motion, never along a synthetic
        # bridge (which may sweep past a mount).
        allowed = {}
        for side in ARM_SIDES:
            motion = self._motion[side]
            in_bridge = (
                motion is not None
                and self.time - motion[1] < motion[0].recorded_from
            )
            allowed[side] = set() if in_bridge else self._grasp_allowed[side]
        data.mocap_pos[:], data.mocap_quat[:] = self.tracker.update(
            eef, allowed=allowed
        )
        state = "revealed" if self.glass_revealed else "occluded"
        self.model.geom_rgba[self._glass_geom] = self._glass[f"{state}_rgba"]
        mujoco.mj_forward(self.model, data)

    def _emit(self, event_type: TaskEventType, spec: TrialSpec | None = None):
        event = TaskEvent(
            t=self.time,
            type=event_type,
            session_id=self.session_id,
            trial_number=-1 if spec is None else spec.trial_number,
            group_name="" if spec is None else spec.group_name,
            object_id="" if spec is None else spec.object_id,
        )
        self.events.append(event)
        self._step_info.events.append(event)

    def _occlude_and_lock(self) -> None:
        self.glass_revealed = False
        self.arm_locked = {s: True for s in ARM_SIDES}

    # --- task logic -----------------------------------------------------

    def _generator_kwargs(self) -> dict:
        """Generator kwargs limited to objects the clip library covers."""
        kwargs = dict(self.config.generator_kwargs)
        grouped = {}
        self.unavailable: dict[str, list[str]] = {}
        for group, object_ids in kwargs["grouped_object_ids"].items():
            side = group.split("_")[0]
            have = self.library.objects(side)
            grouped[group] = [o for o in object_ids if o in have]
            self.unavailable[group] = [o for o in object_ids if o not in have]
        kwargs["grouped_object_ids"] = grouped
        return kwargs

    def _run_task(self):
        self._emit(TaskEventType.TASK_STARTED)
        self._occlude_and_lock()
        active: dict[str, _Process | None] = {
            g: None for g in self.generator.group_names
        }
        specs: dict[str, TrialSpec | None] = dict.fromkeys(active)
        for spec in self.generator:
            group = spec.group_name
            if active[group] is None:
                specs[group] = spec
                self.results[spec.trial_number] = TrialResult(spec, None)
                active[group] = self._spawn(self._run_trial(spec))
                continue
            yield Until(
                lambda: any(p is not None and p.done for p in active.values())
            )
            for g, proc in active.items():
                if proc is not None and proc.done:
                    self.generator.send(specs[g], proc.value)
                    active[g], specs[g] = None, None
                    break
        yield Until(lambda: all(p is None or p.done for p in active.values()))
        self._occlude_and_lock()
        for side in ARM_SIDES:
            yield from self._return_held(side)
        self._emit(TaskEventType.TASK_ENDED)

    def _return_held(self, side: str):
        held = self.tracker.holding(side)
        if held is None:
            return
        name = next(n for n, i in self._mocap_index.items() if i == held)
        self._grasp_allowed[side] = set()
        yield Play(
            side, self.library.get(side, "return", name, near=self.q[side])
        )

    def _run_trial(self, spec: TrialSpec):
        side = spec.group_name.split("_")[0]
        result = self.results[spec.trial_number]
        # reset_manipulation: put back what this arm still holds.
        yield from self._return_held(side)
        # fetch_object
        self._grasp_allowed[side] = {self._mocap_index[spec.object_id]}
        yield Play(
            side,
            self.library.get(side, "fetch", spec.object_id, near=self.q[side]),
        )
        self._grasp_allowed[side] = set()
        if self.tracker.holding(side) != self._mocap_index[spec.object_id]:
            raise RuntimeError(
                f"{side} fetch clip of {spec.object_id} did not grasp it"
            )
        # Presentation lock, present_object, plan_and_move(object_pose)
        yield Lock()
        yield Play(side, self.library.get(side, "present", near=self.q[side]))
        to_pose = self._move_to_object_pose(side, spec)
        if to_pose is not None:
            yield Play(side, to_pose)
        self._emit(TaskEventType.TRIAL_PRESENTED, spec)
        result.presented_at = self.time
        presentation = Presentation(spec, side)
        self.presentation = presentation

        feedback = yield from self._foraging_trial(presentation)
        result.feedback = feedback

        self._occlude_and_lock()
        self._emit(TaskEventType.TRIAL_DEPRESENTING, spec)
        result.depresented_at = self.time
        self.presentation = None
        yield Play(
            side, self.library.get(side, "unpresent", near=self.q[side])
        )
        self._unlock()
        return feedback

    def _foraging_trial(self, presentation: Presentation):
        cfg, spec = self.config, presentation.spec
        # stimulus
        self.glass_revealed = True
        yield Sleep(cfg.stimulus_duration)
        # delay
        if spec.occlude:
            self.glass_revealed = False
        yield Sleep(cfg.delay_duration)
        # response
        released = ARM_SIDES if spec.arm == "both" else (spec.arm,)
        for s in released:
            self.arm_locked[s] = False
        presentation.window_open = True
        presentation.window_start = self.time
        self.response.open(self, presentation)
        deadline = self.time + cfg.response_timeout
        pressed = {"value": False}

        def answered():
            if self.response.pressed(self, presentation):
                pressed["value"] = True
            return pressed["value"] or self.time >= deadline - 1e-9

        yield Until(answered)
        presentation.window_open = False
        if not pressed["value"]:
            return TrialFeedback(reaction_time=None, timeout=True)
        reaction_time = self.time - presentation.window_start
        self.reward_until = self.time + cfg.reward_duration
        self._step_info.reward_started = True
        yield Sleep(cfg.reward_duration)
        if cfg.reveal_duration > 0:
            self.glass_revealed = True
            yield Sleep(cfg.reveal_duration)
        return TrialFeedback(reaction_time=reaction_time, timeout=False)

    def _move_to_object_pose(
        self, side: str, spec: TrialSpec
    ) -> MotionClip | None:
        """IK bridge from the present pose to the trial's object pose."""
        held = self.tracker.holding(side)
        index = self._mocap_index[spec.object_id]
        if held != index:
            return None
        target_pos = np.asarray(spec.object_pose.position)
        w, x, y, z = spec.object_pose.quat
        target_rot = Rotation.from_quat([x, y, z, w])
        cw, cx, cy, cz = self.data.mocap_quat[index]
        current_rot = Rotation.from_quat([cx, cy, cz, cw])
        if np.linalg.norm(self.data.mocap_pos[index] - target_pos) < 1e-3 and (
            current_rot.inv() * target_rot
        ).magnitude() < np.radians(0.5):
            return None
        grasp = self.tracker._held[side]
        eef_rot = target_rot * grasp.rel_rot.inv()
        eef_pos = target_pos - eef_rot.apply(grasp.rel_pos)
        ex, ey, ez, ew = eef_rot.as_quat()
        scratch = mujoco.MjData(self.model)
        scratch.qpos[:] = self.data.qpos
        result = ik.qpos_from_site_pose(
            self.model,
            scratch,
            f"{side}_eef",
            target_pos=eef_pos,
            target_quat=np.array([ew, ex, ey, ez]),
            joint_names=[f"{side}_{j}" for j in ARM_JOINTS],
            max_steps=200,
            tol=1e-10,
        )
        if not result.success and result.err_norm > 1e-3:
            raise RuntimeError(
                f"No IK solution for trial {spec.trial_number} object pose"
            )
        target_q = scratch.qpos[self._arm_qadr[side]].copy()
        return bridge(self.q[side], target_q)


def run_task(
    model: mujoco.MjModel,
    config: ForagingConfig,
    library: MotionLibrary,
    response: responses.ResponseModel | None = None,
    **kwargs,
) -> ForagingSim:
    """Run a whole task without a monkey policy and return the sim."""
    sim = ForagingSim(model, config, library, response, **kwargs)
    sim.run()
    return sim
