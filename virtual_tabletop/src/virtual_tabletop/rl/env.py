"""Minimal presentation task for RL: an object, the occluder, the monkey.

A simplified version of foraging for training, on the minimal RL model
(:func:`virtual_tabletop.assets.build.build_rl`): no robot arms, no object
grid, and optionally no rig frame. The monkey has both arms. Each trial:

1. **intertrial**: the trial's object is already held at its presentation
   pose (as if by a robot, but no robot), behind the opaque smartglass;
2. **stimulus**: the glass clears for ``stimulus_duration``;
3. **delay**: ``delay_duration`` (the glass turns opaque if the trial spec
   says ``occlude``);
4. **response**: the subject arm(s) the spec names (``left``, ``right`` or
   ``both``) are released (observation flags) for ``response_timeout``. The
   trial **ends at the timeout** with no reward. Optionally
   (``reward_on_touch``), a released hand touching the object instead ends
   the trial at once with ``touch_reward``. Touches are always reported in
   ``info``.

Then the next trial starts immediately with its own object, with no
returning or resetting. Objects, poses, released arms and timings come from
the same task YAML as the rig (its trial generator, alternating the object
lists of the two robot arms). One episode is ``num_trials`` trials.

The action space is the same provisional one as the full foraging env:
kinematic joint-position targets for the monkey joints (both arms by
default). It will change once the joint-control design is settled
(``gitignore/claude_plans/rl_joint_control.md``).
"""

from dataclasses import dataclass
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np

from virtual_tabletop.assets import subject
from virtual_tabletop.assets.build import (
    RL_PARKED,
    RL_STIMULI,
    SUBJECT_PREFIX,
    load_rl_model,
    load_sim_config,
)
from virtual_tabletop.assets.rig_config import load_rig_scene
from virtual_tabletop.policies import ReachPolicy  # noqa: F401 (re-export)
from virtual_tabletop.task.config import load_foraging_config
from virtual_tabletop.task.generators import (
    TrialFeedback,
    TrialSpec,
    make_generator,
)

PHASES = ("intertrial", "stimulus", "delay", "response")


@dataclass
class TrialRecord:
    """One trial.

    Attributes:
        spec: The trial spec (object, pose, released arm(s), occlusion).
        start: Sim time the trial began (s).
        touched_at: First time a released hand touched the object during
            the response window, or ``None``.
        touched_by: Which arm touched first (``"left"``/``"right"``).
        rewarded: Whether the touch was rewarded (``reward_on_touch``).
    """

    spec: TrialSpec
    start: float
    touched_at: float | None = None
    touched_by: str | None = None
    rewarded: bool = False


class PresentationEnv(gym.Env):
    """Presented objects behind a switchable occluder; the monkey acts.

    Args:
        task: Task config name or path (timings, objects, poses, arms).
        num_trials: Trials per episode.
        intertrial: Duration of the opaque phase that starts each trial (s).
        reward_on_touch: End the trial with ``touch_reward`` when a released
            hand touches the object in the response window.
        touch_reward: Reward for such a touch.
        include_rig: Show the rig frame CAD (cosmetic).
        joints: Monkey joints the action controls, e.g.
            ``"right/elbow_flexion"`` (default: all of both arms).
        dt: Control tick (s).
        render_mode: ``"rgb_array"`` or ``None``.
        camera: Render camera (``subject`` or ``overview``).
        seed: Seed for the random trial generator.
    """

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(
        self,
        task: str | Path = "foraging_ordered",
        *,
        num_trials: int = 20,
        intertrial: float = 0.5,
        reward_on_touch: bool = False,
        touch_reward: float = 1.0,
        include_rig: bool = False,
        joints: list[str] | None = None,
        dt: float = 0.02,
        render_mode: str | None = None,
        camera: str = "subject",
        seed: int | None = None,
    ):
        self.model = load_rl_model(include_rig)
        self.data = mujoco.MjData(self.model)
        scene = load_rig_scene()
        self._objects = {o.object_id: o for o in scene.grid_objects}
        self.config = load_foraging_config(task, available=set(self._objects))
        self.num_trials = num_trials
        self.intertrial = intertrial
        self.reward_on_touch = reward_on_touch
        self.touch_reward = touch_reward
        self.dt = dt
        self.render_mode = render_mode
        self.camera = camera
        self._seed = seed
        self._renderer = None

        m = self.model
        self.sides = subject.sides_in(m)
        self.joint_names = (
            [SUBJECT_PREFIX + j for j in joints]
            if joints
            else [n for s in self.sides for n in subject.joint_names(m, s)]
        )
        ids = [m.joint(n).id for n in self.joint_names]
        self._qadr = np.array([m.jnt_qposadr[j] for j in ids])
        self.action_space = gym.spaces.Box(
            m.jnt_range[ids, 0].astype(np.float32),
            m.jnt_range[ids, 1].astype(np.float32),
        )
        n_obs = len(ids) + 3 * len(self.sides) + 3 + len(self.sides) + 2
        self.observation_space = gym.spaces.Box(
            -np.inf, np.inf, (n_obs,), np.float32
        )
        self._hands = [m.site(subject.hand_site(m, s)).id for s in self.sides]
        self._side_bodies = {s: subject.body_ids(m, s) for s in self.sides}
        self._stim = {s: m.body(f"stim/{s}").id for s in RL_STIMULI}
        self._stim_mocap = {
            s: int(m.body_mocapid[b]) for s, b in self._stim.items()
        }
        self._stim_geoms = {
            s: [g for g in range(m.ngeom) if m.geom_bodyid[g] == b]
            for s, b in self._stim.items()
        }
        glass = load_sim_config()["smartglass"]
        self._glass_rgba = {
            True: glass["revealed_rgba"],
            False: glass["occluded_rgba"],
        }
        self._glass = m.geom("smartglass").id

    # --- gym API ----------------------------------------------------------

    def reset(self, *, seed: int | None = None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetDataKeyframe(
            self.model, self.data, self.model.key("home").id
        )
        kwargs = dict(self.config.generator_kwargs, num_trials=self.num_trials)
        self.generator = make_generator(
            self.config.generator_class,
            kwargs,
            seed=self._seed if seed is None else seed,
            prefer_feedback_group=False,  # no parallel robot arms: alternate
        )
        self._steps = 0
        self._finished = False
        self.time = 0.0
        self.trials: list[TrialRecord] = []
        self._start_trial()
        mujoco.mj_forward(self.model, self.data)
        return self._obs(), self._info()

    def step(self, action):
        if self._finished:
            raise RuntimeError("The episode has ended; call reset() first")
        self._steps += 1
        self.time = self._steps * self.dt
        self.data.qpos[self._qadr] = np.clip(
            action, self.action_space.low, self.action_space.high
        )
        self._update_phase()
        mujoco.mj_forward(self.model, self.data)

        reward, trial_ended = 0.0, False
        touched_by = self._touching_sides()
        if (
            self.phase == "response"
            and touched_by
            and self.trial.touched_at is None
        ):
            self.trial.touched_at = self.time
            self.trial.touched_by = touched_by[0]
            if self.reward_on_touch:
                self.trial.rewarded = True
                reward, trial_ended = self.touch_reward, True
        if (
            self.phase == "response"
            and self.phase_time >= self._phase_durations["response"] - 1e-9
        ):
            trial_ended = True

        terminated = False
        if trial_ended:
            ended = self.trial
            feedback = TrialFeedback(
                reaction_time=None
                if ended.touched_at is None
                else ended.touched_at - (self.time - self.phase_time),
                timeout=not ended.rewarded,
            )
            self.generator.send(ended.spec, feedback)
            if len(self.trials) >= self.num_trials:
                terminated = self._finished = True
            else:
                self._start_trial()
                mujoco.mj_forward(self.model, self.data)
        info = self._info(touched_by=touched_by, trial_ended=trial_ended)
        return self._obs(), reward, terminated, False, info

    def render(self):
        if self.render_mode != "rgb_array":
            return None
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, 480, 640)
        self._renderer.update_scene(self.data, camera=self.camera)
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    # --- trial logic ------------------------------------------------------

    def _start_trial(self) -> None:
        spec = next(self.generator)
        self.trial = TrialRecord(spec, self.time)
        self.trials.append(self.trial)
        self._trial_start = self.time
        cfg = self.config
        self._phase_durations = {
            "intertrial": self.intertrial,
            "stimulus": cfg.stimulus_duration,
            "delay": cfg.delay_duration,
            "response": cfg.response_timeout,
        }
        obj = self._objects[spec.object_id]
        self.stimulus = obj.mesh_path.resolve().stem
        for stem, mocap in self._stim_mocap.items():
            if stem == self.stimulus:
                self.data.mocap_pos[mocap] = spec.object_pose.position
                self.data.mocap_quat[mocap] = spec.object_pose.quat
            else:
                self.data.mocap_pos[mocap] = RL_PARKED
        for g in self._stim_geoms[self.stimulus]:
            self.model.geom_rgba[g] = (*obj.rgba[:3], 1.0)
        self._update_phase()

    def _update_phase(self) -> None:
        elapsed = self.time - self._trial_start
        for phase in PHASES:
            duration = self._phase_durations[phase]
            if elapsed < duration - 1e-9 or phase == "response":
                self.phase, self.phase_time = phase, elapsed
                break
            elapsed -= duration
        revealed = self.phase == "stimulus" or (
            self.phase in ("delay", "response") and not self.trial.spec.occlude
        )
        self.glass_revealed = revealed
        self.model.geom_rgba[self._glass] = self._glass_rgba[revealed]

    def _touching_sides(self) -> list[str]:
        """Released arms in contact with the presented object."""
        released = self.released_sides
        if not released:
            return []
        target = self._stim[self.stimulus]
        m, d = self.model, self.data
        sides = []
        for i in range(d.ncon):
            bodies = {
                m.geom_bodyid[d.contact[i].geom1],
                m.geom_bodyid[d.contact[i].geom2],
            }
            if target not in bodies:
                continue
            for side in released:
                if bodies & self._side_bodies[side] and side not in sides:
                    sides.append(side)
        return sides

    # --- observations -----------------------------------------------------

    @property
    def released_sides(self) -> list[str]:
        """Subject arms free to respond (response phase, per the spec)."""
        if self.phase != "response":
            return []
        arm = self.trial.spec.arm
        wanted = self.sides if arm == "both" else [arm]
        return [s for s in self.sides if s in wanted]

    @property
    def target_position(self) -> np.ndarray:
        return self.data.xpos[self._stim[self.stimulus]].copy()

    def _obs(self) -> np.ndarray:
        remaining = self._phase_durations[self.phase] - self.phase_time
        released = self.released_sides
        return np.concatenate(
            [
                self.data.qpos[self._qadr],
                self.data.site_xpos[self._hands].ravel(),
                self.target_position,
                [float(s in released) for s in self.sides],
                [float(self.glass_revealed), remaining],
            ]
        ).astype(np.float32)

    def _info(self, touched_by=(), trial_ended: bool = False) -> dict:
        return {
            "time": self.time,
            "trial": len(self.trials) - 1,
            "object_id": self.trial.spec.object_id,
            "arm": self.trial.spec.arm,
            "phase": self.phase,
            "touched_by": list(touched_by),
            "trial_ended": trial_ended,
        }

    def status_text(self) -> str:
        """One line for viewer overlays."""
        spec, trial = self.trial.spec, self.trial
        extra = ""
        if trial.touched_at is not None:
            extra = f"  TOUCH({trial.touched_by})"
            if trial.rewarded:
                extra += " REWARD"
        return (
            f"t={self.time:6.2f}s  trial {len(self.trials) - 1}/"
            f"{self.num_trials}  {spec.object_id}  arm:{spec.arm}  "
            f"{self.phase}  glass:{'clear' if self.glass_revealed else 'opaque'}"
            f"{extra}"
        )
