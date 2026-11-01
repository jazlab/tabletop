"""The RL presentation tasks as a Gymnasium environment.

The scene (:mod:`.scene`): the RL model (:mod:`virtual_tabletop.assets.rl_model`)
with both monkey arms and one presented object per trial; no robot arms,
object grid or smartglass; the rig frame optional (cosmetic). Trials and
their phases (:mod:`.trials`) come from the rig's task YAML, one after
another with no returning or resetting; one episode is ``num_trials``
trials.

Three tasks (``reward``), differing only in what is rewarded in the response
window:

- ``"none"``: nothing; every trial ends at the timeout;
- ``"touch"``: a released hand touching the object ends the trial at once
  with ``reward_value``;
- ``"button"``: a released hand pressing the object's button
  (:mod:`virtual_tabletop.buttons`: the cup's inner floor, the drum's top)
  does. Touching the object elsewhere is not rewarded.

Touches and presses are always recorded (``info``, :attr:`trials`).

**Control.** ``"position"`` (default) simulates physics: the action is a
target angle per controlled joint for the monkey's position servos
(``rl_actuation`` in ``config/subject.yaml``; provisional) and
``mj_step`` runs ``dt / timestep`` substeps, so contact forces act between
the bones and the object (held fixed, as if by the robot) and between bones.
``"kinematic"`` sets the angles directly with no dynamics (about 7× faster;
bodies can pass through each other; velocities are zero).

**Trial start.** Every trial starts with both arms at rest (all joints 0,
at rest velocity), as on the rig, where the robot presents an object only
once the arm is back on its rest. Without this, a new object could appear
around a hand left where the previous trial ended (e.g. a cup enclosing
it), trapping it there.

**Arm locks.** As on the rig (the Teensy's arm locks), a subject arm is
held at rest except while released in the response window (``lock_arms``,
default on): actions for a locked arm's joints are replaced by its rest
posture (all joints 0: hanging, fingers straight). Without locks a policy
could hover at the object before the response window opens.

**Observation.** A flat float32 vector; :attr:`observation_layout` names
its parts (see :meth:`PresentationEnv.observation_parts`).
"""

from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np

from virtual_tabletop.assets import subject
from virtual_tabletop.assets.rl_model import load_rl_model
from virtual_tabletop.rl.scene import (
    PresentationScene,
    grid_objects,
    object_geometry,
)
from virtual_tabletop.rl.trials import TrialRecord, TrialSchedule
from virtual_tabletop.task.config import load_foraging_config

#: The RL tasks: what (if anything) is rewarded in the response window.
REWARDS = ("none", "touch", "button")
#: How actions move the arms: servo targets under physics, or set angles.
CONTROLS = ("position", "kinematic")


class PresentationEnv(gym.Env):
    """Presented objects; the monkey's arms act.

    Args:
        task: Task config name or path (timings, objects, poses, arms).
        num_trials: Trials per episode.
        intertrial: Duration of the phase that starts each trial (s).
        response_timeout: Override the task config's response window (s);
            the rig's 0.5 s is short for learning or for the scripted press.
        arms: Override which subject arm(s) trials release, e.g.
            ``["right"]``.
        trial_order: ``"config"`` (the task YAML's generator) or
            ``"random"`` (objects, poses, arms sampled each trial; see
            :mod:`.trials`).
        reward: The task: ``"none"``, ``"touch"`` or ``"button"``.
        reward_value: Reward for a rewarded response.
        include_rig: Show the rig frame CAD (cosmetic).
        joints: Monkey joints the action controls, e.g.
            ``"right/elbow_flexion"`` (default: all of both arms).
        dt: Control tick (s).
        control: ``"position"`` (physics, servo targets) or
            ``"kinematic"`` (set angles, no dynamics).
        lock_arms: Hold arms at rest unless released (see above).
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
        response_timeout: float | None = None,
        arms: list[str] | None = None,
        trial_order: str = "config",
        reward: str = "none",
        reward_value: float = 1.0,
        include_rig: bool = False,
        joints: list[str] | None = None,
        dt: float = 0.02,
        control: str = "position",
        lock_arms: bool = True,
        render_mode: str | None = None,
        camera: str = "subject",
        seed: int | None = None,
    ):
        if reward not in REWARDS:
            raise ValueError(f"reward must be one of {REWARDS}: {reward!r}")
        if control not in CONTROLS:
            raise ValueError(f"control must be one of {CONTROLS}: {control!r}")
        self.reward, self.reward_value = reward, reward_value
        self.control, self.dt = control, dt
        self.task, self.lock_arms = task, lock_arms
        self.render_mode, self.camera = render_mode, camera
        self._seed = seed
        self._renderer = None
        self._objects = grid_objects()
        self.config = load_foraging_config(task, available=set(self._objects))
        self.schedule = TrialSchedule(
            self.config,
            num_trials,
            intertrial,
            response_timeout,
            order=trial_order,
            arms=arms,
        )
        self.model = load_rl_model(include_rig)
        self.data = mujoco.MjData(self.model)
        self.scene = PresentationScene(self.model, self.data)
        self.sides = self.scene.sides

        m = self.model
        self.joint_names = (
            [subject.SUBJECT_PREFIX + j for j in joints]
            if joints
            else [n for s in self.sides for n in subject.joint_names(m, s)]
        )
        ids = [m.joint(n).id for n in self.joint_names]
        self._qadr = np.array([m.jnt_qposadr[j] for j in ids])
        self._dofadr = np.array([m.jnt_dofadr[j] for j in ids])
        servo = {
            m.actuator_trnid[a, 0]: a
            for a in range(m.nu)
            if m.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT
        }
        self._actuators = np.array([servo[j] for j in ids])
        # Each arm's action columns, and the rest posture locks hold.
        self._side_cols = {
            s: [
                i
                for i, n in enumerate(self.joint_names)
                if n.startswith(subject.prefix(s))
            ]
            for s in self.sides
        }
        self._rest = np.clip(0.0, m.jnt_range[ids, 0], m.jnt_range[ids, 1])
        # Every monkey joint (controlled or not), for the trial-start rest.
        monkey = [
            j
            for s in self.sides
            for j in (m.joint(n).id for n in subject.joint_names(m, s))
        ]
        self._monkey_qadr = m.jnt_qposadr[monkey]
        self._monkey_dofadr = m.jnt_dofadr[monkey]
        self._monkey_rest = np.clip(
            0.0, m.jnt_range[monkey, 0], m.jnt_range[monkey, 1]
        )
        self._monkey_actuators = np.array(
            [servo[j] for j in monkey if j in servo]
        )
        self._substeps = max(1, round(dt / m.opt.timestep))
        self.action_space = gym.spaces.Box(
            m.jnt_range[ids, 0].astype(np.float32),
            m.jnt_range[ids, 1].astype(np.float32),
        )
        self.observation_layout = self._layout()
        size = max(s.stop for s in self.observation_layout.values())
        self.observation_space = gym.spaces.Box(
            -np.inf, np.inf, (size,), np.float32
        )

    # --- public state -------------------------------------------------------

    @property
    def num_trials(self) -> int:
        return self.schedule.num_trials

    @property
    def trials(self) -> list[TrialRecord]:
        return self.schedule.trials

    @property
    def trial(self) -> TrialRecord:
        return self.schedule.trial

    @property
    def phase(self) -> str:
        return self.schedule.phase

    @property
    def stimulus_visible(self) -> bool:
        return self.schedule.stimulus_visible

    @property
    def stimulus(self) -> str:
        """Geometry of the presented object (``small_object``, ...)."""
        return self.scene.current.geometry

    @property
    def released_sides(self) -> list[str]:
        """Subject arms free to respond now (response phase, per the spec)."""
        return self.schedule.released(self.sides)

    @property
    def joint_positions(self) -> np.ndarray:
        """Current angles of the controlled joints (action order)."""
        return self.data.qpos[self._qadr].copy()

    # --- gym API ------------------------------------------------------------

    def reset(self, *, seed: int | None = None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetDataKeyframe(
            self.model, self.data, self.model.key("home").id
        )
        self.schedule.reset(self._seed if seed is None else seed)
        self.episode_steps = 0
        self.time = 0.0
        self.last_target = self.joint_positions
        self._finished = False
        self._start_trial()
        mujoco.mj_forward(self.model, self.data)
        return self._obs(), self._info()

    def step(self, action):
        if self._finished:
            raise RuntimeError("The episode has ended; call reset() first")
        self.episode_steps += 1
        self.time = self.episode_steps * self.dt
        target = np.clip(action, self.action_space.low, self.action_space.high)
        self.schedule.update(self.time)
        if self.lock_arms:
            released = self.released_sides
            for side, cols in self._side_cols.items():
                if side not in released:
                    target[cols] = self._rest[cols]
        self.last_target = target
        if self.control == "kinematic":
            self.data.qpos[self._qadr] = target
            mujoco.mj_forward(self.model, self.data)
        else:
            self.data.ctrl[self._actuators] = target
            mujoco.mj_step(self.model, self.data, self._substeps)

        reward, trial_ended = 0.0, False
        trial, released = self.trial, self.released_sides
        touched_by = self.scene.touching_sides(released)
        pressed_by = self.scene.pressing_sides(released)
        if touched_by and trial.touched_at is None:
            trial.touched_at, trial.touched_by = self.time, touched_by[0]
        if pressed_by and trial.pressed_at is None:
            trial.pressed_at, trial.pressed_by = self.time, pressed_by[0]
        if {"touch": touched_by, "button": pressed_by}.get(self.reward):
            trial.rewarded = True
            reward, trial_ended = self.reward_value, True
        trial_ended = trial_ended or self.schedule.response_over

        terminated = False
        if trial_ended:
            responded_at = (
                trial.pressed_at
                if self.reward == "button"
                else trial.touched_at
            )
            self.schedule.finish(self.time, responded_at)
            if self.schedule.last:
                terminated = self._finished = True
            else:
                self._start_trial()
                mujoco.mj_forward(self.model, self.data)
        info = self._info(
            touched_by, pressed_by, trial_ended, trial_ended and trial.rewarded
        )
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

    # --- internals ----------------------------------------------------------

    def _start_trial(self) -> None:
        d = self.data
        d.qpos[self._monkey_qadr] = self._monkey_rest
        d.qvel[self._monkey_dofadr] = 0.0
        if len(self._monkey_actuators):
            d.ctrl[self._monkey_actuators] = 0.0
        self.last_target = self._rest.copy()
        spec = self.schedule.start(self.time).spec
        obj = self._objects[spec.object_id]
        self.scene.show(
            object_geometry(spec.object_id), spec.object_pose, obj.rgba
        )

    def observation_parts(self) -> list[tuple[str, int]]:
        """The observation's parts, in order: ``(name, size)``.

        - ``joint_positions``, ``joint_velocities``: controlled joints
          (action order; velocities are 0 with kinematic control);
        - ``fingertip_<side>``: each arm's middle fingertip (world, m);
        - ``object_position``: the presented object's base centre;
        - ``button_position``: the top centre of its button;
        - ``released_<side>``: 1 while that arm may respond;
        - ``stimulus_visible``: 1 if the rig's glass would be clear;
        - ``phase_time_left``: seconds left in the current phase.
        """
        n = len(self.joint_names)
        return [
            ("joint_positions", n),
            ("joint_velocities", n),
            *((f"fingertip_{s}", 3) for s in self.sides),
            ("object_position", 3),
            ("button_position", 3),
            *((f"released_{s}", 1) for s in self.sides),
            ("stimulus_visible", 1),
            ("phase_time_left", 1),
        ]

    def _layout(self) -> dict[str, slice]:
        layout, start = {}, 0
        for name, size in self.observation_parts():
            layout[name] = slice(start, start + size)
            start += size
        return layout

    def _obs(self) -> np.ndarray:
        released = self.released_sides
        parts = {
            "joint_positions": self.data.qpos[self._qadr],
            "joint_velocities": self.data.qvel[self._dofadr],
            "object_position": self.scene.object_position,
            "button_position": self.scene.button_position,
            "stimulus_visible": [float(self.stimulus_visible)],
            "phase_time_left": [self.schedule.time_left],
        }
        for side in self.sides:
            parts[f"fingertip_{side}"] = self.scene.hand_position(side)
            parts[f"released_{side}"] = [float(side in released)]
        return np.concatenate(
            [parts[name] for name in self.observation_layout]
        ).astype(np.float32)

    def _info(
        self,
        touched_by=(),
        pressed_by=(),
        trial_ended=False,
        trial_rewarded=False,
    ) -> dict:
        return {
            "time": self.time,
            "trial": self.schedule.index,
            "object_id": self.trial.spec.object_id,
            "arm": self.trial.spec.arm,
            "phase": self.phase,
            "touched_by": list(touched_by),
            "pressed_by": list(pressed_by),
            "trial_ended": trial_ended,
            "trial_rewarded": trial_rewarded,
        }

    def status_text(self) -> str:
        """One line for viewer overlays."""
        spec, trial = self.trial.spec, self.trial
        extra = ""
        if trial.touched_at is not None:
            extra += f"  TOUCH({trial.touched_by})"
        if trial.pressed_at is not None:
            extra += f"  PRESS({trial.pressed_by})"
        if trial.rewarded:
            extra += " REWARD"
        visible = "visible" if self.stimulus_visible else "hidden"
        return (
            f"[{self.reward}] t={self.time:6.2f}s  trial "
            f"{self.schedule.index}/{self.num_trials}  {spec.object_id}  "
            f"arm:{spec.arm}  {self.phase}  stimulus:{visible}{extra}"
        )
