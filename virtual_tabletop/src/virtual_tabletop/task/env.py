"""Gymnasium environment for the MuJoCo foraging task, plus a scripted monkey.

:class:`ForagingEnv` wraps :class:`~virtual_tabletop.task.sim.ForagingSim`:
one episode is one task session (``num_trials`` trials from the config);
reward is 1 when a trial's reward starts. By default a response is a
released monkey arm touching the presented object
(:class:`~virtual_tabletop.task.responses.HandContactResponse`).

**The action space is provisional.** It is kinematic joint-position targets
for the monkey joints (default: all 14, both arms), bounded by the ranges in the arm
file, with no physics for the arm. Which joints to control, their ranges and
how they are actuated is being worked out with the user (see
``gitignore/claude_plans/rl_joint_control.md``); expect this to change.

:class:`~virtual_tabletop.policies.ReachPolicy` is a hand-written baseline
(reach with a released arm, by IK).
"""

from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np

from virtual_tabletop.assets import subject
from virtual_tabletop.assets.build import SUBJECT_PREFIX, load_model
from virtual_tabletop.policies import ReachPolicy
from virtual_tabletop.task.config import load_foraging_config
from virtual_tabletop.task.motion import MotionLibrary, load_library
from virtual_tabletop.task.responses import HandContactResponse, ResponseModel
from virtual_tabletop.task.sim import ForagingSim


class ForagingEnv(gym.Env):
    """The foraging task with the monkey arm as the agent.

    Args:
        task: Task config name or path (``tabletop_tasks/config``).
        model: Rig model (default: the generated model).
        library: Robot motion clips (default: the saved library).
        response: Response source (default: hand contact).
        joints: Monkey joints the action controls, e.g.
            ``"right/elbow_flexion"`` (default: all of both arms).
        dt: Control tick (s).
        max_time: Episode time limit (s); ``None`` for no limit.
        render_mode: ``"rgb_array"`` or ``None``.
        camera: Camera for rendering.
    """

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(
        self,
        task: str | Path = "foraging_ordered",
        *,
        model: mujoco.MjModel | None = None,
        library: MotionLibrary | None = None,
        response: ResponseModel | None = None,
        joints: list[str] | None = None,
        dt: float = 0.02,
        max_time: float | None = None,
        render_mode: str | None = None,
        camera: str = "subject",
    ):
        self.model = model or load_model()
        self.config = load_foraging_config(task)
        self.library = library or load_library()
        self.response = response or HandContactResponse()
        self.dt = dt
        self.max_time = max_time
        self.render_mode = render_mode
        self.camera = camera
        self._renderer = None

        all_joints = [
            self.model.joint(j).name
            for j in range(self.model.njnt)
            if self.model.joint(j).name.startswith(SUBJECT_PREFIX)
        ]
        names = [SUBJECT_PREFIX + j for j in joints] if joints else all_joints
        self.joint_names = names
        self._qadr = np.array(
            [self.model.jnt_qposadr[self.model.joint(n).id] for n in names]
        )
        ranges = np.array(
            [self.model.jnt_range[self.model.joint(n).id] for n in names]
        )
        self.action_space = gym.spaces.Box(
            ranges[:, 0].astype(np.float32), ranges[:, 1].astype(np.float32)
        )
        self.sides = subject.sides_in(self.model)
        n_obs = len(names) + 3 * len(self.sides) + 3 + 3 + 2 * len(self.sides)
        self.observation_space = gym.spaces.Box(
            -np.inf, np.inf, (n_obs,), np.float32
        )
        self._hands = [
            self.model.site(subject.hand_site(self.model, s)).id
            for s in self.sides
        ]
        self.sim: ForagingSim | None = None

    # ------------------------------------------------------------------

    def reset(self, *, seed: int | None = None, options=None):
        super().reset(seed=seed)
        self.sim = ForagingSim(
            self.model,
            self.config,
            self.library,
            self.response,
            dt=self.dt,
            seed=seed,
        )
        return self._obs(), self._info()

    def step(self, action):
        qpos = self.sim.data.qpos.copy()
        qpos[self._qadr] = np.clip(
            action, self.action_space.low, self.action_space.high
        )
        info = self.sim.step(monkey_qpos=qpos[self.sim.subject_qadr])
        reward = 1.0 if info.reward_started else 0.0
        terminated = self.sim.done
        truncated = (
            self.max_time is not None and self.sim.time >= self.max_time
        )
        return self._obs(), reward, terminated, truncated, self._info(info)

    def render(self):
        if self.render_mode != "rgb_array":
            return None
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, 480, 640)
        self._renderer.update_scene(self.sim.data, camera=self.camera)
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    # ------------------------------------------------------------------

    @property
    def target_position(self) -> np.ndarray | None:
        """Position of the presented object, if one is presented."""
        presentation = self.sim.presentation
        if presentation is None:
            return None
        body = self.model.body(presentation.spec.object_id).id
        return self.sim.data.xpos[body].copy()

    @property
    def released_sides(self) -> list[str]:
        """Subject arms free to respond right now."""
        presentation = self.sim.presentation
        if presentation is None or not presentation.window_open:
            return []
        return [s for s in self.sides if not self.sim.arm_locked[s]]

    @property
    def data(self):
        return self.sim.data

    def _obs(self) -> np.ndarray:
        sim = self.sim
        target = self.target_position
        presentation = sim.presentation
        return np.concatenate(
            [
                sim.data.qpos[self._qadr],
                sim.data.site_xpos[self._hands].ravel(),
                np.zeros(3) if target is None else target,
                [float(s in self.released_sides) for s in self.sides],
                [float(sim.arm_locked[s]) for s in self.sides],
                [
                    float(sim.glass_revealed),
                    float(
                        presentation is not None and presentation.window_open
                    ),
                    float(sim.reward_active),
                    float(target is not None),
                ],
            ]
        ).astype(np.float32)

    def _info(self, step_info=None) -> dict:
        return {
            "time": self.sim.time,
            "events": [] if step_info is None else step_info.events,
            "trials_done": sum(
                1 for r in self.sim.results.values() if r.feedback is not None
            ),
        }


def run_scripted_episode(env: ForagingEnv, seed: int | None = None) -> dict:
    """Run one episode with :class:`ReachPolicy`; summarize the outcome."""
    obs, _ = env.reset(seed=seed)
    policy = ReachPolicy(env)
    total, closest = 0.0, np.inf
    done = False
    while not done:
        obs, reward, terminated, truncated, _ = env.step(policy(obs))
        total += reward
        target = env.target_position
        if target is not None and env.released_sides:
            hands = env.sim.data.site_xpos[env._hands]
            closest = min(
                closest, float(np.linalg.norm(hands - target, axis=1).min())
            )
        done = terminated or truncated
    results = env.sim.results.values()
    return {
        "trials": len(results),
        "rewarded": int(total),
        "closest_approach_m": closest,
        "sim_time": env.sim.time,
    }
