"""Gymnasium wrappers that shape the RL tasks for learning.

They wrap :class:`~virtual_tabletop.rl.env.PresentationEnv` (directly or
through other wrappers) and change only the action interface or the
training reward; the task itself (what is rewarded, when trials end) is
untouched.
"""

import gymnasium as gym
import numpy as np


class DeltaJointAction(gym.ActionWrapper):
    """Actions are joint-angle *steps*, normalised to [-1, 1].

    Each component moves its joint's servo target by at most
    ``max_speed * dt`` rad per step (1 = full speed), from the previous
    target. The target then goes to the env (which clips it to the joint
    range, and holds locked arms at rest). After each step the wrapper
    resynchronises with the target the env actually applied, so a locked
    arm's target doesn't drift. Compared with absolute targets, random
    actions explore smoothly around the current posture instead of
    jumping across the joint ranges.

    Args:
        max_speed: Joint speed (rad/s) for an action of 1.
    """

    def __init__(self, env: gym.Env, max_speed: float = 3.0):
        super().__init__(env)
        self.max_speed = max_speed
        self.step_size = max_speed * self.unwrapped.dt
        n = env.action_space.shape[0]
        self.action_space = gym.spaces.Box(-1.0, 1.0, (n,), np.float32)
        self._target: np.ndarray | None = None

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        self._target = self.unwrapped.joint_positions
        return obs, info

    def action(self, action: np.ndarray) -> np.ndarray:
        delta = np.clip(action, -1.0, 1.0) * self.step_size
        return self._target + delta

    def step(self, action):
        result = self.env.step(self.action(action))
        self._target = self.unwrapped.last_target.copy()
        return result


class ReachShaping(gym.Wrapper):
    """Potential-based reward for bringing a released fingertip to the object.

    The potential is ``phi = -scale * min_side |fingertip_side - centre|``
    over the released arms while a response window is open, and 0 otherwise
    (the phase is in the observation, so ``phi`` is a function of the
    state). Each step adds ``gamma * phi(s') - phi(s)`` to the reward (Ng,
    Harada & Russell 1999), which leaves the optimal policy unchanged. The
    task's own reward is kept in ``info["task_reward"]`` and the shaping
    term in ``info["shaping"]``.

    Args:
        scale: Potential per metre of distance.
        gamma: The learner's discount factor.
    """

    def __init__(self, env: gym.Env, scale: float = 1.0, gamma: float = 0.99):
        super().__init__(env)
        self.scale, self.gamma = scale, gamma
        self._phi = 0.0

    def potential(self) -> float:
        env = self.unwrapped
        sides = env.released_sides
        if not sides:
            return 0.0
        centre = env.scene.object_center
        distance = min(
            np.linalg.norm(env.scene.hand_position(s) - centre) for s in sides
        )
        return -self.scale * float(distance)

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        self._phi = self.potential()
        return obs, info

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        # After a trial ends the env is already in the next trial's
        # intertrial, where the potential is 0.
        phi = 0.0 if terminated else self.potential()
        shaping = self.gamma * phi - self._phi
        self._phi = phi
        info = dict(info, task_reward=reward, shaping=shaping)
        return obs, reward + shaping, terminated, truncated, info
