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
    """Potential-based reward for bringing a released fingertip to a target.

    The potential is ``phi = -scale * min_side d(fingertip_side)`` over the
    released arms while a response window is open, and 0 otherwise (the
    phase is in the observation, so ``phi`` is a function of the state).
    Each step adds ``gamma * phi(s') - phi(s)`` to the reward (Ng, Harada &
    Russell 1999), which leaves the optimal policy unchanged. The task's own
    reward is kept in ``info["task_reward"]`` and the shaping term in
    ``info["shaping"]``.

    Targets (the distance ``d``):

    - ``"object"``: straight to the presented object's centre;
    - ``"button"``: straight to the top centre of its button;
    - ``"button_via_opening"``: to the button *through the opening above
      it*. An entry point sits on the button's axis, ``approach`` above it
      (``approach_margin`` above the rim for a hollow geometry such as the
      cup; ``approach_default`` otherwise, where 0 means straight to the
      button), and a funnel (radius ``funnel_radius``, from the button
      up to the entry point) runs along the axis. Inside the funnel ``d``
      is the straight distance to the button; outside it is the distance
      to the entry point plus the entry point's height above the button,
      so the hand is drawn up and over the rim first instead of into the
      wall. (Straight-line shaping led the hand against the outside of the
      cup's wall at floor height.)

    Args:
        scale: Potential per metre of distance.
        gamma: The learner's discount factor.
        target: One of :attr:`TARGETS`.
        funnel_radius: Radius of the funnel above the button (m).
        approach_margin: Entry point height above a hollow geometry's rim
            (m).
        approach_default: Entry point height above the button for other
            geometries (m).
    """

    TARGETS = ("object", "button", "button_via_opening")

    def __init__(
        self,
        env: gym.Env,
        scale: float = 1.0,
        gamma: float = 0.99,
        target: str = "object",
        funnel_radius: float = 0.02,
        approach_margin: float = 0.01,
        approach_default: float = 0.03,
    ):
        super().__init__(env)
        if target not in self.TARGETS:
            raise ValueError(f"target must be one of {self.TARGETS}")
        self.scale, self.gamma, self.target = scale, gamma, target
        self.funnel_radius = funnel_radius
        self.approach = _approach_heights(approach_margin, approach_default)
        self._phi = 0.0

    def distance(self, side: str) -> float:
        """``d`` for one arm's fingertip (m)."""
        scene = self.unwrapped.scene
        tip = scene.hand_position(side)
        if self.target == "object":
            return float(np.linalg.norm(tip - scene.object_center))
        button = scene.button_position
        if self.target == "button":
            return float(np.linalg.norm(tip - button))
        normal = scene.button_normal
        offset = tip - button
        height = float(offset @ normal)
        radial = float(np.linalg.norm(offset - height * normal))
        approach = self.approach[scene.current.geometry]
        if radial <= self.funnel_radius and height <= approach:
            return float(np.linalg.norm(offset))
        entry = button + approach * normal
        return float(np.linalg.norm(tip - entry)) + approach

    def potential(self) -> float:
        sides = self.unwrapped.released_sides
        if not sides:
            return 0.0
        return -self.scale * min(self.distance(s) for s in sides)

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


def _approach_heights(margin: float = 0.01, default: float = 0.03) -> dict:
    """Per geometry: the entry point's height above the button's top (m).

    For a hollow geometry, ``margin`` above its rim; otherwise ``default``.
    """
    from virtual_tabletop.buttons.config import load_buttons, load_hollow

    hollow = load_hollow()
    heights = {}
    for geometry, button in load_buttons().items():
        top = float(button.position[2] + button.height)
        if geometry in hollow:
            heights[geometry] = hollow[geometry].wall_z[1] - top + margin
        else:
            heights[geometry] = default
    return heights
