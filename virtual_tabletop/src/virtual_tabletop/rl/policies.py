"""Scripted (hand-written) monkey policies for the RL tasks.

Baselines and demos, not learned. Each acts on a
:class:`~virtual_tabletop.rl.env.PresentationEnv`: in the response window
the released arm whose shoulder is nearest the object moves toward a goal;
other arms go back to rest. Commands advance toward the goal at most
``max_speed`` rad/s per joint from the previous *command* (not the measured
angles), so servo lag under physics doesn't slow the motion.

- :class:`ReachPolicy`: IK of the fingertip to the object (touch task);
- :class:`ButtonPressPolicy`: follows a planned path down onto the button
  that avoids the object's walls (:mod:`.press_paths`; button task);
- :func:`scripted_policy`: the right one for an env's task.
"""

import logging

import mujoco
import numpy as np

from virtual_tabletop import ik
from virtual_tabletop.assets import subject
from virtual_tabletop.rl.press_paths import load_press_paths


class ScriptedPolicy:
    """Base: arm choice, rest posture and rate-limited commands."""

    def __init__(self, env, max_speed: float = 3.0):
        self.env = env
        self.max_speed = max_speed
        self.rest: np.ndarray | None = None
        self._command: np.ndarray | None = None
        names = list(env.joint_names)
        # Each side's arm joints (fingers stay at rest), as action columns.
        self._cols = {
            side: [
                i
                for i, n in enumerate(names)
                if n.startswith(subject.prefix(side))
                and not subject.is_finger_joint(n)
            ]
            for side in env.sides
        }

    def reaching_side(self) -> str | None:
        """The released arm whose shoulder is nearest the object."""
        env = self.env
        sides = [s for s in env.released_sides if self._cols.get(s)]
        if not sides:
            return None
        target = env.scene.object_position
        return min(
            sides,
            key=lambda s: np.linalg.norm(
                subject.shoulder_position(env.model, env.data, s) - target
            ),
        )

    def arm_goal(self, side: str, goal: np.ndarray) -> None:
        """Write ``side``'s goal angles into ``goal`` (action order)."""
        raise NotImplementedError

    def __call__(self, obs=None) -> np.ndarray:
        env = self.env
        current = env.joint_positions
        if self.rest is None:
            self.rest = current.copy()
        if self._command is None or env.episode_steps == 0:
            self._command = current  # a new episode
        goal = self.rest.copy()
        side = self.reaching_side()
        if side is not None:
            self.arm_goal(side, goal)
        step = self.max_speed * env.dt
        self._command = self._command + np.clip(
            goal - self._command, -step, step
        )
        return self._command.copy()


class ReachPolicy(ScriptedPolicy):
    """Damped least-squares IK of the fingertip to the object's origin."""

    def __init__(self, env, max_speed: float = 3.0):
        super().__init__(env, max_speed)
        self._scratch = mujoco.MjData(env.model)
        # Unreachable targets are expected; don't warn on every tick.
        logging.getLogger(ik.__name__).setLevel(logging.ERROR)

    def arm_goal(self, side: str, goal: np.ndarray) -> None:
        env = self.env
        cols = self._cols[side]
        self._scratch.qpos[:] = env.data.qpos
        result = ik.qpos_from_site_pose(
            env.model,
            self._scratch,
            subject.hand_site(side),
            target_pos=env.scene.object_position,
            joint_names=[env.joint_names[i] for i in cols],
            max_steps=50,
        )
        qadr = [env.model.joint(env.joint_names[i]).qposadr[0] for i in cols]
        goal[cols] = result.qpos[qadr]


class ButtonPressPolicy(ScriptedPolicy):
    """Come down onto the button along a wall-free planned path.

    Paths come from :func:`~virtual_tabletop.rl.press_paths.load_press_paths`
    (planned once, cached). The arm moves to each waypoint in turn,
    advancing when within ``tolerance`` rad of it.
    """

    def __init__(self, env, max_speed: float = 3.0, tolerance: float = 0.03):
        super().__init__(env, max_speed)
        self.tolerance = tolerance
        self.paths = load_press_paths()
        self._waypoint: dict[tuple[int, str], int] = {}

    def arm_goal(self, side: str, goal: np.ndarray) -> None:
        env = self.env
        path = self.paths[env.stimulus, side]
        key = (env.schedule.index, side)
        index = self._waypoint.get(key, 0)
        angles = path[index]
        error = max(
            abs(env.data.qpos[env.model.joint(n).qposadr[0]] - q)
            for n, q in angles.items()
        )
        if error < self.tolerance and index < len(path) - 1:
            index += 1
            angles = path[index]
        self._waypoint[key] = index
        for i in self._cols[side]:
            goal[i] = angles[env.joint_names[i]]


def scripted_policy(env, **kwargs) -> ScriptedPolicy:
    """The scripted policy for ``env``'s task."""
    cls = ButtonPressPolicy if env.reward == "button" else ReachPolicy
    return cls(env, **kwargs)
