"""Scripted monkey policies shared by the foraging and RL environments.

Works with any env exposing ``model``, ``data``, ``dt``, ``joint_names`` and
``_qadr`` (the controlled monkey joints and their ``qpos`` addresses),
``released_sides`` (subject arms free to respond) and ``target_position``
(the presented object, or ``None``).
"""

import logging

import mujoco
import numpy as np

from virtual_tabletop import ik
from virtual_tabletop.assets import subject


class ReachPolicy:
    """Reach for the presented object with a released arm; otherwise rest.

    The reaching arm is the released arm whose shoulder is nearest the
    object; it moves by damped least-squares IK of its hand site over the
    arm joints (fingers stay at rest), with joint speed limited to
    ``max_speed`` rad/s. Other arms return to their rest posture.
    """

    def __init__(self, env, max_speed: float = 3.0):
        self.env = env
        self.max_speed = max_speed
        self._scratch = mujoco.MjData(env.model)
        self.rest: np.ndarray | None = None
        names = list(env.joint_names)
        self._cols = {
            side: [
                i
                for i, n in enumerate(names)
                if n.startswith(subject.prefix(side))
                and not subject.is_finger_joint(n)
            ]
            for side in subject.sides_in(env.model)
        }
        # Unreachable targets are expected; don't warn on every tick.
        logging.getLogger(ik.__name__).setLevel(logging.ERROR)

    def reaching_side(self) -> str | None:
        env = self.env
        target = env.target_position
        sides = [s for s in env.released_sides if self._cols.get(s)]
        if target is None or not sides:
            return None
        return min(
            sides,
            key=lambda s: np.linalg.norm(
                subject.shoulder_position(env.model, env.data, s) - target
            ),
        )

    def __call__(self, obs=None) -> np.ndarray:
        env = self.env
        current = env.data.qpos[env._qadr].copy()
        if self.rest is None:
            self.rest = current.copy()
        goal = self.rest.copy()
        side = self.reaching_side()
        if side is not None:
            self._scratch.qpos[:] = env.data.qpos
            names = [env.joint_names[i] for i in self._cols[side]]
            result = ik.qpos_from_site_pose(
                env.model,
                self._scratch,
                subject.hand_site(env.model, side),
                target_pos=env.target_position,
                joint_names=names,
                max_steps=50,
            )
            cols = self._cols[side]
            goal[cols] = result.qpos[env._qadr[cols]]
        step = self.max_speed * env.dt
        return current + np.clip(goal - current, -step, step)
