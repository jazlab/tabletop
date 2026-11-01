"""Arm and blocks: a 3-joint arm on a pedestal, blocks to push and topple.

The arm has a base yaw joint (about z), a shoulder and an elbow (pitch, about
the arm's y axis), capsule links and a sphere at the tip; no hand. Each
joint has a position servo: the action in [-1, 1]³ sets the joint's target
angle across its range (``control="position"``), or a torque
(``control="torque"``). The arm's weight is compensated. Positive pitch
points a link downward.

Blocks (boxes of random size, mass and colour) spawn on the floor around
the arm, within its reach; with ``towers`` > 0 some start stacked into
towers, which topple.

Agent state (9): joint angles (3), joint velocities (3), tip position (3).

Cameras: ``overview`` (angled), ``top``, ``side``.
"""

import mujoco
import numpy as np

from mujoco_toys.envs.base import ToyEnv
from mujoco_toys.objects import random_shape
from mujoco_toys.scene import AGENT, AGENT_RGBA, STATIC, base_spec

#: (name, axis, range in rad) of the arm's joints, base to tip.
JOINTS = (
    ("yaw", (0, 0, 1), (-np.pi, np.pi)),
    ("shoulder", (0, 1, 0), (-1.6, 1.2)),
    ("elbow", (0, 1, 0), (0.0, 2.4)),
)
#: The arm's resting posture (rad): the middle of each range, i.e. action
#: 0 (tip about 6 cm above the floor, 0.55 m out).
REST = (0.0, -0.2, 1.2)


class ArmBlocksEnv(ToyEnv):
    """See the module docstring; also :class:`~mujoco_toys.envs.base.ToyEnv`.

    Args:
        control: ``"position"`` (servo targets) or ``"torque"``.
        kp: Position-servo stiffness (N·m/rad).
        max_torque: Torque limit (N·m), for either control mode.
        lengths: Upper-arm and forearm lengths (m).
        reach: ``(min, max)`` distance from the base where blocks spawn (m).
        towers: Towers to stack at the start of an episode.
        tower_height: Blocks per tower.
        **kwargs: ``ToyEnv`` options (objects, spawning, observations…).
    """

    def __init__(
        self,
        *,
        control: str = "position",
        kp: float = 20.0,
        max_torque: float = 15.0,
        lengths: tuple[float, float] = (0.4, 0.35),
        reach: tuple[float, float] = (0.3, 0.7),
        towers: int = 1,
        tower_height: int = 3,
        **kwargs,
    ):
        if control not in ("position", "torque"):
            raise ValueError("control must be 'position' or 'torque'")
        self.control, self.kp, self.max_torque = control, kp, max_torque
        self.lengths, self.reach = lengths, reach
        self.towers, self.tower_height = towers, tower_height
        kwargs.setdefault("n_objects", 6)
        kwargs.setdefault("max_objects", 12)
        super().__init__(**kwargs)

    def _random_shapes(self, rng):
        return [
            random_shape(
                rng, kinds=("box",), scale=(0.035, 0.06), mass=(0.05, 0.4)
            )
            for _ in range(self.max_objects)
        ]

    def _build_spec(self, rng) -> mujoco.MjSpec:
        spec = base_spec(floor_half=2.0, name="arm_blocks")
        world = spec.worldbody
        world.add_geom(
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=(0.09, 0.14, 0),
            pos=(0, 0, 0.14),
            rgba=(0.3, 0.32, 0.36, 1),
            contype=STATIC,
            conaffinity=STATIC,
        )
        body = world.add_body(name="yaw", pos=(0, 0, 0.32))
        upper, fore = self.lengths
        links = {"yaw": None, "shoulder": upper, "elbow": fore}
        for name, axis, rng_ in JOINTS:
            if name != "yaw":
                offset = (upper, 0, 0) if name == "elbow" else (0, 0, 0)
                body = body.add_body(name=name, pos=offset)
            body.gravcomp = 1.0
            body.add_joint(
                name=name,
                type=mujoco.mjtJoint.mjJNT_HINGE,
                axis=axis,
                range=rng_,
                damping=0.5,
                armature=0.01,
            )
            length = links[name]
            if length is None:
                body.add_geom(
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=(0.06, 0, 0),
                    rgba=AGENT_RGBA,
                    mass=0.5,
                    contype=AGENT,
                    conaffinity=0,
                )
            else:
                body.add_geom(
                    name=f"{name}_link",
                    type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                    fromto=(0, 0, 0, length, 0, 0),
                    size=(0.03, 0, 0),
                    rgba=AGENT_RGBA,
                    contype=AGENT,
                    conaffinity=STATIC,
                )
            actuator = spec.add_actuator(
                name=name, target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT
            )
            if self.control == "position":
                actuator.set_to_position(
                    kp=self.kp, dampratio=1.0, inheritrange=1.0
                )
            else:
                actuator.ctrllimited = True
                actuator.ctrlrange = (-self.max_torque, self.max_torque)
            actuator.forcelimited = True
            actuator.forcerange = (-self.max_torque, self.max_torque)
        body.add_geom(
            name="tip",
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=(0.045, 0, 0),
            pos=(fore, 0, 0),
            rgba=(0.95, 0.3, 0.05, 1),
            contype=AGENT,
            conaffinity=STATIC,
        )
        body.add_site(name="tip", pos=(fore, 0, 0), size=(0.01, 0, 0))
        world.add_camera(
            name="overview",
            pos=(1.6, -1.6, 1.4),
            xyaxes=(0.71, 0.71, 0, -0.33, 0.33, 0.88),
        )
        world.add_camera(
            name="top", pos=(0, 0, 2.8), xyaxes=(1, 0, 0, 0, 1, 0)
        )
        world.add_camera(
            name="side", pos=(0, -2.2, 0.6), xyaxes=(1, 0, 0, 0, 0.1, 1)
        )
        return spec

    def _reset_agent(self, rng) -> None:
        for (name, _, _), q in zip(JOINTS, REST, strict=True):
            self.data.qpos[self.model.joint(name).qposadr[0]] = q
        if self.control == "position":
            self.data.ctrl[:] = REST

    def reset(self, *, seed=None, options=None):
        obs, info = super().reset(seed=seed, options=options)
        if self.towers:
            self._stack_towers()
            mujoco.mj_forward(self.model, self.data)
            self._last_positions = self.pool.positions().copy()
            obs = self._obs()
        return obs, info

    def _stack_towers(self) -> None:
        """Re-place some active blocks as towers at reachable spots."""
        rng = self.np_random
        active = list(np.flatnonzero(self.pool.active))
        for _ in range(self.towers):
            if len(active) < self.tower_height:
                break
            spot = self._spawn_spot(rng)
            z = 0.0
            for _ in range(self.tower_height):
                i = active.pop()
                half = self.shapes[i].size[2]
                self.pool.place(i, (*spot, z + half), rng.uniform(-0.2, 0.2))
                z += 2 * half

    def _spawn_spot(self, rng) -> np.ndarray:
        r = np.sqrt(rng.uniform(self.reach[0] ** 2, self.reach[1] ** 2))
        a = rng.uniform(0, 2 * np.pi)
        return np.array([r * np.cos(a), r * np.sin(a)])

    def _keep_clear(self) -> list:
        return [(0.0, 0.0, 0.15)]  # the pedestal

    def _agent_geoms(self) -> set[int]:
        m = self.model
        arm = {m.body(n).id for n, _, _ in JOINTS}
        return {g for g in range(m.ngeom) if m.geom_bodyid[g] in arm}

    @property
    def tip_position(self) -> np.ndarray:
        return self.data.site_xpos[self.model.site("tip").id].copy()

    def _ctrl_from_action(self, action: np.ndarray) -> np.ndarray:
        if self.control == "position":
            return super()._ctrl_from_action(action)
        return action * self.max_torque

    def _agent_state(self) -> np.ndarray:
        q = self.data.qpos[:3]
        v = self.data.qvel[:3]
        return np.concatenate([q, v, self.tip_position])
