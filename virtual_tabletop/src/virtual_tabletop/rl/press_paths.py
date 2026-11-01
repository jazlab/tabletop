"""Arm postures that press a button without going through its object.

:class:`PressPlanner` solves, by bounded optimisation over one arm's 7 arm
joints (fingers straight), for postures that bring the fingertip to a point
while penalising overlap between the arm and the presented object's other
collision geoms (e.g. the cup's walls). Kinematic only.

- :meth:`PressPlanner.solve`: the best posture pressing a button;
- :meth:`PressPlanner.press_path`: waypoints coming down onto the button
  along its normal, each free of overlap (the scripted button policy follows
  them);
- :func:`load_press_paths`: every (geometry, arm, presentation pose) path,
  cached in ``<cache>/press_paths/`` keyed by the RL model's contents and
  the poses.

The design tool ``vtt-button-reach`` uses the same planner to check
reachability and scan shoulder placements.
"""

import hashlib
from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.optimize import minimize

from virtual_tabletop import paths
from virtual_tabletop.assets import subject
from virtual_tabletop.assets.rig_config import Pose
from virtual_tabletop.assets.rl_model import load_rl_model, rl_model_path
from virtual_tabletop.rl.scene import (
    PresentationScene,
    pose_key,
    presentation_pose,
    presentation_poses,
)

#: Heights above the button's top (along its normal) of the press path's
#: waypoints (m). The last one is 5 mm *into* the button, so that under
#: physics the servos press on it (a target exactly at its surface leaves
#: the finger resting a hair above it).
PRESS_HEIGHTS = (0.09, 0.06, 0.04, 0.02, -0.005)

PATH_CACHE = paths.CACHE_DIR / "press_paths"


@dataclass
class Posture:
    """An arm posture and how well it reaches its target.

    Attributes:
        geometry, side: The object geometry and arm.
        angles: Joint angles (rad) by full joint name (7 arm joints).
        tip_error: Fingertip to the target (m).
        overlap: Deepest arm overlap with the object, button excluded (m).
        presses: Whether the hand touches the button.
    """

    geometry: str
    side: str
    angles: dict[str, float]
    tip_error: float
    overlap: float
    presses: bool

    def reachable(self, tolerance=0.01, max_overlap=0.002) -> bool:
        return (
            self.tip_error < tolerance
            and self.overlap < max_overlap
            and self.presses
        )


class PressPlanner:
    """The RL model with one object presented; poses one arm at a time."""

    def __init__(self, task: str = "foraging_ordered"):
        self.model = load_rl_model()
        self.data = mujoco.MjData(self.model)
        self.scene = PresentationScene(self.model, self.data)
        self.pose = presentation_pose(task)
        m = self.model
        self.roots = {
            side: m.body(subject.prefix(side) + "sternum").id
            for side in self.scene.sides
        }
        self.root_pos = {
            s: m.body_pos[b].copy() for s, b in self.roots.items()
        }

    def shift_shoulders(self, dy: float, dz: float) -> None:
        """Move both arms (forward ``dy``, up ``dz``) in the compiled model."""
        for side, body in self.roots.items():
            self.model.body_pos[body] = self.root_pos[side] + (0, dy, dz)

    def _show(self, geometry: str, side: str, pose: Pose | None) -> None:
        m, d = self.model, self.data
        self.scene.show(geometry, pose or self.pose)
        self.side = side
        self.joints = subject.arm_joint_names(m, side)
        self.addrs = np.array([m.joint(n).qposadr[0] for n in self.joints])
        self.bounds = np.array([m.joint(n).range for n in self.joints])
        self.site = m.site(subject.hand_site(side)).id
        d.qpos[:] = 0
        mujoco.mj_kinematics(m, d)
        self.button_top = self.scene.button_position
        self.normal = self.scene.button_normal
        self.target = self.button_top

    def _evaluate(self, q) -> tuple[float, bool]:
        """Pose the arm at ``q``: (overlap with the object, presses)."""
        self.data.qpos[self.addrs] = q
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_collision(self.model, self.data)
        presses = bool(self.scene.pressing_sides([self.side]))
        return self.scene.arm_overlap(self.side), presses

    def _cost(self, q) -> float:
        overlap, _ = self._evaluate(q)
        tip = self.data.site_xpos[self.site]
        return float(np.sum((tip - self.target) ** 2) + 2000 * overlap**2)

    def _optimise(self, inits) -> np.ndarray:
        best = None
        for q0 in inits:
            q0 = np.clip(q0, self.bounds[:, 0], self.bounds[:, 1])
            r = minimize(self._cost, q0, method="L-BFGS-B", bounds=self.bounds)
            if best is None or r.fun < best.fun:
                best = r
        return best.x

    def _posture(self, q) -> Posture:
        overlap, presses = self._evaluate(q)
        tip = self.data.site_xpos[self.site]
        return Posture(
            self.scene.current.geometry,
            self.side,
            dict(zip(self.joints, map(float, q), strict=True)),
            float(np.linalg.norm(tip - self.target)),
            overlap,
            presses,
        )

    def _random_inits(self, starts: int, seed: int):
        rng = np.random.default_rng(seed)
        return [np.zeros(len(self.addrs))] + [
            rng.uniform(self.bounds[:, 0], self.bounds[:, 1])
            for _ in range(starts)
        ]

    def solve(
        self,
        geometry: str,
        side: str = "right",
        starts=48,
        seed=0,
        pose: Pose | None = None,
    ) -> Posture:
        """The best posture pressing ``geometry``'s button with ``side``
        (object at ``pose``; default the task's first pose)."""
        self._show(geometry, side, pose)
        return self._posture(self._optimise(self._random_inits(starts, seed)))

    def press_path(
        self,
        geometry: str,
        side: str = "right",
        heights=PRESS_HEIGHTS,
        starts=48,
        seed=0,
        pose: Pose | None = None,
    ) -> list[Posture]:
        """Waypoints (highest first) down onto the button, free of overlap.

        Solves the pressing posture first, then each higher waypoint
        warm-started from the one below it, so consecutive postures stay
        close (no swinging through the walls in between).
        """
        self._show(geometry, side, pose)
        path, q = [], None
        for height in sorted(heights):
            self.target = self.button_top + height * self.normal
            inits = self._random_inits(starts, seed) if q is None else [q]
            q = self._optimise(inits)
            path.append(self._posture(q))
        return path[::-1]


def load_press_paths(
    task: str = "foraging_ordered", heights=PRESS_HEIGHTS
) -> dict[tuple[str, str, str], list[dict[str, float]]]:
    """Press paths (joint angles per waypoint), by
    ``(geometry, side, pose_key(pose))`` for every pose of ``task``.

    Planned once (about 20 s) and cached, keyed by the RL model file's
    contents and the poses, so any change (placement, buttons, arm, task
    poses) replans.
    """
    model_file = rl_model_path()
    if not model_file.exists():
        load_rl_model()  # builds it
    poses = presentation_poses(task)
    digest = hashlib.sha1(
        model_file.read_bytes()
        + repr(heights).encode()
        + repr([pose_key(p) for p in poses]).encode()
    )
    key = digest.hexdigest()[:16]
    cache = PATH_CACHE / f"{key}.npz"
    if cache.exists():
        with np.load(cache) as saved:
            return _unpack(saved)
    planner = PressPlanner(task)
    arrays = {}
    for pose in poses:
        for geometry in planner.scene.stimuli:
            for side in planner.scene.sides:
                path = planner.press_path(geometry, side, heights, pose=pose)
                names = list(path[0].angles)
                name = f"{geometry}|{side}|{pose_key(pose)}"
                arrays[f"{name}|joints"] = np.array(names)
                arrays[f"{name}|angles"] = np.array(
                    [[p.angles[n] for n in names] for p in path]
                )
    PATH_CACHE.mkdir(parents=True, exist_ok=True)
    np.savez(cache, **arrays)
    return _unpack(arrays)


def _unpack(arrays) -> dict[tuple[str, str, str], list[dict[str, float]]]:
    paths_ = {}
    for name in arrays:
        geometry, side, pose, kind = name.split("|")
        if kind != "joints":
            continue
        joints = [str(j) for j in arrays[name]]
        angles = arrays[f"{geometry}|{side}|{pose}|angles"]
        paths_[geometry, side, pose] = [
            dict(zip(joints, map(float, row), strict=True)) for row in angles
        ]
    return paths_
