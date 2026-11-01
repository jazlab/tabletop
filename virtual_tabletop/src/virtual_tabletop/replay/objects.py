"""Infer task-object poses from end-effector motion.

Bags record the robots' joint states but not which object each arm holds.
The rig's manipulation state machine (``commander.yaml``,
``manipulation_state_goals``) grasps with the end-effector frame exactly at
the object's seated pose (``attach`` offset ``[0, 0, 0]``), carries it, and
later places it back the same way. So:

- an arm that is not holding anything **attaches** a seated object when its
  ``eef`` origin comes within ``attach_tol`` of that object's mount pose, and
  the object then keeps its pose relative to the ``eef``;
- once the ``eef`` has moved more than ``depart_dist`` away from the mount,
  coming back within ``attach_tol`` of it **detaches** the object, which
  snaps back to its mount.

Not modelled yet: objects left at the reset stations, and grasp failures.
"""

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation


def _rot(quat_wxyz: np.ndarray) -> Rotation:
    w, x, y, z = quat_wxyz
    return Rotation.from_quat([x, y, z, w])


def _wxyz(rotation: Rotation) -> np.ndarray:
    x, y, z, w = rotation.as_quat()
    return np.array([w, x, y, z])


@dataclass
class _Grasp:
    index: int
    rel_pos: np.ndarray
    rel_rot: Rotation
    departed: bool = False


@dataclass
class ObjectTracker:
    """Tracks which arm holds which object, frame by frame.

    Attributes:
        mount_pos: Seated object positions, shape ``(n_objects, 3)``.
        mount_quat: Seated object orientations (wxyz), ``(n_objects, 4)``.
        attach_tol: Distance (m) at which an eef grasps/releases at a mount.
        depart_dist: Distance (m) the eef must leave a mount before a return
            counts as a release, or, after a release, before it can grasp
            again.
    """

    mount_pos: np.ndarray
    mount_quat: np.ndarray
    attach_tol: float = 0.02
    depart_dist: float = 0.05
    _held: dict[str, _Grasp] = field(default_factory=dict, init=False)
    # Mount each arm just released at; it must leave before grasping again.
    _released: dict[str, int] = field(default_factory=dict, init=False)

    def holding(self, arm: str) -> int | None:
        """Index of the object ``arm`` holds, if any."""
        grasp = self._held.get(arm)
        return None if grasp is None else grasp.index

    def update(
        self,
        eef: dict[str, tuple[np.ndarray, np.ndarray]],
        allowed: dict[str, set[int]] | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Advance one frame.

        Args:
            eef: ``{arm: (position, quat_wxyz)}`` end-effector poses.
            allowed: Optional ``{arm: object indices}`` an arm may grasp
                this frame (e.g. only the object being fetched); arms
                missing from the dict may grasp any object.

        Returns:
            Object positions ``(n, 3)`` and orientations ``(n, 4)``.
        """
        held = {g.index for g in self._held.values()}
        for arm, (pos, quat) in eef.items():
            grasp = self._held.get(arm)
            if arm in self._released:
                mount = self.mount_pos[self._released[arm]]
                if np.linalg.norm(mount - pos) <= self.depart_dist:
                    continue
                del self._released[arm]
            if grasp is None:
                dist = np.linalg.norm(self.mount_pos - pos, axis=1)
                if held:
                    dist[list(held)] = np.inf
                if allowed is not None and arm in allowed:
                    mask = np.ones(len(dist), bool)
                    mask[list(allowed[arm])] = False
                    dist[mask] = np.inf
                index = int(np.argmin(dist))
                if dist[index] < self.attach_tol:
                    eef_rot = _rot(quat)
                    self._held[arm] = _Grasp(
                        index=index,
                        rel_pos=eef_rot.inv().apply(
                            self.mount_pos[index] - pos
                        ),
                        rel_rot=eef_rot.inv() * _rot(self.mount_quat[index]),
                    )
                    held.add(index)
            else:
                dist = np.linalg.norm(self.mount_pos[grasp.index] - pos)
                if dist > self.depart_dist:
                    grasp.departed = True
                elif grasp.departed and dist < self.attach_tol:
                    del self._held[arm]
                    held.discard(grasp.index)
                    self._released[arm] = grasp.index

        positions = self.mount_pos.copy()
        quats = self.mount_quat.copy()
        for arm, grasp in self._held.items():
            pos, quat = eef[arm]
            eef_rot = _rot(quat)
            positions[grasp.index] = pos + eef_rot.apply(grasp.rel_pos)
            quats[grasp.index] = _wxyz(eef_rot * grasp.rel_rot)
        return positions, quats
