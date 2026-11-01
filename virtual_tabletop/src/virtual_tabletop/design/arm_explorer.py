"""Pose the monkey arms with sliders and read off joint angles.

::

    uv run vtt-arm-explorer [--independent] [--rig]

Opens the RL model (both arms and the presented object) in MuJoCo's viewer.
Each monkey joint gets a slider in the viewer's **Control** panel
(right-hand side), in **degrees**, spanning the joint's range. Posing is
kinematic (no physics or gravity): the sliders set the joint angles
directly. By default the left arm mirrors the right, so there is one slider
per right-arm joint; ``--independent`` gives both arms sliders.

Keys: P print the posture (YAML, degrees) to the terminal; 0 reset to the
rest posture; O switch the object (small/big); H hide/show the object.

The overlay shows each fingertip's position and its arm's gap to the object
(negative = the hand overlaps it, which counts as a touch).
"""

import argparse

import mujoco
import numpy as np

from virtual_tabletop import viewer
from virtual_tabletop.assets import subject
from virtual_tabletop.assets.rl_model import (
    RL_PARKED,
    RL_STIMULI,
    load_rl_model,
    rl_model_path,
)
from virtual_tabletop.rl.scene import PresentationScene, presentation_pose


def explorer_model(
    include_rig: bool = False, independent: bool = False
) -> mujoco.MjModel:
    """The RL model with one degree-valued slider actuator per joint.

    The servos and keyframe are removed. Slider actuators have no gain
    (they never apply force); the explorer copies their ``ctrl`` into
    ``qpos``.
    """
    load_rl_model(include_rig)  # make sure it's built
    spec = mujoco.MjSpec.from_file(str(rl_model_path(include_rig)))
    reference = spec.compile()
    for key in list(spec.keys):
        spec.delete(key)
    for actuator in list(spec.actuators):
        spec.delete(actuator)
    sides = subject.SIDES if independent else ("right",)
    for side in sides:
        for joint in subject.joint_names(reference, side):
            low, high = np.degrees(reference.joint(joint).range)
            spec.add_actuator(
                name=joint.removeprefix(subject.SUBJECT_PREFIX),
                target=joint,
                trntype=mujoco.mjtTrn.mjTRN_JOINT,
                ctrllimited=True,
                ctrlrange=[low, high],
                gainprm=[0] * 10,
                biasprm=[0] * 10,
            )
    return spec.compile()


def _arm_gap(model, data, side: str, target_body: int) -> float:
    """Smallest distance from ``side``'s arm to the target body (m)."""
    arm = subject.body_ids(model, side)
    hand = [g for g in range(model.ngeom) if model.geom_bodyid[g] in arm]
    obj = [
        g
        for g in range(model.ngeom)
        if model.geom_bodyid[g] == target_body and model.geom_conaffinity[g]
    ]
    return min(
        mujoco.mj_geomDistance(model, data, a, b, 0.5, None)
        for a in hand
        for b in obj
    )


class Explorer:
    """Slider → joint-angle mapping, posture printing, overlay text."""

    def __init__(self, model, independent: bool, task: str):
        self.model, self.data = model, mujoco.MjData(model)
        self.independent = independent
        self.scene = PresentationScene(model, self.data)
        self.pose = presentation_pose(task)
        m = model
        self._qadr = np.array(
            [m.jnt_qposadr[m.actuator_trnid[a, 0]] for a in range(m.nu)]
        )
        self._mirror = []  # (right qpos addr, left qpos addr) pairs
        if not independent:
            for name in subject.joint_names(m, "right"):
                other = subject.prefix("left") + name.split("/")[-1]
                self._mirror.append(
                    (m.joint(name).qposadr[0], m.joint(other).qposadr[0])
                )
        self.stimulus = RL_STIMULI[0]
        self.visible = True
        self.reset_posture()
        self.place_object()

    def reset_posture(self) -> None:
        low, high = self.model.actuator_ctrlrange.T
        self.data.ctrl[:] = np.clip(0.0, low, high)

    def place_object(self) -> None:
        self.scene.show(self.stimulus, self.pose)
        if not self.visible:
            self.data.mocap_pos[self.scene.current.mocap] = RL_PARKED

    def on_key(self, key: int) -> None:
        if key == viewer.KEY_P:
            print(self.posture_yaml(), flush=True)
        elif key == viewer.KEY_0:
            self.reset_posture()
        elif key == viewer.KEY_O:
            i = RL_STIMULI.index(self.stimulus)
            self.stimulus = RL_STIMULI[(i + 1) % len(RL_STIMULI)]
            self.place_object()
        elif key == viewer.KEY_H:
            self.visible = not self.visible
            self.place_object()

    def apply(self) -> None:
        """Copy the sliders into the joints."""
        d = self.data
        d.qpos[self._qadr] = np.radians(d.ctrl)
        for right, left in self._mirror:
            d.qpos[left] = d.qpos[right]
        mujoco.mj_forward(self.model, d)

    def posture_yaml(self) -> str:
        lines = ["posture_deg:"]
        for a in range(self.model.nu):
            name = self.model.actuator(a).name
            lines.append(f"  {name}: {self.data.ctrl[a]:.1f}")
        return "\n".join(lines)

    def status_text(self) -> str:
        m, d = self.model, self.data
        parts = [f"object: {self.stimulus if self.visible else 'hidden'}"]
        for side in self.scene.sides:
            tip = self.scene.hand_position(side)
            text = f"{side} tip ({tip[0]:.3f}, {tip[1]:.3f}, {tip[2]:.3f})"
            if self.visible:
                gap = _arm_gap(m, d, side, self.scene.current.body)
                text += f" gap {100 * gap:+.1f} cm"
                if gap <= 0:
                    text += " TOUCH"
            parts.append(text)
        mode = "both arms" if self.independent else "left mirrors right"
        parts.append(f"{mode}; P print, 0 rest, O object, H hide")
        return "\n".join(parts)

    def update(self, wall_dt: float) -> str:
        self.apply()
        return self.status_text()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--independent",
        action="store_true",
        help="separate sliders for each arm (default: left mirrors right)",
    )
    parser.add_argument(
        "--rig", action="store_true", help="show the rig frame"
    )
    parser.add_argument(
        "--task", default="foraging_ordered", help="for the object pose"
    )
    args = parser.parse_args()
    viewer.ensure_mjpython("virtual_tabletop.design.arm_explorer")
    explorer = Explorer(
        explorer_model(args.rig, args.independent), args.independent, args.task
    )

    def camera(cam: mujoco.MjvCamera) -> None:
        cam.lookat[:] = (0.95, 0.25, 0.6)
        cam.distance, cam.azimuth, cam.elevation = 1.0, -60, -20

    viewer.run_passive(
        explorer.model,
        explorer.data,
        explorer.update,
        on_key=explorer.on_key,
        setup_camera=camera,
    )


if __name__ == "__main__":
    main()
