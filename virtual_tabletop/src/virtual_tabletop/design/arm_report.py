"""Evidence for the joint-control design: sweeps, workspace and ablation.

::

    uv run vtt-arm-report [--out DIR] [--samples N] [--no-video]

Writes to ``DIR`` (default ``.cache/arm_report/``):

- ``arm_sweep.mp4``: each arm joint through its range (rest → min → max →
  rest), others at rest, both arms (the left mirrors the right); subject
  view and side view;
- ``finger_sweep.mp4``: thumb abduction, then each finger's flexion joints
  together, then all fingers; close-up of the right hand;
- ``workspace.png``: where the right fingertip can go (uniform samples over
  the arm joint ranges, fingers at rest), with the presented object;
- ``report.md``: reach numbers, the best-reach posture, and a joint
  ablation (which joints can be frozen at rest and still touch).

Everything is kinematic and uses the right arm; the left is its exact
mirror image.
"""

import argparse
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import minimize

from virtual_tabletop import paths
from virtual_tabletop.assets import subject
from virtual_tabletop.assets.rl_model import RL_STIMULI, load_rl_model
from virtual_tabletop.assets.sim_config import load_rig_config
from virtual_tabletop.rl.scene import PresentationScene, presentation_pose

DEFAULT_OUT = paths.CACHE_DIR / "arm_report"
SIDE = "right"
FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")


@dataclass
class Arm:
    """The right arm's joints in the RL model, and the presented object."""

    model: mujoco.MjModel
    data: mujoco.MjData
    scene: PresentationScene
    joints: list[str]  # arm joints, short names
    fingers: list[str]  # finger joints, short names

    @classmethod
    def load(cls, task: str = "foraging_ordered") -> "Arm":
        model = load_rl_model(False)
        data = mujoco.MjData(model)
        names = [n.split("/")[-1] for n in subject.joint_names(model, SIDE)]
        arm = cls(
            model,
            data,
            PresentationScene(model, data),
            [n for n in names if not subject.is_finger_joint(n)],
            [n for n in names if subject.is_finger_joint(n)],
        )
        arm.pose = presentation_pose(task)
        arm.show(RL_STIMULI[0])
        return arm

    @property
    def stimulus(self) -> str:
        return self.scene.current.geometry

    def addr(self, joint: str, side: str = SIDE) -> int:
        return self.model.joint(subject.prefix(side) + joint).qposadr[0]

    def range(self, joint: str) -> np.ndarray:
        return self.model.joint(subject.prefix(SIDE) + joint).range

    def show(self, stimulus: str) -> None:
        self.scene.show(stimulus, self.pose)
        self._update()

    def set(self, angles: dict[str, float], mirror: bool = True) -> None:
        """Set joint angles (rad), others unchanged; then kinematics."""
        for joint, value in angles.items():
            self.data.qpos[self.addr(joint)] = value
            if mirror:
                self.data.qpos[self.addr(joint, "left")] = value
        self._update()

    def _update(self) -> None:
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_camlight(self.model, self.data)  # fixed cameras

    def rest(self) -> None:
        self.data.qpos[:] = 0
        self._update()

    @property
    def tip(self) -> np.ndarray:
        return self.scene.hand_position(SIDE)

    @property
    def shoulder(self) -> np.ndarray:
        return subject.shoulder_position(self.model, self.data, SIDE)

    @property
    def object_center(self) -> np.ndarray:
        """Centre of the presented object's mesh."""
        return self.data.geom_xpos[list(self.scene.current.meshes)].mean(0)

    @property
    def object_size(self) -> float:
        """Largest extent of the presented object's mesh (m)."""
        m = self.model
        sizes = []
        for g in self.scene.current.meshes:
            mesh = m.geom_dataid[g]
            start = m.mesh_vertadr[mesh]
            v = m.mesh_vert[start : start + m.mesh_vertnum[mesh]]
            sizes.append((v.max(axis=0) - v.min(axis=0)).max())
        return float(max(sizes))

    def gap(self) -> float:
        """Smallest arm-to-object distance (m); negative = overlap."""
        m, d = self.model, self.data
        bodies = subject.body_ids(m, SIDE)
        arm = [g for g in range(m.ngeom) if m.geom_bodyid[g] in bodies]
        return min(
            mujoco.mj_geomDistance(m, d, a, b, 0.5, None)
            for a in arm
            for b in self.scene.current.contact
        )


# --- sweeps -----------------------------------------------------------------


def _ease(n: int) -> np.ndarray:
    return 0.5 - 0.5 * np.cos(np.linspace(0, np.pi, n))


def _sweep(low: float, high: float, fps: int, seconds: float) -> np.ndarray:
    """rest(0) → low → high → rest, eased."""
    n = max(2, int(fps * seconds / 4))
    start = float(np.clip(0.0, low, high))
    parts = [
        start + (low - start) * _ease(n),
        low + (high - low) * _ease(2 * n),
        high + (start - high) * _ease(n),
    ]
    return np.concatenate(parts)


def _label(frame: np.ndarray, text: str) -> np.ndarray:
    from PIL import Image, ImageDraw, ImageFont

    image = Image.fromarray(frame)
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default(size=22)
    draw.rectangle((0, 0, image.width, 34), fill=(0, 0, 0))
    draw.text((10, 5), text, fill=(255, 255, 255), font=font)
    return np.asarray(image)


def render_arm_sweep(arm: Arm, output: Path, fps: int = 30) -> Path:
    import imageio.v2 as imageio

    m = arm.model
    renderer = mujoco.Renderer(m, 480, 640)
    side = mujoco.MjvCamera()
    side.lookat[:] = (0.95, 0.25, 0.5)
    side.distance, side.azimuth, side.elevation = 1.3, 180, -10
    with imageio.get_writer(output, fps=fps, macro_block_size=8) as writer:
        for joint in arm.joints:
            low, high = arm.range(joint)
            arm.rest()
            for q in _sweep(low, high, fps, 4.0):
                arm.set({joint: q})
                views = []
                for camera in ("subject", side):
                    renderer.update_scene(arm.data, camera=camera)
                    views.append(renderer.render())
                text = (
                    f"{joint}  {np.degrees(q):+6.1f} deg   range "
                    f"[{np.degrees(low):.0f}, {np.degrees(high):.0f}]"
                )
                writer.append_data(_label(np.hstack(views), text))
    renderer.close()
    return output


def _finger_groups(arm: Arm) -> list[tuple[str, list[str]]]:
    flex = [j for j in arm.fingers if "flexion" in j]
    groups = [("thumb_d0_abduction", ["thumb_d0_abduction"])]
    for finger in FINGER_NAMES:
        groups.append(
            (f"{finger} flexion (d0-d2)", [j for j in flex if finger in j])
        )
    groups.append(("all finger flexion", flex))
    return groups


def render_finger_sweep(arm: Arm, output: Path, fps: int = 30) -> Path:
    import imageio.v2 as imageio

    m = arm.model
    renderer = mujoco.Renderer(m, 480, 640)
    camera = mujoco.MjvCamera()
    arm.rest()
    # Raise the forearm so the hand is in view, palm toward the camera.
    pose = {"shoulder_flexion": 0.6, "elbow_flexion": 1.2}
    palm = m.body(subject.prefix(SIDE) + "palm").id
    with imageio.get_writer(output, fps=fps, macro_block_size=8) as writer:
        for name, joints in _finger_groups(arm):
            arm.rest()
            arm.set(pose)
            ranges = np.array([arm.range(j) for j in joints])
            for s in (
                _ease(int(fps * 1.2)).tolist()
                + _ease(int(fps * 1.2))[::-1].tolist()
            ):
                # 0 → each joint's max (flexion/abduction) and back.
                arm.set(dict(zip(joints, s * ranges[:, 1], strict=True)))
                camera.lookat[:] = arm.data.xpos[palm] + (0, 0.04, 0)
                camera.distance, camera.azimuth = 0.16, 150
                camera.elevation = -25
                renderer.update_scene(arm.data, camera=camera)
                text = f"{name}  {100 * s:3.0f}% of max"
                writer.append_data(_label(renderer.render(), text))
    renderer.close()
    return output


# --- workspace ------------------------------------------------------------


def sample_workspace(arm: Arm, n: int, seed: int = 0) -> np.ndarray:
    """Fingertip positions for uniform samples over the arm ranges."""
    rng = np.random.default_rng(seed)
    ranges = np.array([arm.range(j) for j in arm.joints])
    qs = rng.uniform(ranges[:, 0], ranges[:, 1], (n, len(arm.joints)))
    arm.rest()
    addrs = [arm.addr(j) for j in arm.joints]
    site = arm.model.site(subject.hand_site(SIDE)).id
    tips = np.empty((n, 3))
    for i, q in enumerate(qs):
        arm.data.qpos[addrs] = q
        mujoco.mj_kinematics(arm.model, arm.data)
        tips[i] = arm.data.site_xpos[site]
    arm.rest()
    return tips


def plot_workspace(arm: Arm, tips: np.ndarray, output: Path) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    glass = load_rig_config()["smartglass"]
    center = arm.object_center
    shoulder = arm.shoulder
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5))
    views = [
        (0, 1, "x (m), subject's right →", "y (m), toward the rig →", "Top"),
        (1, 2, "y (m), toward the rig →", "z (m)", "Side (from the right)"),
    ]
    for ax, (i, j, xlabel, ylabel, title) in zip(axes, views, strict=True):
        ax.hexbin(
            tips[:, i], tips[:, j], gridsize=70, bins="log", cmap="Blues"
        )
        ax.plot(*shoulder[[i, j]], "k^", ms=9, label="right shoulder")
        ax.plot(*center[[i, j]], "o", color="tab:orange", ms=10)
        ax.add_patch(
            plt.Circle(
                center[[i, j]],
                arm.object_size / 2,
                fill=False,
                color="tab:orange",
                label=f"{arm.stimulus} ({100 * arm.object_size:.0f} cm)",
            )
        )
        if i == 0:
            ax.axhline(
                glass["center"][1],
                color="gray",
                ls="--",
                label="smartglass (rig only)",
            )
        else:
            ax.axvline(glass["center"][1], color="gray", ls="--")
        ax.set(xlabel=xlabel, ylabel=ylabel, title=title, aspect="equal")
    axes[0].legend(loc="lower left", fontsize=8)
    fig.suptitle(
        "Right fingertip workspace (arm joints uniform over their ranges, "
        "fingers at rest; darker = more postures)"
    )
    fig.tight_layout()
    fig.savefig(output, dpi=110)
    plt.close(fig)
    return output


# --- best reach and ablation -----------------------------------------------


@dataclass
class Reach:
    """The posture that brings the fingertip closest to the object."""

    free: list[str]
    angles: dict[str, float]  # rad, all arm joints
    tip_to_center: float  # m
    gap: float  # m, arm hull to object; negative = overlap (a touch)
    at_limit: list[str]


def best_reach(
    arm: Arm,
    free: list[str],
    *,
    unlimited: bool = False,
    starts: int = 24,
    seed: int = 0,
) -> Reach:
    """Bring the fingertip as close as possible to the object center.

    Joints not in ``free`` stay at rest (0, or the nearest in-range value).
    Bounded optimisation (L-BFGS-B) from several starts.
    """
    rng = np.random.default_rng(seed)
    fixed = {
        j: float(np.clip(0.0, *arm.range(j)))
        for j in arm.joints
        if j not in free
    }
    bounds = np.array(
        [(-np.pi, np.pi) if unlimited else arm.range(j) for j in free]
    )
    arm.rest()
    arm.set(fixed, mirror=False)
    target = arm.object_center
    addrs = [arm.addr(j) for j in free]
    site = arm.model.site(subject.hand_site(SIDE)).id

    def cost(q):
        arm.data.qpos[addrs] = q
        mujoco.mj_kinematics(arm.model, arm.data)
        return float(np.sum((arm.data.site_xpos[site] - target) ** 2))

    best = None
    inits = [np.clip(0.0, bounds[:, 0], bounds[:, 1])] + [
        rng.uniform(bounds[:, 0], bounds[:, 1]) for _ in range(starts)
    ]
    for q0 in inits:
        result = minimize(cost, q0, method="L-BFGS-B", bounds=bounds)
        if best is None or result.fun < best.fun:
            best = result
    cost(best.x)
    angles = {**fixed, **dict(zip(free, best.x, strict=True))}
    at_limit = [
        j
        for j, q, (lo, hi) in zip(free, best.x, bounds, strict=True)
        if min(q - lo, hi - q) < np.radians(1)
    ]
    reach = Reach(
        free,
        angles,
        float(np.sqrt(best.fun)),
        arm.gap(),
        at_limit,
    )
    arm.rest()
    return reach


def ablation(arm: Arm) -> list[tuple[str, Reach]]:
    j = arm.joints
    cases = [("all 7 arm joints", list(j))]
    cases += [(f"without {x}", [y for y in j if y != x]) for x in j]
    core = ["shoulder_abduction", "shoulder_flexion", "elbow_flexion"]
    cases += [
        ("shoulder abd + flex, elbow (3)", core),
        ("+ shoulder_rotation (4)", core + ["shoulder_rotation"]),
        ("+ wrist_flexion (4)", core + ["wrist_flexion"]),
    ]
    return [(name, best_reach(arm, free)) for name, free in cases]


# --- report ---------------------------------------------------------------


def _deg(angles: dict[str, float]) -> str:
    return ", ".join(f"{k} {np.degrees(v):+.0f}°" for k, v in angles.items())


def write_report(arm: Arm, tips: np.ndarray, out: Path) -> Path:
    lines = ["# Monkey arm report (right arm; the left is its mirror)", ""]
    m = arm.model
    lines += ["## Joints", "", "| Joint | Range (deg) |", "|---|---|"]
    for j in arm.joints + arm.fingers:
        lo, hi = np.degrees(arm.range(j))
        lines.append(f"| {j} | {lo:.0f} … {hi:.0f} |")
    mass = sum(m.body_mass[b] for b in subject.body_ids(m, SIDE))
    lines += ["", f"Mass: {1000 * mass:.0f} g per arm.", ""]

    for stimulus in RL_STIMULI:
        arm.show(stimulus)
        arm.rest()
        center = arm.object_center
        shoulder = arm.shoulder
        dist = np.linalg.norm(tips - center, axis=1)
        reach = np.linalg.norm(tips - shoulder, axis=1)
        lines += [
            f"## Reach: {stimulus}",
            "",
            f"- Shoulder → object center: "
            f"{np.linalg.norm(center - shoulder):.3f} m. Longest "
            f"shoulder → fingertip over {len(tips):,} samples: "
            f"{reach.max():.3f} m.",
            "- Share of uniformly sampled arm postures with the fingertip "
            "within "
            + ", ".join(
                f"{100 * r:.0f} cm: {100 * np.mean(dist < r):.3f}%"
                for r in (0.03, 0.05, 0.10)
            )
            + " of the object center.",
            "",
            "### Best reach and joint ablation",
            "",
            "Joints not listed as free are frozen at rest (0°). *Tip* is "
            "the closest the middle fingertip gets to the object center; "
            "*gap* is the arm's collision hull to the object (negative = "
            "overlap, i.e. a touch, with that much margin).",
            "",
            "| Free joints | Tip → center (cm) | Gap (cm) | Touch | "
            "Joints at a limit |",
            "|---|---|---|---|---|",
        ]
        results = ablation(arm)
        unlimited = best_reach(arm, list(arm.joints), unlimited=True)
        results.append(("all 7, limits removed (±180°)", unlimited))
        for name, r in results:
            lines.append(
                f"| {name} | {100 * r.tip_to_center:.1f} | "
                f"{100 * r.gap:+.1f} | {'yes' if r.gap <= 0 else 'no'} | "
                f"{', '.join(r.at_limit) or '—'} |"
            )
        best = results[0][1]
        lines += [
            "",
            f"Best posture, all 7 free: {_deg(best.angles)}.",
            "",
        ]
    arm.show(RL_STIMULI[0])
    path = out / "report.md"
    path.write_text("\n".join(lines))
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--samples", type=int, default=100_000)
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    arm = Arm.load()
    if not args.no_video:
        print(render_arm_sweep(arm, args.out / "arm_sweep.mp4"))
        print(render_finger_sweep(arm, args.out / "finger_sweep.mp4"))
    tips = sample_workspace(arm, args.samples)
    print(plot_workspace(arm, tips, args.out / "workspace.png"))
    print(write_report(arm, tips, args.out))


if __name__ == "__main__":
    main()
