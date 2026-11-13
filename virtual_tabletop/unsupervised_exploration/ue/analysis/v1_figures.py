"""Version 1 report figures for the hand, the shapes and the scripted
controller.

usage: python -m ue.analysis.v1_figures
Writes version_1/report/figures/:
- hand_ranges.png: v0 vs v1 joint ranges of the 16 hand joints;
- hand_random.png: random hand postures, v0 (top) vs v1 (bottom);
- hand_extremes.png: "bent back" and fist extremes, v0 vs v1;
- shapes_v1.png: v1 shape library (base shapes x 3 sizes);
- scripted_v1.png: stills from the improved scripted grasp.
"""

import imageio
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

from .. import paths  # noqa: E402
from ..sim.env import PlayEnv  # noqa: E402
from . import plots  # noqa: E402
from .hand_demo import HandPoser  # noqa: E402

OUT = paths.report_dir(1) / "figures"


def tag(img, text):
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, im.width, 14], fill=(255, 255, 255))
    d.text((3, 1), text, fill=(20, 20, 20))
    return np.asarray(im)


def ranges(posers):
    plots.style()
    joints = posers["v0"].names[7:]
    fig, ax = plt.subplots(figsize=(7.5, 5.2))
    for i, j in enumerate(joints):
        for k, (name, c) in enumerate([("v0", "#9a9893"),
                                       ("v1", plots.PALETTE[0])]):
            lo, hi = posers[name].range_deg(j)
            ax.plot([lo, hi], [i + 0.18 * (k - 0.5) * 2] * 2, color=c,
                    lw=4, solid_capstyle="butt",
                    label=name if i == 0 else None)
    ax.axvline(0, color="#0b0b0b", lw=0.8)
    ax.axvspan(-62, 0, color="#e34948", alpha=0.06)
    ax.text(-58, len(joints) - 0.3, "bending back (hyperextension)",
            fontsize=7.5, color="#e34948")
    ax.text(5, len(joints) - 0.3, "flexion (toward the palm)",
            fontsize=7.5, color="#52514e")
    ax.set_yticks(range(len(joints)))
    ax.set_yticklabels(joints, fontsize=7.5)
    ax.invert_yaxis()
    ax.set_xlabel("joint range (deg)")
    ax.set_title("Hand joint ranges: v0 (arm file) vs v1", loc="left")
    ax.legend(loc="lower right")
    ax.grid(axis="y", visible=False)
    fig.tight_layout()
    fig.savefig(OUT / "hand_ranges.png")
    plt.close(fig)


def random_postures(posers):
    rng = np.random.default_rng(0)
    rows = []
    for name, p in posers.items():
        joints = p.names[7:]
        tiles = []
        for _ in range(6):
            u = rng.uniform(0, 1, len(joints))
            pose = {j: p.range_deg(j)[0] + ui * (p.range_deg(j)[1]
                                                  - p.range_deg(j)[0])
                    for j, ui in zip(joints, u)}
            p.pose(pose)
            p.look("index_d0_flexion")
            tiles.append(tag(p.render()[::2, ::2], name))
        rows.append(np.concatenate(tiles, 1))
    Image.fromarray(np.concatenate(rows, 0)).save(OUT / "hand_random.png")


def extremes(posers):
    tiles = []
    for which, lab in ((0, "bent back (lower limits)"),
                       (1, "fist (upper limits)")):
        for name, p in posers.items():
            p.pose({j: p.range_deg(j)[which] for j in p.names[7:]})
            for view, vlab in (("index_d0_flexion", "side"),
                               (None, "palm")):
                p.look(view, view=None if view else [0, -1, 0.3])
                tiles.append(tag(p.render()[::2, ::2],
                                 f"{name}: {lab}, {vlab}"))
    rows = [np.concatenate(tiles[i:i + 4], 1) for i in range(0, 8, 4)]
    Image.fromarray(np.concatenate(rows, 0)).save(OUT / "hand_extremes.png")


def shapes_gallery():
    env = PlayEnv(seed=9000, shape_seed=9000, render_size=(170, 170),
                  hand="v1", shapes="v1", p_fixed=1.0)
    rows = []
    shapes = env.pm.shapes
    for b in range(0, len(shapes), 3):
        tiles = []
        for k in range(3):
            idx = b + k
            env.reset()
            env._set_shape(idx, (*plots_colour(b // 3), 1.0))
            mujoco.mj_forward(env.model, env.data)
            s = shapes[idx]
            tiles.append(tag(env.render("close_side"),
                             f"{s.family} {'SML'[k]} "
                             f"{100 * s.extent.max():.1f} cm"))
        rows.append(np.concatenate(tiles, 1))
    half = (len(rows) + 1) // 2
    left = np.concatenate(rows[:half], 0)
    right = np.concatenate(rows[half:], 0)
    if right.shape[0] < left.shape[0]:
        right = np.pad(right, ((0, left.shape[0] - right.shape[0]), (0, 0),
                               (0, 0)), constant_values=255)
    Image.fromarray(np.concatenate([left, right], 1)).save(
        OUT / "shapes_v1.png")


def plots_colour(i):
    c = plots.PALETTE[i % 8].lstrip("#")
    return [int(c[k:k + 2], 16) / 255 for k in (0, 2, 4)]


def scripted_stills():
    v = paths.version_dir(1) / "runs" / "scripted_v1_demo.mp4"
    if not v.exists():
        return
    frames = [f for f in imageio.get_reader(v)]
    n = len(frames) // 8  # 8 trials in the demo
    rows = []
    for t in range(4):
        tr = frames[t * n:(t + 1) * n]
        idx = np.linspace(2, len(tr) - 1, 5).astype(int)
        rows.append(np.concatenate([tr[i] for i in idx], 1))
    img = np.concatenate(rows, 0)
    Image.fromarray(img[::2, ::2]).save(OUT / "scripted_v1.png")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    posers = {"v0": HandPoser("v0", size=(300, 300)),
              "v1": HandPoser("v1", size=(300, 300))}
    ranges(posers)
    random_postures(posers)
    extremes(posers)
    shapes_gallery()
    scripted_stills()
    print("wrote", sorted(p.name for p in OUT.glob("*.png")))


if __name__ == "__main__":
    main()
