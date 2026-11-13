"""Environment figures for a version's report (report/figures/env_*.png).

usage: python -m ue.analysis.env_figures VERSION

- env_shapes.png: a gallery of shapes, several per family, each rendered
  alone (held in space) by a camera tracking the object;
- env_trials.png: 8 trial spawns (fixed and free) from the overview camera;
- env_cameras.png: one trial from every camera;
- env_workspace.png: reachable palm positions vs the spawn box.
"""

import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

from ue import paths  # noqa: E402
from ue.analysis.plots import style  # noqa: E402
from ue.sim.env import WORKSPACE, PlayEnv  # noqa: E402
from ue.sim.shapes import FAMILIES  # noqa: E402

OUT = paths.report_dir(sys.argv[1] if len(sys.argv) > 1 else "1") / "figures"
OUT.mkdir(parents=True, exist_ok=True)


def tag(img, text):
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, im.width, 15], fill=(255, 255, 255))
    d.text((4, 2), text, fill=(20, 20, 20))
    return np.asarray(im)


def shape_gallery(per_family=4):
    tiles = {f: [] for f in FAMILIES}
    seed = 0
    while min(len(v) for v in tiles.values()) < per_family and seed < 40:
        env = PlayEnv(seed=seed, shape_seed=seed, render_size=(200, 200),
                      p_fixed=1.0)
        for k, s in enumerate(env.pm.shapes):
            if len(tiles[s.family]) >= per_family:
                continue
            env.reset()
            # Force this shape, fixed, at a standard pose.
            env._set_shape(k, (*env.rng.uniform(0.2, 0.95, 3), 1.0))
            mujoco.mj_forward(env.model, env.data)
            img = env.render("close_side")
            meta = s.meta.get("kind") or (f"n={s.meta['n']}"
                                          if "n" in s.meta else "")
            ext = "x".join(f"{e * 100:.1f}" for e in s.extent)
            tiles[s.family].append(tag(img, f"{s.family} {meta} {ext} cm"))
        env.close()
        seed += 1
    rows = [np.concatenate(tiles[f], 1) for f in FAMILIES]
    Image.fromarray(np.concatenate(rows, 0)).save(OUT / "env_shapes.png")


def trials():
    env = PlayEnv(seed=11, shape_seed=11, render_size=(240, 320))
    imgs = []
    for i in range(8):
        env.p_fixed = float(i % 2 == 0)
        env.reset()
        info = env._obs_and_info()[1]
        imgs.append(tag(env.render("persp"),
                        f"{'fixed' if env.fixed else 'free'} "
                        f"{info['shape']}"))
    grid = np.concatenate([np.concatenate(imgs[:4], 1),
                           np.concatenate(imgs[4:], 1)], 0)
    Image.fromarray(grid).save(OUT / "env_trials.png")
    env.close()


def cameras():
    env = PlayEnv(seed=4, shape_seed=4, render_size=(240, 320), p_fixed=0.0)
    env.reset()
    for _ in range(25):  # raise the arm a little for the picture
        a = np.zeros(23)
        a[0], a[1], a[3] = 0.6, 0.8, 1.0
        env.step(a)
    cams = ["persp", "side", "front", "top", "close", "close_side"]
    imgs = [tag(env.render(c), c) for c in cams]
    grid = np.concatenate([np.concatenate(imgs[:3], 1),
                           np.concatenate(imgs[3:], 1)], 0)
    Image.fromarray(grid).save(OUT / "env_cameras.png")
    env.close()


def workspace():
    style()
    env = PlayEnv(seed=0, n_shapes=2)
    m, d, pm = env.model, mujoco.MjData(env.model), env.pm
    rng = np.random.default_rng(0)
    P = []
    for _ in range(20000):
        d.qpos[pm.qadr[:7]] = rng.uniform(pm.jnt_range[:7, 0],
                                          pm.jnt_range[:7, 1])
        mujoco.mj_kinematics(m, d)
        P.append(d.geom_xpos[pm.palm_geom].copy())
    P = np.array(P)
    fig, axes = plt.subplots(1, 2, figsize=(7.5, 3.2))
    for ax, (i, j, lab) in zip(axes, [(0, 1, "top view (x, y)"),
                                      (1, 2, "side view (y, z)")]):
        ax.scatter(P[:, i], P[:, j], s=1, color="#9a9893", alpha=0.25,
                   rasterized=True, label="reachable palm centre")
        ax.scatter([pm.shoulder[i]], [pm.shoulder[j]], s=40,
                   color="#0b0b0b", label="shoulder")
        box = {0: WORKSPACE["x"], 1: WORKSPACE["y"],
               2: (WORKSPACE["z_platform"][0], WORKSPACE["z_fixed"][1])}
        ax.add_patch(plt.Rectangle((box[i][0], box[j][0]),
                                   box[i][1] - box[i][0],
                                   box[j][1] - box[j][0], fill=False,
                                   lw=2, ec="#2a78d6",
                                   label="object spawn box"))
        ax.set_title(lab, loc="left")
        ax.set_xlabel("xyz"[i] + " (m)")
        ax.set_ylabel("xyz"[j] + " (m)")
        ax.set_aspect("equal")
    axes[0].legend(loc="lower left", fontsize=7)
    fig.tight_layout()
    fig.savefig(OUT / "env_workspace.png")
    plt.close(fig)


if __name__ == "__main__":
    shape_gallery()
    trials()
    cameras()
    workspace()
    print("wrote", OUT)
