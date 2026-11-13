"""Commanded reach goals for a goal-babbling run: is "reach" a controllable
primitive?

For 3 held-out trials, the reach module is given 8 goals: palm-to-object
vectors at 4 cm from the block's centre, in 8 directions (4 horizontal,
up, and three oblique). The palm paths are plotted (top and side views)
together with the commanded targets. Also reports the final error.

usage: python -m ue.analysis.goal_reach version_<v>/runs/<name>
Writes version_<v>/report/figures/goalreach_<name>.png.
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

from ue import paths  # noqa: E402
from ue.analysis import plots  # noqa: E402
from ue.analysis.evaluate import Policy  # noqa: E402
from ue.sim.env import PlayEnv  # noqa: E402

run = Path(sys.argv[1])
FIG = paths.version_of(run) / "report" / "figures"
FIG.mkdir(parents=True, exist_ok=True)
torch.set_num_threads(1)
probe = PlayEnv(seed=0, n_shapes=2)
pol = Policy(run, probe)
gm = getattr(pol.module, "goal_module", None) or pol.module
dirs = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1],
                 [1, -1, 1], [-1, -1, 1], [0, -1, 1]], float)
dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
R = 0.04
plots.style()
fig, axes = plt.subplots(3, 2, figsize=(7.5, 9.5))
errs = []
for t, (seed, fixed) in enumerate([(9201, 1.0), (9202, 1.0), (9203, 0.0)]):
    for k, dvec in enumerate(dirs):
        env = PlayEnv(seed=seed, shape_seed=seed, p_fixed=fixed)
        obs, _ = env.reset(seed=seed)
        # obj_rel = object - palm (x5); the palm target is object - goal.
        g = -dvec * R * 5
        z = np.zeros((1, gm.z_dim), np.float32)
        z[0, gm.MODULES.index("reach")] = 1
        z[0, gm.slot["reach"]] = g
        pol.z = z
        path = [env.palm_centre]
        for s in range(env.max_steps):
            obs, *_ = env.step(pol(obs, env))
            path.append(env.palm_centre)
        P = np.array(path)
        oc = env.object_centre
        tgt = oc + dvec * R
        errs.append(np.linalg.norm(P[-1] - tgt))
        c = plots.PALETTE[k]
        for ax, (i, j) in zip(axes[t], [(0, 1), (1, 2)]):
            ax.plot(P[:, i], P[:, j], color=c, lw=1.2, alpha=0.9)
            ax.plot(tgt[i], tgt[j], "x", color=c, ms=7, mew=2)
            ax.plot(P[-1, i], P[-1, j], "o", color=c, ms=4)
            if k == 0:
                ax.plot(oc[i], oc[j], "s", color="#0b0b0b", ms=9,
                        mfc="none", mew=2)
        env.close()
    for ax, lab in zip(axes[t], ["top view: x, y (m)", "side view: y, z (m)"]):
        ax.set_title(f"trial {t + 1} ({'fixed' if fixed else 'free'}): "
                     f"{lab}", loc="left")
        ax.set_aspect("equal")
fig.suptitle(f"{plots.label_of(plots.method_key(run.name))}: commanded "
             f"reach goals 4 cm around the block (x = target, o = end; "
             f"median error {100 * np.median(errs):.1f} cm)", x=0.01,
             ha="left", fontsize=9)
fig.tight_layout()
fig.savefig(FIG / f"goalreach_{run.name}.png")
print("median error cm", 100 * np.median(errs), "frac < 2.5 cm",
      np.mean(np.array(errs) < 0.025))
