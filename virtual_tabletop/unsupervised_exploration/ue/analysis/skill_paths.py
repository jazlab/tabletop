"""Palm trajectories per skill (or goal module) for a run.

For a skill run (CSF: the 8 axis directions; DIAYN: up to 16 skills),
each skill is run on the same 3 held-out trials (2 fixed, 1 free). The
palm-centre paths are plotted in top and side views, coloured by skill,
with the object position marked. Also renders the frame at 2 s of each
skill on the first trial as a contact sheet.

usage: python -m ue.analysis.skill_paths version_<v>/runs/<name> [n_skills]
Writes version_<v>/report/figures/skillpaths_<name>.png and
skillframes_<name>.png.
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from PIL import Image  # noqa: E402

from ue import paths  # noqa: E402
from ue.analysis import plots  # noqa: E402
from ue.analysis.evaluate import Policy, label  # noqa: E402
from ue.sim.env import PlayEnv  # noqa: E402

run = Path(sys.argv[1])
n_sk = int(sys.argv[2]) if len(sys.argv) > 2 else 8
torch.set_num_threads(1)
probe = PlayEnv(seed=0, n_shapes=2)
pol = Policy(run, probe)
if n_sk <= 8:  # the validated categorical palette, fixed order
    def cmap(k):
        return plots.PALETTE[k]
else:
    cmap = plt.get_cmap("tab20")
plots.style()
fig, axes = plt.subplots(3, 2, figsize=(7.5, 9.5))
frames = []
for t, (seed, fixed) in enumerate([(9101, 1.0), (9102, 1.0), (9103, 0.0)]):
    for k in range(n_sk):
        env = PlayEnv(seed=seed, shape_seed=seed, render_size=(200, 240),
                      p_fixed=fixed)
        np.random.seed(k)
        torch.manual_seed(k)
        obs, _ = env.reset(seed=seed)
        pol.new_trial(k, obs)
        path = [env.palm_centre]
        for s in range(env.max_steps):
            obs, *_ = env.step(pol(obs, env))
            path.append(env.palm_centre)
            if t == 0 and s == 49:
                frames.append(label(env.render("persp"), f"skill {k}"))
        P = np.array(path)
        oc = env.object_centre
        for ax, (i, j) in zip(axes[t], [(0, 1), (1, 2)]):
            ax.plot(P[:, i], P[:, j], color=cmap(k), lw=1.4,
                    label=f"skill {k}" if t == 0 and i == 0 else None)
            ax.plot(P[-1, i], P[-1, j], "o", color=cmap(k), ms=4)
            if k == 0:
                ax.plot(oc[i], oc[j], "s", color="#0b0b0b", ms=9,
                        mfc="none", mew=2)
        env.close()
    for ax, lab in zip(axes[t], ["top view: x, y (m)", "side view: y, z (m)"]):
        ax.set_title(f"trial {t + 1} ({'fixed' if fixed else 'free'}): "
                     f"{lab}", loc="left")
        ax.set_aspect("equal")
axes[0, 0].legend(fontsize=6.5, ncol=2, loc="best")
fig.suptitle(f"{plots.label_of(plots.method_key(run.name))}: palm paths "
             "per skill (square = object)", x=0.01, ha="left")
fig.tight_layout()
out = paths.version_of(run) / "report" / "figures"
out.mkdir(parents=True, exist_ok=True)
fig.savefig(out / f"skillpaths_{run.name}.png")
cols = 4
rows = [np.concatenate(frames[i:i + cols], 1)
        for i in range(0, len(frames), cols)]
w = max(r.shape[1] for r in rows)
rows = [np.pad(r, ((0, 0), (0, w - r.shape[1]), (0, 0))) for r in rows]
Image.fromarray(np.concatenate(rows, 0)).save(
    out / f"skillframes_{run.name}.png")
print("ok")
