"""Emergent finger synergies: PCA of finger postures during contact.

For each policy, 40 held-out trials are run. The 16 finger joint angles
(scaled to [-1, 1] by range) are collected on every step where the hand
touches the block; for random actions and DEP, every step is used. PCA
gives the variance explained by the first k components (low = many
independent finger DoFs; high = coordinated synergies). The loadings of
the first three components are shown for the chosen policies.

usage: python -m ue.analysis.synergies version_<v>/runs/<name>... [random]
    [dep]
Writes version_<v>/report/figures/synergies.png and
report/src/generated_table_synergies.md.
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
from ue.analysis.evaluate import Baseline, Policy  # noqa: E402
from ue.sim.env import PlayEnv  # noqa: E402

torch.set_num_threads(1)
targets = sys.argv[1:]
VDIR = next(paths.version_of(t) for t in targets
            if t not in ("random", "dep"))
(VDIR / "report/figures").mkdir(parents=True, exist_ok=True)
(VDIR / "report/src").mkdir(parents=True, exist_ok=True)
probe = PlayEnv(seed=0, n_shapes=2)
names = [n.split("/")[-1] for n in probe.pm.joint_names[7:]]
res = {}
for tg in targets:
    if tg in ("random", "dep"):
        pol, key = Baseline(tg, probe), tg
    else:
        pol, key = Policy(Path(tg), probe), plots.method_key(Path(tg).name)
    X = []
    np.random.seed(0)
    for i in range(40):
        if i % 10 == 0:
            env = PlayEnv(seed=9500 + i // 10, shape_seed=9500 + i // 10)
        env.p_fixed = float(i % 2 == 0)
        obs, info = env.reset()
        pol.new_trial(i, obs)
        for _ in range(env.max_steps):
            obs, _, _, _, info = env.step(pol(obs, env))
            if info["hand_touching"] or tg in ("random", "dep"):
                X.append(obs[env.observation_layout["qpos"]][7:])
    X = np.array(X)
    if len(X) < 50:
        print(key, "too few contact steps", len(X))
        continue
    Xc = X - X.mean(0)
    U, S, Vt = np.linalg.svd(Xc, full_matrices=False)
    ev = S ** 2 / (S ** 2).sum()
    res[key] = dict(n=len(X), ev=ev, Vt=Vt, std=X.std(0).mean())
    print(key, len(X), np.round(np.cumsum(ev)[:4], 2))

plots.style()
show = [k for k in res if k not in ("random", "dep")][:3]
fig = plt.figure(figsize=(10, 3.2 + 1.6 * len(show)))
gs = fig.add_gridspec(1 + len(show), 1, height_ratios=[2.2] + [1.4] *
                      len(show))
ax = fig.add_subplot(gs[0])
for k, v in res.items():
    c, ls = plots.color_of(k)
    ax.plot(np.arange(1, 17), np.cumsum(v["ev"]), color=c, ls=ls,
            marker="o", ms=3, label=plots.label_of(k))
ax.set_xlabel("number of principal components")
ax.set_ylabel("cumulative variance")
ax.set_title("Finger posture dimensionality during contact (16 joints)",
             loc="left")
ax.legend(fontsize=7, ncol=2, loc="lower right")
for r, k in enumerate(show):
    a = fig.add_subplot(gs[1 + r])
    L = res[k]["Vt"][:3]
    a.imshow(L, cmap="RdBu_r", vmin=-0.6, vmax=0.6, aspect="auto")
    a.set_yticks(range(3))
    a.set_yticklabels([f"PC{i + 1} ({100 * res[k]['ev'][i]:.0f}%)"
                       for i in range(3)], fontsize=7)
    a.set_xticks(range(16))
    a.set_xticklabels(names if r == len(show) - 1 else [], rotation=60,
                      ha="right", fontsize=6.5)
    a.set_title(f"{plots.label_of(k)}: loadings (red +, blue −)",
                loc="left", fontsize=8)
    a.grid(False)
fig.tight_layout()
fig.savefig(VDIR / "report/figures/synergies.png")
rows = ["| Policy | Samples | PC1 | PC1–3 | PC1–5 | Components for 90% | "
        "Mean finger SD |", "|---|---|---|---|---|---|---|"]
for k, v in res.items():
    c = np.cumsum(v["ev"])
    rows.append(f"| {plots.label_of(k)} | {v['n']} | {c[0]:.2f} | "
                f"{c[2]:.2f} | {c[4]:.2f} | {int(np.searchsorted(c, 0.9)) + 1}"
                f" | {v['std']:.2f} |")
(VDIR / "report/src/generated_table_synergies.md").write_text(
    "\n".join(rows))
