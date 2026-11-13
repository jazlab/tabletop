"""Behaviour repertoire: cluster evaluation trials of all methods.

Each evaluation trial is summarised by a behaviour vector:
- contact time;
- opposed-grasp time;
- lifted time;
- mean active patches;
- displacement and rotation (clipped);
- knocked off;
- palm path length;
- minimum hand–surface distance;
- the fraction of time each hand unit (palm, thumb, index, middle, ring,
  pinky) touches the object.

The vectors are standardised and the trials of all methods (and
baselines) are pooled and clustered with k-means. Each cluster is a
post-hoc "primitive".

Writes:
- report/figures/repertoire.png (centroid profiles + per-method usage);
- report/src/generated_table_repertoire.md (per-method entropy: the
  diversity of the repertoire).

usage: python -m ue.analysis.repertoire VERSION [K]   (needs scikit-learn:
    uv run --with scikit-learn ...)
"""

import json
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from sklearn.cluster import KMeans  # noqa: E402

from ue import paths  # noqa: E402
from ue.analysis import plots  # noqa: E402

VDIR = paths.version_dir(sys.argv[1] if len(sys.argv) > 1 else "1")
K = int(sys.argv[2]) if len(sys.argv) > 2 else 7
FEATS = ["contact", "grasp", "lift", "patches", "disp", "rot", "fell",
         "path", "min dist", "palm", "thumb", "index", "middle", "ring",
         "pinky"]


def vec(r):
    u = r.get("unit_use", [0] * 6)
    return [r["touch_frac"], r["grasp_frac"], r["lift_frac"],
            r["mean_ntouch"] / 6, min(r["max_disp"], 0.2) / 0.2,
            min(r["max_rot"], np.pi) / np.pi, float(r.get("fell", 0)),
            min(r.get("palm_path", 0), 2.0) / 2.0,
            min(r["min_dist"], 0.05) / 0.05, *u]


def main():
    data, owner = [], []
    for d in sorted((VDIR / "runs").iterdir()):
        f = d / "eval.json"
        if not f.exists() or d.name.startswith("smoke"):
            continue
        e = json.loads(f.read_text())
        if "unit_use" not in e["trials"][0]:
            continue
        for r in e["trials"]:
            data.append(vec(r))
            owner.append(plots.method_key(d.name))
    X = np.array(data)
    mu, sd = X.mean(0), X.std(0) + 1e-6
    km = KMeans(K, n_init=20, random_state=0).fit((X - mu) / sd)
    lab = km.labels_
    C = np.array([X[lab == k].mean(0) for k in range(K)])
    # Order clusters by "engagement" (contact + grasp + lift + motion).
    eng = C[:, 0] + 2 * C[:, 1] + 4 * C[:, 2] + C[:, 4]
    order = np.argsort(eng)
    remap = {old: new for new, old in enumerate(order)}
    lab = np.array([remap[v] for v in lab])
    C = C[order]
    keys = sorted(set(owner), key=lambda k: -np.mean(
        [eng[order][lab[i]] for i in range(len(owner)) if owner[i] == k]))
    U = np.array([[np.mean(lab[np.array(owner) == k] == c)
                   for c in range(K)] for k in keys])
    H = [-(u[u > 0] * np.log(u[u > 0])).sum() for u in U]
    plots.style()
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 0.3 * len(keys) + 2.5),
                                 gridspec_kw={"width_ratios": [1.1, 1]})
    a1.imshow(C, aspect="auto", cmap="Blues", vmin=0, vmax=1)
    for i in range(K):
        for j in range(len(FEATS)):
            a1.text(j, i, f"{C[i, j]:.2f}", ha="center", va="center",
                    fontsize=6)
    a1.set_xticks(range(len(FEATS)))
    a1.set_xticklabels(FEATS, rotation=45, ha="right")
    a1.set_yticks(range(K))
    a1.set_yticklabels([f"B{k}" for k in range(K)])
    a1.set_title("Behaviour clusters: mean features (0–1 scaled)",
                 loc="left")
    a1.grid(False)
    a2.imshow(U, aspect="auto", cmap="Blues", vmin=0, vmax=1)
    for i in range(len(keys)):
        for j in range(K):
            if U[i, j] >= 0.01:
                a2.text(j, i, f"{100 * U[i, j]:.0f}", ha="center",
                        va="center", fontsize=6)
    a2.set_xticks(range(K))
    a2.set_xticklabels([f"B{k}" for k in range(K)])
    a2.set_yticks(range(len(keys)))
    a2.set_yticklabels([f"{plots.label_of(k)} (H={h:.2f})"
                        for k, h in zip(keys, H)], fontsize=7)
    a2.set_title("Trials per cluster (%), by method", loc="left")
    a2.grid(False)
    fig.tight_layout()
    fig.savefig(VDIR / "report/figures/repertoire.png")
    rows = ["| Cluster | " + " | ".join(FEATS) + " |",
            "|---|" + "---|" * len(FEATS)]
    for k in range(K):
        rows.append(f"| B{k} | " + " | ".join(f"{v:.2f}" for v in C[k])
                    + " |")
    rows += ["", "| Method | Repertoire entropy (nats) | Clusters used "
             "(≥5% of trials) |", "|---|---|---|"]
    for k, h, u in sorted(zip(keys, H, U), key=lambda x: -x[1]):
        used = ", ".join(f"B{c}" for c in range(K) if u[c] >= 0.05)
        rows.append(f"| {plots.label_of(k)} | {h:.2f} | {used} |")
    (VDIR / "report/src/generated_table_repertoire.md").write_text(
        "\n".join(rows))
    print("trials", len(X), "methods", len(keys))


if __name__ == "__main__":
    main()
