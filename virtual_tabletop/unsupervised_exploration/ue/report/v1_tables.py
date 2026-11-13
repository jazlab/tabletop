"""Version 1 results: tables and figures aggregated over seeds.

usage: python -m ue.report.v1_tables
Writes version_1/report/src/generated_v1_*.md and
version_1/report/figures/v1_*.png.

A *setup* is a run name without its round prefix and seed suffix
(``r1_ab_lift_s1`` -> ``ab_lift``). Tables give the mean over seeds with
each seed's value in brackets. Version 0's RND (``r1_rnd``, v0 hand and
shapes) is included as the "before" reference for the naturalness metrics.
"""

import json
import re

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .. import paths  # noqa: E402
from ..analysis import plots  # noqa: E402

V1 = paths.version_dir(1)
SRC, FIG = V1 / "report" / "src", V1 / "report" / "figures"

LABEL = {
    "rnd": "RND (no teacher)",
    "ab": "A+B: demos + restarts",
    "ab_lift": "A+B+D: + lift bonus",
    "res": "C: residual",
    "res_lift": "C+D: residual + lift bonus",
    "baseline_scripted": "Scripted teacher (v1)",
    "baseline_random": "Random actions",
    "baseline_still": "Still",
    "v0_rnd": "Version 0 RND (v0 hand)",
}
ORDER = ["rnd", "ab", "ab_lift", "res", "res_lift", "baseline_scripted",
         "baseline_random", "baseline_still", "v0_rnd"]
COLOR = {"rnd": plots.PALETTE[1], "ab": plots.PALETTE[0],
         "ab_lift": plots.PALETTE[6], "res": plots.PALETTE[2],
         "res_lift": plots.PALETTE[3], "baseline_scripted": "#0b0b0b",
         "baseline_random": "#9a9893", "baseline_still": "#c3c2b7",
         "v0_rnd": "#b5b3ad"}


def setup_of(name: str) -> str:
    if name.startswith("baseline_"):
        return name
    return re.sub(r"_s\d+$", "", re.sub(r"^r\d+_", "", name))


def load():
    groups = {}
    for d in sorted((V1 / "runs").iterdir()):
        f = d / "eval.json"
        if f.exists() and not d.name.startswith("smoke"):
            groups.setdefault(setup_of(d.name), []).append(
                (d.name, json.loads(f.read_text())))
    # Version 0 RND re-evaluated with the version 1 metrics (same
    # held-out protocol; eval.json itself is left as it was).
    f = paths.runs_dir(0) / "r1_rnd" / "eval_v1metrics.json"
    if f.exists():
        groups["v0_rnd"] = [("v0 r1_rnd", json.loads(f.read_text()))]
    return {k: groups[k] for k in ORDER if k in groups}


def get(e, group, key):
    v = e["summary"].get(group, {}).get(key)
    if v is None and key in ("contact_sets", "contact_cells"):
        v = e["summary"].get(key)
    return v


def cell(runs, group, key, pct=True, fmt="{:.1f}"):
    vals = [get(e, group, key) for _, e in runs]
    vals = [v for v in vals if v is not None]
    if not vals:
        return "–"
    f = (lambda v: f"{100 * v:.0f}%") if pct else (lambda v: fmt.format(v))
    s = f(np.mean(vals))
    if len(vals) > 1:
        s += " (" + ", ".join(f(v) for v in vals) + ")"
    return s


def tables(G):
    rows = ["| Setup | Seeds | Fixed: grasp | Free: grasp | Free: lift | "
            "Free: held at end | Free: knocked off |",
            "|---|---|---|---|---|---|---|"]
    for k, runs in G.items():
        rows.append(f"| {LABEL.get(k, k)} | {len(runs)} | "
                    f"{cell(runs, 'fixed', 'any_grasp')} | "
                    f"{cell(runs, 'free', 'any_grasp')} | "
                    f"{cell(runs, 'free', 'any_lift')} | "
                    f"{cell(runs, 'free', 'held_end')} | "
                    f"{cell(runs, 'free', 'fell')} |")
    (SRC / "generated_v1_main.md").write_text("\n".join(rows))
    rows = ["| Setup | Lifted time (s) | Max height (cm) | Carry while "
            "lifted (cm) | Turn while lifted (rad) | Lift events |",
            "|---|---|---|---|---|---|"]
    for k, runs in G.items():
        lt = [get(e, "free", "lift_frac") for _, e in runs]
        lt_s = ("–" if None in lt else f"{3 * np.mean(lt):.2f}")
        mh = [get(e, "free", "max_height") for _, e in runs]
        mh_s = ("–" if None in mh else f"{100 * np.mean(mh):.1f}")
        cr = [get(e, "free", "carry") for _, e in runs]
        cr_s = ("–" if None in cr else f"{100 * np.mean(cr):.1f}")
        rows.append(f"| {LABEL.get(k, k)} | {lt_s} | {mh_s} | {cr_s} | "
                    f"{cell(runs, 'free', 'turn_lifted', False, '{:.2f}')} "
                    f"| {cell(runs, 'free', 'lift_events', False, '{:.2f}')}"
                    " |")
    (SRC / "generated_v1_manip.md").write_text("\n".join(rows))
    rows = ["| Setup | Hand joints past natural limits | Jerk | Effort | "
            "Residual ratio | Contact sets | Contact cells |",
            "|---|---|---|---|---|---|---|"]
    for k, runs in G.items():
        rows.append(f"| {LABEL.get(k, k)} | "
                    f"{cell(runs, 'all', 'hyper_frac')} | "
                    f"{cell(runs, 'all', 'jerk', False, '{:.3f}')} | "
                    f"{cell(runs, 'all', 'effort', False, '{:.3f}')} | "
                    f"{cell(runs, 'all', 'res_ratio', False, '{:.2f}')} | "
                    f"{cell(runs, 'all', 'contact_sets', False, '{:.0f}')}"
                    f" | {cell(runs, 'all', 'contact_cells', False, '{:.0f}')}"
                    " |")
    (SRC / "generated_v1_natural.md").write_text("\n".join(rows))


def bars(G):
    plots.style()
    panels = [("fixed", "any_grasp", "Fixed blocks: P(grasp)"),
              ("free", "any_grasp", "Free blocks: P(grasp)"),
              ("free", "any_lift", "Free blocks: P(lift)"),
              ("free", "held_end", "Free blocks: P(held at end)")]
    keys = [k for k in G if k != "v0_rnd"]
    fig, axes = plt.subplots(1, 4, figsize=(11, 0.32 * len(keys) + 1.2),
                             sharey=True)
    for ax, (g, m, title) in zip(axes, panels):
        for i, k in enumerate(keys):
            vals = [get(e, g, m) for _, e in G[k]]
            vals = [v for v in vals if v is not None]
            if not vals:
                continue
            ax.barh(i, np.mean(vals), color=COLOR.get(k, "#888"),
                    height=0.62)
            if len(vals) > 1:  # each seed as a dot
                ax.plot(vals, [i] * len(vals), "o", ms=3.5, color="#0b0b0b",
                        mfc="white", mew=1)
            ax.text(max(vals) + 0.02, i, f" {100 * np.mean(vals):.0f}%",
                    va="center", fontsize=7)
        ax.set_xlim(0, 1.15)
        ax.set_title(title, loc="left", fontsize=8.5)
        ax.grid(axis="y", visible=False)
    axes[0].set_yticks(range(len(keys)))
    axes[0].set_yticklabels([LABEL.get(k, k) for k in keys])
    axes[0].invert_yaxis()
    fig.tight_layout()
    fig.savefig(FIG / "v1_bars.png")
    plt.close(fig)


def curves():
    found = [d for d in (V1 / "runs").glob("r1_*")
             if (d / "progress.csv").exists()]
    found.sort(key=lambda d: (ORDER.index(setup_of(d.name)), d.name))
    runs = {d.name: d for d in found}

    def style_fn(name):
        k, s1 = setup_of(name), name.endswith("_s1")
        return (COLOR.get(k, "#888"), "--" if s1 else "-",
                LABEL.get(k, k) + (" (seed 1)" if s1 else ""))

    metrics = [("tr_any_touch", "P(touch)"), ("tr_any_grasp", "P(grasp)"),
               ("tr_free_lift", "P(lift), free blocks"),
               ("tr_free_disp", "free-block displacement (m)"),
               ("bc_lam", "BC weight λ (A)"),
               ("rterm_Smoothness_mean", "smoothness reward (mean)")]
    plots.training_curves(runs, metrics, FIG / "v1_curves.png",
                          title="Version 1 training (training trials, "
                                "including restarted ones for A+B)",
                          style_fn=style_fn)


def scripted_table():
    """Held-out scripted controller: version 0 (v0 controller, hand,
    shapes) vs version 1."""
    rows = ["| Scripted controller (held-out, 120 trials) | Fixed: grasp | "
            "Free: grasp | Free: lift | Free: held at end | "
            "Free: knocked off |", "|---|---|---|---|---|---|"]
    for lab, f in (("version 0 (v0 controller, hand, shapes)",
                    paths.runs_dir(0) / "baseline_scripted" / "eval.json"),
                   ("version 1 (v1 controller, hand, shapes)",
                    V1 / "runs" / "baseline_scripted" / "eval.json")):
        if not f.exists():
            continue
        e = [(f.parent.name, json.loads(f.read_text()))]
        rows.append(f"| {lab} | {cell(e, 'fixed', 'any_grasp')} | "
                    f"{cell(e, 'free', 'any_grasp')} | "
                    f"{cell(e, 'free', 'any_lift')} | "
                    f"{cell(e, 'free', 'held_end')} | "
                    f"{cell(e, 'free', 'fell')} |")
    (SRC / "generated_table_scripted.md").write_text("\n".join(rows))


def main():
    FIG.mkdir(parents=True, exist_ok=True)
    G = load()
    scripted_table()
    tables(G)
    bars(G)
    curves()
    print("setups:", {k: len(v) for k, v in G.items()})


if __name__ == "__main__":
    main()
