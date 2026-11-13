"""Build report figures and tables from runs/*/eval.json and progress.csv.

usage: python -m ue.report.figures VERSION

Writes report/figures/*.png and report/src/generated_*.md (markdown
tables that the results document includes via {{name}} placeholders; see
build_report.py).
"""

import json
import sys

import imageio
import numpy as np

from .. import paths
from ..analysis import plots

# Set by main() for the chosen version.
VDIR = FIG = SRC = None

#: Method type labels for the tables.
KIND = {
    "contact": "reward-free (touch)", "touch": "reward-free (touch)",
    "rnd": "reward-free", "rnd_full": "reward-free (full state)",
    "disagreement": "reward-free", "apt": "reward-free",
    "music_contact": "reward-free", "csf": "reward-free skills",
    "diayn": "reward-free skills", "goals": "reward-free goals",
    "none": "control (no reward)", "goals_reflex": "reward-free + priors",
    "contact_reflex": "reward-free + priors",
    "rnd_pink_dep": "reward-free + DEP/pink noise",
    "shaped": "**supervised**", "scripted": "hand-coded",
    "shaped_freerestart": "**supervised** + restarts",
    "rnd_freerestart": "reward-free + restarts",
    "disagreement_freerestart": "reward-free + restarts",
    "rnd_restart": "reward-free + restarts",
    "contact_restart": "reward-free + restarts",
    "music_v3": "reward-free", "goals_v2": "reward-free goals",
    "goals_v2_reflex": "reward-free goals + priors",
    "rnd_contact": "reward-free", "rnd_long": "reward-free (2x steps)",
    "rnd_seed1": "reward-free (seed 1)",
    "contact_seed1": "reward-free (touch; seed 1)",
    "random": "control", "still": "control", "dep": "reward-free (no NN)",
}


def load_evals(det=False):
    out = []
    name = "eval_det.json" if det else "eval.json"
    for d in sorted((VDIR / "runs").iterdir()):
        f = d / name
        if not f.exists() or d.name.startswith("smoke"):
            continue
        e = json.loads(f.read_text())
        e["name"], e["key"] = d.name, plots.method_key(d.name)
        e["dir"] = d
        steps = None
        if (d / "progress.csv").exists():
            p = plots.read_progress(d / "progress.csv")
            steps = int(p["step"][-1]) if len(p.get("step", [])) else None
        e["steps"] = steps
        out.append(e)
    return out


def pct(v):
    return "–" if v is None else f"{100 * v:.0f}%"


def main_tables(evals):
    order = sorted(evals, key=lambda e: -(
        e["summary"]["all"]["any_grasp"] + e["summary"]["all"]["any_touch"]
        + 2 * e["summary"].get("free", {}).get("any_lift", 0)))
    head = ("| Method | Type | Steps | Touch | Pinch | Grasp | Contact "
            "time | Time to touch (s) |\n|---|---|---|---|---|---|---|---|")
    fixed, free, cov = [head], [], []
    for e in order:
        s, k = e["summary"], e["key"]
        f = s["fixed"]
        steps = f"{e['steps'] / 1e3:.0f}k" if e["steps"] else "–"
        ttt = f.get("time_to_touch")
        fixed.append(
            f"| {plots.label_of(k)} | {KIND.get(k, '')} | {steps} | "
            f"{pct(f['any_touch'])} | {pct(f['any_pinch'])} | "
            f"{pct(f['any_grasp'])} | {pct(f['touch_frac'])} | "
            f"{'–' if ttt is None else f'{ttt:.2f}'} |")
        g = s["free"]
        free.append(
            f"| {plots.label_of(k)} | {pct(g['any_touch'])} | "
            f"{pct(g['any_pinch'])} | {pct(g['any_grasp'])} | "
            f"{pct(g['any_lift'])} | "
            f"{100 * g.get('disp_kept', g['max_disp']):.1f} | "
            f"{pct(g.get("fell"))} | {g['max_rot']:.2f} |")
        cov.append(f"| {plots.label_of(k)} | {s['contact_cells']} | "
                   f"{s['contact_sets']} | {s['disp_voxels']} |")
    free = ["| Method | Touch | Pinch | Grasp | Lift | Displacement, "
            "kept on pedestal (cm) | Knocked off | Max rotation (rad) |",
            "|---|---|---|---|---|---|---|---|"] + free
    cov = ["| Method | Contact cells | Contact sets | Displacement "
           "voxels |", "|---|---|---|---|"] + cov
    (SRC / "generated_table_fixed.md").write_text("\n".join(fixed))
    (SRC / "generated_table_free.md").write_text("\n".join(free))
    (SRC / "generated_table_coverage.md").write_text("\n".join(cov))


def det_table(evals_det):
    rows = ["| Method | Touch (fixed) | Grasp (fixed) | Touch (free) | "
            "Grasp (free) | Lift (free) |", "|---|---|---|---|---|---|"]
    for e in sorted(evals_det, key=lambda e: e["key"]):
        f, g = e["summary"]["fixed"], e["summary"]["free"]
        rows.append(f"| {plots.label_of(e['key'])} | {pct(f['any_touch'])}"
                    f" | {pct(f['any_grasp'])} | {pct(g['any_touch'])} | "
                    f"{pct(g['any_grasp'])} | {pct(g['any_lift'])} |")
    (SRC / "generated_table_det.md").write_text("\n".join(rows))


def goals_table(evals):
    rows = ["| Run | Module | Trials | Success | Non-trivial goals | "
            "Success (non-trivial) | Start dist | Final dist |",
            "|---|---|---|---|---|---|---|---|"]
    for e in evals:
        gs = e["summary"].get("goals")
        if not gs:
            continue
        for m, v in gs.items():
            snt = v["success_nontrivial"]
            rows.append(
                f"| {plots.label_of(e['key'])} | {m} | {v['n']} | "
                f"{pct(v['success'])} | {v['n_nontrivial']} | "
                f"{pct(snt)} | {v['d_start']:.2f} | {v['d_final']:.2f} |")
    (SRC / "generated_table_goals.md").write_text("\n".join(rows))


def curves():
    runs = {d.name: d for d in sorted((VDIR / "runs").glob("r*_*"))
            if (d / "progress.csv").exists()}
    metrics = [("tr_any_touch", "P(touch) per trial"),
               ("tr_any_pinch", "P(pinch)"), ("tr_any_grasp", "P(grasp)"),
               ("tr_free_lift", "P(lift), free blocks"),
               ("tr_free_disp", "free-block displacement (m)"),
               ("tr_min_dist", "min hand–surface distance (m)")]
    main = {k: v for k, v in runs.items()
            if (k.startswith(("r1_", "r2_")) and "music" not in k)
            or plots.method_key(k) in ("none", "diayn", "goals_v2",
                                       "music_v3")}
    plots.training_curves(main, metrics, FIG / "curves_main.png",
                          title="Training (trial metrics during "
                                "exploration, all trials)")
    rnd_var = {k: v for k, v in runs.items() if plots.method_key(k) in (
        "rnd", "rnd_full", "rnd_long", "rnd_seed1", "rnd_pink_dep",
        "rnd_contact", "rnd_restart", "rnd_freerestart")}
    plots.training_curves(rnd_var, metrics, FIG / "curves_rnd_variants.png",
                          title="RND variants: features, length, seed, "
                                "noise, mixing, restarts", by_index=True)
    other = {k: v for k, v in runs.items() if plots.method_key(k) in (
        "contact", "contact_reflex", "contact_restart", "contact_seed1",
        "goals", "goals_v2", "goals_v2_reflex", "music_contact_noclip",
        "music_contact_clip", "music_v3", "disagreement",
        "disagreement_freerestart", "shaped", "shaped_freerestart")}
    plots.training_curves(other, metrics,
                          FIG / "curves_other_variants.png",
                          title="Contact, goal-babbling, MUSIC, "
                                "disagreement and shaped variants",
                          by_index=True)


#: One representative per method family, plus references (bar chart).
MAIN = ("rnd", "disagreement", "contact", "apt", "touch", "music_v3",
        "csf", "diayn", "goals_v2", "none", "rnd_full", "shaped",
        "scripted", "dep", "random")


def bars(evals):
    import matplotlib.pyplot as plt
    plots.style()
    sel = [e for e in evals if e["key"] in MAIN]
    panels = [("any_grasp", "fixed", "P(grasp), fixed blocks"),
              ("any_grasp", "free", "P(grasp), free blocks"),
              ("any_lift", "free", "P(lift), free blocks"),
              ("fell", "free", "P(knocked off), free blocks")]
    fig, axes = plt.subplots(2, 2, figsize=(10, 0.3 * len(sel) * 2 + 1.5))
    for ax, (m, g, lab) in zip(axes.flat, panels):
        plots.bar_compare(sel, m, lab, None, group=g, ax=ax)
        ax.set_xlim(0, 1.18)
    fig.tight_layout()
    fig.savefig(FIG / "bars_main.png")
    plt.close(fig)


def stills(evals, n_frames=5):
    """A strip of frames from each run's evaluation video."""
    for e in evals:
        v = e["dir"] / "video.mp4"
        if not v.exists():
            continue
        try:
            frames = [f for f in imageio.get_reader(v)]
        except Exception:  # noqa: BLE001
            continue
        # Video pages are 3 trials side by side, 75 frames per page.
        pages = [frames[i:i + 75] for i in range(0, len(frames), 75)][:2]
        rows = []
        for p in pages:
            idx = np.linspace(8, len(p) - 1, n_frames).astype(int)
            rows.append(np.concatenate([p[i] for i in idx], 0))
        strip = np.concatenate(rows, 1)
        imageio.imwrite(FIG / f"stills_{e['name']}.png", strip[::2, ::2])


def main(argv=None):
    global VDIR, FIG, SRC
    argv = sys.argv[1:] if argv is None else argv
    VDIR = paths.version_dir(argv[0] if argv else "1")
    FIG, SRC = VDIR / "report" / "figures", VDIR / "report" / "src"
    FIG.mkdir(parents=True, exist_ok=True)
    SRC.mkdir(parents=True, exist_ok=True)
    evals = load_evals()
    curves()
    if evals:
        main_tables(evals)
        goals_table(evals)
        bars(evals)
        plots.ethogram(evals, FIG / "ethogram_all.png",
                       title="Highest behaviour per trial (all trials)")
        plots.ethogram(evals, FIG / "ethogram_fixed.png", group="fixed",
                       title="Fixed blocks")
        plots.ethogram(evals, FIG / "ethogram_free.png", group="free",
                       title="Free blocks")
        for e in evals:
            if e["key"] in ("csf", "diayn"):
                plots.skill_heatmap(e, FIG / f"skills_{e['name']}.png",
                                    title=plots.label_of(e["key"]))
        stills(evals)
    det = load_evals(det=True)
    if det:
        det_table(det)
    print("figures:", len(list(FIG.glob("*.png"))), "evals:", len(evals))


if __name__ == "__main__":
    main()
