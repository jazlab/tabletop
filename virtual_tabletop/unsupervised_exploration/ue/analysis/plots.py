"""Figures for the report: training curves and evaluation comparisons.

Colour follows the method, never its rank: each method has a fixed slot in
the validated reference categorical palette (light mode, since the figures
go into PDFs). Reference policies (supervised shaped reward, scripted,
random, still, DEP) are neutral greys with dashed lines, so the eight hues
are kept for the reward-free learners. One y-axis per panel; small
multiples instead of dual axes.
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import csv  # noqa: E402

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

PALETTE = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4",
           "#008300", "#6250d6", "#e34948"]
GREYS = {"shaped": "#52514e", "scripted": "#0b0b0b", "random": "#9a9893",
         "none": "#b5b3ad",
         "still": "#c3c2b7", "dep": "#7a7873"}
TEXT, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"

#: Fixed colour slot per learned method (by family); variants share it
#: and are told apart by line style.
FAMILY_SLOT = {"contact": 0, "rnd": 1, "touch": 2, "disagreement": 3,
               "music": 4, "csf": 5, "goals": 6, "apt": 7, "diayn": 6,
               "none": 7,
               # Version 1 setups.
               "ab": 0, "res": 2}

NICE = {
    "contact": "Contact coverage (CCGE-lite)",
    "rnd": "RND (interaction)",
    "rnd_full": "RND (full state)",
    "touch": "Tactile surprise",
    "disagreement": "Disagreement (interaction)",
    "music_contact": "MUSIC + contact",
    "music_contact_noclip": "MUSIC + contact (unclipped)",
    "music_contact_clip": "MUSIC + contact (clipped)",
    "music_v3": "MUSIC v3 + contact",
    "goals_v2": "Goal babbling v2 (+contact)",
    "goals_v2_reflex": "Goal babbling v2 + reflex",
    "contact_reflex": "Contact + reflex + curriculum",
    "rnd_pink_dep": "RND + pink noise + DEP",
    "rnd_contact": "RND + contact",
    "rnd_long": "RND (1.4M steps)",
    "rnd_seed1": "RND (seed 1)",
    "contact_seed1": "Contact coverage (seed 1)",
    "rnd_restart": "RND + restarts",
    "contact_restart": "Contact + restarts",
    "rnd_freerestart": "RND + free-block restarts",
    "disagreement_freerestart": "Disagreement + free restarts",
    "shaped_freerestart": "Shaped + free restarts (supervised)",
    "music": "MUSIC",
    "apt": "APT (kNN entropy)",
    "csf": "CSF skills",
    "diayn": "DIAYN skills",
    "goals": "Goal babbling (IMGEP+HER)",
    "none": "Max-entropy (no reward)",
    "shaped": "Shaped reward (supervised)",
    "scripted": "Scripted grasp",
    # Version 1 setups (``_s1`` = seed 1).
    "ab": "A+B: demos + restarts", "ab_s1": "A+B (seed 1)",
    "ab_lift": "A+B+D: + lift bonus", "ab_lift_s1": "A+B+D (seed 1)",
    "res": "C: residual", "res_s1": "C (seed 1)",
    "res_lift": "C+D: residual + lift bonus",
    "rnd_s1": "RND (seed 1)",
    "random": "Random actions",
    "still": "Still",
    "dep": "DEP",
}


def style():
    plt.rcParams.update({
        "font.size": 8.5, "axes.titlesize": 9, "axes.labelsize": 8.5,
        "axes.edgecolor": MUTED, "axes.labelcolor": TEXT,
        "xtick.color": MUTED, "ytick.color": MUTED, "text.color": TEXT,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
        "axes.spines.top": False, "axes.spines.right": False,
        "legend.frameon": False, "figure.dpi": 150, "savefig.dpi": 200,
        "savefig.bbox": "tight", "lines.linewidth": 2.0,
    })


def method_key(name: str) -> str:
    """Run name -> method key (strip round prefix, keep variant)."""
    for p in ("r1_", "r2_", "r3_", "r4_", "r5_", "r6_", "baseline_"):
        if name.startswith(p):
            return name[len(p):]
    return name


def color_of(key: str) -> tuple[str, str]:
    """(colour, linestyle) for a method key; variants get dash styles."""
    for g, c in GREYS.items():
        if key == g or key.startswith(g + "_"):
            return c, "--"
    fam = key.split("_")[0]
    if key.startswith("music"):
        fam = "music"
    slot = FAMILY_SLOT.get(fam, 7)
    variant = key != fam and key not in ("music_contact",)
    return PALETTE[slot], (":" if variant else "-")


def label_of(key: str) -> str:
    return NICE.get(key, key.replace("_", " "))


def smooth(y, k=5):
    """Centred moving average (shorter window at the ends)."""
    y = np.asarray(y, float)
    out = np.empty_like(y)
    h = k // 2
    for i in range(len(y)):
        w = y[max(0, i - h):i + h + 1]
        w = w[~np.isnan(w)]
        out[i] = w.mean() if len(w) else np.nan
    return out


def read_progress(path: Path) -> dict[str, np.ndarray]:
    """progress.csv -> column arrays (missing values as NaN)."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    cols = rows[0].keys() if rows else []
    out = {}
    for c in cols:
        vals = []
        for r in rows:
            try:
                vals.append(float(r[c]))
            except (TypeError, ValueError):
                vals.append(np.nan)
        out[c] = np.array(vals)
    return out


def training_curves(runs: dict[str, Path], metrics: list[tuple[str, str]],
                    out: Path, ncols=3, xmax=None, title=None,
                    by_index=False, style_fn=None):
    """Small multiples: one panel per metric, one line per run.

    ``style_fn``: optional ``name -> (colour, linestyle, label)``, which
    overrides the default styling (e.g. one colour per setup, dashed for
    the second seed).

    ``by_index``: colour runs by their position in ``runs`` (palette
    order) instead of by method family. Used for variant figures, where
    every run belongs to the same family.
    """
    style()
    n = len(metrics)
    nrows = int(np.ceil(n / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(3.0 * ncols,
                                                    2.3 * nrows),
                             squeeze=False)
    for ax, (col, lab) in zip(axes.flat, metrics):
        for name, run in runs.items():
            f = run / "progress.csv"
            if not f.exists():
                continue
            df = read_progress(f)
            if col not in df:
                continue
            key = method_key(name)
            c, ls = color_of(key)
            if by_index:
                i = list(runs).index(name)
                c = PALETTE[i % 8] if key not in GREYS else c
                ls = "-" if i < 8 else "--"
            lab_run = label_of(key)
            if style_fn:
                c, ls, lab_run = style_fn(name)
            ax.plot(df["step"] / 1e3, smooth(df[col]), color=c, ls=ls,
                    lw=1.8, label=lab_run)
        ax.set_title(lab, loc="left")
        ax.set_xlabel("env steps (k)")
        if xmax:
            ax.set_xlim(0, xmax / 1e3)
    for ax in list(axes.flat)[n:]:
        ax.axis("off")
    h, lab = axes.flat[0].get_legend_handles_labels()
    fig.legend(h, lab, loc="lower center", ncol=min(4, len(lab)),
               bbox_to_anchor=(0.5, -0.02 - 0.03 * (len(lab) // 4)))
    if title:
        fig.suptitle(title, x=0.01, ha="left", fontsize=10)
    fig.tight_layout(rect=(0, 0.06 + 0.03 * (len(lab) // 4), 1, 0.97))
    fig.savefig(out)
    plt.close(fig)


def bar_compare(rows: list[dict], metric: str, label: str, out: Path,
                group: str = "all", ax=None, fmt="{:.0%}"):
    """Horizontal bars of one evaluation metric across methods (sorted)."""
    style()
    rows = [r for r in rows if group in r["summary"]]
    rows = sorted(rows, key=lambda r: r["summary"][group][metric])
    own = ax is None
    if own:
        fig, ax = plt.subplots(figsize=(4.2, 0.28 * len(rows) + 0.8))
    for i, r in enumerate(rows):
        v = r["summary"][group][metric]
        c, ls = color_of(r["key"])
        ax.barh(i, v, color=c, height=0.62,
                hatch="///" if ls != "-" else None, edgecolor="white",
                linewidth=0.0)
        ax.text(v, i, " " + fmt.format(v), va="center", fontsize=7.5,
                color=TEXT)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([label_of(r["key"]) for r in rows])
    ax.set_title(label, loc="left")
    ax.grid(axis="y", visible=False)
    if own:
        fig.tight_layout()
        fig.savefig(out)
        plt.close(fig)


#: Ethogram categories, lowest to highest (each trial gets its highest).
ETHO = ["no approach", "approach (<1 cm)", "touch", "pinch", "grasp",
        "lift"]
#: Sequential single-hue ramp (light -> dark blue) for ordered categories.
ETHO_COLORS = ["#e4e3df", "#c6dbf3", "#8fbbe9", "#5797de", "#2a78d6",
               "#174a8a"]


def ethogram_category(r):
    if r.get("any_lift"):
        return 5
    if r["any_grasp"]:
        return 4
    if r["any_pinch"]:
        return 3
    if r["any_touch"]:
        return 2
    if r["min_dist"] < 0.01:
        return 1
    return 0


def ethogram(evals: list[dict], out: Path, group=None, title=None):
    """Stacked horizontal bars: fraction of trials whose highest behaviour
    is each category (a sequential ramp: ordered categories), plus the
    fraction of free blocks knocked off the pedestal (marker)."""
    style()
    rows = []
    for e in evals:
        tr = [r for r in e["trials"] if group is None
              or (group == "fixed") == r["fixed"]]
        cats = np.bincount([ethogram_category(r) for r in tr],
                           minlength=len(ETHO)) / max(len(tr), 1)
        free = [r for r in tr if not r["fixed"]]
        fell = np.mean([r.get("fell", False) for r in free]) if free \
            else np.nan
        rows.append((e["key"], cats, fell))
    rows.sort(key=lambda x: (x[1] * np.arange(len(ETHO))).sum())
    fig, ax = plt.subplots(figsize=(7.2, 0.32 * len(rows) + 1.3))
    for i, (key, cats, fell) in enumerate(rows):
        left = 0.0
        for c, v in enumerate(cats):
            ax.barh(i, v, left=left, color=ETHO_COLORS[c], height=0.7,
                    edgecolor="white", linewidth=1.0,
                    label=ETHO[c] if i == 0 else None)
            left += v
        if not np.isnan(fell):
            ax.plot([fell], [i], marker="v", color="#e34948", ms=6,
                    ls="none", label="free block knocked off"
                    if i == 0 else None)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([label_of(k) for k, _, _ in rows])
    ax.set_xlim(0, 1)
    ax.set_xlabel("fraction of trials (highest behaviour reached)")
    ax.grid(axis="y", visible=False)
    if title:
        ax.set_title(title, loc="left")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12 - 0.3 /
                                                  len(rows)), ncol=4,
              fontsize=7.5)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def skill_heatmap(e: dict, out: Path, metrics=None, title=None):
    """Per-skill means of evaluation metrics (skills on rows)."""
    style()
    metrics = metrics or [("any_touch", "P(touch)"),
                          ("any_pinch", "P(pinch)"),
                          ("any_grasp", "P(grasp)"),
                          ("max_disp", "disp (m)"),
                          ("max_rot", "rot (rad)"),
                          ("min_dist", "min dist (m)")]
    by = {}
    for r in e["trials"]:
        z = np.asarray(r["z"]) if r["z"] else np.zeros(1)
        # Skill index: one-hot (DIAYN) or signed axis (CSF eval: +-e_k).
        j = int(np.argmax(np.abs(z)))
        k = 2 * j + int(z[j] < 0) if len(z) <= 8 else j
        by.setdefault(k, []).append(r)
    keys = sorted(by, key=lambda k: str(k))
    M = np.array([[np.mean([r[m] for r in by[k]]) for m, _ in metrics]
                  for k in keys])
    Mn = (M - M.min(0)) / (np.ptp(M, 0) + 1e-9)
    fig, ax = plt.subplots(figsize=(0.9 * len(metrics) + 1.5,
                                    0.22 * len(keys) + 1.0))
    ax.imshow(Mn, aspect="auto", cmap="Blues", vmin=0, vmax=1.2)
    for i in range(len(keys)):
        for j in range(len(metrics)):
            ax.text(j, i, f"{M[i, j]:.2f}", ha="center", va="center",
                    fontsize=6.5, color=TEXT)
    ax.set_xticks(range(len(metrics)))
    ax.set_xticklabels([lab for _, lab in metrics], rotation=30,
                       ha="right")
    ax.set_yticks(range(len(keys)))
    ax.set_yticklabels([f"skill {k}" if isinstance(k, int) else "z"
                        for k in keys], fontsize=6.5)
    ax.grid(False)
    if title:
        ax.set_title(title, loc="left")
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
