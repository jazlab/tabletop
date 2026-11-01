"""Learning-curve plots for a training run (written to the run directory).

``plot_run(run_dir)`` draws ``learning_curves.png`` from ``evals.csv``
(evaluation success rate and time to respond) and ``progress.csv``
(training trial success rate per PPO update). ``vtt-rl-train`` calls it at
the end of a run.
"""

import csv
from pathlib import Path


def _read(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def plot_run(run_dir: Path) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    evals = _read(run_dir / "evals.csv")
    progress = _read(run_dir / "progress.csv")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4))
    if progress:
        ax1.plot(
            [int(r["timesteps"]) / 1e6 for r in progress],
            [100 * float(r["success_rate"]) for r in progress],
            color="0.7",
            lw=1,
            label="training (stochastic)",
        )
    if evals:
        ax1.plot(
            [int(r["timesteps"]) / 1e6 for r in evals],
            [100 * float(r["success_rate"]) for r in evals],
            "o-",
            color="tab:blue",
            label="evaluation (deterministic)",
        )
        timed = [r for r in evals if r["time_to_respond"]]
        ax2.plot(
            [int(r["timesteps"]) / 1e6 for r in timed],
            [float(r["time_to_respond"]) for r in timed],
            "o-",
            color="tab:orange",
        )
    ax1.set(
        xlabel="env steps (millions)",
        ylabel="trials rewarded (%)",
        title="Success rate",
        ylim=(-2, 102),
    )
    ax1.legend(loc="lower right")
    ax2.set(
        xlabel="env steps (millions)",
        ylabel="seconds after the window opens",
        title="Time to respond (rewarded trials, evaluation)",
    )
    for ax in (ax1, ax2):
        ax.grid(alpha=0.3)
    fig.tight_layout()
    path = run_dir / "learning_curves.png"
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path
