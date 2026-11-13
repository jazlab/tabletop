"""Generate report/src/05_appendix.md: every run's config, final training
statistics and evaluation, and an index of the videos in report/videos/."""

import sys

import yaml

from .. import paths
from ..analysis import plots
from ..learn.train import DEFAULTS

DEFAULT_KEYS = ("name", "torch_threads", "log_every", "snapshot_every",
                "buffer")


def last_row(run):
    p = plots.read_progress(run / "progress.csv")
    return {k: v[-1] for k, v in p.items() if len(v)}


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    vdir = paths.version_dir(argv[0] if argv else "1")
    out = ["# Part 5: Appendix: runs, configs, videos", "",
           "Every training run in `runs/`, in launch order: its config (only "
           "the keys that differ from the defaults in `ue/train.py`), its "
           "final training statistics (trials of the last 10k steps), and "
           "its held-out evaluation where one exists.", ""]
    runs = sorted([d for d in (vdir / "runs").iterdir()
                   if d.name[:2] in ("r1", "r2", "r3", "r4", "r5", "r6")
                   and (d / "progress.csv").exists()],
                  key=lambda d: (d / "progress.csv").stat().st_ctime)
    out += ["## Run inventory", "",
            "One row per run. Statistics come from the last logging window "
            "of training (training shapes, exploratory policy). Held-out "
            "results are in Part 4.", "",
            "| Run | Method | Steps | Wall time (min) | Final train "
            "P(touch) | P(grasp) | Status |", "|---|---|---|---|---|---|---|"]
    for d in runs:
        r = last_row(d)
        done = (d / "agent.pt").exists()
        status = "finished" if done else (
            "stopped (diagnosed failure)" if "clip" in d.name else "running")
        out.append(
            f"| `{d.name}` | {plots.label_of(plots.method_key(d.name))} | "
            f"{int(r.get('step', 0)) // 1000}k | "
            f"{r.get('time', 0) / 60:.0f} | "
            f"{100 * r.get('tr_any_touch', 0):.0f}% | "
            f"{100 * r.get('tr_any_grasp', 0):.0f}% | {status} |")
    out += ["", "## Configs", ""]
    for d in runs:
        cfg = yaml.safe_load(open(d / "config.yaml"))
        diff = {k: v for k, v in cfg.items()
                if k not in DEFAULT_KEYS and DEFAULTS.get(k, "§") != v}
        out += [f"**`{d.name}`**", "", "```",
                yaml.safe_dump(diff, sort_keys=False).strip(), "```", ""]
    out += ["## Videos", ""]
    if (vdir / "report/src/06_videos.md").exists():
        out += ["Part 6 (`06_videos.pdf`) lists every video in "
                "`report/videos/`, the model it shows, its layout and the "
                "outcome of each trial shown.", ""]
    else:
        out += ["Videos are in `report/videos/` (also next to each run as "
                "`runs/<name>/video.mp4`). Each shows held-out trials from "
                "the `close` camera, three side by side, labelled with the "
                "trial, fixed or free, the shape family, and TOUCH / GRASP "
                "/ LIFT when they happen.", "",
                "| File | Policy |", "|---|---|"]
        vids = vdir / "report" / "videos"
        for v in sorted(vids.glob("*.mp4")) if vids.exists() else []:
            show = v.stem.startswith("showcase_")
            key = v.stem.replace("video_", "").replace("showcase_", "")
            lab = plots.label_of(plots.method_key(key))
            out.append(f"| `{v.name}` | {lab}"
                       f"{' (showcase)' if show else ''} |")
    (vdir / "report/src/05_appendix.md").write_text("\n".join(out))
    print("ok", len(runs))


if __name__ == "__main__":
    main()
