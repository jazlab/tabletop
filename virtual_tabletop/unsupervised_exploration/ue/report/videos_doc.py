"""Version 1 video guide: which model each video shows, and what is in it.

usage: python -m ue.report.videos_doc
Writes version_1/report/src/06_videos.md (built into 06_videos.pdf) and
version_1/report/videos/README.md.

Everything is read from the runs themselves:
- the model, from ``runs/<run>/config.yaml``;
- the per-trial outcomes, from the evaluation file written by the same
  evaluation call that rendered the video:
  - ``eval.json`` for ``video_<run>.mp4``;
  - ``eval_showcase.json`` for ``showcase_<run>.mp4``.
"""

import json
import re

import yaml

from .. import paths
from ..analysis.evaluate import EVAL_SEED
from .v1_tables import LABEL, setup_of

V1 = paths.version_dir(1)
VIDS = V1 / "report" / "videos"

# What each setup is, in one or two sentences.
WHAT = {
    "rnd": "Learned policy (SAC). Reward: RND novelty on hand–object "
           "interaction features, plus small smoothness costs. No teacher, "
           "no task reward.",
    "ab": "Learned policy (SAC). Reward: RND novelty plus smoothness costs. "
          "Taught by the scripted controller through (A) its "
          "demonstrations in replay with fading imitation and (B) trials "
          "restarted from its demonstration states. No task reward.",
    "ab_lift": "As A+B, plus (D) a lift bonus paid while a free block is "
               "lifted. This is the only term that rewards the task.",
    "res": "Learned residual policy (SAC) on top of the scripted "
           "controller (C): each step executes the script's action plus "
           "0.5 × the policy's action. Reward: RND novelty plus "
           "smoothness costs. No task reward.",
    "res_lift": "As C, plus (D) the lift bonus.",
    "baseline_scripted": "Not learned. The hand-coded version 1 "
                         "controller (`ue/teach/scripted.py`): it plans a "
                         "top-down thumb-against-fingers pinch, then lifts. "
                         "It is the teacher for A, B and C.",
    "baseline_random": "Not learned. A new uniformly random action every "
                       "step (a floor for chance contact).",
    "baseline_still": "Not learned. Zero action every step, so the arm "
                      "holds its starting posture (a floor: the block is "
                      "never reached).",
}


def model_of(run: str):
    """(setup, seed, what) for a run directory name."""
    k = setup_of(run)
    seed = None
    f = V1 / "runs" / run / "config.yaml"
    if f.exists() and not run.startswith("baseline_"):
        seed = yaml.safe_load(open(f)).get("seed")
    return k, seed, WHAT.get(k, "")


def trials_of(run: str, showcase: bool):
    f = V1 / "runs" / run / ("eval_showcase.json" if showcase
                             else "eval.json")
    return json.loads(f.read_text())["trials"] if f.exists() else None


def outcome(t):
    if t["fixed"]:
        return "grasp" if t["any_grasp"] else (
            "touch" if t["any_touch"] else "no contact")
    if t["any_lift"]:
        return "lifted, held to the end" if t["held_end"] else (
            "lifted, then dropped")
    if t["fell"]:
        return "knocked off"
    if t["any_grasp"]:
        return "grasp, no lift"
    return "touch" if t["any_touch"] else "no contact"


def videos():
    """[(file, run, showcase)] in a fixed, readable order."""
    out = []
    for v in VIDS.glob("*.mp4"):
        m = re.match(r"(video|showcase)_(.+)\.mp4$", v.name)
        if m:
            out.append((v.name, m.group(2), m.group(1) == "showcase"))
    order = list(LABEL)
    return sorted(out, key=lambda x: (x[2], order.index(setup_of(x[1]))
                                      if setup_of(x[1]) in order else 99,
                                      x[1]))


def seed_str(seed):
    return "–" if seed is None else str(seed)


def main():
    vids = videos()
    ref = trials_of("r1_rnd", False)
    L = ["# Part 6: Video guide", "",
         "This part says which model every video in "
         "`version_1/report/videos/` shows, and how to read it.", "",
         "## Which video is which model", "",
         "| Video file | Model | Seed | Trials shown |",
         "|---|---|---|---|"]
    for name, run, show in vids:
        k, seed, _ = model_of(run)
        L.append(f"| `{name}` | {LABEL.get(k, k)} | {seed_str(seed)} | "
                 f"{'0–7 (showcase, large)' if show else '0–8'} |")
    L += ["", "File names:",
          "- `video_<run>.mp4`: the standard video of training run "
          "`<run>`, 9 trials, 3 side by side.",
          "- `showcase_<run>.mp4`: the same model on 8 trials, 2 side by "
          "side at a larger size. Showcases exist for one seed (seed 0) of "
          "each learned setup.",
          "- Run names: `r1_` is the training round, `_s1` means seed 1 "
          "(no suffix: seed 0). `baseline_` videos are not learned.", "",
          "## What each model is", "",
          "| Model | What it is |", "|---|---|"]
    seen = []
    for _, run, _ in vids:
        k = setup_of(run)
        if k not in seen:
            seen.append(k)
            L.append(f"| {LABEL.get(k, k)} | {WHAT.get(k, '')} |")
    L += ["",
          "Every learned model was trained for 700k environment steps "
          "(about 9,300 trials) on the version 1 hand (natural joint "
          "ranges, 19 actions) and version 1 blocks. Part 3 has the "
          "details.", "",
          "## How to read a video", "",
          "**The trials.**",
          f"- Every video shows the first trials of the held-out "
          f"evaluation: shape libraries {EVAL_SEED}+, which no model saw "
          f"in training.",
          "- **Every video shows the same blocks, in the same order**, so "
          "videos can be compared trial by trial.",
          "- Even trials have a **fixed** block, held in space on a thin "
          "pole. Odd trials have a **free** block, resting on a pedestal.",
          "- Each trial lasts 3 s (75 control steps). The block then "
          "respawns.",
          "- Learned models act **stochastically** (sampled actions), as "
          "they do while exploring and as in the Part 4 tables. A rerun "
          "would not give exactly the same movements.", "",
          "**The layout.**",
          "- A video plays its trials in groups, side by side, left to "
          "right.",
          "- `video_*.mp4`: 3 groups of 3 trials (0–2, 3–5, 6–8), 9 s in "
          "total.",
          "- `showcase_*.mp4`: 4 groups of 2 trials (0–1, 2–3, 4–5, 6–7), "
          "12 s in total.",
          "- The camera (`close`) stays in one place but turns to keep the "
          "block centred, so the background moves when the block moves.",
          "", "**The label strip** at the top of each tile reads "
          "`trial <n> <fixed|free> <shape family>`, followed by the events "
          "happening on that frame:",
          "- **TOUCH:** some part of the hand touches the block.",
          "- **GRASP:** two hand parts push on the block from opposite "
          "sides (an opposed grasp).",
          "- **LIFT:** a free block is more than 2 cm above where it "
          "spawned, off the pedestal, and touched by the hand.", "",
          "## The trials (identical in every video)", "",
          "| Trial | Block | Shape family | video_*: group, position | "
          "showcase_*: group, position |", "|---|---|---|---|---|"]
    pos = ["left", "middle", "right"]
    for t in ref[:9]:
        i = t["trial"]
        sc = (f"{i // 2 + 1}, {'left' if i % 2 == 0 else 'right'}"
              if i < 8 else "not shown")
        L.append(f"| {i} | {'fixed' if t['fixed'] else 'free'} | "
                 f"{t['shape'].replace('_', ' ')} | {i // 3 + 1}, "
                 f"{pos[i % 3]} | {sc} |")
    L += ["", "## Each video in detail", "",
          "What happened in each trial shown, from the same evaluation "
          "that rendered the video:",
          "- **Fixed blocks:** grasp, touch or no contact.",
          "- **Free blocks:** lifted (held to the end, or dropped), "
          "knocked off the pedestal, grasp without a lift, touch, or no "
          "contact.", "",
          "The last line of each entry gives the model's rates over all "
          "120 held-out trials (the Part 4 numbers for that run), because "
          "9 trials are too few to judge a model.", ""]
    for name, run, show in vids:
        k, seed, what = model_of(run)
        tr = trials_of(run, show)
        L += [f"### `{name}`", "",
              f"**Model:** {LABEL.get(k, k)}"
              + (f", seed {seed}" if seed is not None else "")
              + f" (run `{run}`). {what}", ""]
        if tr is None:
            L += ["(No evaluation file found.)", ""]
            continue
        n = 8 if show else 9
        L += ["| Trial | Block | Shape | Outcome |", "|---|---|---|---|"]
        for t in tr[:n]:
            L.append(f"| {t['trial']} | "
                     f"{'fixed' if t['fixed'] else 'free'} | "
                     f"{t['shape'].replace('_', ' ')} | {outcome(t)} |")
        full = trials_of(run, False)
        if full:
            fx = [t for t in full if t["fixed"]]
            fr = [t for t in full if not t["fixed"]]

            def pct(xs, key):
                return f"{100 * sum(bool(x[key]) for x in xs) / len(xs):.0f}%"

            L += ["", f"All 120 held-out trials: fixed grasp "
                  f"{pct(fx, 'any_grasp')}; free grasp "
                  f"{pct(fr, 'any_grasp')}, lift {pct(fr, 'any_lift')}, "
                  f"held at end {pct(fr, 'held_end')}, knocked off "
                  f"{pct(fr, 'fell')}.", ""]
    (V1 / "report" / "src" / "06_videos.md").write_text("\n".join(L))

    # A short index next to the videos themselves.
    R = ["# Version 1 videos", "",
         "The full guide is `../06_videos.pdf` (Part 6 of the report).", "",
         "- Every video shows the **same held-out blocks in the same "
         "order**:",
         "  - even trials: a fixed block, held on a pole;",
         "  - odd trials: a free block, on a pedestal.",
         "- Trials play in groups, side by side, left to right:",
         "  - `video_*`: 3 at a time, trials 0–8;",
         "  - `showcase_*`: 2 at a time, larger, trials 0–7.",
         "- The label on each tile gives the trial, fixed/free and shape, "
         "plus TOUCH / GRASP / LIFT while they happen.", "",
         "| File | Model | Seed |", "|---|---|---|"]
    for name, run, show in vids:
        k, seed, _ = model_of(run)
        R.append(f"| `{name}` | {LABEL.get(k, k)}"
                 f"{' (showcase)' if show else ''} | {seed_str(seed)} |")
    R += ["", "Models:", ""]
    for k in seen:
        R.append(f"- **{LABEL.get(k, k)}:** {WHAT.get(k, '')}")
    (VIDS / "README.md").write_text("\n".join(R) + "\n")
    print("ok", len(vids))


if __name__ == "__main__":
    main()
