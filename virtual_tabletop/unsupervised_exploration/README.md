# Reward-free exploration for the monkey arm

Unsupervised RL experiments on the virtual tabletop. The right monkey arm
skeleton (7 arm and 16 finger joints, joint position control) plays with
one block per trial, which is either held in space or resting free on a
pedestal. Every learner works from state (proprioception, touch, object
state) and gets **no task reward**. A supervised shaped-reward run and a
scripted controller serve as references.

Each `version_*/` holds one experiment generation: its `PLAN.md`
(with dated revisions), configs, runs, logs and report (PDFs and
videos). `research/` holds the literature notes.

## Layout

```text
ue/                    the shared package
  paths.py               directory layout (versions, runs, reports)
  sim/                   shapes, hand versions (hand.py), model, env, vec
  learn/                 nets, replay, SAC, noise, training loop
  rewards/               exploration objectives (prediction, coverage,
                         mutual_info, skills, goals, shaped, mix)
  teach/                 scripted IK grasp controller, DEP
  analysis/              evaluation, plots, repertoire, synergies, skill
                         and goal figures, hand_demo (joint-range videos)
  report/                markdown -> PDF, figures/tables, appendix, build
version_0/             first generation (12 Nov): configs, runs, logs,
                       report (6 PDFs + videos), PLAN.md
version_1/             second generation: v1 hand, demonstrations,
                       residual policies
notebooks/             hand_ranges_demo.ipynb (v0 vs v1 joint ranges)
research/              literature notes (shared)
scripts/               queue.sh, eval_runs.sh, autoeval.sh,
                       collect_videos.sh, make_hand_notebook.py, dev tools
```

## Commands

From `virtual_tabletop/unsupervised_exploration/`:

```bash
export VIRTUAL_ENV= PYTHONPATH=$PWD
uv run --project .. python -m ue.learn.train version_1/configs/<...>.yaml
uv run --project .. python -m ue.analysis.evaluate version_1/runs/<name> --video
uv run --project .. python -m ue.analysis.evaluate scripted --version 1
scripts/queue.sh 8 version_1/configs/round1/*.yaml     # at most 8 at once
uv run --project .. python -m ue.report.figures 1
uv run --project .. python -m ue.report.videos_doc   # version 1 video guide
uv run --project .. --with reportlab python -m ue.report.build 1
```

Training writes to the config's version directory. Version 0 is
reproducible with the current code: its configs use the default
`hand: v0`, and evaluation and training were regression-tested as
bit-identical after the refactor. Each run uses about 1.1 CPU cores, so at
most 8 runs at once keeps the machine at or under 10 cores.

## Version 0 results in one paragraph

On held-out shapes, random actions and zero-reward SAC almost never touch
the block (≤ 3% of trials). Reward-free RND on interaction features grasps 67% of fixed and
23% of free blocks, which beats a supervised shaped reward (57% / 8%),
and it explores twice as many contact configurations. Ensemble
disagreement and contact-coverage counts are close behind. Skill methods
(CSF, DIAYN) learn arm-swing or approach primitives that rarely touch the
block. Goal babbling learns a controllable reach primitive (median error
2.2 cm). Lifting free blocks is rare (≤ 3%), while a scripted pinch lifts
43%. See `version_0/report/00_overview.pdf`.

## Version 1 results in one paragraph

Version 1 limits finger extension to natural ranges and couples each DIP
to ⅔ of its PIP (19 actions). It shrinks the blocks to three sizes
(about 3/4/5 cm) and improves the scripted grasp (held-out free-block
lift 43% → 82%). No version 1 policy bends a finger backwards (version 0
RND: 31% of hand-joint steps past natural limits). Reward-free RND now
grasps 75% of fixed and 43% of free blocks, but still lifts only 2%.
Teaching with the scripted controller:
- demonstrations in replay with fading imitation, plus restarts from
  demonstration states (A+B), give 16% lift;
- adding a lift bonus (A+B+D) gives 34% lift (2 seeds), with blocks
  carried 6 cm and turned 0.55 rad. It knocks off about half of the free
  blocks;
- a residual policy on the script (C) breaks the script's lift (3%;
  27% with the bonus). Its correction is as large as the script's
  action.

Teaching widens exploration rather than collapsing it. See
`version_1/report/00_overview.pdf`.
