# Part 5: Appendix: runs, configs, videos

Every training run in `runs/`, in launch order: its config (only the keys that differ from the defaults in `ue/train.py`), its final training statistics (trials of the last 10k steps), and its held-out evaluation where one exists.

## Run inventory

One row per run. Statistics come from the last logging window of training (training shapes, exploratory policy). Held-out results are in Part 4.

| Run | Method | Steps | Wall time (min) | Final train P(touch) | P(grasp) | Status |
|---|---|---|---|---|---|---|
| `r1_rnd` | RND (interaction) | 700k | 80 | 98% | 64% | finished |
| `r1_rnd_s1` | RND (seed 1) | 700k | 81 | 97% | 48% | finished |
| `r1_ab_lift` | A+B+D: + lift bonus | 700k | 91 | 97% | 80% | finished |
| `r1_ab` | A+B: demos + restarts | 700k | 93 | 100% | 79% | finished |
| `r1_res` | C: residual | 700k | 169 | 100% | 77% | finished |
| `r1_res_lift` | C+D: residual + lift bonus | 700k | 169 | 99% | 77% | finished |
| `r1_res_s1` | C (seed 1) | 700k | 168 | 99% | 73% | finished |
| `r1_ab_s1` | A+B (seed 1) | 700k | 137 | 99% | 82% | finished |
| `r1_ab_lift_s1` | A+B+D (seed 1) | 700k | 137 | 98% | 81% | finished |

## Configs

**`r1_rnd`**

```
env:
  hand: v1
  shapes: v1
mix:
- - rnd
  - 1.0
  - feats: interaction
- - smooth
  - 1.0
  - w_effort: 0.1
    w_jerk: 0.2
steps: 700000
```

**`r1_rnd_s1`**

```
env:
  hand: v1
  shapes: v1
mix:
- - rnd
  - 1.0
  - feats: interaction
- - smooth
  - 1.0
  - w_effort: 0.1
    w_jerk: 0.2
seed: 1
steps: 700000
```

**`r1_ab_lift`**

```
demos:
  bc:
  - 1.0
  - 0.0
  - 0.5
  frac: 0.25
  path: version_1/demos/scripted_v1.npz
env:
  demo_restart_p: 0.5
  demo_tmin: 0.8
  demo_trials: 40
  hand: v1
  shapes: v1
mix:
- - rnd
  - 1.0
  - feats: interaction
- - smooth
  - 1.0
  - w_effort: 0.1
    w_jerk: 0.2
- - lift
  - 1.0
  - {}
schedule:
  demo_restart_p:
  - 0.5
  - 0.1
  - 0.6
  demo_tmin:
  - 0.8
  - 0.0
  - 0.6
steps: 700000
```

**`r1_ab`**

```
demos:
  bc:
  - 1.0
  - 0.0
  - 0.5
  frac: 0.25
  path: version_1/demos/scripted_v1.npz
env:
  demo_restart_p: 0.5
  demo_tmin: 0.8
  demo_trials: 40
  hand: v1
  shapes: v1
mix:
- - rnd
  - 1.0
  - feats: interaction
- - smooth
  - 1.0
  - w_effort: 0.1
    w_jerk: 0.2
schedule:
  demo_restart_p:
  - 0.5
  - 0.1
  - 0.6
  demo_tmin:
  - 0.8
  - 0.0
  - 0.6
steps: 700000
```

**`r1_res`**

```
env:
  hand: v1
  residual_scale: 0.5
  shapes: v1
mix:
- - rnd
  - 1.0
  - feats: interaction
- - smooth
  - 1.0
  - w_effort: 0.1
    w_jerk: 0.2
n_workers: 2
steps: 700000
```

**`r1_res_lift`**

```
env:
  hand: v1
  residual_scale: 0.5
  shapes: v1
mix:
- - rnd
  - 1.0
  - feats: interaction
- - smooth
  - 1.0
  - w_effort: 0.1
    w_jerk: 0.2
- - lift
  - 1.0
  - {}
n_workers: 2
steps: 700000
```

**`r1_res_s1`**

```
env:
  hand: v1
  residual_scale: 0.5
  shapes: v1
mix:
- - rnd
  - 1.0
  - feats: interaction
- - smooth
  - 1.0
  - w_effort: 0.1
    w_jerk: 0.2
n_workers: 2
seed: 1
steps: 700000
```

**`r1_ab_s1`**

```
demos:
  bc:
  - 1.0
  - 0.0
  - 0.5
  frac: 0.25
  path: version_1/demos/scripted_v1.npz
env:
  demo_restart_p: 0.5
  demo_tmin: 0.8
  demo_trials: 40
  hand: v1
  shapes: v1
mix:
- - rnd
  - 1.0
  - feats: interaction
- - smooth
  - 1.0
  - w_effort: 0.1
    w_jerk: 0.2
schedule:
  demo_restart_p:
  - 0.5
  - 0.1
  - 0.6
  demo_tmin:
  - 0.8
  - 0.0
  - 0.6
seed: 1
steps: 700000
```

**`r1_ab_lift_s1`**

```
demos:
  bc:
  - 1.0
  - 0.0
  - 0.5
  frac: 0.25
  path: version_1/demos/scripted_v1.npz
env:
  demo_restart_p: 0.5
  demo_tmin: 0.8
  demo_trials: 40
  hand: v1
  shapes: v1
mix:
- - rnd
  - 1.0
  - feats: interaction
- - smooth
  - 1.0
  - w_effort: 0.1
    w_jerk: 0.2
- - lift
  - 1.0
  - {}
schedule:
  demo_restart_p:
  - 0.5
  - 0.1
  - 0.6
  demo_tmin:
  - 0.8
  - 0.0
  - 0.6
seed: 1
steps: 700000
```

## Videos

Part 6 (`06_videos.pdf`) lists every video in `report/videos/`, the model it shows, its layout and the outcome of each trial shown.
