# Part 5: Appendix: runs, configs, videos

Every training run in `runs/`, in launch order: its config (only the keys that differ from the defaults in `ue/train.py`), its final training statistics (trials of the last 10k steps), and its held-out evaluation where one exists.

## Run inventory

One row per run. Statistics come from the last logging window of training (training shapes, exploratory policy). Held-out results are in Part 4.

| Run | Method | Steps | Wall time (min) | Final train P(touch) | P(grasp) | Status |
|---|---|---|---|---|---|---|
| `r2_shaped` | Shaped reward (supervised) | 700k | 55 | 97% | 32% | finished |
| `r1_contact` | Contact coverage (CCGE-lite) | 700k | 58 | 96% | 33% | finished |
| `r2_music_contact_noclip` | MUSIC + contact (unclipped) | 280k | 40 | 5% | 0% | stopped (diagnosed failure) |
| `r2_apt` | APT (kNN entropy) | 700k | 72 | 85% | 12% | finished |
| `r1_rnd` | RND (interaction) | 700k | 86 | 97% | 46% | finished |
| `r2_music_contact_clip` | MUSIC + contact (clipped) | 220k | 29 | 8% | 1% | stopped (diagnosed failure) |
| `r1_csf` | CSF skills | 700k | 88 | 2% | 0% | finished |
| `r3_goals` | Goal babbling (IMGEP+HER) | 700k | 59 | 1% | 0% | finished |
| `r1_touch` | Tactile surprise | 700k | 113 | 38% | 2% | finished |
| `r1_disagreement` | Disagreement (interaction) | 700k | 121 | 98% | 30% | finished |
| `r3_contact_reflex` | Contact + reflex + curriculum | 700k | 59 | 90% | 34% | finished |
| `r4_goals_v2` | Goal babbling v2 (+contact) | 700k | 65 | 56% | 23% | finished |
| `r3_rnd_full` | RND (full state) | 700k | 85 | 84% | 35% | finished |
| `r4_music_v3` | MUSIC v3 + contact | 700k | 97 | 25% | 2% | finished |
| `r5_contact_restart` | Contact + restarts | 700k | 58 | 98% | 41% | finished |
| `r5_rnd_restart` | RND + restarts | 700k | 83 | 98% | 56% | finished |
| `r4_goals_v2_reflex` | Goal babbling v2 + reflex | 700k | 67 | 62% | 16% | finished |
| `r3_rnd_pink_dep` | RND + pink noise + DEP | 700k | 83 | 96% | 35% | finished |
| `r3_none` | Max-entropy (no reward) | 700k | 51 | 9% | 0% | finished |
| `r4_rnd_contact` | RND + contact | 700k | 85 | 95% | 21% | finished |
| `r4_rnd_seed1` | RND (seed 1) | 700k | 83 | 97% | 46% | finished |
| `r3_diayn` | DIAYN skills | 700k | 98 | 23% | 4% | finished |
| `r4_contact_seed1` | Contact coverage (seed 1) | 700k | 59 | 99% | 41% | finished |
| `r6_shaped_freerestart` | Shaped + free restarts (supervised) | 600k | 46 | 90% | 11% | finished |
| `r4_rnd_long` | RND (1.4M steps) | 1400k | 168 | 98% | 49% | finished |
| `r6_rnd_freerestart` | RND + free-block restarts | 600k | 69 | 86% | 28% | finished |
| `r6_disagreement_freerestart` | Disagreement + free restarts | 600k | 92 | 91% | 17% | finished |

## Configs

**`r2_shaped`**

```
method: shaped
steps: 700000
```

**`r1_contact`**

```
method: contact
method_kwargs:
  w_reach: 0.1
  w_set: 0.5
steps: 700000
```

**`r2_music_contact_noclip`**

```
mix:
- - music
  - 1.0
  - {}
- - contact
  - 0.5
  - w_reach: 0.1
    w_set: 0.5
steps: 700000
```

**`r2_apt`**

```
method: apt
method_kwargs:
  feats: interaction
  k: 12
steps: 700000
```

**`r1_rnd`**

```
method: rnd
method_kwargs:
  feats: interaction
steps: 700000
```

**`r2_music_contact_clip`**

```
mix:
- - music
  - 1.0
  - clip_min: 0.0
- - contact
  - 0.5
  - w_reach: 0.1
    w_set: 0.5
steps: 700000
```

**`r1_csf`**

```
method: csf
method_kwargs:
  feats: interaction
  z_dim: 4
steps: 700000
```

**`r3_goals`**

```
method: goals
method_kwargs:
  her: 0.7
steps: 700000
```

**`r1_touch`**

```
method: touch
method_kwargs:
  w_obj: 0.5
steps: 700000
```

**`r1_disagreement`**

```
method: disagreement
method_kwargs:
  feats: interaction
steps: 700000
```

**`r3_contact_reflex`**

```
method: contact
method_kwargs:
  w_reach: 0.1
  w_set: 0.5
schedule:
  p_fixed:
  - 0.8
  - 0.5
  - 0.5
  reflex:
  - 1.0
  - 0.0
  - 0.7
steps: 700000
```

**`r4_goals_v2`**

```
mix:
- - goals
  - 1.0
  - her: 0.7
    reach_radius: 0.3
- - contact
  - 0.5
  - w_reach: 0.1
    w_set: 0.5
steps: 700000
```

**`r3_rnd_full`**

```
method: rnd
method_kwargs:
  feats: all
steps: 700000
```

**`r4_music_v3`**

```
mix:
- - music
  - 1.0
  - clip_min: 0.0
    s_parts:
    - obj_vel
    - wrench
- - contact
  - 0.5
  - w_reach: 0.1
    w_set: 0.5
steps: 700000
```

**`r5_contact_restart`**

```
env:
  restart_p: 0.3
method: contact
method_kwargs:
  w_reach: 0.1
  w_set: 0.5
steps: 700000
```

**`r5_rnd_restart`**

```
env:
  restart_p: 0.3
method: rnd
method_kwargs:
  feats: interaction
steps: 700000
```

**`r4_goals_v2_reflex`**

```
mix:
- - goals
  - 1.0
  - her: 0.7
    reach_radius: 0.3
- - contact
  - 0.5
  - w_reach: 0.1
    w_set: 0.5
schedule:
  p_fixed:
  - 0.8
  - 0.5
  - 0.5
  reflex:
  - 1.0
  - 0.0
  - 0.7
steps: 700000
```

**`r3_rnd_pink_dep`**

```
dep_switch:
  h: 15
  p: 0.01
method: rnd
method_kwargs:
  feats: interaction
noise: pink
steps: 700000
```

**`r3_none`**

```
steps: 700000
```

**`r4_rnd_contact`**

```
mix:
- - rnd
  - 1.0
  - feats: interaction
- - contact
  - 1.0
  - w_reach: 0.1
    w_set: 0.5
steps: 700000
```

**`r4_rnd_seed1`**

```
method: rnd
method_kwargs:
  feats: interaction
seed: 1
steps: 700000
```

**`r3_diayn`**

```
method: diayn
method_kwargs:
  n_skills: 16
  w_rnd: 0.5
steps: 700000
```

**`r4_contact_seed1`**

```
method: contact
method_kwargs:
  w_reach: 0.1
  w_set: 0.5
seed: 1
steps: 700000
```

**`r6_shaped_freerestart`**

```
env:
  p_fixed: 0.25
  restart_free_only: true
  restart_p: 0.5
method: shaped
```

**`r4_rnd_long`**

```
method: rnd
method_kwargs:
  feats: interaction
steps: 1400000
```

**`r6_rnd_freerestart`**

```
env:
  p_fixed: 0.25
  restart_free_only: true
  restart_p: 0.5
method: rnd
method_kwargs:
  feats: interaction
```

**`r6_disagreement_freerestart`**

```
env:
  p_fixed: 0.25
  restart_free_only: true
  restart_p: 0.5
method: disagreement
method_kwargs:
  feats: interaction
```

## Videos

Videos are in `report/videos/` (also next to each run as `runs/<name>/video.mp4`). Each one shows held-out trials (shape libraries never seen in training) from the `close` camera, which tracks the block, three trials side by side. The label shows the trial, fixed or free, the shape family, and TOUCH / GRASP / LIFT when the evaluation metrics detect them.

| File | Policy |
|---|---|
| `showcase_r1_contact.mp4` | showcase r1 contact |
| `showcase_r1_csf.mp4` | showcase r1 csf |
| `showcase_r1_disagreement.mp4` | showcase r1 disagreement |
| `showcase_r1_rnd.mp4` | showcase r1 rnd |
| `showcase_r1_touch.mp4` | showcase r1 touch |
| `showcase_r2_apt.mp4` | showcase r2 apt |
| `showcase_r2_shaped.mp4` | showcase r2 shaped |
| `showcase_r3_diayn.mp4` | showcase r3 diayn |
| `showcase_r3_rnd_full.mp4` | showcase r3 rnd full |
| `showcase_r4_contact_seed1.mp4` | showcase r4 contact seed1 |
| `showcase_r4_goals_v2.mp4` | showcase r4 goals v2 |
| `showcase_r4_goals_v2_reflex.mp4` | showcase r4 goals v2 reflex |
| `showcase_r4_music_v3.mp4` | showcase r4 music v3 |
| `showcase_r4_rnd_long.mp4` | showcase r4 rnd long |
| `showcase_r5_rnd_restart.mp4` | showcase r5 rnd restart |
| `showcase_scripted.mp4` | showcase scripted |
| `video_baseline_dep.mp4` | DEP |
| `video_baseline_random.mp4` | Random actions |
| `video_baseline_scripted.mp4` | Scripted grasp |
| `video_baseline_still.mp4` | Still |
| `video_r1_contact.mp4` | Contact coverage (CCGE-lite) |
| `video_r1_csf.mp4` | CSF skills |
| `video_r1_disagreement.mp4` | Disagreement (interaction) |
| `video_r1_rnd.mp4` | RND (interaction) |
| `video_r1_touch.mp4` | Tactile surprise |
| `video_r2_apt.mp4` | APT (kNN entropy) |
| `video_r2_shaped.mp4` | Shaped reward (supervised) |
| `video_r3_contact_reflex.mp4` | Contact + reflex + curriculum |
| `video_r3_diayn.mp4` | DIAYN skills |
| `video_r3_goals.mp4` | Goal babbling (IMGEP+HER) |
| `video_r3_none.mp4` | Max-entropy (no reward) |
| `video_r3_rnd_full.mp4` | RND (full state) |
| `video_r3_rnd_pink_dep.mp4` | RND + pink noise + DEP |
| `video_r4_contact_seed1.mp4` | Contact coverage (seed 1) |
| `video_r4_goals_v2.mp4` | Goal babbling v2 (+contact) |
| `video_r4_goals_v2_reflex.mp4` | Goal babbling v2 + reflex |
| `video_r4_music_v3.mp4` | MUSIC v3 + contact |
| `video_r4_rnd_contact.mp4` | RND + contact |
| `video_r4_rnd_long.mp4` | RND (1.4M steps) |
| `video_r4_rnd_seed1.mp4` | RND (seed 1) |
| `video_r5_contact_restart.mp4` | Contact + restarts |
| `video_r5_rnd_restart.mp4` | RND + restarts |
| `video_r6_disagreement_freerestart.mp4` | Disagreement + free restarts |
| `video_r6_rnd_freerestart.mp4` | RND + free-block restarts |
| `video_r6_shaped_freerestart.mp4` | Shaped + free restarts (supervised) |