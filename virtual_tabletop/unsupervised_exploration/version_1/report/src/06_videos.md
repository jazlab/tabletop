# Part 6: Video guide

This part says which model every video in `version_1/report/videos/` shows, and how to read it.

## Which video is which model

| Video file | Model | Seed | Trials shown |
|---|---|---|---|
| `video_r1_rnd.mp4` | RND (no teacher) | 0 | 0–8 |
| `video_r1_rnd_s1.mp4` | RND (no teacher) | 1 | 0–8 |
| `video_r1_ab.mp4` | A+B: demos + restarts | 0 | 0–8 |
| `video_r1_ab_s1.mp4` | A+B: demos + restarts | 1 | 0–8 |
| `video_r1_ab_lift.mp4` | A+B+D: + lift bonus | 0 | 0–8 |
| `video_r1_ab_lift_s1.mp4` | A+B+D: + lift bonus | 1 | 0–8 |
| `video_r1_res.mp4` | C: residual | 0 | 0–8 |
| `video_r1_res_s1.mp4` | C: residual | 1 | 0–8 |
| `video_r1_res_lift.mp4` | C+D: residual + lift bonus | 0 | 0–8 |
| `video_baseline_scripted.mp4` | Scripted teacher (v1) | – | 0–8 |
| `video_baseline_random.mp4` | Random actions | – | 0–8 |
| `video_baseline_still.mp4` | Still | – | 0–8 |
| `showcase_r1_rnd.mp4` | RND (no teacher) | 0 | 0–7 (showcase, large) |
| `showcase_r1_ab.mp4` | A+B: demos + restarts | 0 | 0–7 (showcase, large) |
| `showcase_r1_ab_lift.mp4` | A+B+D: + lift bonus | 0 | 0–7 (showcase, large) |
| `showcase_r1_res.mp4` | C: residual | 0 | 0–7 (showcase, large) |
| `showcase_r1_res_lift.mp4` | C+D: residual + lift bonus | 0 | 0–7 (showcase, large) |

File names:
- `video_<run>.mp4`: the standard video of training run `<run>`, 9 trials, 3 side by side.
- `showcase_<run>.mp4`: the same model on 8 trials, 2 side by side at a larger size. Showcases exist for one seed (seed 0) of each learned setup.
- Run names: `r1_` is the training round, `_s1` means seed 1 (no suffix: seed 0). `baseline_` videos are not learned.

## What each model is

| Model | What it is |
|---|---|
| RND (no teacher) | Learned policy (SAC). Reward: RND novelty on hand–object interaction features, plus small smoothness costs. No teacher, no task reward. |
| A+B: demos + restarts | Learned policy (SAC). Reward: RND novelty plus smoothness costs. Taught by the scripted controller through (A) its demonstrations in replay with fading imitation and (B) trials restarted from its demonstration states. No task reward. |
| A+B+D: + lift bonus | As A+B, plus (D) a lift bonus paid while a free block is lifted. This is the only term that rewards the task. |
| C: residual | Learned residual policy (SAC) on top of the scripted controller (C): each step executes the script's action plus 0.5 × the policy's action. Reward: RND novelty plus smoothness costs. No task reward. |
| C+D: residual + lift bonus | As C, plus (D) the lift bonus. |
| Scripted teacher (v1) | Not learned. The hand-coded version 1 controller (`ue/teach/scripted.py`): it plans a top-down thumb-against-fingers pinch, then lifts. It is the teacher for A, B and C. |
| Random actions | Not learned. A new uniformly random action every step (a floor for chance contact). |
| Still | Not learned. Zero action every step, so the arm holds its starting posture (a floor: the block is never reached). |

Every learned model was trained for 700k environment steps (about 9,300 trials) on the version 1 hand (natural joint ranges, 19 actions) and version 1 blocks. Part 3 has the details.

## How to read a video

**The trials.**
- Every video shows the first trials of the held-out evaluation: shape libraries 9000+, which no model saw in training.
- **Every video shows the same blocks, in the same order**, so videos can be compared trial by trial.
- Even trials have a **fixed** block, held in space on a thin pole. Odd trials have a **free** block, resting on a pedestal.
- Each trial lasts 3 s (75 control steps). The block then respawns.
- Learned models act **stochastically** (sampled actions), as they do while exploring and as in the Part 4 tables. A rerun would not give exactly the same movements.

**The layout.**
- A video plays its trials in groups, side by side, left to right.
- `video_*.mp4`: 3 groups of 3 trials (0–2, 3–5, 6–8), 9 s in total.
- `showcase_*.mp4`: 4 groups of 2 trials (0–1, 2–3, 4–5, 6–7), 12 s in total.
- The camera (`close`) stays in one place but turns to keep the block centred, so the background moves when the block moves.

**The label strip** at the top of each tile reads `trial <n> <fixed|free> <shape family>`, followed by the events happening on that frame:
- **TOUCH:** some part of the hand touches the block.
- **GRASP:** two hand parts push on the block from opposite sides (an opposed grasp).
- **LIFT:** a free block is more than 2 cm above where it spawned, off the pedestal, and touched by the hand.

## The trials (identical in every video)

| Trial | Block | Shape family | video_*: group, position | showcase_*: group, position |
|---|---|---|---|---|
| 0 | fixed | prism | 1, left | 1, left |
| 1 | free | random hull | 1, middle | 1, right |
| 2 | fixed | random hull | 1, right | 2, left |
| 3 | free | compound | 2, left | 2, right |
| 4 | fixed | platonic | 2, middle | 3, left |
| 5 | free | platonic | 2, right | 3, right |
| 6 | fixed | box | 3, left | 4, left |
| 7 | free | random hull | 3, middle | 4, right |
| 8 | fixed | cylinder | 3, right | not shown |

## Each video in detail

What happened in each trial shown, from the same evaluation that rendered the video:
- **Fixed blocks:** grasp, touch or no contact.
- **Free blocks:** lifted (held to the end, or dropped), knocked off the pedestal, grasp without a lift, touch, or no contact.

The last line of each entry gives the model's rates over all 120 held-out trials (the Part 4 numbers for that run), because 9 trials are too few to judge a model.

### `video_r1_rnd.mp4`

**Model:** RND (no teacher), seed 0 (run `r1_rnd`). Learned policy (SAC). Reward: RND novelty on hand–object interaction features, plus small smoothness costs. No teacher, no task reward.

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | touch |
| 1 | free | random hull | grasp, no lift |
| 2 | fixed | random hull | grasp |
| 3 | free | compound | grasp, no lift |
| 4 | fixed | platonic | grasp |
| 5 | free | platonic | grasp, no lift |
| 6 | fixed | box | grasp |
| 7 | free | random hull | grasp, no lift |
| 8 | fixed | cylinder | grasp |

All 120 held-out trials: fixed grasp 83%; free grasp 48%, lift 3%, held at end 0%, knocked off 28%.

### `video_r1_rnd_s1.mp4`

**Model:** RND (no teacher), seed 1 (run `r1_rnd_s1`). Learned policy (SAC). Reward: RND novelty on hand–object interaction features, plus small smoothness costs. No teacher, no task reward.

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | touch |
| 1 | free | random hull | no contact |
| 2 | fixed | random hull | grasp |
| 3 | free | compound | touch |
| 4 | fixed | platonic | grasp |
| 5 | free | platonic | touch |
| 6 | fixed | box | touch |
| 7 | free | random hull | knocked off |
| 8 | fixed | cylinder | touch |

All 120 held-out trials: fixed grasp 67%; free grasp 38%, lift 2%, held at end 0%, knocked off 38%.

### `video_r1_ab.mp4`

**Model:** A+B: demos + restarts, seed 0 (run `r1_ab`). Learned policy (SAC). Reward: RND novelty plus smoothness costs. Taught by the scripted controller through (A) its demonstrations in replay with fading imitation and (B) trials restarted from its demonstration states. No task reward.

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | touch |
| 1 | free | random hull | knocked off |
| 2 | fixed | random hull | grasp |
| 3 | free | compound | grasp, no lift |
| 4 | fixed | platonic | grasp |
| 5 | free | platonic | knocked off |
| 6 | fixed | box | grasp |
| 7 | free | random hull | touch |
| 8 | fixed | cylinder | grasp |

All 120 held-out trials: fixed grasp 87%; free grasp 65%, lift 10%, held at end 7%, knocked off 33%.

### `video_r1_ab_s1.mp4`

**Model:** A+B: demos + restarts, seed 1 (run `r1_ab_s1`). Learned policy (SAC). Reward: RND novelty plus smoothness costs. Taught by the scripted controller through (A) its demonstrations in replay with fading imitation and (B) trials restarted from its demonstration states. No task reward.

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | touch |
| 1 | free | random hull | knocked off |
| 2 | fixed | random hull | touch |
| 3 | free | compound | no contact |
| 4 | fixed | platonic | grasp |
| 5 | free | platonic | lifted, then dropped |
| 6 | fixed | box | grasp |
| 7 | free | random hull | touch |
| 8 | fixed | cylinder | grasp |

All 120 held-out trials: fixed grasp 87%; free grasp 75%, lift 22%, held at end 7%, knocked off 52%.

### `video_r1_ab_lift.mp4`

**Model:** A+B+D: + lift bonus, seed 0 (run `r1_ab_lift`). As A+B, plus (D) a lift bonus paid while a free block is lifted. This is the only term that rewards the task.

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | touch |
| 1 | free | random hull | grasp, no lift |
| 2 | fixed | random hull | grasp |
| 3 | free | compound | grasp, no lift |
| 4 | fixed | platonic | grasp |
| 5 | free | platonic | lifted, held to the end |
| 6 | fixed | box | grasp |
| 7 | free | random hull | touch |
| 8 | fixed | cylinder | grasp |

All 120 held-out trials: fixed grasp 85%; free grasp 67%, lift 32%, held at end 27%, knocked off 45%.

### `video_r1_ab_lift_s1.mp4`

**Model:** A+B+D: + lift bonus, seed 1 (run `r1_ab_lift_s1`). As A+B, plus (D) a lift bonus paid while a free block is lifted. This is the only term that rewards the task.

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | touch |
| 1 | free | random hull | knocked off |
| 2 | fixed | random hull | grasp |
| 3 | free | compound | knocked off |
| 4 | fixed | platonic | grasp |
| 5 | free | platonic | knocked off |
| 6 | fixed | box | grasp |
| 7 | free | random hull | no contact |
| 8 | fixed | cylinder | grasp |

All 120 held-out trials: fixed grasp 83%; free grasp 67%, lift 37%, held at end 25%, knocked off 52%.

### `video_r1_res.mp4`

**Model:** C: residual, seed 0 (run `r1_res`). Learned residual policy (SAC) on top of the scripted controller (C): each step executes the script's action plus 0.5 × the policy's action. Reward: RND novelty plus smoothness costs. No task reward.

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | touch |
| 1 | free | random hull | touch |
| 2 | fixed | random hull | grasp |
| 3 | free | compound | grasp, no lift |
| 4 | fixed | platonic | grasp |
| 5 | free | platonic | touch |
| 6 | fixed | box | grasp |
| 7 | free | random hull | touch |
| 8 | fixed | cylinder | touch |

All 120 held-out trials: fixed grasp 72%; free grasp 67%, lift 0%, held at end 0%, knocked off 10%.

### `video_r1_res_s1.mp4`

**Model:** C: residual, seed 1 (run `r1_res_s1`). Learned residual policy (SAC) on top of the scripted controller (C): each step executes the script's action plus 0.5 × the policy's action. Reward: RND novelty plus smoothness costs. No task reward.

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | touch |
| 1 | free | random hull | touch |
| 2 | fixed | random hull | touch |
| 3 | free | compound | grasp, no lift |
| 4 | fixed | platonic | grasp |
| 5 | free | platonic | touch |
| 6 | fixed | box | grasp |
| 7 | free | random hull | grasp, no lift |
| 8 | fixed | cylinder | grasp |

All 120 held-out trials: fixed grasp 60%; free grasp 72%, lift 7%, held at end 7%, knocked off 10%.

### `video_r1_res_lift.mp4`

**Model:** C+D: residual + lift bonus, seed 0 (run `r1_res_lift`). As C, plus (D) the lift bonus.

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | touch |
| 1 | free | random hull | knocked off |
| 2 | fixed | random hull | grasp |
| 3 | free | compound | grasp, no lift |
| 4 | fixed | platonic | grasp |
| 5 | free | platonic | no contact |
| 6 | fixed | box | grasp |
| 7 | free | random hull | touch |
| 8 | fixed | cylinder | grasp |

All 120 held-out trials: fixed grasp 72%; free grasp 67%, lift 27%, held at end 23%, knocked off 10%.

### `video_baseline_scripted.mp4`

**Model:** Scripted teacher (v1) (run `baseline_scripted`). Not learned. The hand-coded version 1 controller (`ue/teach/scripted.py`): it plans a top-down thumb-against-fingers pinch, then lifts. It is the teacher for A, B and C.

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | touch |
| 1 | free | random hull | touch |
| 2 | fixed | random hull | grasp |
| 3 | free | compound | lifted, held to the end |
| 4 | fixed | platonic | grasp |
| 5 | free | platonic | lifted, held to the end |
| 6 | fixed | box | grasp |
| 7 | free | random hull | no contact |
| 8 | fixed | cylinder | grasp |

All 120 held-out trials: fixed grasp 75%; free grasp 85%, lift 82%, held at end 78%, knocked off 2%.

### `video_baseline_random.mp4`

**Model:** Random actions (run `baseline_random`). Not learned. A new uniformly random action every step (a floor for chance contact).

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | no contact |
| 1 | free | random hull | no contact |
| 2 | fixed | random hull | no contact |
| 3 | free | compound | no contact |
| 4 | fixed | platonic | no contact |
| 5 | free | platonic | no contact |
| 6 | fixed | box | no contact |
| 7 | free | random hull | no contact |
| 8 | fixed | cylinder | no contact |

All 120 held-out trials: fixed grasp 0%; free grasp 0%, lift 0%, held at end 0%, knocked off 0%.

### `video_baseline_still.mp4`

**Model:** Still (run `baseline_still`). Not learned. Zero action every step, so the arm holds its starting posture (a floor: the block is never reached).

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | no contact |
| 1 | free | random hull | no contact |
| 2 | fixed | random hull | no contact |
| 3 | free | compound | no contact |
| 4 | fixed | platonic | no contact |
| 5 | free | platonic | no contact |
| 6 | fixed | box | no contact |
| 7 | free | random hull | no contact |
| 8 | fixed | cylinder | no contact |

All 120 held-out trials: fixed grasp 0%; free grasp 0%, lift 0%, held at end 0%, knocked off 0%.

### `showcase_r1_rnd.mp4`

**Model:** RND (no teacher), seed 0 (run `r1_rnd`). Learned policy (SAC). Reward: RND novelty on hand–object interaction features, plus small smoothness costs. No teacher, no task reward.

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | touch |
| 1 | free | random hull | grasp, no lift |
| 2 | fixed | random hull | grasp |
| 3 | free | compound | grasp, no lift |
| 4 | fixed | platonic | grasp |
| 5 | free | platonic | grasp, no lift |
| 6 | fixed | box | grasp |
| 7 | free | random hull | grasp, no lift |

All 120 held-out trials: fixed grasp 83%; free grasp 48%, lift 3%, held at end 0%, knocked off 28%.

### `showcase_r1_ab.mp4`

**Model:** A+B: demos + restarts, seed 0 (run `r1_ab`). Learned policy (SAC). Reward: RND novelty plus smoothness costs. Taught by the scripted controller through (A) its demonstrations in replay with fading imitation and (B) trials restarted from its demonstration states. No task reward.

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | touch |
| 1 | free | random hull | knocked off |
| 2 | fixed | random hull | grasp |
| 3 | free | compound | grasp, no lift |
| 4 | fixed | platonic | grasp |
| 5 | free | platonic | knocked off |
| 6 | fixed | box | grasp |
| 7 | free | random hull | touch |

All 120 held-out trials: fixed grasp 87%; free grasp 65%, lift 10%, held at end 7%, knocked off 33%.

### `showcase_r1_ab_lift.mp4`

**Model:** A+B+D: + lift bonus, seed 0 (run `r1_ab_lift`). As A+B, plus (D) a lift bonus paid while a free block is lifted. This is the only term that rewards the task.

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | touch |
| 1 | free | random hull | grasp, no lift |
| 2 | fixed | random hull | grasp |
| 3 | free | compound | grasp, no lift |
| 4 | fixed | platonic | grasp |
| 5 | free | platonic | lifted, held to the end |
| 6 | fixed | box | grasp |
| 7 | free | random hull | touch |

All 120 held-out trials: fixed grasp 85%; free grasp 67%, lift 32%, held at end 27%, knocked off 45%.

### `showcase_r1_res.mp4`

**Model:** C: residual, seed 0 (run `r1_res`). Learned residual policy (SAC) on top of the scripted controller (C): each step executes the script's action plus 0.5 × the policy's action. Reward: RND novelty plus smoothness costs. No task reward.

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | touch |
| 1 | free | random hull | touch |
| 2 | fixed | random hull | grasp |
| 3 | free | compound | grasp, no lift |
| 4 | fixed | platonic | grasp |
| 5 | free | platonic | touch |
| 6 | fixed | box | grasp |
| 7 | free | random hull | touch |

All 120 held-out trials: fixed grasp 72%; free grasp 67%, lift 0%, held at end 0%, knocked off 10%.

### `showcase_r1_res_lift.mp4`

**Model:** C+D: residual + lift bonus, seed 0 (run `r1_res_lift`). As C, plus (D) the lift bonus.

| Trial | Block | Shape | Outcome |
|---|---|---|---|
| 0 | fixed | prism | touch |
| 1 | free | random hull | knocked off |
| 2 | fixed | random hull | grasp |
| 3 | free | compound | grasp, no lift |
| 4 | fixed | platonic | grasp |
| 5 | free | platonic | no contact |
| 6 | fixed | box | grasp |
| 7 | free | random hull | touch |

All 120 held-out trials: fixed grasp 72%; free grasp 67%, lift 27%, held at end 23%, knocked off 10%.
