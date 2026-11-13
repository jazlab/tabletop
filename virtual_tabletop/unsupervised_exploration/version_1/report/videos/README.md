# Version 1 videos

The full guide is `../06_videos.pdf` (Part 6 of the report).

- Every video shows the **same held-out blocks in the same order**:
  - even trials: a fixed block, held on a pole;
  - odd trials: a free block, on a pedestal.
- Trials play in groups, side by side, left to right:
  - `video_*`: 3 at a time, trials 0–8;
  - `showcase_*`: 2 at a time, larger, trials 0–7.
- The label on each tile gives the trial, fixed/free and shape, plus TOUCH / GRASP / LIFT while they happen.

| File | Model | Seed |
|---|---|---|
| `video_r1_rnd.mp4` | RND (no teacher) | 0 |
| `video_r1_rnd_s1.mp4` | RND (no teacher) | 1 |
| `video_r1_ab.mp4` | A+B: demos + restarts | 0 |
| `video_r1_ab_s1.mp4` | A+B: demos + restarts | 1 |
| `video_r1_ab_lift.mp4` | A+B+D: + lift bonus | 0 |
| `video_r1_ab_lift_s1.mp4` | A+B+D: + lift bonus | 1 |
| `video_r1_res.mp4` | C: residual | 0 |
| `video_r1_res_s1.mp4` | C: residual | 1 |
| `video_r1_res_lift.mp4` | C+D: residual + lift bonus | 0 |
| `video_baseline_scripted.mp4` | Scripted teacher (v1) | – |
| `video_baseline_random.mp4` | Random actions | – |
| `video_baseline_still.mp4` | Still | – |
| `showcase_r1_rnd.mp4` | RND (no teacher) (showcase) | 0 |
| `showcase_r1_ab.mp4` | A+B: demos + restarts (showcase) | 0 |
| `showcase_r1_ab_lift.mp4` | A+B+D: + lift bonus (showcase) | 0 |
| `showcase_r1_res.mp4` | C: residual (showcase) | 0 |
| `showcase_r1_res_lift.mp4` | C+D: residual + lift bonus (showcase) | 0 |

Models:

- **RND (no teacher):** Learned policy (SAC). Reward: RND novelty on hand–object interaction features, plus small smoothness costs. No teacher, no task reward.
- **A+B: demos + restarts:** Learned policy (SAC). Reward: RND novelty plus smoothness costs. Taught by the scripted controller through (A) its demonstrations in replay with fading imitation and (B) trials restarted from its demonstration states. No task reward.
- **A+B+D: + lift bonus:** As A+B, plus (D) a lift bonus paid while a free block is lifted. This is the only term that rewards the task.
- **C: residual:** Learned residual policy (SAC) on top of the scripted controller (C): each step executes the script's action plus 0.5 × the policy's action. Reward: RND novelty plus smoothness costs. No task reward.
- **C+D: residual + lift bonus:** As C, plus (D) the lift bonus.
- **Scripted teacher (v1):** Not learned. The hand-coded version 1 controller (`ue/teach/scripted.py`): it plans a top-down thumb-against-fingers pinch, then lifts. It is the teacher for A, B and C.
- **Random actions:** Not learned. A new uniformly random action every step (a floor for chance contact).
- **Still:** Not learned. Zero action every step, so the arm holds its starting posture (a floor: the block is never reached).
