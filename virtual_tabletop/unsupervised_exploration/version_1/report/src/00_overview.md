# Version 1: natural hands and learning to lift (overview)

Virtual tabletop, 12 November 2026. A follow-up to version 0
(`version_0/report/`), on the same laptop with at most 10 CPU cores.

## What version 0 left open

Version 0 trained twelve reward-free exploration objectives on the
monkey's right arm. The best of them (RND on interaction features)
learned to touch and grasp held-out blocks. Two problems remained:

1. **Unnatural hands.** The arm file lets every finger segment bend
   backwards by up to 60°. The learned policies used this freely, and
   their fingers looked double-jointed. Version 0's RND spent 30% of its
   hand-joint steps more than 2° past natural limits.
2. **No lifting.** No reward-free method lifted a free block (held-out
   lift rate 0–2%). The scripted IK grasp lifted 43%.

## What version 1 changes

1. **A natural hand** (Part 2). Only this project's model changes; the
   shared arm file and the PPO environment are untouched.
   - Joint ranges limit extension (for example, PIP and DIP to −5°).
   - Each finger's DIP follows ⅔ of its PIP, as tendons couple them, so
     there are 19 actions instead of 23.
   - The notebook `notebooks/hand_ranges_demo.ipynb` shows a video of
     every joint, v0 against v1.
2. **Blocks that fit the hand** (Part 2). Each base shape now comes in
   three sizes, about 3, 4 and 5 cm. Version 0 drew sizes from 3–7 cm,
   and the largest blocks did not fit the hand.
3. **A better scripted teacher** (Part 2). It plans a flat finger sweep
   against the thumb, selects grasps and uses straight vertical
   approaches. Its held-out free-block lift rate rose from 43% to 82%.
4. **Teaching the explorer** (Part 3). Every learner keeps RND as its
   intrinsic reward and adds small smoothness costs (effort and jerk).
   The scripted controller is used in two ways:
   - **A+B:** demonstrations in replay with fading, Q-filtered
     imitation (A), plus a reverse curriculum that starts some trials
     from demonstration states (B);
   - **C:** a residual policy that adds a correction to the script's
     action.

   Each is run with and without **D**, a lift bonus, which is the only
   term that rewards the task.
5. **New metrics** (Part 4): held at the end, carry and turn while
   lifted, regrasps, joints past natural limits, jerk and the residual
   ratio.

## Headline results

{{generated_v1_headlines}}

## Reading guide

| Part | Content |
|---|---|
| 1 Related work | Learning from demonstrations, residual RL, reverse curricula, natural hand priors |
| 2 Hand, shapes, teacher | Joint ranges and coupling, the v1 shape library, the scripted controller |
| 3 Methods | A, B, C and D, the run matrix, evaluation |
| 4 Results | Grasp/lift tables, manipulation, naturalness, learning curves, findings, behaviour |
| 5 Appendix | Every run's config and final training statistics |
| 6 Video guide | Which model each video shows, how to read it, and what happens in each trial shown |

Videos are in `version_1/report/videos/`. Part 6 says which model each one shows:
- `video_<run>.mp4`: 9 held-out trials, 3 side by side;
- `showcase_<run>.mp4`: 8 held-out trials, larger, 2 side by side.
