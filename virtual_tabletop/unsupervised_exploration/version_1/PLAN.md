# Version 1 plan: natural hands, and learning to lift

Living document. Revisions are appended at the bottom with timestamps.

## The brief (from the user)

1. **The hand moves unnaturally** in version 0: fingers bend backwards as
   if double-jointed. Fix it.
2. **The exploration policies should learn to lift and manipulate the
   blocks.** Supervision from the hand-crafted (scripted) policy is
   allowed.
   - Try **A+B**: demonstrations in the replay buffer with a fading
     imitation term, plus a reverse curriculum of restarts from
     demonstration states.
   - Try **C**: residual learning on top of the scripted controller.
   - Run both **with and without D**, a sparse lift bonus.
   - Report results for every combination.
3. Joint ranges change **only in this project's model**.
4. Order of work:
   1. joint ranges and a demo notebook;
   2. **wait for the user's OK**;
   3. training runs;
   4. a multi-PDF report and videos.
5. Code: a shared library (`ue/`) with versioned experiments
   (`version_0/`, `version_1/`). The user chose this layout.

## Done before the user's OK

- **Refactor:**
  - `ue/{sim,learn,rewards,teach,analysis,report}`, with `ue/paths.py`
    for the layout.
  - Version 0 artefacts moved to `version_0/`.
  - Regression: version 0 evaluation outputs and the training log are
    bit-identical before and after the refactor.
- **Hands** (`ue/sim/hand.py`, env option `hand`):
  - `v0`: the arm file's ranges.
  - `v1_ranges`: extension limits, independent joints (23 actions).
  - `v1`: the same limits plus DIP = ⅔·PIP coupling for the index to
    pinky (19 actions).
  - The limits are:
    - digits: MCP −20°, PIP −5°, DIP −5°;
    - thumb: MCP and IP −10°;
    - thumb CMC: kept as in the arm file;
    - flexion: unchanged.
  - The compiled ranges are asserted against the spec, after an
    attachment-units bug was caught.
- **Scripted controller on v1:** free blocks 57% lift and 60% grasp (v0:
  57% / 62%); fixed blocks 68% grasp. A first, stricter thumb-CMC limit
  dropped lifting to 20% and was reverted.
- **Simulation instability** is now detected (`info["unstable"]`) and
  ends the trial, instead of MuJoCo's silent reset.
- **Notebook:** `notebooks/hand_ranges_demo.ipynb`, 20 embedded videos.

## Design for the agents (after the OK)

### Common to every version 1 run

- **Environment:** `hand: v1` (19 actions); everything else as in
  version 0 (shapes, trials, observation, 50% fixed blocks).
- **Base explorer: RND on interaction features**, the best reward-free
  method in version 0. All version 1 variants are built on it, so the
  comparisons isolate A, B, C and D.
- **Smoothness costs (hand fix 3)**, added to every run:
  `r_smooth = −w_e · mean(a²) − w_j · mean((a_t − a_{t−1})²)`.
  - Computed in the env, stored in the replay `info`, read by a reward
    module.
  - Weights w_e = w_j = 0.05, relative to the normalised intrinsic reward
    (std 1). Tune them in a smoke test so that `r_smooth` is about 10% of
    the reward's spread.
- **Learner:** the version 0 SAC (n-step 3, batch 512, UTD 0.25), 700k
  steps, 8 envs.

### A: demonstrations in replay + fading imitation (DDPGfD / Nair et al. 2018 style)

- **Demonstration data** (`ue/teach/demos.py`):
  - Run the scripted controller with the v1 hand on *training-type* shape
    libraries (seeds 5000+, disjoint from evaluation seeds 9000+).
  - Run about 1,500 trials, half free and half fixed.
  - Keep trials with a grasp, and every free trial with a lift.
  - Store (obs, action, next_obs, done, info) as `.npz` under
    `version_1/demos/`.
- **Action conversion:** the scripted controller outputs 23-D delta
  actions. Map them to the 19-D action space (drop the coupled DIPs).
  Check that replaying the 19-D action reproduces the same servo targets
  up to the coupling.
- **Replay mixing:** each SAC batch takes `demo_frac` = 25% of its samples
  from a separate demonstration buffer. Rewards for those samples are
  computed by the same reward modules as the rest.
- **Imitation term:** `λ(t) · 1[Q(s, a_d) > Q(s, π(s))] · ‖μ(s) − a_d‖²` on
  the demonstration samples. This is a Q-filtered BC loss on the actor's
  mean.
  - λ decays linearly from `bc_start` = 1 (relative to the actor loss
    scale, which is checked in the smoke test) to 0 at 50% of training.
  - After that the demonstrations stay in replay, but there is no
    imitation term.

### B: reverse curriculum from demonstration states (Florensa et al. 2017; Salimans & Chen 2018)

- Each env generates its own demonstration states at start-up: about 40
  scripted trials on its *own* shape library, because restoring a state
  needs the same shapes. It keeps the full state snapshots
  (`get_state`) of successful trials.
- At reset, with probability `p_restart(t)` a trial starts from a
  snapshot taken at time index ≥ `t_min(t)` of a successful demonstration.
  - `t_min` goes from late (the block already lifted) to 0 over the
    first 60% of training.
  - `p_restart` goes from 0.5 to 0.1.
- Most late trials therefore start from rest. This avoids the train/test
  shift that sank version 0's round 6, where 50% of trials restarted from
  mid-contact states for the whole run.
- **Trial-time handling:** a restarted trial gets the full 3 s from the
  restored state, and the time feature restarts at 0, as in version 0's
  restarts.

### C: residual policy (Silver et al. 2018; Johannink et al. 2019)

- The env wrapper option `residual: {scale: β}` computes the scripted
  action `a_s` each step (the env holds a `ScriptedGrasp` that re-plans
  every trial). The executed action is `clip(a_s + β · a_π)`.
- The policy observation is augmented with `a_s` (19-D) so the policy
  knows what it is correcting (as in Silver et al.).
- β = 0.5. Optionally, β ramps from 0.2 to 0.5 over the first 30%.
- The intrinsic RND reward and the smoothness costs are unchanged.
  Smoothness applies to the *executed* action.
- **Evaluation** uses the same residual env, so results show the combined
  controller. Also report:
  - the mean |β·a_π| / |a_s|, i.e. how far the policy departs from the
    script;
  - the script alone with the v1 hand, as the reference.

### D: lift bonus (sparse, supervised)

- `r_lift = 1[lifted] · (0.5 + 10 · min(Δh, 0.05 m))`, so 0.5–1.0 per
  lifted step.
- "Lifted" is defined by the env: a free block > 2 cm above spawn height,
  off the pedestal, and in the hand.
- The reward module reads the stored `info`, so it can't be fooled by
  blocks that tip up. Added unnormalised, with weight 1.

### Run matrix (2 seeds each, 700k steps)

| id | explorer | A | B | C | D |
|---|---|---|---|---|---|
| rnd | RND + smooth | | | | |
| ab | RND + smooth | ✓ | ✓ | | |
| ab_lift | RND + smooth | ✓ | ✓ | | ✓ |
| res | RND + smooth | | | ✓ | |
| res_lift | RND + smooth | | | ✓ | ✓ |

That's 10 runs at about 1.1 cores each: one wave of 8, then 2 more, about
2.5 h in total. If time allows, add `ab` with contact coverage as the
explorer, to check that A+B is not RND-specific.

**Baselines on v1:** scripted, random, still. Version 0's `r1_rnd` gives
the "before" picture for the hand-naturalness metrics.

### Evaluation (held-out, same protocol as version 0, extended)

- Same 120 held-out trials and metrics as version 0: touch, pinch, grasp,
  lift, knocked off, coverage, ethogram, repertoire.
- **New manipulation metrics:**
  - lift rate;
  - time held aloft;
  - max lift height;
  - carry distance while lifted;
  - in-hand rotation while lifted;
  - regrasp count.
- **New naturalness metrics:**
  - fraction of finger-joint steps below −5° (the v1 limit, so ≈0 by
    construction) and joint-limit dwelling;
  - action jerk (mean |Δa|);
  - finger angular speed.
  - Version 0's `r1_rnd` gives the comparison.
- **Exploration retained?** Repertoire entropy and contact coverage: does
  teaching collapse the repertoire onto the scripted pinch?
- **Videos:** the standard grid for every run, plus large showcases
  (stochastic and deterministic) for the best run of each of A+B, C and
  ±D.

### Report (version_1/report, multi-PDF)

1. Overview.
2. Hand: the problem, the new ranges and coupling, figures from the
   notebook, the scripted check.
3. Methods: A, B, C and D with equations and literature, and the
   smoothness costs.
4. Results: the matrix table, curves, manipulation and naturalness
   metrics, repertoires, videos.
5. Appendix: runs, configs, video index.

The version 0 literature review is referenced rather than repeated. A
short related-work section covers learning from demonstrations, reverse
curricula and residual RL.

### Risks and checks

- **Demo action mapping (23 → 19):** verify that a replayed demo trial in
  the 19-D action space lifts as often as the original.
- **BC scale vs actor loss:** check in the smoke test. Q-filtering avoids
  imitating demos that the critic rates as worse than the policy.
- **Residual + scripted planning cost per step:** profile it. The script
  plans once per trial on a kinematic copy and is cheap per step after
  that.
- **Reverse curriculum:** each env's own demonstrations may include few
  lifts on hard shapes. Fall back to grasp states if fewer than about 5
  lift snapshots exist.
- **Compute:** keep ≤ 8 concurrent runs (≤ 10 cores) and evaluate as
  runs finish.
- **Time:** smoke-test every new mechanism for about 5 minutes before
  launching the matrix.

## Revisions
- (after user feedback) The joint limits were approved. New requirement:
  lifting must stay possible.
  - Added **shape set v1** (`shapes: v1`): 12 base shapes × 3 size classes
    (largest extent about 3 / 4 / 5 cm; v0 was 3–7 cm continuous), 36
    shapes per library. All version 1 runs use `hand: v1, shapes: v1`.
  - Scripted on v1 + v1, before improvement: free lift 70% (v0 shapes:
    57%), grasp 79%. Weak spots: large blocks (58%), pyramids (40%),
    prisms (50%).
  - The controller is being improved (agent, `ue/teach/scripted.py`). The
    version 0 controller is frozen as `ue/teach/scripted_v0.py`, so the
    version 0 baselines stay reproducible.
  - Plan re-read before implementing the agents; the design is unchanged.
- Infrastructure implemented and smoke-tested:
  - env: smoothness signals (`effort`, `jerk`); residual mode
    (`residual_scale`; adds `base_action` to the obs; `STRUCTURAL` keys
    keep evaluation consistent); demonstration restarts (`demo_trials`,
    `demo_restart_p`, `demo_tmin`).
  - rewards: `smooth` and `lift` read the stored info.
  - learn: demo replay mixing (`demos: {path, frac, bc}`) and a Q-filtered
    BC term scaled by |Q| (TD3+BC style). In a unit test, the BC error to
    the demos fell from 2.95 to 0.78.
  - evaluation: held_end, carry, turn_lifted, lift_events, effort, jerk,
    res_ratio, and `hyper_frac` (hand joints more than 2° past the natural,
    v1, limits). Version 0 RND scores 30% hyper_frac, version 1 0%.
  - Version 0 regression re-checked: identical.
- Smoothness weights: w_e 0.1, w_j 0.2. In the smoke test the smoothness
  term's std was about 17% of RND's.
- Residual costs about 1 ms per step for the script (plus about 64 ms
  planning per trial), so residual runs use 2 env workers (about 2.2
  cores). Queue at max 5 concurrent runs.
- 12:03: improved scripted controller (agent). On its test seeds: free
  lift 81% (68% before), held at end 74%, fell 1%, fixed grasp 85%.
  Demonstration dataset (1,500 trials, seeds 5000+): free lift 74%, fixed
  grasp 77%; 1,129 kept (`version_1/demos/scripted_v1.npz`).
  - Launched r1_rnd and r1_rnd_s1 at 11:50.
  - The other 8 runs are queued at max 6 concurrent.
- 12:15: the user accepted one seed for C+D (res_lift) to finish earlier.
  r1_res_lift_s1 was dropped from the queue; ab_s1 and ab_lift_s1 remain
  queued (max 7 concurrent). Expected end of training about 14:30, report
  about 15:15.
- 14:50: the residual runs ran at about 68 steps/s (5 runs sharing 10
  cores), not about 90. The end of training slipped to about 15:25 and
  the report to about 16:00. While waiting, the report text (overview,
  results structure) was drafted from the finished A+B and RND runs.
- 15:35: all runs finished and evaluated (`STOP_AUTOEVAL` written).
  Showcase videos for rnd, ab, ab_lift, res and res_lift. Report built
  (6 PDFs). Headline: A+B+D lifts 34% of held-out free blocks (2 seeds
  agree); C undermines the script (residual ratio 0.9).
