# Part 3: Methods

Every version 1 run uses the same learner as version 0 (SAC, 3-step
returns, rewards recomputed at sampling time; see
version_0/report/03_methods.pdf) and the same base explorer. The runs
differ only in how the scripted teacher is used (A, B, C) and whether the
lift bonus (D) is added.

## Common to all runs

- **Environment:** hand v1 (19 actions) and shapes v1, otherwise as in
  version 0 (3 s trials, 50% fixed blocks, 180-D state observation).
- **Explorer: RND on interaction features**, the best reward-free method
  in version 0, normalised by its running std.
- **Smoothness costs** (naturalness), computed by the env on the executed
  action `a` (19-D):

```
r_smooth = −(0.1 · mean(a²) + 0.2 · mean((a_t − a_{t−1})²))
```

  In a smoke test, its spread was about 17% of RND's.
- **Total reward:** `r = r_RND + r_smooth (+ r_lift for D)`.
- **Budget:** 700k env steps (about 9,300 trials), 8 envs, 2 seeds per
  setup (C+D: one seed, to finish earlier).

## A: demonstrations in replay + fading imitation

**Data.** The scripted controller ran 1,500 trials on demonstration
shape libraries (seeds 5000+), alternating free and fixed blocks. Kept
trials:
- free blocks that were lifted;
- fixed blocks with an opposed grasp.

That leaves 1,129 trials (about 85k transitions) in
`version_1/demos/scripted_v1.npz`. Actions are stored in the 19-D action
space. The demonstration rollouts were run with those 19-D actions, so
their success already accounts for the coupling.

**Mixing.** Each SAC batch of 512 takes 25% of its samples from the
demonstration replay. Their intrinsic rewards are recomputed like all
others.

**Imitation.** A Q-filtered behaviour-cloning term on the demonstration
samples is added to the actor loss:

```
L_BC = λ(t) · mean|Q| · mean( 1[Q(s,a_d) > Q(s,π(s))] · ‖tanh μ(s) − a_d‖² )
```

- λ falls linearly from 1 to 0 over the first half of training.
- The demonstrations stay in replay afterwards.
- mean|Q| makes λ independent of the reward scale (TD3+BC).

## B: reverse curriculum from demonstration states

- At its first reset, each env records 40 scripted trials on **its own**
  shape library, since restoring a state needs the same shapes.
- It keeps the full simulator snapshots of the successful ones.
- A new trial then starts, with probability p(t), from a snapshot at time
  index ≥ t_min(t) of a random successful trajectory.
- Both are annealed over the first 60% of training:
  - p from 0.5 to 0.1;
  - t_min from 80% of the trajectory (the block already lifted) to 0.
- Restarted trials get the full 3 s.

## C: residual policy on the scripted controller

- The env runs the scripted controller every step. The executed action is

  ```
  a = clip(a_script + 0.5 · a_π, −1, 1)
  ```

- The script's 19-D action is appended to the observation (199-D).
- The intrinsic reward and smoothness costs are unchanged; smoothness
  applies to the executed action.
- During warm-up the residual is random, which already perturbs the
  script.
- The script plans about 0.3 s per trial and costs about 1 ms per step, so
  these runs use 2 env workers.
- Evaluation uses the same residual environment and reports the residual
  ratio mean|0.5·a_π| / mean|a_script|.

## D: lift bonus

```
r_lift = 1[lifted] · (0.5 + 10 · min(Δh, 0.05 m))
```

This gives 0.5–1.0 per step while a free block is lifted, as defined by
the env: more than 2 cm above its spawn height, off the pedestal, and
touched by the hand. It is read from the env's stored `info`, so a block
that merely tips over to stand taller does not count. It is added
unnormalised, with weight 1. This is the only term that encodes the task.

## Run matrix

| Setup | A | B | C | D | Seeds |
|---|---|---|---|---|---|
| `rnd` (no teacher) | | | | | 0, 1 |
| `ab` | ✓ | ✓ | | | 0, 1 |
| `ab_lift` | ✓ | ✓ | | ✓ | 0, 1 |
| `res` | | | ✓ | | 0, 1 |
| `res_lift` | | | ✓ | ✓ | 0 |

## Evaluation

Held-out shape libraries (seeds 9000+), 120 trials alternating fixed and
free, stochastic policy (deterministic too, on 60 trials). The version 0
metrics are kept: touch, pinch, grasp, lift, knocked off, coverage,
ethogram and repertoire. Version 1 adds:

- **held at end**: the block is still lifted at the last step;
- **carry**: distance the block travels while lifted;
- **turn**: rotation of the block while lifted (in-hand reorientation);
- **lift events**: separate lifts per trial (regrasps);
- **hyper_frac**: fraction of hand-joint steps more than 2° past the
  natural (v1) limits (version 0 RND: 30%);
- **jerk** and **effort**: mean squared action change and size;
- **res_ratio**: for C, how far the policy departs from the script.
