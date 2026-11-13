# Part 3: Methods

This part describes the shared learner and the exploration objectives built
on it. The code is in `ue/agent.py`, `ue/intrinsic.py`, `ue/train.py`,
`ue/dep.py` and `ue/evaluate.py`.

## Design principles, from the literature (Part 1)

1. **One learner, many objectives.** Following URLB, every method trains
   the same SAC agent. Methods differ only in the intrinsic reward module,
   so differences in behaviour come from the objective.
2. **Off-policy, with rewards recomputed when sampled.** At laptop scale,
   SAC/DDPG with n-step returns is far more sample-efficient than PPO.
   Intrinsic rewards are non-stationary, so they are never stored: each
   module recomputes them for every sampled batch with its current
   parameters.
3. **Compute intrinsic rewards on interaction features, not the body.**
   Several papers (CEE-US, CSD, MUSIC, SkiLD, TacEx) report that novelty
   over the full state makes a high-DoF agent explore its own body and
   ignore the object. Our arm has 23 joints and the block 6 DoF. By
   default the reward modules therefore read the 81-D `interaction`
   features (touch, contact map, wrench, hand–surface distance, object
   displacement, rotation and velocity), while the **policy** still sees
   the full 180-D observation, including proprioception and touch. A
   full-state RND run tests this choice directly.
4. **Purely unsupervised versus helped.**
   - The core methods get no task knowledge beyond the choice of
     features.
   - Separately labelled variants add developmental priors (a fading grasp
     reflex, a static-to-dynamic curriculum, DEP exploration). These are
     still reward-free.
   - One supervised run (a shaped reach-grasp-lift reward) and a scripted
     controller are references.

## The learner

**SAC** (Haarnoja et al. 2018) with:
- an automatically tuned temperature, target entropy −|A|/2 = −11.5 (as
  recommended for high-dimensional action spaces);
- twin critics with Polyak averaging (τ = 0.01);
- 3-step returns, γ = 0.97 (the horizon matches the 75-step trial);
- MLPs with 2 × 256 units (LayerNorm and tanh on the first layer, ELU
  afterwards);
- Adam with lr 3e-4, batch 512;
- 0.25 gradient updates per env step;
- 8,000 steps of random warm-up;
- a 600k-transition buffer.

**Replay** is a ring buffer of shape `[T, n_env]`, so n-step chains are
contiguous and are cut at trial boundaries. Trials end only by time limit,
and the trial-time feature is in the observation, so targets always
bootstrap.

**Data collection:** 8 envs in one worker process. Each run uses about
1.1 cores; 8 runs at once stay within the 10-core budget.

**Skill- and goal-conditioning:** the actor and critic see [obs, z], with
z sampled per trial.

**Exploration noise:** white Gaussian by default. Two options:
- **pink** (1/f) noise, as one coloured sequence per trial;
- **DEP switching** (DEP-RL): with probability p per step, the DEP
  controller drives that env for h steps. Its actions are stored and
  learned from off-policy.

Rewards are normalised by a running standard deviation and clipped to
±5 (except for skills, goals and the shaped reward).

## Exploration objectives

### Knowledge-based (prediction error and uncertainty)

**RND** (Burda et al. 2019). A fixed random network f and a predictor g,
both 2 × 256 MLPs with 64 outputs. The input x′ is the next-state
interaction features, normalised by running statistics.

```
r = ‖g(x′) − f(x′)‖²
```

**RND (full state)** is the same on the whole 180-D observation. It is a
control for principle 3.

**Disagreement** (Pathak et al. 2019; object-centric as in CEE-US,
Sancaktar et al. 2022). An ensemble of K = 5 MLPs (2 × 128) predicts the
change in normalised interaction features, Δx, from (obs, action). Each
member trains on a random half of each batch.

```
r = mean over dimensions of Var_k[Δx̂_k]
```

**Tactile surprise** (haptics-based curiosity, Rajeswar et al. 2022;
TacEx). A 5-member ensemble of Bernoulli classifiers predicts which of the
36 contact-map cells and 18 touch patches will be active at t+1, plus
object motion, from (obs, action). The reward is epistemic uncertainty
about *touch*:

```
r = 10 · mean Var_k[p_k] + 0.5 · mean Var_k[Δobject_k]
```

The dynamics term follows ToC/HaC. Without it, touch curiosity settles
into constant passive contact. Disagreement is used rather than prediction
error, because contact chatter is aleatoric and prediction error would
chase it.

### Data-based (state entropy)

**APT** (Liu & Abbeel 2021). A particle estimate of state entropy over the
normalised interaction features: the reward is log(1 + mean distance to
the k = 12 nearest neighbours) in the current batch.

**Contact coverage** (CCGE-lite, after Liu et al. 2026). Counts are kept
over cells
c = (object-state bin s, hand unit u, object region k):
- s is one of 36 bins: fixed or free × height gain (3 bins) × which object
  axis points up (6);
- u is one of 6 hand units, k one of 6 bounding-box faces.

Each env step adds one count to every active cell, and one count to the
step's whole contact set (the set of active cells, per object-state bin).

```
r = mean over active cells c of 1/√(C_c + 1)
  + 0.5 · 1/√(C_set + 1)                      (if any contact)
  + 0.1 · mean_u exp(−d_u / 2 cm) · ḡ_u
```

The last term pulls hand units toward the object in proportion to how
rarely that unit has touched it in the current object state; d_u is unit
u's distance to the surface. The object-state bins matter: without them,
touching a resting block becomes "boring" and nothing pushes toward
lifting or turning it.

### Competence-based (mutual information and skills)

**MUSIC** (Zhao et al. 2021). Maximises I(S_hand; S_object):
- S_hand = palm and fingertip positions;
- S_object = displacement, rotation since spawn, and wrench (the wrench
  carries the signal for fixed blocks).

The critic is separable, T(a, s) = f(a)·g(s), so every other sample in the
batch is a negative at the cost of one matrix product (InfoNCE form). The
per-sample reward is

```
r = T(a′, s′) − log mean_j exp T(a′, s_j)
```

The contrast is over *object* states for the same hand state, so the
reward is exactly 0 while the object does not vary. It is high when the
object moves with the hand. Because it is about zero until the object
moves, it is bootstrapped with 0.5 × contact coverage.

Two pitfalls came up, and both are reported in Part 4:
- A first version contrasted over hand states. With an unvarying object
  state, that rewards an arbitrary hand region.
- Unclipped, the pointwise MI of *novel* hand–object states (a first
  touch) is negative, so touching gets punished. The final version clips
  at 0, as MUSIC does.

**CSF** (contrastive successor features; Zheng et al. 2025, METRA-like):
- φ: interaction features → R⁴; skills z are uniform on the unit sphere,
  one per trial;
- φ is trained with InfoNCE on (φ(s′) − φ(s))·z against in-batch skills
  (ξ = 5);
- reward (φ(s′) − φ(s))·z; the policy is z-conditioned.

**DIAYN** (Eysenbach et al. 2019):
- 16 discrete skills;
- the discriminator q(z | s′) sees only the `skill` features, quantities
  the agent changes;
- r = log q(z | s′) + log 16, plus 0.5 × RND.

### Goal-based: modular goal babbling (IMGEP + HER)

After Forestier, Portelas, Mollard & Oudeyer (2022), Baranes & Oudeyer
(2013), Andrychowicz et al. (2017) and Pong et al. (2020). The agent sets
its own goals in four goal spaces (modules):

| Module | Goal space | Success |
|---|---|---|
| reach | palm-to-object vector | 2.5 cm |
| contact | which of the 6 hand units touch the object | exact pattern |
| move | object displacement (free blocks only) | 2 cm |
| turn | object rotation since spawn (free blocks only) | 0.25 rad |

At each trial:
1. **Choose a module** in proportion to its absolute learning progress:
   the change in success rate between the last two windows of 100
   attempts, with 20% uniform choice.
2. **Draw a goal** from that module's archive of outcomes the agent has
   achieved before. Rare outcomes are preferred: of 16 random archive
   points, take the one furthest from its nearest neighbours. This is a
   cheap Skew-Fit.
3. **Reward:** 1[success] − 0.1 · min(d/ε, 10).
4. **Hindsight:** 70% of sampled transitions are relabelled with an
   outcome reached later in the same trial (HER "future").

Goals are the agent's own past outcomes, so there is still no task
reward.

### Controls and references

- **Max-entropy SAC** with zero reward.
- **Random actions** and **still** (no movement), evaluated directly.
- **DEP** (Der & Martius 2015; DEP-RL, Schumacher et al. 2023). A
  self-organising sensorimotor controller with no networks:
  - y = tanh(κ C̃ x + h), with sensors x = joint angles and outputs
    y = servo targets;
  - C ← C + (ẋ_t ẋ_{t−1}ᵀ − C)/τ, with C̃ = C/‖C‖;
  - separate arm and finger blocks (κ = 2 and 1.5, τ = 20 steps).
- **Scripted:** the IK reach-grasp-lift controller (Part 2).

**Shaped (supervised).** d̄ is the mean palm/thumb/index surface
distance; the height term applies to free blocks only.

```
r = 1 − tanh(10 d̄) + 0.5·[thumb + digit contact]
  + 0.5·[opposed faces] + 3·clip(height gain / 5 cm, 0, 1)
```

### Developmental priors (reward-free variants)

- **Fading grasp reflex** (like the infant palmar reflex). While the palm,
  or at least 2 hand units, touch the object, the env adds a closing bias
  of reflex × 3 rad/s to the finger flexion targets. reflex is annealed
  from 1 to 0 over the first 70% of training, so the policy has to take
  over the closing itself.
- **Static → dynamic curriculum** (Chiappa et al. 2024). p_fixed is
  annealed from 0.8 to 0.5 over the first half of training.

## Implementation notes and pitfalls found

- **Trial-identity confound.** The DIAYN discriminator reached 100%
  accuracy during the *random warm-up*. Absolute object orientation and
  spawn-dependent positions identify the trial, and z is constant within
  a trial, so the discriminator was memorising trials. The fix was to give
  intrinsic objectives rotation *since spawn* (`obj_drot`) instead of
  absolute orientation, and to give skill discriminators only quantities
  the agent changes. Its accuracy at warm-up then fell to 0.25. The same
  confound would have made RND and APT novelty reward spawn randomness.
- **Cost.** Under 8-way load, a SAC update takes about 40 ms. A 5-member
  256-unit ensemble scoring 1,536 transitions added about 30 ms, and the
  first MUSIC version (an MLP critic with 64 negatives) about 40 ms. The
  ensembles were shrunk to 128 units, and MUSIC got the separable critic.
- **Torque limits** (Part 2) and **collision bits**, both found when
  testing the env.

## Evaluation protocol

All policies are evaluated on **held-out shape libraries** (seeds 9000+),
with 120 trials alternating fixed and free, and the stochastic policy (the
exploratory behaviour). Skill methods sample a skill per trial; DIAYN
cycles through its skills. For each trial the metrics are:
- P(touch), P(pinch), P(grasp), P(lift);
- fraction of time touching or grasping;
- maximum displacement and rotation;
- minimum hand–surface distance.

Coverage over all trials counts:
- distinct contact cells (out of 36 × 6 × 6);
- distinct contact sets;
- 2-cm voxels of object displacement.

For goal babbling, per-module goal success is also recorded.

## Compute

Everything ran on the laptop (14 cores, of which at most 10 were used;
CPU-only torch, 1 thread per learner). A standard run is 700k env steps
(9,300 trials; 700k × 0.04 s ≈ 7.8 h of simulated play) and takes 40–90
min with 8 runs in parallel.
