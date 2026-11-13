# Part 1: Literature review

The review covered about 65 papers in two strands, each read with its
references followed up:

- **unsupervised RL**: knowledge-based, data-based and competence-based
  exploration, recent skill discovery, and touch- or contact-driven
  curiosity for manipulation;
- **skeletal and musculoskeletal agents**: KINESIS and the Mathis-lab
  line of work, MyoSuite, DEP and DEP-RL, Lattice, synergies, virtual
  animals, goal babbling, and developmental models with touch.

The full per-paper notes are Appendices A and B. Each entry has a
citation, a method summary, the core equation, hyperparameters, results
and relevance; tags mark whether a detail was read during the review or
recalled. This part opens with a synthesis of how the literature shaped
the study.

## Five lessons that shaped the design

**1. Choose what the intrinsic reward looks at.** Generic novelty over the
full state (RND, ICM, APT, DIAYN, DADS, LSD) mostly produces flailing in
manipulation. Moving the robot is easier than moving the object, so the
robot's own state dominates novelty and skill objectives:

- CEE-US (Sancaktar et al. 2022): the URLB baselines "mostly stay in free
  space";
- CSD (Park et al. 2023): DIAYN, DADS and LSD move the arm, not the
  object;
- MUSIC (Zhao et al. 2021) and SkiLD (2024): the same finding.

Methods that did make objects move without task reward share a common
ingredient: the objective is computed on object or interaction variables.
Examples are object-factored world-model uncertainty (CEE-US), hand–object
mutual information (MUSIC), controllability-aware distances (CSD),
contact-coverage counts (CCGE, Liu et al. 2026) and tactile-only
uncertainty (TacEx). Our arm has 23 joints against the block's 6, which
makes this sharper. → All our objectives read an `interaction` feature
vector, and a full-state RND run tests the choice.

**2. Touch is the bridge between reaching and manipulating.**
- Touch-based curiosity (Rajeswar et al. 2022) yields about 4× more
  object interactions than vision-only curiosity.
- Tactile intrinsic motivation (Vulin et al. 2021) speeds up manipulation
  learning.
- CCGE counts (object state, finger, object region) contact cells and is
  the most directly relevant recent result: a 16-DoF hand from low-
  dimensional state.
- Developmental models (MIMo; "Baby Sophia" in BabyBench) use touch
  novelty for self-exploration.
- Pitfalls: contact signals are spiky and partly random (aleatoric), so
  predict binary contacts or log-forces, prefer ensemble *disagreement*
  over prediction error, and pair touch with an object-dynamics term to
  avoid passive constant contact.

→ Contact coverage and tactile-surprise methods.

**3. Laptop scale favours off-policy learning.** PPO-based results (CCGE,
KINESIS, most MyoChallenge entries) use thousands of GPU environments or
128 CPU threads. URLB's off-policy DDPG with n-step returns and
sample-time reward recomputation is the reference for reward-free
pre-training at small scale. → A single SAC learner shared by all
objectives.

**4. Skeletal-model lessons (Mathis lab, DEP-RL, virtual animals).**
- **KINESIS** (Simos, Chiappa & Mathis 2025) is motion imitation, not
  reward-free, but several of its components transfer:
  - a curriculum by hard-negative mining (from PHC), giving a mixture of
    frozen experts with a learned gate;
  - Lattice exploration noise;
  - the observation that a policy trained on goals from scratch moves
    unnaturally, while a pre-trained motor prior fixes this;
  - the caution that task-relevant control is high-dimensional even when
    a few principal components explain most of the variance.
- **DEP-RL** (Schumacher et al. 2023): a self-organising DEP controller
  explores redundant bodies far better than Gaussian noise. Switching
  between DEP and the policy works; summing their actions does not.
- **Synergies** (SAR, Berg et al. 2023; Chiappa et al. 2024) speed up
  dexterous learning, provided a residual pathway is kept.
- **Curricula**: static before dynamic (Chiappa et al. 2024 on Baoding
  balls). → DEP baseline and DEP switching, pink noise, a static-to-dynamic
  curriculum.

**5. Goal babbling builds primitives.** Intrinsically motivated goal
exploration (Baranes & Oudeyer 2013; Forestier et al. 2022) with
*modular* goal spaces and learning-progress selection explores tool use
far better than a flat goal space: about 60% versus 10% coverage. Combined
with hindsight relabelling (HER) and rare-goal sampling (Skew-Fit), it
turns exploration into reusable, testable primitives such as "move the
block by Δ".
→ Modular goal babbling, plus a fading grasp reflex as a developmental
prior.

## From papers to implemented methods

| Implemented method | Main sources | What was taken |
|---|---|---|
| RND (interaction / full) | Burda et al. 2019; URLB | prediction error of a random network; the full-state version as a control |
| Disagreement | Pathak et al. 2019; CEE-US 2022 | 5-member ensemble variance, object-centric target |
| Tactile surprise | Rajeswar et al. 2022 (ToC/HaC); TacEx; Vulin et al. 2021 | uncertainty over next contacts + object-dynamics term |
| APT | Liu & Abbeel 2021; URLB | kNN particle entropy |
| Contact coverage | CCGE (Liu et al. 2026); Baby Sophia | counts over (object state, finger, region), contact-set novelty, reach-to-rare |
| MUSIC + contact | Zhao et al. 2021 | hand–object mutual information (here with a separable InfoNCE critic) |
| CSF skills | Zheng et al. 2025; METRA (Park et al. 2024) | contrastive successor features over interaction features |
| DIAYN + RND | Eysenbach et al. 2019; APS | discrete skills on agent-changed features |
| Goal babbling | Forestier et al. 2022; HER; Skew-Fit | 4 goal modules, LP selection, rare goals, hindsight |
| DEP / DEP switching | Der & Martius 2015; Schumacher et al. 2023 | self-organised exploration, DEP-RL switching |
| Pink noise | Eberhard et al. 2023; Lattice (Chiappa et al. 2023) | temporally correlated action noise |
| Grasp reflex, static → dynamic | MIMo / developmental; Chiappa et al. 2024 | fading palmar-like reflex, p_fixed curriculum |
| Shaped reward (supervised) | MyoChallenge practice | reach + grasp + lift reference |

## What the literature predicted for this budget

The skeletal-model review expected touching, pushing and occasional
grasps or lifts from reward-free training within a few hours, not reliable
relocation. For comparison:
- the best MyoChallenge 2023 entry reached 34% relocation, with shaped
  rewards and far more compute;
- CSD needed about 4M steps to move Fetch objects;
- CEE-US produced lifting after about 60k steps, but with a model-based
  planner and a GPU.

Our runs used 0.7–1.5M steps each.
