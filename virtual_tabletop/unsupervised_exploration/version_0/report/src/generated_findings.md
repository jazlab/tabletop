**F1. Reward-free exploration on interaction features learns to reach, touch and grasp. The best method matches or beats a supervised shaped reward.**
- Random actions and a still arm never touch the block. Max-entropy SAC
  with **zero reward** (the same learner, entropy bonus only) touches 3%
  of fixed blocks and no free ones. The behaviour therefore comes from
  the intrinsic objectives, not from the learner's entropy-driven
  exploration.
- Several purely reward-free explorers touch it in nearly every trial:
  RND, disagreement, contact coverage and APT.
- RND on interaction features grasps 67% of fixed and 23% of free blocks,
  against 57% and 8% for the shaped reach-grasp-lift reward with the same
  learner and step budget.
- It also covers the most contact configurations: 407 cells and 1,964
  contact sets, against 217 and 844 for the shaped reward.
- The supervised policy specialises in one behaviour, a thumb–index pinch
  (repertoire cluster B2, 58% of its trials). The explorers keep a
  broader repertoire.

**F2. What the reward looks at matters more than the estimator.**
- *Features.* RND on the full 180-D observation is clearly worse than the
  same RND on the 81-D interaction features: fixed grasp 55% vs 67%, free
  grasp 12% vs 23%, contact cells 305 vs 407, blocks knocked off 70% vs
  37%. This confirms the literature (CEE-US, CSD, MUSIC), although less
  dramatically than reported for Fetch. Here the hanging start makes
  reaching the block a large novelty source even in joint space.
- *Trial-identity confounds.* Absolute object orientation and spawn
  offsets identify the trial. DIAYN's discriminator reached 100% accuracy
  on random warm-up data. MUSIC's critic learned "hand states typical of
  this trial" when its object features included settling offsets and its
  negatives came from other trials. Every intrinsic objective here has to
  use agent-caused quantities (rotation *since spawn*, velocity, wrench).

**F3. Novelty, uncertainty and coverage objectives work; skill and MI objectives need care.** Grouping the methods by family:
- *Prediction-based:* RND is the best overall. Ensemble disagreement on
  interaction features is close behind (100% touch, 13% free grasp) and
  produced the first held-out lifts (2% of free blocks).
- *Data-based:* contact coverage (CCGE-lite) is the gentlest explorer
  (15% knocked off) with a broad contact repertoire. APT also reaches
  everything, but its kNN entropy over object displacement rewards
  throwing: 88% of free blocks knocked off.
- *Tactile surprise* (uncertainty over the next contacts) explores
  weakly: 15% touch on fixed blocks. Before any contact its reward is
  flat, so it lacks the reaching drive that novelty over distances and
  positions gives.
- *Competence-based:*
  - **CSF** learned 8 distinct, consistent whole-arm swings (a forward
    loop over the block, back-and-up, lateral) but never touched the
    block. This is the textbook "skills become body motions" failure:
    arm motion is the easiest way to make skills distinguishable.
  - **MUSIC** (hand–object mutual information) failed here. Three
    diagnosed fixes were applied:
    - contrast over object states, not hand states;
    - clip negative pointwise MI, which otherwise punishes the first
      touch;
    - remove trial-identifying object features.
  - Result: even so, v3 touched only 20% of blocks, *less* than its own contact-coverage bootstrap term achieves alone. The MI reward is noisy until the hand and the object actually co-move, and it drowns out the contact signal. MUSIC's Fetch results relied on a 4-DoF gripper, which reaches the block by chance far more often.
- *Goal babbling*:
  - v1 never touched the block, because rare-goal sampling favours
    far-away palm targets.
  - v2 caps reach goals near the block and adds a contact bonus. It
    learned a **controllable reach primitive**: 55% success on
    self-generated reach goals, and a median error of 2.2 cm on 8
    commanded targets around the block.
  - Its contact-pattern, move and turn goals were rarely achieved
    (0–7%).

**F4. Free blocks are the hard part. Lifting emerges, but slowly and rarely.**
- Every learned policy grasps fixed blocks far more often than free ones.
  Free blocks are mostly pushed, rotated or knocked off.
- Held-out lifts appear only after enough experience:
  - disagreement 2% at 700k steps;
  - RND 0% at 700k steps, 3% at 1.4M steps;
  - RND seed 1 and contact coverage seed 1: 2% each.
- The longer RND run keeps its grasping (fixed 68%) but knocks off more
  blocks (58%).
- The scripted controller shows lifting is physically possible (43%), but
  only with a precise top-down thumb-against-fingers pinch planned per
  block. The ±30° wrist range and the small hand leave a narrow set of
  feasible grasps.

**F4b. Seed variance is substantial.** Two seeds of the same method
differ by up to about 15 points on some metrics:
- RND fixed grasp 67% vs 52%, free grasp 23% vs 28%;
- contact coverage free grasp 5% vs 17%, contact sets 1,430 vs 1,675.

Most comparisons here are single-seed. Differences under about 10–15
points between methods should not be over-read. The large gaps hold
across seeds:
- explorers vs controls;
- interaction vs full-state features;
- RND/disagreement/contact vs APT, tactile surprise, MUSIC, CSF, DIAYN
  and goal babbling.

**F5. Priors, noise and mixtures: mostly no help to the best explorer.**
- *Grasp reflex + static-to-dynamic curriculum.* No help for contact
  coverage on held-out blocks (fixed grasp 48% vs 43%, free 3% vs 5%), but
  it more than doubled fixed grasping for goal babbling v2 (28% vs 12%,
  contact sets 778 vs 512). The reflex helps the method that explores
  least.
- *Pink noise + DEP switching (DEP-RL)* made RND worse: fixed grasp 40%
  vs 67%, free grasp 10% vs 23%, contact sets 1,380 vs 1,964. DEP's
  self-organised oscillations are object-agnostic. In a 3 s trial, DEP
  bursts (about 13% of steps) interrupt approaches more than they
  diversify them.
- *Mixing RND with contact coverage* gave a compromise rather than a sum:
  fixed grasp 28%, free 17%, fewer blocks knocked off (28%).
- *Go-Explore-style restarts* from archived contact states:
  - RND + restarts raised free-block grasping (30% vs 23%) but knocked off
    more (52%);
  - contact + restarts raised fixed grasping (52% vs 43%);
  - neither produced held-out lifts.
- *Free-block-focused restarts* (round 6: 75% free blocks, and 50% of
  trials restarting from archived free-block contact states) **hurt every method**:
  - RND: fixed grasp 15%, free 15%;
  - disagreement: 17% and 13%;
  - the *supervised* shaped reward: 5% and 0%, against 57% and 8%
    without restarts.
  
  None lifted. Evaluation trials always start from the hanging rest
  posture, so policies trained mostly from mid-contact states get too
  little practice at the approach: a train/test distribution shift. They
  do knock fewer blocks off (10–35%). Restarts need to stay a minority of
  trials (the 30% runs did better), or be paired with rest-start
  practice.
