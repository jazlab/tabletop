All numbers are on **held-out shapes**, 120 trials per policy (half
fixed, half free blocks).

1. **Reward-free exploration learns to reach, touch and grasp.**
   - Random actions, a still arm, and SAC with zero reward (entropy only)
     essentially never touch the block: 0–3%.
   - Four reward-free explorers touch it in 93–100% of trials: RND,
     ensemble disagreement and contact coverage on interaction features,
     and APT.

2. **The best reward-free explorer rivals supervision.**
   - RND on interaction features grasps (opposed contacts):
     - 67% of fixed blocks, against 57% for a dense shaped reach-grasp-
       lift reward with the same learner and budget;
     - 23% of free blocks, against 8%.
   - It also covers about 2× as many hand–object contact configurations
     (1,964 vs 844 contact sets).
   - The supervised policy collapses onto one stereotyped fingertip pinch.
     The explorers keep a broad repertoire: whole-hand wraps, radial pinches,
     ulnar-side contact, turning.

3. **What the intrinsic reward looks at matters most.**
   - The same RND on the full 180-D state is clearly worse: grasps 55% /
     12%, blocks knocked off 70% vs 37%.
   - Several failures came from rewards that could be gamed through
     trial identity (absolute object pose, spawn offsets) rather than
     through behaviour. Fixing them was essential for DIAYN and MUSIC.

4. **Skills and goals give primitives, but body- or reach-centred ones.**
   - **CSF** learned 8 consistent whole-arm swings and never touched the
     block.
   - **DIAYN** learned 16 object-directed approaches that stop at
     different places around the block.
   - **Goal babbling** (IMGEP + HER, v2) learned a controllable *reach*
     primitive: 55% success on its own goals, and a median error of
     2.2 cm on commanded targets around the block. Its contact, move and
     turn goals stayed mostly out of reach.

5. **Lifting is the open problem.**
   - Free blocks get pushed, turned and knocked off far more often than
     lifted.
   - Held-out lifts appear only rarely and late: 2–3% for disagreement,
     RND at 2× steps, and replicate seeds.
   - A scripted top-down pinch lifts 43%, so the physics allows it, but
     the hand's small size and ±30° wrist leave little room for error.

6. **Priors and noise.**
   - A fading grasp reflex helped the weakest explorer (goal babbling:
     fixed grasp 12% → 28%), not the strongest.
   - DEP switching with pink noise hurt RND.
   - Go-Explore restarts from contact states raised RND's free-block
     grasping (23% → 30%).
   - Heavier, free-block-focused restarts (50% of trials starting
     mid-contact) hurt every method, including the supervised one
     (shaped: fixed grasp 57% → 5%). Held-out trials always start from
     rest, so the policies got too little practice at the approach: a
     train/test shift.

7. **Learned finger coordination.** Finger postures during contact are
   more coordinated than random (PC1 22–33% vs 12%) but not
   low-dimensional (10–11 of 16 components for 90%). This matches
   KINESIS's caution that task-relevant control stays high-dimensional.

**Caveat:** most methods have one seed, and seed-to-seed differences
reach about 15 points on some metrics (Part 4).
