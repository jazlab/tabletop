# Plan: reward-free exploration for the monkey arm (living document)

Started 2026-11-12 01:30. Budget 8 h (until about 09:30), at most 10 CPU
cores at a time. Revisions are appended at the bottom with timestamps.

## The brief, restated

- Monkey skeleton, **right arm only**, **joint control**, MuJoCo, from
  **state** (no vision).
- New env: like the PPO env (servo physics, rest start, `subject.yaml`
  placement) but no left arm. Each trial has **one block**, either **fixed in
  space** or **free on a fixed pedestal**, within reach. Shapes vary a lot:
  rectangular blocks, prisms, polyhedra and more. Size, colour, position and
  pose also vary. Trials last a few seconds, then the block re-spawns.
- Several **unsupervised exploration** algorithms, chosen from the
  literature. Some must use proprioceptive and tactile inputs. The goal is
  to **grab and manipulate without reward**. Supervision or curricula are
  allowed for some methods, as long as others stay purely unsupervised.
- Organised code in one directory. Multi-PDF report in `report/`. Videos of
  trained behaviour.

## What the literature says (research/notes_*.md, about 65 papers)

1. Novelty over the full body state makes a high-DoF arm flail and ignore
   the object (CEE-US, CSD, MUSIC, SkiLD, TacEx). → Compute intrinsic
   rewards on **interaction features**: touch, contact map, wrench, hand-
   to-surface distance and object pose. Keep a full-state version as a
   control.
2. Methods that produced grasping or lifting without task reward:
   - CEE-US: object-centric ensemble disagreement, with MPC;
   - MUSIC: hand–object mutual information; learned pick-up on Fetch;
   - CSD: controllability-aware skills;
   - CCGE: contact-coverage counts on a 16-DoF hand;
   - TacEx: tactile disagreement.
3. At laptop scale, off-policy (SAC/DDPG, n-step) beats PPO. Recompute
   intrinsic rewards at sample time (URLB).
4. Skeletal-model lessons:
   - KINESIS: curriculum by hard-negative mining, mixture of experts, Lattice
     noise; motor priors matter.
   - DEP-RL: self-organised DEP exploration with switching.
   - SAR: synergies, keeping a residual pathway.
   - Static → dynamic curriculum (Chiappa 2024).
   - Grasp reflex (developmental).
   - IMGEP: modular goal babbling with learning progress.

## Environment (done: ue/shapes.py, ue/model.py, ue/env.py)

- Right arm (23 joints) with position servos and gravity compensation, at
  the PPO env's `rl` shoulder placement. **Change:** per-group torque
  limits (fingers 0.4 N·m, wrist 1.5, arm 5). With 5 N·m everywhere,
  fingertips could push about 250 N and crushed blocks into bones.
- Procedural library of 10 families: box (cube, slab, bar, brick), n-prism,
  cylinder, pyramid, frustum, wedge, platonic, bipyramid, random convex
  hull, and compound L/T/cross/step. Largest extent is 3–7 cm. Library
  seeds differ per env worker; evaluation uses held-out seeds.
- 50% fixed (welded, random 3-D pose, on a rod) and 50% free (dropped on a
  pedestal of random height and position, settled).
- Trial: 3 s at 25 Hz (75 steps). Delta joint actions (2.5 rad/s).
- Observation (≈180-D):
  - proprioception: q, q̇, servo targets, palm position, fingertips;
  - touch: 18 patch forces, a 6×6 hand-unit × object-region contact map,
    the wrench on the object;
  - surface distances;
  - object: relative position, displacement, rotation, velocity, size;
  - the fixed flag and trial time.
- Evaluation metrics in `info`, never used as reward by the unsupervised
  methods: touching, n_touch, pinch, grasp (opposed contacts), lifted,
  displacement, rotation, force.

## Learner (ue/agent.py, ue/train.py)

SAC with auto-α (target entropy −|A|/2), 3-step returns, a `[T, env]` ring
buffer, and rewards recomputed on sampled batches. Skill/goal-conditioned
inputs. 8 envs in 1–2 worker processes plus a learner process: about 2
cores per run, so up to 5 concurrent runs.

## Methods (ue/intrinsic.py)

| id | method | inputs to reward | type |
|---|---|---|---|
| contact | CCGE-lite contact-coverage counts + contact-set novelty + reach-to-rare | contact map, obj state bin, surf dist | unsupervised (tactile) |
| touch | tactile disagreement (HaC/TacEx on state) | next contacts + Δobject | unsupervised (tactile) |
| disagreement | object-centric ensemble disagreement (CEE-US-lite, model-free) | interaction feats | unsupervised |
| rnd | RND on interaction features | interaction feats | unsupervised |
| rnd_full | RND on the full observation (naive control) | everything | unsupervised |
| apt | kNN particle entropy | interaction feats | unsupervised |
| music | hand–object MI (MINE), + contact counts to bootstrap | hand pos, obj pose, wrench | unsupervised |
| csf | contrastive successor-feature skills (METRA-like), d=4 | interaction feats | unsupervised skills |
| diayn | 16 discrete skills + RND bonus | interaction feats | unsupervised skills |
| none | max-entropy SAC, zero reward | – | control |
| dep | DEP self-organising controller (no learning) | proprioception | unsupervised, no NN |
| random | uniform random actions | – | control |
| shaped | dense reach + grasp + lift reward | – | **supervised** reference |
| scripted | IK reach-grasp-lift | – | hand-coded reference |

Round-3 candidates, depending on results:
- modular goal babbling (IMGEP + HER + LP) over palm / contact / object
  goals;
- DEP switching inside the best learner;
- grasp-reflex curriculum (fading);
- static → dynamic curriculum (p_fixed annealed);
- finger synergies (SAR-style, with residual);
- Go-Explore restarts from contact states.

## Schedule

| time | work |
|---|---|
| 01:30–02:20 | research (agents) ∥ env |
| 02:20–02:50 | learner, intrinsic modules, smoke tests, throughput |
| 02:50–04:00 | **Round 1** (≈65 min, 5 runs × 2 cores): contact, touch, disagreement, rnd, csf. In parallel: DEP + goal-babbling implementation (agents, ≤1 core each), eval/video tooling |
| 04:00–05:10 | **Round 2**: music, apt, diayn, rnd_full / none, shaped. Evaluate round 1 |
| 05:10–06:40 | **Round 3**: best methods improved (curricula, reflex, DEP switching, goal babbling), longer runs |
| 06:40–07:20 | final evaluations on held-out shapes, videos |
| 07:20–09:15 | report: multiple PDFs (overview, environment, methods, results, per-method appendices, literature) |

## Evaluation protocol

Use held-out shape libraries (seeds 9000+), 120 trials (60 fixed, 60
free), the stochastic policy (skills sampled uniformly), and deterministic
too.

Metrics per trial:
- P(touch), P(pinch), P(grasp), P(lift);
- mean contact time;
- mean number of patches;
- object displacement and rotation;
- coverage of contact cells (36 × 6 × 6) and of object displacement voxels.

For skill methods, also report the best skill per metric.

Videos: a grid of trials per method from the `close` camera, plus a
multi-camera clip for the best ones.

## Revisions

- 01:50: plan written after both literature reviews. The scripted grasp
  controller has been handed to an agent: it is a physics sanity check
  (can this hand lift these blocks at all?).
- 01:53: Round 1 launched (contact, touch, disagreement, rnd, csf; 700k
  steps). Runs are learner-bound at about 1.1 cores, so 3 Round-2 runs
  (music+contact, apt, shaped) started at once, about 9 cores in total.
- 01:55: baselines on held-out shapes (40 trials): random 0% touch, still
  0%, DEP 5% touch. The hanging start makes contact a genuine exploration
  problem.
- 01:57: confound found and fixed. Absolute object orientation and spawn
  offsets identify the trial, so DIAYN's discriminator reached 100% at
  random warm-up. Added `obj_drot` (rotation since spawn). Intrinsic
  `interaction` features now use it, and a stricter `skill` group drops
  spawn-dependent offsets.
- 01:58: profiling under load. A SAC update takes about 42 ms. Ensembles
  and MUSIC added 30–40 ms, so those runs ran at about 70 sps. Changes:
  ensembles (disagreement, touch) now use 128-unit layers, and MUSIC uses a
  separable critic f(a)·g(s) (InfoNCE form, negatives from one matmul).
  disagreement, touch and music_contact were restarted.
- 01:58: early results at about 60k steps. contact: 55% of trials touch,
  6% grasp. rnd: 42% / 4%. shaped (supervised): 52% / 4%. csf: 0%.
- Implemented and ready for Round 3: GoalBabbling (IMGEP with 4 goal
  modules, LP selection, rare-goal sampling, HER), DEP baseline, grasp
  reflex (env `reflex`), annealed env schedules (`schedule:` in configs).
- 02:08: the scripted controller (agent-tuned thumb-against-fingers pinch,
  planned per trial) lifts 60% of free blocks and grasps 67.5%. The
  physics allows lifting, but the small hand and ±30° wrist make it hard.
- 02:10: MUSIC bug found. The first separable version contrasted over
  *hand* states, so with a constant object state (before any contact) it
  rewarded an arbitrary hand region, amplified by std normalisation: touch
  was 1.5% at 80k steps vs 55% for contact alone. It now contrasts over
  object states (pointwise MI, exactly 0 when the object doesn't vary).
  Restarted.
- 02:10: at 150–230k steps: contact 79% touch / 11% grasp / 35% pinch;
  rnd 80/8/29 (and it pushes free blocks about 39 cm, i.e. off the
  pedestal); shaped (supervised) 77/8/45; apt 41/0/6;
  disagreement 21% and touch 19% at 70k (slower); csf 2%. Round 3 queued
  (goals, rnd_full, contact_reflex, goals_reflex, rnd_pink_dep, none,
  diayn).
- Report: literature PDF built (42 pp). Environment and methods drafts
  written.
- 02:52: first held-out results (120 trials).
  - contact (reward-free): fixed 100% touch / 83% pinch / 43% grasp;
    free 93% touch / 5% grasp / 0% lift; 344 contact cells.
  - shaped (supervised): fixed 100 / 97 / 57; free 83 / 8 / 0; 217 cells.
  - Reward-free contact coverage nearly matches supervision on fixed
    blocks and explores about 60% more contact configurations. **Nobody
    lifts free blocks yet**, while the scripted reference lifts 43%.
- 02:52: MUSIC+contact still about 5% touch at 280k steps. Cause: the
  pointwise MI of novel hand–object states is negative, so first touches
  are punished. Added `clip_min: 0` (MUSIC clips its reward). The failed
  run is kept as `r2_music_contact_noclip`; the clipped one is queued.
- Round 4 focus (revised): the free-block gap (grasp → lift). Candidates:
  - longer runs of the best explorers (rnd, disagreement, contact);
  - RND + contact mix;
  - the reflex/curriculum variants already queued in Round 3;
  - goal babbling's `move` module.
  Final videos at a larger tile size (360×480).
- 03:20: held-out evaluations (120 trials).
  - RND (interaction) is the best reward-free explorer:
    - fixed 67% grasp vs 57% for the supervised reward;
    - free 23% grasp vs 8% supervised;
    - most coverage (407 cells, 1964 contact sets);
    - but 37% of free blocks knocked off.
  - APT: 88% knocked off (destructive).
  - No method lifts yet.
- 03:20: two diagnosed failures, fixed in round 4.
  - Goal babbling: rare-goal sampling favours far-from-object palm
    targets, so contact goals were never achieved (0%) and only 2% of
    trials touched. v2 caps reach goals at 6 cm from the object and adds
    a 0.5 contact-coverage bonus.
  - MUSIC (clipped) still about 5% touch. Cause: the trial-identity
    confound again. Negatives come from other trials, and tiny settling
    offsets in displacement/rotation identify the trial. v3 uses only
    object velocity and wrench, which are zero unless the block is acted
    on.
  - Kept as evidence: r2_music_contact_noclip, r2_music_contact_clip.
- 03:20: round-4 queue (max 7 concurrent):
  - music_v3, goals_v2, rnd_long (1.4M steps), rnd_pink_dep,
    rnd_contact, goals_v2_reflex, none, diayn;
  - seed replicates of rnd and contact.
  - r3_goals_reflex dropped (superseded).
- Surprise: RND on the FULL state is also touching (64% at 110k). The
  hanging start may make reaching the object the dominant source of novelty
  even in full state. Compare at the end.
- 03:30: CSF (held-out) never touches the block, but learned 8 consistent
  whole-arm swing primitives: back-up, a forward loop over the object,
  lateral. This is the textbook "skills = body motion" failure
  (figure skillpaths_r1_csf).
- 03:30: added Go-Explore-style restarts (env `restart_p`). Contact states
  with >= 2 hand units are archived and 30% of trials restart from one: a
  curriculum prior aimed at the lift gap. Queued round 5: rnd_restart,
  contact_restart.
- Timeline: the queue (12 runs) should finish by about 06:30; the report
  is written in parallel, with the final build 06:30–09:15.
- 04:31: held-out results.
  - Feature ablation confirms the literature. RND on full state vs
    interaction features: fixed grasp 55% vs 67%, free grasp 12% vs 23%,
    knocked off 70% vs 37%, contact cells 305 vs 407.
  - Reflex + curriculum did not help contact coverage (free grasp 3% vs
    5%; fixed 48% vs 43%).
  - Disagreement gives the first held-out lifts (2% of free blocks).
  - Tactile surprise is weak (fixed touch 15%).
  - In training, RND + restarts reaches 48% grasp and 1.8% lift (best);
    contact + restarts also lifts (1.6%).
- Remaining queue: none, diayn, rnd_seed1, contact_seed1, then the evals
  of everything. Expected end of training about 06:30 (rnd_long). Report
  writing in parallel.
- 05:15: restarts (held-out): RND + restarts reaches free grasp 30% (vs
  23%) but knocks off more (52%); contact + restarts reaches fixed grasp
  52% (vs 43%). Still no held-out lifts. MUSIC v3: 20% touch (failed
  despite 3 fixes). Goals v2: reach primitive 55% success, commanded-reach
  median error 2.2 cm.
- 05:15: round 6 queued (600k steps, aimed at the lift gap):
  free-block-focused restarts (p_fixed 0.25, restart 0.5, free-only
  archive) for rnd, disagreement, and shaped (supervised reference).
  All training should end by about 07:15; final report 07:15–09:15.
- 06:40: seeds and long runs.
  - Held-out lifts appear with experience: RND at 1.4M steps 3%, RND
    seed 1 2%, contact seed 1 2%.
  - Seed variance is up to about 15 points (contact free grasp 5% vs
    17%).
  - DIAYN: 16 object-directed approach skills that rarely touch (38%
    fixed touch).
- 07:20: round 6 (free-focused restarts, 50%) hurt every method, the
  supervised one included (shaped fixed grasp 57% → 5%). Cause: a
  train/test shift (eval always starts at rest).
- 07:20: all training done (27 runs). Final evaluations, figures, videos
  and the report from here on.
- 07:22: report built — 6 PDFs (overview 5 pp, literature 42, environment
  7, methods 5, results 26, appendix 8) + 45 videos in report/videos/.
  Final state: 27 training runs, 29 held-out evaluations (incl. baselines).
