### 1. Natural joint limits cost nothing

- Version 1 RND grasps more than version 0 RND: 75% vs 67% of fixed
  blocks, and 43% vs 23% of free blocks.
- The v1 hand differs in three ways: limited extension, DIP coupling and
  smaller blocks. These runs cannot separate their effects. The smaller
  blocks are the likely cause of the free-block gain, since they now fit
  the hand's opening.
- No version 1 policy uses back-bent fingers. The limits prevent them by
  construction, so the policies do not have to learn to avoid them.

### 2. Lifting needs both the teacher and the task bonus

| | No lift bonus | With lift bonus (D) |
|---|---|---|
| No teacher (RND) | 2% | not run |
| A+B (demos + restarts) | 16% (10%, 22%) | **34%** (32%, 37%) |
| C (residual) | 3% (0%, 7%) | 27% (one seed) |

Free-block lift rates, held out.

- **Without D,** the teacher's demonstrations change *how* the policy
  grasps: grasp rates go up, and first touch comes as fast as RND's.
  Lifts stay rare. RND's novelty reward has no reason to keep a block in
  the air once lifting has been seen.
- **With D,** lifting becomes worth repeating. The lift rate doubles
  under A+B, and the time spent lifted nearly triples (0.25 s against 0.09 s
  per 3 s trial).
- **The deterministic policies agree.** On the deterministic 60-trial
  evaluation, A+B+D lifts 30% and 33% (two seeds) and A+B lifts 7% and
  17%.
- **Seeds agree** to within about 6 points on every grasp and lift rate.

### 3. A+B is the better use of the teacher; C undermines the teacher

The residual policy starts from a controller that lifts 82% of free
blocks, yet it ends below 30%. There are two reasons:

1. **The residual is not small.**
   - Policy actions are scaled by 0.5, and SAC keeps its policy
     stochastic to maximise entropy.
   - The result: the residual is about as large as the script's own
     action (ratio 0.92).
   - A precise pinch-and-lift does not survive noise of that size. During
     training, the lift rate stayed at 1–5% (C) and 15–30% (C+D).
2. **Nothing protects the script's success.** RND rewards new
   interactions, not reproducing the script's lift. The policy has no
   reason to shrink its correction toward zero.

Standard fixes from the residual RL literature were not tried, to keep
the run budget:
- a smaller residual scale, or one that grows from 0 over training;
- a critic warm-start on demonstrations;
- a lower target entropy.

Where C does better than A+B:
- It is gentler: 10% of free blocks knocked off, against 43–48% for A+B.
- It is smoother: jerk 0.20 against 0.28–0.29.

Both follow from the script carrying most of the motion. The cost is
exploration: C's contact sets are well below RND's, and close to the
script's own.

### 4. A+B+D manipulates, but roughly

The A+B+D policy does more with a lifted block than the script does. It
carries it twice as far and turns it three times as much. The script
lifts straight up and holds still; A+B+D swings the arm out.

The same swing knocks off about half of the free blocks, often before
the grasp is secure. That makes A+B+D a more active but less careful
manipulator. It is closer to "play" than to a pick-and-place skill.
Whether that is wanted depends on the goal:
- The knock-offs are part of what RND finds interesting.
- The lift bonus is paid only while the block is lifted, so it does not
  penalise them.

### 5. Teaching keeps exploration broad

A concern with demonstrations is that the policy collapses onto the
teacher's single grasp. That did not happen with A+B:
- A+B and A+B+D visit about 18–21% more contact sets than RND alone,
  and 30–47% more hand-unit × object-face contact cells.
- The script visits 622 sets.

A plausible reason: restarts from demonstration states put the policy in
contact configurations (fingers around a lifted block) that RND would
rarely reach. RND then explores around them.

### 6. Smoothness costs: a small effect

- The effort and jerk costs lower jerk only modestly: 0.33 for version 0
  RND, 0.30 for version 1 RND, 0.28–0.29 with the teacher.
- The learners' movements are still visibly jittery compared with the
  script (jerk 0.05).
- The costs were kept small (about 17% of RND's spread) so that they
  would not suppress exploration. A stronger jerk weight, pink action
  noise or action smoothing would be the next step.

### 7. Limits of these results

- One run budget (700k steps, about 9,300 trials) and 2 seeds (1 for
  C+D).
- No hyperparameter search for A, B, C or D. In particular, the BC
  weight, the restart schedule, the residual scale and the bonus size
  are all first choices.
- Held-out evaluation on 60 free and 60 fixed trials per policy. A
  10-point difference between two single runs is within noise; seed
  agreement is the better guide.
