All numbers are on held-out shapes (120 trials per policy, half free
blocks). Means are over 2 seeds unless noted.

- **The hand looks natural now.** Every version 1 policy keeps all hand
  joints within natural limits (0% of steps past them, against 31% for
  version 0's RND). Grasps still work: version 1 RND grasps 75% of fixed
  and 43% of free blocks, against 67% and 23% in version 0.
- **Reward-free exploration still does not lift.** RND lifts 2% of free
  blocks, even with the smaller blocks and the natural hand.
- **The teacher plus a lift bonus teaches lifting.** A+B+D lifts **34%**
  of free blocks (seeds: 32%, 37%) and holds 26% at the end of the
  trial. Without the bonus, A+B lifts 16% and holds 7%.
- **A+B+D manipulates more than the teacher does.** While lifted, it
  carries blocks 6.4 cm and turns them 0.55 rad (script: 3.4 cm,
  0.18 rad). Its lifts are less secure: it knocks off 48% of free
  blocks, against 2% for the script.
- **The residual policy (C) makes the teacher worse, not better.** The
  learned correction is as large as the script's action (ratio 0.9) and
  breaks the script's lift: 82% for the script, 3% for C, and 27% for
  C+D (one seed). C is gentler (10% knocked off) but explores much less
  (842 contact sets, against about 2,250 for A+B).
- **Teaching does not collapse exploration.** A+B and A+B+D explore
  more contact configurations than RND alone (about 2,250–2,300 sets
  against 1,900) and reach more contact cells (560–630 against 430).
- **Smoothness costs help only a little.** Jerk falls from 0.33
  (version 0) to 0.28–0.30, still about 6 times the scripted
  controller's.
