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


\pagebreak

## Appendix A: Unsupervised RL, skill discovery and touch-based curiosity


Project context: a simulated monkey right arm in MuJoCo (7 arm joints + 16 finger joints, joint-position control, contacts) learning from low-dimensional state to reach, grab and manipulate one block per trial (the block is either fixed in space or resting free on a platform). Compute: one Apple Silicon laptop (10 CPU cores, torch MPS), about 5 h of total training across all methods.

Provenance:
- **[read]**: details I checked against the arXiv abstract or HTML (method and/or hyperparameter appendix) during this review.
- **[known]**: well-established details I did not re-fetch in this review. Equations and the main hyperparameters are standard. Double-check exact numbers against the code before relying on them.

---

### 0. TL;DR

1. **Generic novelty over the full state mostly produces arm/finger flailing.** URLB's data-based methods (APT, ProtoRL) and RND do best on DMC locomotion. In manipulation, however, every paper that looked (CEE-US, CSD, MUSIC, CCGE, SkiLD) found that state novelty, curiosity and skill methods over the whole state ignore the object, because moving the robot is easier than moving the object.
2. **What made objects move without task reward:**
   - (a) restricting the intrinsic objective to **object or hand-object state** (MUSIC, CSD, RND-Dist, DUSDi/SkiLD factorisation);
   - (b) **controllability-aware or "hard-transition" weighting** (CSD, SUSD);
   - (c) **contact-based rewards** (Vulin; ToC/HaC; CCGE contact coverage);
   - (d) **structured, object-factored world-model uncertainty** (CEE-US, RaIR).
3. **Practical recipe for our budget:** an off-policy actor-critic (SAC or DDPG with n-step returns) at a moderate update-to-data ratio, running-std reward normalisation, and an intrinsic reward computed on a *small hand-crafted feature vector*: object pose/velocity, fingertip-to-object distances, a per-finger contact vector, and contact forces. Use skill methods (METRA/CSF/CSD) on that same object-centric feature space.

---

### 1. Benchmarks and surveys

#### 1.1 URLB: Unsupervised Reinforcement Learning Benchmark [read]
- **Citation:** Laskin, Yarats, Liu, Lee, Zhan, Lu, Cang, Pinto, Abbeel. NeurIPS 2021 Datasets & Benchmarks. arXiv:2110.15191.
- **Method/setup:** Two phases: reward-free pretraining (up to 2M frames), then fine-tuning on 12 DMC tasks (Walker, Quadruped, Jaco reach) for 100k frames. It groups methods into three families:
  - knowledge-based (ICM, Disagreement, RND), with reward = prediction error or variance;
  - data-based (APT, ProtoRL), with reward = kNN particle entropy;
  - competence-based (SMM, DIAYN, APS), with reward = log q(z|s).
- **Hyperparameters:** DDPG base agent, hidden 1024, lr 1e-4, batch 1024, n-step 3, update every 2 env steps, gamma 0.99, buffer 1e6. Skill dimension for DIAYN is 16 (one-hot); APS uses a 10-d successor-feature space [known].
- **Results:** From states, the data-based methods (APT, ProtoRL) and RND are best. Competence-based methods lag behind, and none solve the benchmark. Longer pretraining does not reliably help: performance was flat or worse in 9 of 18 settings. The Jaco reach (manipulation) domain is the hardest for every method, and competence-based methods do especially poorly there.
- **Relevance to our project:** This is the reference implementation for 8 algorithms (rll-research/url_benchmark) and gives sensible DDPG defaults. It shows that skill methods are weak on arm reaching unless the skill space is restricted.

#### 1.2 ExORL: Don't Change the Algorithm, Change the Data [known]
- **Citation:** Yarats, Brandfonbrener, Liu, Laskin, Abbeel, Lazaric, Pinto. 2022. arXiv:2201.13425.
- **Method:** Collects reward-free datasets with the URLB explorers, relabels them with task rewards, and runs offline RL (TD3, CQL).
- **Results:** Data from RND, APT and ProtoRL lets plain TD3 solve many tasks. The *diversity of exploratory data* matters more than the offline algorithm.
- **Relevance to our project:** We can evaluate each explorer by the coverage of its replay buffer (object displacement, contact diversity, grasp events) and by offline relabelling. This is cheaper than fine-tuning every method.

---

### 2. Knowledge-based (prediction-error / uncertainty) methods

#### 2.1 ICM: Curiosity-driven Exploration by Self-supervised Prediction [known]
- **Citation:** Pathak, Agrawal, Efros, Darrell. ICML 2017. arXiv:1705.05363.
- **Method:** An inverse model trains features φ; a forward model predicts φ(s') from (φ(s), a).
- **Reward:** r = η/2 · ‖f(φ(s), a) − φ(s')‖².
- **Hyperparameters:** A3C; β = 0.2 weights the forward loss against the inverse loss.
- **Results:** Exploration in VizDoom and Mario.
- **Relevance to our project:** Low. With state input and contact-induced stochasticity, the forward error is dominated by aleatoric noise from contact chatter (the "noisy-TV" problem). The baseline is only useful if the forward error is restricted to object-state prediction.

#### 2.2 RND: Exploration by Random Network Distillation [known]
- **Citation:** Burda, Edwards, Storkey, Klimov. ICLR 2019. arXiv:1810.12894.
- **Method:** A predictor f_θ regresses a fixed random network g on next states.
- **Reward:** r = ‖f_θ(s') − g(s')‖².
- **Key tricks:** Normalise observations with running mean/std and clip to [-5, 5]. Divide the intrinsic reward by a running std of intrinsic returns. Use separate value heads for intrinsic and extrinsic rewards, a non-episodic intrinsic return, and γ_int = 0.99. Train the predictor on a fraction of each batch (25%) to slow down novelty decay.
- **Results:** First to reach above-human scores on Montezuma's Revenge.
- **Relevance to our project:** Very cheap (two small MLPs). It is the strongest simple knowledge-based method in URLB from states. Apply it to *object and hand-object features* rather than the full proprioceptive state (see RND-Dist, §6.4).

#### 2.3 Disagreement: Self-Supervised Exploration via Disagreement [known]
- **Citation:** Pathak, Gandhi, Gupta. ICML 2019. arXiv:1906.04161.
- **Method:** An ensemble of K forward models.
- **Reward:** the variance across members, r = Var_k[f_k(s, a)] (summed over dimensions). The reward is differentiable with respect to the action, which enables "differentiable exploration".
- **Results:** Robust to stochasticity (aleatoric noise averages out); a real Sawyer arm learned to push objects.
- **Hyperparameters:** K = 5 (URLB uses an ensemble of 5 MLPs, hidden 1024).
- **Relevance to our project:** A good fit, because MuJoCo contacts are noisy in practice and disagreement suppresses that noise. Predict only object state deltas plus the contact vector.

#### 2.4 Plan2Explore: Planning to Explore via Self-Supervised World Models [known]
- **Citation:** Sekar, Rybkin, Daniilidis, Abbeel, Hafner, Pathak. ICML 2020. arXiv:2005.05960.
- **Method:** A Dreamer (RSSM) world model plus an ensemble of one-step latent predictors. The disagreement variance is the reward for an imagination-trained exploration policy.
- **Results:** Zero-shot/few-shot adaptation on DMC from pixels.
- **Relevance to our project:** Shows that optimising the *expected future* novelty inside a model is more sample-efficient than reacting to past novelty. Training a full Dreamer is too expensive for us. A cheap analogue is a one-step ensemble used for reward in an off-policy learner, or MPC (see CEE-US).

#### 2.5 CEE-US: Curious Exploration via Structured World Models Yields Zero-Shot Object Manipulation [read]
- **Citation:** Sancaktar, Blaes, Martius. NeurIPS 2022. arXiv:2206.11403.
- **World model:** An ensemble (M = 5) of message-passing GNNs, one node per object plus a global agent node, fully connected, one message-passing step. It predicts state deltas.
- **Reward:** r = tr(Cov_m[f_m(s, a)]), the ensemble disagreement.
- **Planner:** iCEM MPC, 128 samples, horizon 30, 10 elites, 3 iterations, colored-noise exponent β = 3.5, replanning every step.
- **Hyperparameters:** MLP blocks 2×128 (Construction); 2,000 transitions per iteration; 300 iterations (about 72 h on one RTX 3060, though lifting appeared after about 30 iterations ≈ 60k steps).
- **Results:** In the Fetch-style Construction environment with 4 blocks, free play yields lifting, flipping and throwing. Zero-shot downstream success with the same MPC: stacking 2 blocks 0.91 (MLP model 0); pick & place with 4 objects 0.97. URLB baselines (Disagreement, RND, ICM with DDPG) "mostly stay in free space or at walls". Offline RL on CEE-US data beats MLP-model data.
- **Relevance to our project:** The strongest evidence that *object-factored epistemic uncertainty* yields grasping and lifting from state with no reward. The full MPC loop is slow (72 h on a GPU). For us, the takeaway is to use ensemble disagreement over *object-centric* predictions (a factored or small model) as the reward for a model-free learner. A short-horizon iCEM explorer is an optional stretch goal.

#### 2.6 RaIR: Regularity as Intrinsic Reward for Free Play [read]
- **Citation:** Sancaktar, Piater, Martius. NeurIPS 2023. arXiv:2312.01473.
- **Method:** A relational descriptor ϕ(s_i, s_j) is computed for every object pair (e.g. absolute relative x-y position, discretised to bins of size b). The reward is the negative Shannon entropy of the histogram of descriptors: r = Σ_x p(x) log p(x). It rewards symmetric or regular configurations.
- **Combined reward:** r = r_RaIR + λ · r_Dis, with λ = 0.1 (Construction) and λ = 0.02 (Quadruped). Planned with iCEM on CEE-US GNN ensembles.
- **Results:** Zero-shot tower building improves (Singletower-3: 0.75 vs 0.40 for CEE-US). Pure RaIR (λ = 0) causes too few interactions, so the disagreement term is necessary.
- **Relevance to our project:** Low. We have a single object, so pairwise regularity among objects is degenerate. The idea of "regularity" could be applied to hand-object relations (e.g. symmetric finger placement on block faces), but that is speculative. Main lesson: structure-seeking rewards need an exploration term alongside them.

---

### 3. Data-based (state-entropy maximisation) methods

#### 3.1 APT: Behavior From the Void: Unsupervised Active Pre-Training [known]
- **Citation:** Liu, Abbeel. NeurIPS 2021. arXiv:2103.04551.
- **Method:** A particle-based (kNN) estimate of state entropy computed within a minibatch.
- **Reward:** r(s) = log(1 + (1/k) Σ_{j∈kNN(s)} ‖h(s) − h(s_j)‖), with k = 12 and averaging over the neighbours. h is a contrastively learned embedding (from pixels); for states, URLB uses the raw state or a learned forward/inverse embedding.
- **Learner:** DDPG/DrQ.
- **Results:** Strongest URLB family overall.
- **Relevance to our project:** Very simple and needs no extra network if computed on a hand-designed feature vector. Caveat: entropy over all 23 joint angles rewards flailing. Use object pose plus fingertip-object relative vectors.

#### 3.2 ProtoRL: Reinforcement Learning with Prototypical Representations [known]
- **Citation:** Yarats, Fergus, Lazaric, Pinto. ICML 2021. arXiv:2102.11271.
- **Method:** SwAV-style prototypes give a representation. The kNN entropy reward is computed against a queue of past embeddings (a coverage estimate across time rather than within a batch).
- **Hyperparameters:** 512 prototypes, k = 3, queue 2048.
- **Relevance to our project:** Low as a separate method. The useful idea is to compute kNN against a *memory of past states*, not just the current batch, which estimates coverage across the whole run.

#### 3.3 SMM: Efficient Exploration via State Marginal Matching [known]
- **Citation:** Lee, Eysenbach, Parisotto, Xing, Levine, Salakhutdinov. 2019. arXiv:1906.05274.
- **Method:** Matches the state marginal to a target p*(s) (uniform by default).
- **Reward:** r = log p*(s) − log q(s) (a VAE density), plus a skill-mixture term (log d(z|s) − log p(z)).
- **Results:** Fetch block-pushing, where the target places mass on object positions.
- **Relevance to our project:** The idea of a *target distribution over object positions* (e.g. uniform over the platform, or lifted heights) is an easy way to inject a prior.

#### 3.4 Particle-based entropy estimators in general [known]
- **Kozachenko-Leonenko estimator:** Ĥ ∝ Σ_i log ‖s_i − s_i^{kNN}‖ + const.
- **APT's change:** log(1 + mean distance to the k neighbours), with k = 10–12, for stability.
- **Practical pitfalls:** The reward scale is set by the feature scale, so normalise features per dimension (running std). Batch-level kNN shrinks as the policy concentrates, so use a large batch (≥1024) or a FIFO memory of about 10k states. Reward normalisation is needed when mixing with other terms.

---

### 4. Competence-based (mutual-information / skill-discovery) methods

#### 4.1 DIAYN: Diversity Is All You Need [known]
- **Citation:** Eysenbach, Gupta, Ibarz, Levine. ICLR 2019. arXiv:1802.06070.
- **Objective:** maximise I(S; Z) + H(A|S, Z) with a discrete skill z (e.g. 50 skills).
- **Reward:** r = log q_φ(z|s) − log p(z).
- **Learner:** SAC, α = 0.1, hidden 300×2.
- **Weakness:** Converges to static, easily discriminable poses with poor coverage. Under the I(S; Z) objective, moving the arm to distinct places is enough, so the object is ignored (shown by CSD, MUSIC and DUSDi).
- **Relevance to our project:** Use it as a baseline, and only with the discriminator input restricted to object or hand-object features.

#### 4.2 DADS: Dynamics-Aware Unsupervised Discovery of Skills [known]
- **Citation:** Sharma, Gu, Levine, Kumar, Hausman. ICLR 2020. arXiv:1907.01657.
- **Objective:** maximise I(S'; Z | S) using a skill-dynamics model q(s'|s, z).
- **Reward:** r = log q(s'|s, z) − log (1/L) Σ_i q(s'|s, z_i), with L = 500 prior samples.
- **Results:** Predictable skills usable for model-based (MPC) composition.
- **Learner:** On-policy-ish SAC.
- **Relevance to our project:** Medium-low. It is expensive and URLB/METRA show weak coverage. Predictable skills are attractive for later MPC, though.

#### 4.3 APS: Active Pretraining with Successor Features [known]
- **Citation:** Liu, Abbeel. ICML 2021. arXiv:2108.13956.
- **Reward:** r = r_APT(s) + log q(s|z), where the second term is a successor-feature-style term φ(s)ᵀz with z on the unit sphere.
- **Results:** Combines entropy and skills; good in URLB on Walker, weaker on Jaco.
- **Relevance to our project:** Shows that "exploration + skill" sums help. It is an easy add-on to APT.

#### 4.4 CIC: Contrastive Intrinsic Control [known]
- **Citation:** Laskin, Liu, Peng, Yarats, Rajeswaran, Abbeel. NeurIPS 2022. arXiv:2202.00161.
- **Representation:** A contrastive (CPC/NCE) loss between transition embeddings g(s, s') and skill embeddings, with temperature 0.5.
- **Reward:** kNN particle entropy over the *transition embedding* (APT-style, k = 12). The discriminator is used for representation only, not as reward.
- **Hyperparameters:** Skill dim 64 (continuous uniform [0, 1]^64), hidden 1024, DDPG as in URLB.
- **Results:** State of the art on URLB states at publication (+79% interquartile mean over the next best), including the best result on Jaco.
- **Relevance to our project:** A strong, simple, well-documented method that drops into a URLB-style codebase. Feed the embedding networks object-centric features.

#### 4.5 LSD: Lipschitz-constrained Unsupervised Skill Discovery [known]
- **Citation:** Park, Choi, Kim, Lee, Kim. ICLR 2022. arXiv:2202.00914.
- **Reward:** r = (φ(s') − φ(s))ᵀz, with φ spectrally normalised to be 1-Lipschitz in *Euclidean state distance*. This forces large state changes.
- **Results:** Larger coverage than DIAYN/DADS on Ant and Humanoid.
- **Relevance to our project:** Superseded by CSD and METRA. The Euclidean metric means "big arm motions".

#### 4.6 CSD: Controllability-Aware Unsupervised Skill Discovery [read]
- **Citation:** Park, Lee, Lee, Abbeel. ICML 2023. arXiv:2302.05103.
- **Distance:** d_CSD(s, s') = (s' − μ_θ(s))ᵀ Σ_θ(s)⁻¹ (s' − μ_θ(s)). It is proportional to −log q_θ(s'|s), where q_θ is a Gaussian density model fitted to current skill transitions, so rare (hard-to-achieve) transitions count as "far".
- **Constraint:** ‖φ(s) − φ(s')‖ ≤ d_CSD(s, s'), enforced by dual gradient descent with λ (init 3000, slack ε = 1e-6).
- **Reward:** r = (φ(s') − φ(s))ᵀz, optimised with SAC.
- **Hyperparameters (manipulation):** 2-hidden-layer 256 MLPs, lr 1e-3, batch 256, γ = 0.98, 10 gradient steps per episode, skill dim 2–3 (continuous) or 16 discrete. Fetch runs use 40k epochs × 2 episodes × 50 steps ≈ 4M steps.
- **Results:** In FetchPush, Slide and PickAndPlace, CSD is the *only* method among CSD, LSD, DIAYN and DADS that learns to move the object without supervision; the other three mostly move the arm. In Kitchen, CSD learns about 10/13 tasks vs LSD 2, DIAYN 4, DADS 0. Disagreement gives more chaotic behaviour and lower coverage.
- **Relevance to our project:** Highly relevant: it directly addresses "arm motion is easy, object motion is hard". Note that Fetch had object state *in the observation*, and that took about 4M steps; we must cut that heavily.

#### 4.7 METRA: Scalable Unsupervised RL with Metric-Aware Abstraction [read]
- **Citation:** Park, Rybkin, Levine. ICLR 2024. arXiv:2310.08887.
- **Objective:** maximise E[Σ_t (φ(s_{t+1}) − φ(s_t))ᵀz] subject to ‖φ(s) − φ(s')‖₂ ≤ 1 for adjacent states. This is equivalent to bounding latent distances by temporal distance.
- **Dual updates:** φ maximises (φ(s') − φ(s))ᵀz + λ·min(ε, 1 − ‖φ(s) − φ(s')‖²); λ minimises λ·min(ε, 1 − ‖·‖²).
- **Reward:** r = (φ(s') − φ(s))ᵀz.
- **Skill prior:** continuous isotropic Gaussian, or zero-centred one-hot.
- **Hyperparameters:** SAC; skill dim 2 (Ant, Humanoid), 4 (Quadruped), 16 discrete (Cheetah), 24 discrete (Kitchen); update-to-data ratio 1/4 to 1/16; [known] 1024-unit 2-layer MLPs, lr 1e-4, batch 256, 8 parallel workers, 8 trajectories per epoch.
- **Results:** Best state coverage in most domains. It is the first method to get locomotion from pixels on Quadruped and Humanoid. Zero-shot goal reaching beats LEXA, and hierarchical control is strong.
- **Limitations stated by the authors:** Symmetric metric; linear latent behaviour; low sample efficiency (designed for wall-clock).
- **Relevance to our project:** The temporal-distance metric means the latent space covers the directions that take longest to traverse. For a free block those directions are *object displacement*, which is good. However, it needs millions of steps. Use a small skill dim (2–4) and object-centric inputs to φ.

#### 4.8 CSF: Contrastive Successor Features ("Can a MISL Fly?") [read]
- **Citation:** Zheng, Tuyls, Peng, Eysenbach. 2024/2025 (arXiv v2 March 2025; ICLR 2025 [known]). arXiv:2412.08021.
- **Representation loss:** InfoNCE with an inner-product critic, (φ(s') − φ(s))ᵀz, against in-batch negatives z' (256). The negative log-sum-exp term is scaled by ξ = 5.
- **Reward:** r = (φ(s') − φ(s))ᵀz, with z ~ Unif(S^{d−1}).
- **Policy learning:** A vector-valued successor-feature critic ψ(s, a, z) ∈ R^d is trained with TD toward φ(s') − φ(s) + γψ̄(s', a', z). The policy maximises ψᵀz. There is no dual variable or Lagrangian.
- **Ablations:** The inner-product critic is crucial; MLP or kernel critics fail. The information-bottleneck reward (dropping the negative term) is crucial.
- **Hyperparameters:** lr 1e-4, hidden 1024 (one hidden layer), batch 256, EMA τ = 5e-3, SAC with automatic entropy tuning, 8 workers × 8 trajectories per round, 50–200 gradient steps per round, horizon 200 (50 for Kitchen). Skill dim: Ant 2, Cheetah 2, Quadruped 4, Humanoid 8, Kitchen 4, Robobin 9.
- **Results:** Matches METRA on coverage, zero-shot goal reaching and hierarchical control, with fewer moving parts and more stability.
- **Follow-up:** Reizinger et al. 2025 (arXiv:2507.14748) prove CSF recovers ground-truth state features up to a linear map.
- **Relevance to our project:** The simplest modern distance-style skill method. It is the preferred skill-discovery implementation for us (no dual λ to tune).

#### 4.9 DUSDi: Disentangled Unsupervised Skill Discovery for Efficient Hierarchical RL [read]
- **Citation:** Hu, Wang, Stone. NeurIPS 2024. arXiv:2410.11251.
- **Setup:** A factored state S = (S¹, …, S^N); the skill z = (z¹, …, z^N) has one k-way discrete component per factor.
- **Objective:** Σ_i [I(S^i; z^i) − λ I(S^{−i}; z^i)], with λ = 0.1, using DIAYN-style discriminators per factor.
- **Value factorisation:** One Q-head per reward term (summed for the policy), which reduces variance.
- **Learner:** SAC; a downstream PPO high-level policy picks skills every 50 steps.
- **Results:** DCI disentanglement 0.86 vs 0.016 (DIAYN). Beats DIAYN, CIC, CSD and METRA on multi-factor downstream tasks.
- **Relevance to our project:** Medium. We have natural factors: arm/wrist pose, hand aperture/finger config, object position, object orientation. A DUSDi-style factorised discriminator with an *object factor* forces some skill components to control the object. The value-factorisation trick (separate critics per reward term) is broadly useful when summing intrinsic rewards.

#### 4.10 SkiLD: Unsupervised Skill Discovery Guided by Factor Interactions [read]
- **Citation:** Wang, Hu, Chuck, Chen, Martín-Martín, Zhang, Niekum, Stone. NeurIPS 2024 [known] (arXiv Oct 2024). arXiv:2410.18416.
- **Skill:** z = (target local-dependency graph g, diversity index b). Dependencies are detected with pointwise conditional MI from a learned attention dynamics model.
- **Reward:** r = 1[induced graph = g] · (1 + λ log q(b|s, g)), with λ = 0.5 and hindsight relabelling. The graph-selection policy is rewarded with 1/√count(g).
- **Learners:** TD3/Rainbow skill policy; PPO graph selector.
- **Results:** The only method that achieves complex multi-object interactions (soak rag, cut peach) in Mini-behavior and iGibson. DIAYN and CSD mostly manipulate single objects.
- **Relevance to our project:** The concept translates directly: *reward inducing specific interactions* (e.g. "finger i in contact with the object"; "object velocity depends on hand"). Our contact vector gives these dependency graphs for free, with no learned causal model. A cheap version is "contact-pattern skills": z = target set of fingers in contact.

#### 4.11 SUSD: Structured Unsupervised Skill Discovery through State Factorization [read: abstract/summaries]
- **Citation:** Hosseini, Soleymani Baghshah. ICLR 2026. arXiv:2602.01619.
- **Method:** A distance-maximising (METRA/CSD-style) skill objective per state factor. The factor-wise rewards are weighted by curiosity, w_i ∝ −log q(s'_i | s), from a density model, so under-explored or hard-to-control factors get more weight.
- **Results:** Beats LSD, CSD, METRA, DIAYN and DUSDi on factorised environments (Multi-Particle, Kitchen).
- **Relevance to our project:** Supports the recipe "per-factor METRA/CSF reward + upweight the object factor by its novelty".

#### 4.12 Choreographer: Learning and Adapting Skills in Imagination [read: abstract]
- **Citation:** Mazzaglia, Verbelen, Dhoedt, Lacoste, Rajeswar. ICLR 2023. arXiv:2211.13350.
- **Method:** A DreamerV2-style world model. Exploration is decoupled from skill learning: a VQ-VAE codebook over latent states defines skills (reach codebook codes), learned in imagination. A meta-controller evaluates skills in imagination for adaptation.
- **Results:** Best URLB results from states and pixels at the time; good sparse-goal finding in Meta-World.
- **Relevance to our project:** The conceptual takeaway is "explore first (data-based), then fit skills as goal-reaching to clustered states". The world-model training is too costly for a 5-hour budget.

#### 4.13 MUSIC: Mutual Information State Intrinsic Control [read]
- **Citation:** Zhao, Gao, Abbeel, Tresp, Xu. ICLR 2021 (spotlight). arXiv:2103.08107.
- **Method:** Split the state into *agent state* Sᵃ (gripper position) and *surrounding state* Sˢ (object position), and maximise I(Sᵃ; Sˢ). The MI is estimated with MINE: a statistics network T_φ is trained with E_joint[T] − log E_marg[e^T], where marginal samples come from shuffling agent states in time within a trajectory.
- **Per-transition reward:** r = 0.5 Σ_{i=t,t+1} T(sˢ_i, sᵃ_i) − log(0.5 Σ_i e^{T(sˢ_i, s̄ᵃ_i)}), scaled by α (5000) and clipped.
- **Hyperparameters:** DDPG/SAC, 3×256 MLP, lr 1e-3, batch 256, 16 workers.
- **Results:** With *no task reward*, the agent learns reaching, pushing and **picking up** the block in Fetch Pick&Place (first time reported). As an add-on it boosts DISCERN success on Push from 21% to 95% and on Pick&Place from 4% to 49%. Combining it with DIAYN gives diverse manipulation skills.
- **Relevance to our project:** Very high and cheap. Our agent/surround split is natural: hand and fingertip positions vs block pose. The reward is non-zero only when the object moves *with* the hand. That is exactly grasp-and-move, and it is zero for a fixed block, which is a limitation there. Use contact/force features as Sˢ for the fixed block.

#### 4.14 Empowerment-based manipulation [read: abstract]
- **Citation:** Dai, Xu, Hofmann, Williams. "An Empowerment-based Solution to Robotic Manipulation Tasks with Sparse Rewards." RSS 2021 / Autonomous Robots 2023. arXiv:2010.07986.
- **Method:** Intrinsic reward = estimated empowerment max I(A; S'_object | S), i.e. the MI between actions and future *object* states, with a neural (MINE-like) estimator. It needs novelty-driven exploration early to bootstrap the estimator. Combined with a diversity term.
- **Results:** Beats other intrinsic methods on object lifting and pick-and-place with sparse rewards.
- **Relevance to our project:** Conceptually ideal ("states where my actions affect the object"), but estimator-heavy. A cheap proxy is the disagreement of an *action-conditioned* object-delta predictor, or MUSIC.

#### 4.15 Asymmetric self-play for manipulation skill discovery [read: abstract]
- **Citation:** Jansonnie, Wu, Perez, Peters. "Unsupervised Skill Discovery for Robotic Manipulation through Automatic Task Generation." IEEE-RAS Humanoids 2024. arXiv:2410.04855.
- **Method:** Asymmetric self-play (Alice proposes goal states by acting, Bob reaches them) generates tasks. The behaviours are embedded as Multiplicative Compositional Policy primitives and composed by HRL.
- **Results:** Skills are more object-interactive than skill-learning baselines; transferred to a real robot.
- **Relevance to our project:** Medium-low. Self-play doubles the policy cost, but "Alice moves the block, Bob reproduces the block pose" is a powerful later stage.

---

### 5. Touch / contact / haptics as intrinsic signal

#### 5.1 Vulin et al., Tactile intrinsic motivation [read: abstract; details partly known]
- **Citation:** Vulin, Christen, Stevšić, Hilliges. "Improved Learning of Robot Manipulation Tasks via Tactile Intrinsic Motivation." IEEE RA-L 6(2):2194–2201, 2021. arXiv:2102.11051.
- **Method:** The intrinsic reward is proportional to the summed contact forces between finger force sensors and the object (clipped/scaled) [known]. Contact-Prioritized Experience Replay (CPER) preferentially samples episodes and transitions with contact. Built on DDPG + HER [known].
- **Results:** Faster learning on Pick-and-Place, Push and Slide (sparse goals) than HER, PER, and curiosity baselines.
- **Relevance to our project:** Cheap and directly implementable: MuJoCo gives finger-object contact forces through `mj_contactForce` or touch sensors. The reward must be *bounded and saturating*, otherwise the agent learns to squeeze. Contact-prioritised replay is a trivially cheap add-on for any of our learners.

#### 5.2 ToC / HaC: Touch-based and Haptics-based Curiosity [read]
- **Citation:** Rajeswar, Ibrahim, Surya, Golemo, Vazquez, Courville, Pinheiro.
  - "Touch-based Curiosity for Sparse-Reward Tasks." arXiv:2104.00442 (2021).
  - Published as "Haptics-based Curiosity for Sparse-reward Tasks," CoRL 2021, PMLR 164:395–405 (proceedings dated 2022).
- **Reward:** r = (1−λ)·‖ĥ_t − h_t‖ + λ·‖ẑ_{t+1} − z_{t+1}‖. The first term is the *cross-modal* error of predicting force/torque (touch) from vision; the second is a forward-dynamics error. λ is swept (≈0.5 balances).
- **Hyperparameters:** SAC, lr 3e-5, batch 128, reward scale 100, 2×128 MLP, 1M steps.
- **Results:** Beats SAC, ICM and RND on Push, Open-door and Pick-up (MiniTouch, Panda arm). The touch-only variant gives about 4× more interactions, but touch-only can lead to "passive constant contact" with little object movement, so the dynamics term is needed.
- **Relevance to our project:** The cross-modal idea maps to state as "predict the contact vector or force from proprioception + object pose". Contacts that geometry does not predict (new contact configurations) are surprising. Pitfall: force signals are non-smooth, so predict *binary* contacts or log-forces.

#### 5.3 CCGE: Contact Coverage-Guided Exploration for General-Purpose Dexterous Manipulation [read]
- **Citation:** Liu, Qiao, Tie, Liu, Lou, Gao, Xu, Shao. arXiv:2603.10971 (March 2026).
- **Contact definition:** A finger f contacts object surface region k if its nearest keypoint is within δ_dist of an object point and force > δ_force. Object surface points are clustered into K regions (FPS + K-means on position and normal).
- **Object-state conditioning:** Object state is hashed by an autoencoder with a binary code and SimHash (H = 5 bits), giving S clusters. Counts are kept as C[s, f, k].
- **Contact reward:** mean over contacting fingers of g(C[s, f, k]), with g(c) = 1/√(c+1).
- **Energy-based reaching reward:** for each finger, Σ_k g(C[s, f, k]) · exp(−dist(f, region k)/δ), which pulls fingers toward *rarely contacted* regions.
- **Anti-premature-convergence:** Both terms only pay out for *increases* above the episode's running max.
- **Scales:** α = 200, β = 1.28 (retrieval task).
- **Learner:** PPO, Isaac Gym, 2048 environments; LEAP hand (16 DoF) on an xArm.
- **Results:** Average success 91% vs RND-Dist 65%, task reward only 62%, learned-hash counts (LHCC) 62%, Haptics curiosity (HaC) 58%. Constrained retrieval 88% vs 0% for all baselines. 2–3× fewer steps. The object-state-conditioned counter is crucial (Box Push 100% vs 18% single-state).
- **Relevance to our project:** **The most directly relevant recent paper**: a 16-DoF hand, contacts, low-dimensional state. It is task-reward-augmented in the paper, but the contact-coverage reward works on its own as a reward-free exploration objective. For a block, the K regions are the 6 faces (or 6 faces × 4 quadrants), so no point-cloud clustering is needed. Object-state clusters can come from a coarse grid over block position/orientation (or "fixed" vs "lifted").

#### 5.4 Gentle manipulation with curiosity [known]
- **Citation:** Huang, Hao, Gal, et al. "Learning gentle object manipulation with curiosity-driven deep RL." 2019. arXiv:1903.08542.
- **Method:** Penalises impact force ("pain") and uses a *surprise about pain* curiosity term. Without it, agents learn to hit objects or avoid them entirely.
- **Relevance to our project:** Pitfall reference. Force-based rewards need a penalty or saturation on large forces, otherwise the arm learns to slam the block (and the fixed block gives arbitrarily high forces).

#### 5.5 RND-Dist: Curiosity-driven learning of joint locomotion and manipulation [read: abstract]
- **Citation:** Schwarke, Klemm, van der Boon, Bjelonic, Hutter. CoRL 2023, PMLR 229:2594–2610 (no arXiv found).
- **Method:** RND applied *only to a low-dimensional task-relevant state* (e.g. hand-object distance, door angle) rather than full observations, with PPO and a single sparse +1 task reward.
- **Results:** Learns door opening and package moving on a real wheeled-legged robot.
- **Relevance to our project:** Practical confirmation that **RND on a curated low-dimensional "curiosity state"** works where full-state RND fails. This is the CCGE baseline "RND-Dist" (65% vs 91%).

---

### 6. Other relevant pieces (brief)

- **LEXA, "Discovering and Achieving Goals via World Models"** (Mendonca, Rybkin, Daniilidis, Hafner, Pathak; NeurIPS 2021; arXiv:2110.09514) [known]. An explorer (Plan2Explore) plus an achiever (goal-conditioned, temporal-distance reward) trained in imagination; tested on Robobin and Kitchen. *Relevance:* explore-then-goal-reach structure; METRA and CSF benchmark against it.
- **Go-Explore** (Ecoffet et al.; arXiv:1901.10995) [known]. Archive of cells; return to a cell, then explore. *Relevance:* with a simulator we can reset to archived states (e.g. "block grasped" states) to amplify rare events. Cheap and very effective in MuJoCo via `mj_setState`, if resets from saved states are allowed.
- **#Exploration** (Tang et al.; NeurIPS 2017; arXiv:1611.04717) [known]. SimHash counts with r = β/√n(φ(s)). *Relevance:* for our low-dimensional object/contact features, explicit discretised counts are simpler than RND and non-stationarity-free.
- **Constrained Ensemble Exploration for Unsupervised Skill Discovery** (Bai et al.; ICML 2024; arXiv:2405.16030) [read: title/abstract only]. An ensemble of skills, each maximising state entropy within a partitioned region. Not examined further.
- **Reizinger et al. 2025** (arXiv:2507.14748). Identifiability theory for CSF-style representations.

---

### 7. Practical lessons for manipulation from state with intrinsic rewards

1. **Choose the intrinsic "lens" deliberately.**
   - Every successful manipulation result computed the intrinsic reward on object or interaction variables (MUSIC: agent vs object; CSD: hard transitions; RND-Dist; CCGE: contacts; DUSDi/SkiLD: factors).
   - Full-state novelty is dominated by the many cheap DoFs. We have 23 joints vs a 6-DoF object, which is worse than Fetch's 4-DoF action.
2. **Normalise rewards.**
   - Divide each intrinsic term by the running std of that term (or of its discounted return, as in RND). Clip to [−5, 5] or [0, 5].
   - Keep observation normalisation for predictors (RND, ensembles) frozen-ish: update the statistics slowly or after warm-up.
3. **Combine rewards with care.**
   - Mixtures help: RaIR needed disagreement; ToC needed forward dynamics; APS = APT + skill; MUSIC + DIAYN; CCGE = contact counts + reaching energy.
   - Use fixed weights after per-term normalisation, or separate critic heads per term (DUSDi value factorisation, RND's two value heads). Separate heads let you change weights later without retraining.
4. **Off-policy beats on-policy at our scale.**
   - PPO results (CCGE, SkiLD's selector) rely on thousands of parallel environments on a GPU.
   - On a 10-core laptop, use SAC/DDPG/TD3 with 4–8 environment workers.
   - Update-to-data (UTD) ratio: URLB uses 0.5 (DDPG); METRA/CSF use 1/16–1/4 (built for wall-clock); CSD manipulation uses 10 steps per 50-step episode = 0.2.
   - Recommendation: UTD ≈ 0.25–0.5, batch 256–1024.
5. **Non-stationary rewards in a replay buffer.**
   - Recompute intrinsic rewards at *sample time* (as URLB does) rather than storing them. This is critical for kNN, RND, counts and skill rewards.
6. **Ensembles.** K = 5 small MLPs (2×256) are cheap. Disagreement is robust to contact noise; prediction error is not.
7. **Skill methods need many steps.**
   - METRA/CSF/CSD use about 4M–64M steps. We should expect only the coarse first directions (e.g. "push block in x/y", "lift").
   - Use small skill dimensions (2–4) and object-centric φ inputs.
8. **Contact signals.**
   - Use binary or saturating transforms (e.g. tanh(F/F₀)), since raw forces are spiky (CCGE, Huang et al.).
   - Penalise very large forces or self-collision so the explorer does not learn to slam.
   - Contact-prioritised replay (Vulin) is a free improvement.
9. **Reset design matters as much as reward.**
   - Episode length 100–200 steps at a 20–50 Hz control rate.
   - Randomise the initial hand pose and block placement.
   - For the free block, reset when it falls off the platform.
   - Optional: Go-Explore-style resets to archived interesting states.
10. **MPS vs CPU.** For 256–1024-wide MLPs and batch ≤1024, CPU torch on Apple Silicon is often as fast as MPS once host-device transfer overhead is counted. Benchmark both. Running MuJoCo in subprocess workers (4–8) is the main source of throughput.

---

### 8. Recommendations (ranked) for our project

**Shared infrastructure for all methods**
- **Learner:** SAC (auto-α, target entropy −|A|/2 rather than −|A|, because 23-D actions make the default too exploratory), or DDPG with n-step 3 as in URLB.
- **Networks:** 2×256 MLPs (actor, twin critics), lr 3e-4, batch 512, γ = 0.98–0.99, τ = 0.005, buffer 1M, UTD ≈ 0.5, 2,000-step random warm-up.
- **Action space:** joint-position deltas clipped to a small range per step (smoother exploration than absolute targets).
- **Environments:** 6–8 parallel workers; episode length 150; control at about 25 Hz.
- **Intrinsic feature vector x (≈ 40–60 dims)**, used by every method below:
  - block position, orientation (6-D rotation or quaternion) and linear/angular velocity;
  - palm-to-block vector;
  - 5 fingertip-to-block-surface distances;
  - 5-or-more-dimensional binary finger-block contact vector (thumb, index, middle, ring, little, palm);
  - total normal force, squashed: tanh(F/F₀);
  - block height above platform.
  For the **fixed block**, pose and velocity are constant, so the contact/force/distance parts carry the signal.
- **Rewards:** recompute the intrinsic reward on sampling. Normalise each term by its running std and clip to [0, 5].
- **Budget:** about 40–50 min per method, roughly 300–600k env steps at 150–250 steps/s with 8 workers. Run a quick 5-min smoke test first.
- **Evaluation metrics, common to all methods:**
  - fraction of steps in contact (≥1 and ≥3 fingers);
  - grasp events: ≥2 opposing contacts and block lifted > 2 cm for the free block;
  - block displacement coverage: a 3-D occupancy grid of block position plus an orientation histogram;
  - contact-pattern coverage: number of distinct (finger × face) sets.

**1. Contact-coverage count bonus (CCGE-lite) + reaching energy.** Highest prior for grasping; cheapest.
- **Discretisation:** contact cell c = (object-state bin s, finger f, block face k).
  - Faces: 6, or 24 with quadrants.
  - Object-state bin: coarse (z-height in 3 bins × lifted/not × orientation bucketed into 6 "which face up" classes). For the fixed block, s is constant.
- **Rewards:**
  - r_contact = mean_{f in contact} 1/√(C[c]+1)
  - r_reach = mean_f Σ_k (1/√(C[s, f, k]+1)) · exp(−d(f, face k)/δ), with δ ≈ 2 cm.
  - r = α·r_contact + β·r_reach, starting with α = 1 and β ≈ 0.1. Use CCGE's "only increases over the episode max" trick for r_reach to avoid hovering.
- **Optional add-ons:** a bonus for multi-finger opposition (contacts on opposite faces), and contact-prioritised replay.
- **Pitfalls:**
  - Counts must be global (not per episode).
  - The reward is non-stationary, so recompute it from counts at sample time.
  - Without the object-state bin, the agent exhausts touching the resting block and stops; lifting creates new bins.
  - Penalise forces > F_max.

**2. RND (or ensemble Disagreement) on the object/interaction feature vector x (RND-Dist style).**
- **RND:**
  - target and predictor 3×256 MLPs, output dim 64;
  - x normalised by running mean/std;
  - r = ‖f(x') − g(x')‖², divided by its running std;
  - the predictor trains on 25% of each batch.
- **Disagreement variant:**
  - 5 MLPs (2×256) predict Δ(block pose, contact logits) from (x, proprio, a);
  - r = Σ_dim Var_k (CEE-US-lite, action-conditioned, so it also approximates empowerment);
  - this variant is the better choice when contacts are noisy.
- **Pitfalls:**
  - Full-proprioception input makes the agent flail; keep x object-centric.
  - The RND reward decays fast at small scale, so keep the predictor lr small (1e-4).

**3. MUSIC: MI between hand state and object state.**
- **Inputs:** Sᵃ = (palm position, fingertip positions); Sˢ = (block position and orientation) for the free block, or (contact vector, squashed force/torque on the block) for the fixed block.
- **Statistics network:** T(sᵃ, sˢ) MLP 2×256, trained with MINE; negatives come from time-shuffled sᵃ within the same trajectory.
- **Reward:** per-transition as in §4.13. Multiply by a scale so its std is ≈ 1 (the paper used α = 5000 with raw positions), then clip.
- **Why:** it directly rewards making the block move *with* the hand, i.e. grasp-and-carry; it learned pick-up on Fetch with no task reward.
- **Pitfall:** the reward is zero until the block moves at all. Mix it with #1 or #2 (e.g. 0.5 · r_MUSIC + 0.5 · r_contact) to bootstrap.

**4. APT/kNN state entropy over x (data-based baseline).**
- **Reward:** r = log(1 + mean of k = 12 neighbour distances) in a batch of 1024, computed over normalised x (or a FIFO memory of 10–50k recent x for global coverage).
- **Variant:** add CIC's representation (transition embedding contrasted with a 16–64-D skill) for the APS/CIC flavour.
- **Pitfall:** weight the feature groups (contacts vs block pose) so one group does not dominate the distances.

**5. CSF (or METRA) on object-centric φ: skills.**
- **Representation:** φ: x → R^d, with d = 3 for the free block and d = 2 for the fixed block (or d = 4); MLP 2×512.
- **Skills:** z ~ Unif(S^{d−1}).
- **Losses:** InfoNCE on (φ(x') − φ(x))ᵀz with in-batch negatives and ξ = 5.
- **Policy learning:** reward (φ(x') − φ(x))ᵀz. The policy is conditioned on z and resamples z each episode. SAC with z-conditioned critics is acceptable instead of vector successor features, but CSF's ablation favours the inner-product critic: implement ψ(s, a, z) ∈ R^d with Q = ψᵀz.
- **Alternatives:** METRA needs the dual λ (ε = 1e-3, λ init 30, lr 1e-4) [known]. CSD adds the controllability-aware metric: replace the "1" bound with the Mahalanobis distance from a Gaussian q(x'|x) fitted on current data. CSD is the version that made Fetch objects move.
- **Pitfalls:**
  - With fewer than 1M steps, expect only crude skills.
  - If φ sees proprioception, skills become arm poses, so keep φ on object/contact features.
  - Fixed block: φ over contact/force features yields "press/touch in different ways" skills.

**6. DIAYN-factored / DUSDi-lite baseline (classic competence-based).**
- **Skills:** discrete, 8–16, with the discriminator q(z | x_object) on object/contact features only.
- **Reward:** log q(z|x) − log p(z) + 0.1 · r_RND-x for exploration (the APS-style sum).
- **Value factorisation:** separate critic heads for the two terms.
- **Why include it:** a simple, well-understood baseline to compare against #5. URLB and CSD say it will underperform on its own.

**7. Stretch goal: CEE-US-lite model-based explorer.**
- **Components:** the ensemble from #2, used inside iCEM MPC with horizon 15, 64 samples, 3 iterations, colored noise β = 3, replanning every 2–4 steps, maximising summed predicted disagreement (+ r_contact).
- **Cost:** model rollouts are cheap on CPU for MLPs; no policy learning is needed.
- **Why:** it produced lifting and throwing from state in about 60k steps in CEE-US.
- **Risk:** the dynamics of 23-DoF contacts are hard to predict, and MPC at 128 × 15 × ensemble-5 per step limits speed to roughly 5–20 steps/s. Only worth it if 1–5 underwhelm.

#### Ideas specific to contact/tactile and object state

- **Contact-set novelty:** count distinct (finger-set, face-set) patterns (2⁶ × faces) with 1/√n bonuses. A *new combination* pays more than repeated single-finger touches. This is a SkiLD-like "interaction graph" with no learned causal model.
- **Opposition/grasp bonus inside novelty:** novelty computed only on states with ≥2 contacts on opposing faces, or a count bin "stable grasp" defined by contact persistence over ≥5 steps.
- **Cross-modal contact surprise (HaC on state):** a small classifier predicts the contact vector at t+1 from (proprio, x_t, a). The reward is the BCE error, or ensemble disagreement over the contact logits. Use *binary* contacts to avoid the non-smooth force problem.
- **Object-controllability (empowerment proxy):** r = ‖E[Δ block pose | a] − E[Δ block pose | a']‖ over sampled actions, or simply the action-conditioned disagreement on block motion. This is zero for unreachable or resting blocks and large when the hand holds the block.
- **Fixed-block variant:** the block cannot move, so replace "object state" with the net wrench applied to the block (force/torque sensor via `mj_contactForce` summed in the block frame, squashed). Novelty over the wrench direction (6-D, normalised) gives "push or pull in all directions" plus "squeeze" exploration.
- **Saturation and safety:** cap the force reward with tanh(F/F₀), with F₀ ≈ 0.5 × the block's weight-equivalent. Add a small penalty for joint-limit hitting and for very large contact forces, so curiosity does not become slamming (Huang et al.).
- **Replay:** contact-prioritised replay (Vulin): sample 50% of each batch from transitions with ≥1 contact once those exist.
- **Optional curriculum via resets:** archive states with new contact cells or block-lifted states, and restart 20–30% of episodes from them (Go-Explore). This multiplies the effective sample efficiency of all methods above.

\pagebreak

## Appendix B: Skeletal and musculoskeletal agents, DEP, goal babbling and developmental touch


**Project context:** a simulated monkey right arm skeleton in MuJoCo with 23 joint position servos:
- 7 arm joints: shoulder abduction, flexion and rotation; elbow flexion; radial pronation; wrist flexion and abduction.
- 16 finger joints.

The control step is dt = 0.04 s, so a 3 s trial is 75 steps. In the default `action_mode="delta"`, each action moves its servo target by up to ±2.5 rad/s · dt. Observations are low-dimensional state (see `ue/env.py`): proprioception, 18 touch patches, a contact map, a wrench, and object pose/velocity/size. There is one block per trial, either welded in space or resting free on a pedestal. Budget: one laptop with 10 CPU cores, about 5 h of training in total.

This file complements `notes_unsupervised_rl.md`, which covers the URLB-style families (ICM, RND, APT, DIAYN, METRA, CSD, CEE-US, ToC/HaC, CCGE and others). Here the focus is:
- musculoskeletal and skeletal control (Mathis lab, MyoSuite, DeepMind/Ölveczky virtual animals);
- embodied self-organisation (DEP);
- structured exploration noise and synergies;
- goal babbling and IMGEPs;
- developmental and tactile intrinsic motivation.

**Provenance tags:**
- **[read]:** checked against the arXiv HTML, abstract or full text during this review.
- **[read: abstract]:** only the abstract or a secondary summary was checked.
- **[known]:** well-established details I did not re-fetch. Verify exact numbers before relying on them.

---

### 0. TL;DR

1. **The Mathis-lab / MyoSuite recipe is not unsupervised.** It is PPO (often with an LSTM) plus a heavily shaped task reward, plus a **curriculum** (static-to-dynamic, or hard-negative mining), plus structured exploration (**Lattice**), plus sometimes **mixture-of-experts** or **synergy** priors. Most components transfer to reward-free learning:
   - the curriculum idea becomes curricula over goals or spawn difficulty;
   - Lattice / gSDE / pink noise become the action-noise model;
   - MoE / NPMP-style distillation becomes the way to consolidate explorers into one controller;
   - synergies extracted from exploration data become an action representation.
2. **DEP is the only method here that produces rich, coordinated, whole-body movement with zero learning budget.** It works within seconds of simulated time and needs no networks or reward. It is also the closest to "playful" self-organised behaviour. DEP-RL shows that mixing DEP into an off-policy learner (DEP takes over for H steps with a small probability, and DEP-only data pre-fills the replay buffer) helps both learning speed and state coverage. DEP is natively designed for servo or position-controlled robots (lpzrobots), so it fits our joint position servos very well. Its weakness: it only generates oscillatory, object-agnostic movement. It needs "guidance" or an RL layer to reach for and grasp the object.
3. **Goal babbling / IMGEPs with modular, object-centred goal spaces** are the developmental-robotics answer to "reaching, grasping and manipulating emerge without reward":
   - goal spaces: hand position → contact pattern → object pose;
   - module selection by learning progress;
   - in deep RL, goal-conditioned SAC with hindsight relabelling (HER, RIG, Skew-Fit).
   
   Flat goal spaces over the full state fail (Forestier et al.: about 10% vs 60% tool exploration). This is the strongest candidate for producing *usable, controllable* primitives in our budget.
4. **Touch helps intrinsic motivation focus on the object.** Count or novelty rewards over contact patterns (Baby Sophia, MIMo/BabyBench), contact-force rewards (Vulin), and tactile-weighted epistemic uncertainty (TacEx, CoRL 2026) all move exploration from free-space flailing to object interaction. TacEx explicitly reports grasping emerging from tactile curiosity without task reward.
5. **Synergies:** SAR extracts PCA → ICA synergies from an early "play" policy and lets a new policy act in synergy space blended with raw actions. This helped MyoHand manipulation enormously (>70% vs <20%). The Mathis lab cautions (Chiappa et al. 2024; KINESIS appendix) that low-variance components still carry task-relevant control. So always keep a residual pathway.

---

### 1. Musculoskeletal and skeletal motor control (Mathis lab, MyoSuite)

#### 1.1 KINESIS: Motion Imitation for Human Musculoskeletal Locomotion [read]
- **Citation:** Merkourios Simos, Alberto Silvio Chiappa, Alexander Mathis. 2025. arXiv:2503.14637. v1 was titled "Reinforcement learning-based motion imitation for physiologically plausible musculoskeletal motor control". Code: github.com/amathislab/Kinesis.
- **Model and data:**
  - MyoLeg models (MyoSuite) scaled to the mean SMPL body, in three variants: Legs (80 muscles), Legs+Abs (86) and Legs+Back (290).
  - Reference motion is KIT-Locomotion from AMASS: 1,054 clips (about 1.8 h; 946 train / 108 test), retargeted by IK and downsampled to 30 FPS.
- **Policy:**
  - model-free **PPO**;
  - Gaussian MLP with 6 hidden layers [2048, 1536, 1024, 1024, 512, 512] and SiLU; the critic has the same shape;
  - **Lattice** latent exploration (§1.3).
- **Observation:** pelvis height, tilt and velocity; joint positions, velocities and angles; foot contact forces; next reference pose (absolute and relative). The size is 309 to 453 dimensions.
- **Action:** direct muscle activation m = (a+1)/2. This beat a PD-on-muscle-length mapping.
- **Reward:**
  - r = w_pos·exp(−200 Σ‖p−p̂‖²) + w_vel·exp(−5 Σ‖v−v̂‖²) − w_e·(activation energy) [+ optional upright term exp(−3(θ_f²+θ_s²))];
  - weights: w_pos 0.6, w_vel 0.2, w_e 0.1 to 0.2;
  - early termination when any keypoint deviates more than 0.15 m;
  - episodes start at a random reference frame.
- **Curriculum and expert structure:**
  - **Curriculum by hard-negative mining (taken from PHC, Luo et al. 2023):** train on all clips until success plateaus. Collect the failed clips. Copy the weights and continue training on the failures only. Repeat until nothing fails. This yields a set of experts for increasingly hard subsets.
  - **Mixture of experts:** the experts are frozen. A gating MLP [1024, 512, 256] outputs a softmax over experts, and one expert is sampled per step. Three experts were enough.
- **Hyperparameters:**
  - Adam, lr 5e-5, γ 0.99, GAE λ 0.95, clip 0.2;
  - 128 parallel environments, 51,200 steps per rollout, 10 PPO epochs per update;
  - 150 Hz simulation, 30 Hz control;
  - compute: one A100 and 128 CPU threads (total wall-clock not reported).
- **Results:**
  - Test-set success is 91 to 97% and MPJPE about 43 mm.
  - Muscle activity correlates with human EMG significantly better than DEP-RL and DynSyn.
  - The policy transfers zero-shot to text-to-motion (MDM) outputs, can be fine-tuned for goal reaching (progress reward + success bonus), and solves football penalty kicks. It was the basis of the winning MyoChallenge 2025 entry.
  - Appendix: PCA on muscle activity shows a few components explain most of the variance, but task performance needs many components. "Task-relevant dimensionality is high."
- **Relevance to our project:** KINESIS is imitation, not unsupervised, but several of its components transfer directly:
  1. **Hard-negative mining → goal / spawn curriculum.** Track per-(shape, fixed/free, location) competence and re-train or over-sample failures.
  2. **MoE of frozen experts with a learned gate.** This is a cheap way to combine separately trained explorers (e.g., a reach expert and a grasp expert) into one controller.
  3. **Lattice** as the exploration noise model.
  4. **Their pretrained imitation policy acts as a motor prior for later goal tasks.** Without it, goal reaching learned an unnatural one-leg-dragging gait. The lesson: reward-free tasks learned from scratch produce odd gaits, and a movement prior (DEP data, synergies, or babbling-pretrained controllers) helps naturalness.
  5. **The caution about synergies:** low-variance dimensions matter.

#### 1.2 DEP-RL: Embodied Exploration for RL in Overactuated and Musculoskeletal Systems [read]
- **Citation:** Pierre Schumacher, Daniel Häufle, Dieter Büchler, Syn Schmitt, Georg Martius. ICLR 2023 (notable top 25%). arXiv:2206.00484. Code: github.com/martius-lab/depRL.
- **Method:** DEP (§2.1) is used as an *exploration policy* inside an off-policy learner (MPO; also HER-MPO).
  - **DEP controller:** a = tanh(κ·C̃·s + h). C is learned online with τĊ = f(ṡ_t) ṡ_{t−Δt}ᵀ − C, using finite-difference velocities and per-row normalisation C̃_ij = C_ij/(‖C_i‖+ε). The bias follows ḣ ∝ −a. For muscles, f(ṡ) = −ṡ, because muscles shorten when excited. For torque or position actuators, f(ṡ) = ṡ.
  - **Sensor input:** for muscles, one scalar per actuator: normalised muscle length + c·normalised force. Joint angles are used for the torque arm.
  - **Integration:**
    - (i) an unsupervised **DEP-only warm-up** pre-fills the replay buffer;
    - (ii) **intra-episode switching:** after each policy action, with probability p_switch, DEP takes control for H_DEP steps, then hands back.
    
    Summing DEP and policy actions caused chaotic behaviour. Switching worked better.
- **Hyperparameters:**
  - Arm-reaching: κ = 1000, τ = 80, buffer 600, Δt = 60, p_switch = 0.01, H_DEP = 20.
  - Human-run: κ ≈ 1900, τ = 26, Δt = 4, p_switch = 0.01, H_DEP = 10.
  - Ostrich: κ = 20, τ = 8, p_switch = 0.0004, H_DEP = 4.
  - Treat all of these as needing per-system tuning.
- **Results:**
  - arm26 success: 0.95 vs 0.62 for MPO.
  - humanreacher (7 DoF, 50 muscles), success with HER: 0.82 vs 0.65 for HER-MPO.
  - Human hopping: 472 vs 0 return.
  - Human running: 5/5 seeds found an alternating gait vs 1/5.
  - DEP alone covers the state space within seconds. Gaussian or OU noise performs badly once the action space is redundant ("virtual" actuators duplicated up to 600).
- **Relevance to our project:** high. Our arm is not muscle-overactuated, but the 16 finger joints form a highly coupled, redundant system. Independent per-joint noise produces jitter that averages out, and DEP produces coordinated whole-hand and arm motion.
  - With position servos, use f(ṡ) = ṡ. In absolute mode, the servo target is the action y and the measured joint angle is the sensor x, which is exactly the lpzrobots setting DEP was invented in.
  - The switching scheme (p_switch, H_DEP) is directly reusable with any SAC-based intrinsic learner.

#### 1.3 Lattice: Latent Exploration for Reinforcement Learning [read]
- **Citation:** Alberto Silvio Chiappa, Alessandro Marin Vargas, Ann Zixiang Huang, Alexander Mathis. NeurIPS 2023. arXiv:2305.20065. Code: github.com/amathislab/lattice.
- **Method:** it extends gSDE (Raffin et al. 2022, state-dependent exploration). Noise is injected into the policy's last latent layer x as well as into the action:
  - a = (W + P_a + α·W·P_x)·x, where P_x ~ N(0, S_x²) and P_a ~ N(0, S_a²);
  - so the action covariance is Σ = Diag(S_a² x²) + α² W Diag(S_x² x²) Wᵀ, a full covariance that follows the policy's own actuator correlations;
  - the noise matrices are resampled every T steps, giving temporal correlation;
  - α = 0 recovers gSDE.
  
  The log-likelihood under the full covariance is used in PPO or SAC.
- **Hyperparameters:**
  - α = 1;
  - resampling period T ∈ {1, 4, 8};
  - initial log-std 0 (or 1 for pose tasks);
  - std clipped to (1e-3, 10);
  - log S rescaled by −0.5·log N_x;
  - a small diagonal jitter is added for stability;
  - PPO uses an LSTM (MyoChallenge 2022 setup);
  - about 20 to 30% compute overhead.
- **Results:**
  - Humanoid (SAC): +18% reward vs unstructured noise. On low-DoF systems it is comparable to gSDE and pink noise.
  - MyoSuite reach and manipulation (Reorient, Pen, Baoding): higher reward and **20 to 60% less energy**. Pose tasks show no gain.
  - It was part of the winning MyoChallenge 2023 manipulation-track recipe (PPO + curriculum + reward shaping + Lattice; 34% success on relocating unseen objects with the full MyoArm).
- **Relevance to our project:** moderate. For 23 position-controlled joints, the gain over pink or gSDE noise may be small. Still, its "correlated noise follows the policy's learned coupling" is cheap to add to SAC (one extra matrix multiply). Recommended ablation order: **pink noise (zero cost) → gSDE → Lattice.**

#### 1.4 Acquiring musculoskeletal skills with curriculum-based RL (Baoding balls) [read]
- **Citation:** Alberto S. Chiappa, Pablo Tano, Nisheet Patel, Abigaïl Ingster, Alexandre Pouget, Alexander Mathis. *Neuron* 112(23):3969–3983, 2024. bioRxiv 10.1101/2024.01.24.577123. Code: github.com/amathislab/myochallenge.
- **Method:** PPO (SB3) with LSTM actor and critic controls MyoHand (39 muscles) to rotate two Baoding balls.
  - **Observation:** 86-D (joint angles, ball pose and velocity, target, previous muscle activations).
  - **Reward:** −distance + on-palm bonus + within-1.5 cm bonus.
  - **Curriculum:** **"static-to-dynamic stabilisation" (SDS)**, 32 stages. It first learns to *hold* the balls at poses along the trajectory, then to move between them at increasing speed, and only then adds domain randomisation.
- **Results:**
  - Won MyoChallenge 2022: Phase I 100%, Phase II 55%, with about 40% less energy than the runner-up.
  - Without the curriculum: 41 to 45% in Phase I and about 0% in Phase II.
  - Cost: about 300 M steps and about 400 h.
- **Representation findings:**
  - Muscle-space dimensionality was about 12 PCs for Baoding vs about 7 for pose.
  - Ablating the control subspace shows that about 12 PCs retain 50% of performance and about 25 are needed for saturation. Synergies transfer poorly across tasks.
  - LSTM states are less tangled than actions, as in motor cortex vs EMG (Russo et al.).
- **Relevance to our project:**
  - (a) A "**static before dynamic**" curriculum maps naturally onto our welded (fixed) vs free blocks. Learn contact and grasp on welded blocks first (no object dynamics), then free blocks on the pedestal.
  - (b) The budget lesson: dexterous object rotation is out of reach for 5 h. Target reach, touch, grasp, push and lift.
  - (c) Synergy caution again.

#### 1.5 SAR: Synergistic Action Representations [read]
- **Citation:** Cameron Berg, Vittorio Caggiano, Vikash Kumar. RSS 2023. arXiv:2307.03716.
- **Method:**
  1. Train a "play" policy on a simpler precursor task (e.g., MyoHand reorienting 8 objects for 1 M steps).
  2. Record its muscle activations and apply **PCA, then ICA** ("ICAPCA").
  3. Keep N = 20 synergies (more than 80% of the variance), normalised to [−1, 1].
  4. The new policy outputs N synergy coefficients plus O raw activations. The final action is a* = φ·SAR(a_syn) + (1−φ)·a_raw.
  
  Algorithm: SAC.
- **Results:**
  - MyoHand 100-object reorientation: more than 70% success vs less than 20% for end-to-end RL and curriculum baselines. Zero-shot generalisation 2 to 3× better.
  - MyoLegs walking: 11.7 m vs 0.8 m.
  - Also helps Shadow Hand (about 2×) and Humanoid-v2.
- **Relevance to our project:** high, as a *second stage*. SAR's key point is that synergies can come from an imperfect, early-stage play policy. For us, the data can come from DEP play or intrinsic-exploration buffers:
  - extract finger (16-D) synergies from contact-rich segments;
  - train downstream goal-conditioned policies in [synergy coefficients ⊕ arm joints ⊕ residual] space.
  
  With only 16 finger DoF, use about 4 to 6 synergies plus a residual (φ ≈ 0.7 to 0.8).

#### 1.6 DynSyn: Dynamical Synergistic Representation [read: abstract]
- **Citation:** Kaibo He, Chenhui Zuo, Chengtian Ma, Yanan Sui. ICML 2024. arXiv:2407.11472.
- **Method:** builds actuator groups ("synergies") from the system's *dynamical structure*, by clustering actuators that produce similar joint motions when excited in simulation. The policy then adapts the synergy weights per state and task.
- **Results:** state-of-the-art sample efficiency on musculoskeletal models up to 700 muscles. It is the only method that worked on FullBody-Gait. KINESIS reports its EMG correlation is worse than KINESIS.
- **Relevance to our project:** low to moderate. A model-based way to get synergies *without* any play data. You could cluster our 16 finger joints by their effect on fingertip positions. Hand-defining "finger flexion" groups (all phalanges of a finger flex together) is a cheap analogue.

#### 1.7 MyoDex: A Generalizable Prior for Dexterous Manipulation [read]
- **Citation:** Vittorio Caggiano, Sudeep Dasari, Vikash Kumar. ICML 2023. arXiv:2309.03130.
- **Method:**
  - multi-task PPO (SB3) on 14 of the 57 MyoDM tasks at once;
  - TCDM-style object-trajectory tracking reward (exponential position and orientation error, lift bonus, effort penalty);
  - GRAB pre-grasp postures and a planner for the free-space reach;
  - policy and value MLPs 256×128;
  - the prior is the checkpoint at 12.5 k iterations.
- **Results:**
  - Fine-tuning solves 37 of 43 unseen tasks, vs 22 for single-task experts and 13 for distillation, and is about 3× faster to reach 80% success.
  - Zero-shot success is at most 10%, but the hand grasps and lifts unseen objects.
  - The prior shares more NMF synergies across tasks (about 6 vs 2).
- **Relevance to our project:** a multi-task prior over *many objects* beats per-task experts. That supports training one goal-conditioned policy across all 32+ shapes rather than per-shape. The checkpoint trade-off (earlier is more general but slower to fine-tune) applies to choosing when to stop reward-free pretraining. Its pre-grasp pose initialisation is a "cheat" that we lack. Reach-to-pregrasp has to emerge from exploration (e.g., hand-position goals).

#### 1.8 MyoSuite and MyoChallenge (2022, 2023) [read: retrospect and abstract]
- **Citation:**
  - Vittorio Caggiano, Huawei Wang, Guillaume Durandau, Massimo Sartori, Vikash Kumar. "MyoSuite: A contact-rich simulation suite for musculoskeletal motor control." L4DC 2022. arXiv:2205.13600 [known].
  - Caggiano et al. "MyoChallenge 2022: Learning contact-rich manipulation using a musculoskeletal hand." PMLR 220 (NeurIPS 2022 competition track) [read: PDF listing].
  - MyoChallenge 2023 retrospect (MyoSuite blog).
- **Summary:**
  - The 2022 winners (Mathis lab) used curriculum + PPO-LSTM.
  - The 2023 manipulation-track winners ("Team Lattice", EPFL) relocated unknown objects with the full MyoArm (63 muscles + hand) and reached 34% success.
  - Across top entries, the common recipe was PPO with large batch, curricula, reward shaping, and structured exploration.
- **Relevance to our project:** the MyoChallenge 2023 "relocate" task is the closest benchmark to ours: arm + hand, one object, random shape and pose. Even with shaped rewards and very large compute, 34% was the top. Expect reward-free exploration in 5 h to yield touch, push and occasional grasp/lift, not reliable relocation.

#### 1.9 Mathis-lab representation work (proprioception / sensorimotor models) [read: abstract]
- **Citations:**
  - Kai J. Sandbrink, Pranav Mamidanna, Claudio Michaelis, Matthias Bethge, Mackenzie W. Mathis, Alexander Mathis. "Contrasting action and posture coding with hierarchical deep neural network models of proprioception." *eLife* 2023.
  - Alessandro Marin Vargas, Axel Bisi, Alberto S. Chiappa, Chris Versteeg, Lee E. Miller, Alexander Mathis. "Task-driven neural network models predict neural dynamics of proprioception." *Cell* 2024.
- **Summary:**
  - Networks are trained on proprioceptive (muscle spindle) inputs from a musculoskeletal arm with supervised objectives, e.g., predicting limb state or recognising handwritten characters from passive movement.
  - Units resemble S1 and cuneate-nucleus neurons.
  - Task-driven (especially state-estimation) objectives best predict neural dynamics.
  - Marin Vargas's thesis links this to curriculum + Lattice RL and imitation-learned grasping.
- **Relevance to our project:** indirect. It supports adding a self-supervised **state-estimation auxiliary objective** (predict palm/fingertip positions or object-relative pose from proprioception + touch) as the representation trunk for intrinsic-reward models. Such objectives produce the most "brain-like" proprioceptive features, and they are cheap.

---

### 2. Self-organised (playful) behaviour: DEP and "playful machines"

#### 2.1 DEP: A novel plasticity rule can explain the development of sensorimotor intelligence [read]
- **Citation:** Ralf Der, Georg Martius. *PNAS* 112(45):E6224–E6232, 2015. arXiv:1505.00835.
- **Method:**
  - **Controller:** a one-layer network y = tanh(C·x + h), mapping sensors x (joint angles) to motor commands y (servo targets).
  - **Inverse model:** maps sensor velocities back to motor velocities, y˙ ≈ M·x˙ (M = I for servos).
  - **"Differential extrinsic plasticity":** τĊ = M·x˙(t)·x˙(t−θ)ᵀ − C. It is a differential Hebbian rule, but the postsynaptic term is the *inverse-model-reconstructed* motor change rather than the actual motor change.
  - **Normalisation:** global or per-neuron, C ← κC/(‖C‖+ρ). κ sets the feedback gain, around 1 to 2.2, near the "edge of chaos".
  - **Optional threshold dynamics:** τ_h ḣ = −y (favours periodic motion).
- **Behaviour:** without any goal or reward, a humanoid develops crawling, a hexapod discovers several gaits, and a robot at a heavy wheel discovers rotating it ("affordance discovery"). Two robots coupled through a wheel synchronise.
  - **"Guided self-organisation":** hand-set entries in M (e.g., antiphase between legs), sensor delays, or external kinaesthetic hints (torque on the wheel) steer which attractor is found.
  - **Timescales:** τ about 0.7 s at 50 Hz controls how long the system stays in one attractor. Sensor delays of about 0.2 s are useful.
- **Limitations (stated):**
  - Behaviours are **all oscillatory**. There are no discrete point-to-point movements, because fixed points are destabilised.
  - It does not seek objects, so contact happens only if the object is in the attractor's path.
  - Learning M concurrently with C gives wrong results. Here M is simply I.
- **Relevance to our project:** high as a zero-training "motor babbling generator" that is far richer than random noise. It needs a mechanism (an RL wrapper, guidance, or goal babbling) to produce *directed* reaching and grasping.

#### 2.2 Playful Machines: Robotics Implementation of Self-Organised Behaviour [known]
- **Citation:** Ralf Der, Georg Martius. *The Playful Machine: Theoretical Foundation and Practical Realization of Self-Organizing Robots.* Springer (Cognitive Systems Monographs 15), 2012. Software: lpzrobots.
- **Summary:** homeokinesis (the predecessor of DEP). The controller adapts to maximise sensitivity (time-loop error minimisation) while keeping predictability. This yields playful, embodiment-specific behaviours (rolling, climbing, crawling) in many lpzrobots bodies. Martius, Der and Ay 2013 (PLoS ONE) derived a related rule from maximising predictive information (TiPI).
- **Relevance to our project:** conceptual background. The DEP rule is the simpler, more robust descendant. Use DEP rather than homeokinesis.

---

### 3. Motor priors from imitation and hierarchical control (DeepMind / Ölveczky lineage)

#### 3.1 Neural Probabilistic Motor Primitives (NPMP) [read: abstract; details known]
- **Citation:** Josh Merel, Leonard Hasenclever, Alexandre Galashov, Arun Ahuja, Vu Pham, Greg Wayne, Yee Whye Teh, Nicolas Heess. ICLR 2019. arXiv:1811.11711.
- **Method:**
  - Train thousands of single-clip tracking experts. Distil them *offline* into an encoder-decoder:
    - the encoder q(z_t | z_{t−1}, future reference frames) feeds an AR(1)-prior-regularised latent z (KL weight β);
    - the decoder π(a | s_proprio, z) is the reusable low-level controller.
  - "Linear-feedback policy cloning" (LFPC) augments behavioural cloning with Jacobian-based local feedback to make distillation data-efficient.
  - **Downstream:** a new high-level policy outputs z (latent about 60-D [known]).
- **Results:** one-shot imitation of unseen motions. The reusable decoder yields naturalistic movement in go-to-target and similar tasks.
- **Relevance to our project:** a template for consolidating many exploration behaviours (DEP episodes, goal-reaching trajectories) into one **latent-conditioned motor controller** that later goal-directed or intrinsic agents command. That is beyond a 5 h budget as a full pipeline. A small version (a VAE-bottleneck BC policy over goal-reaching trajectories) is feasible.

#### 3.2 Hierarchical motor control in mammals and machines [known]
- **Citation:** Josh Merel, Matthew Botvinick, Greg Wayne. *Nature Communications* 10:5489, 2019.
- **Summary:** a review arguing for:
  - low-level controllers that are reusable, partially autonomous and sensory-driven;
  - high-level controllers that issue abstract commands;
  - motor learning that should produce reusable "motor primitives" (spinal / brainstem-like).
  
  It draws on NPMP, the virtual rodent, and neuroscience of basal ganglia and motor cortex.
- **Relevance to our project:** it frames our goal. "Reasonable primitives" should be a goal- or latent-conditioned low-level controller (reach-to-point, close-hand-on-contact) that a higher level can sequence.

#### 3.3 Deep neuroethology of a virtual rodent [read: abstract]
- **Citation:** Josh Merel, Diego Aldarondo, Jesse Marshall, Yuval Tassa, Greg Wayne, Bence Ölveczky. ICLR 2020. arXiv:1911.09451.
- **Summary:** a MuJoCo rat with vision + proprioception, trained multi-task by distributed off-policy RL on four tasks (gaps, maze forage, escape bowl, two-tap timing).
  - Network analysis finds task-specific "strategy" representations and task-invariant kinematic representations.
  - Neural sequences and population dynamics resemble rodent motor circuits.
- **Relevance to our project:** shows that *analysing* emergent representations (e.g., which units encode contact or grasp phase) is a scientific outcome in its own right. Plan to log hidden activations of our explorers.

#### 3.4 A virtual rodent predicts the structure of neural activity across behaviours (MIMIC) [read: summaries and MIMIC-MJX]
- **Citations:**
  - Diego Aldarondo, Josh Merel, Jesse D. Marshall, Leonard Hasenclever, Ugne Klibaite, Amanda Gellis, Yuval Tassa, Greg Wayne, Matthew Botvinick, Bence P. Ölveczky. *Nature* 632:594–602, 2024. doi:10.1038/s41586-024-07633-4.
  - Follow-up: C. Y. Zhang et al. (with T. Pereira). "MIMIC-MJX: Neuromechanical emulation of animal behavior." arXiv:2511.20532, 2025.
- **Method:** **MIMIC** (Motor IMItation and Control).
  - 3D pose of freely moving rats (DANNCE) is registered to a MuJoCo skeletal rat (STAC).
  - Deep RL trains an **inverse-dynamics** model: an encoder of future reference frames feeds a stochastic latent "motor intention" with a KL bottleneck, and a decoder conditioned on proprioception outputs the action.
  - MIMIC-MJX reimplements it in PPO/Brax on GPU, with about 842 rat clips and about 0.65 B steps to 95% of peak.
  - The frozen decoder transfers to new tasks (escape bowl solved only by transfer).
- **Results:**
  - Network activity predicts DLS and MC neural activity better than kinematic features. This is consistent with an inverse-dynamics representation.
  - Latent variability is structured like the minimal-intervention principle of optimal feedback control.
- **Relevance to our project:** (i) The stochastic latent bottleneck (KL to N(0, I)) both regularises and yields biologically relevant variability. If we build a goal- or latent-conditioned controller, include it. (ii) For neuroscience comparison to monkey data, an inverse-dynamics-style controller (goal = desired next hand/object state) is the architecture that best matched brain data.

#### 3.5 MoCapAct [read: abstract; details known]
- **Citation:** Nolan Wagener, Andrey Kolobov, Felipe Vieira Frujeri, Ricky Loynd, Ching-An Cheng, Matthew Hausknecht. NeurIPS 2022 Datasets & Benchmarks. arXiv:2208.07363.
- **Summary:**
  - Releases clip-tracking experts (about 2,500 clip snippets, more than 3 h of CMU MoCap) for the dm_control CMU humanoid, plus rollouts.
  - Distils them into an NPMP-style multi-clip policy whose low level is reused for downstream tasks, and a GPT for motion completion.
  - Training the experts reportedly took very large CPU budgets [known].
- **Relevance to our project:** low directly (no monkey hand MoCap). It shows the "experts → distilled latent controller" pattern is reproducible with open code, if we later want to distil our explorers.

#### 3.6 PHC: Perpetual Humanoid Control (reference of KINESIS) [known]
- **Citation:** Zhengyi Luo, Jinkun Cao, Alexander Winkler, Kris Kitani, Weipeng Xu. ICCV 2023. arXiv:2305.06456.
- **Summary:** a progressive multiplicative control policy (PMCP). New network "primitives" are allocated for hard sequences mined from failures, then combined with a composer. It also learns fail-state recovery.
- **Relevance to our project:** the source of the hard-negative mining + MoE scheme used by KINESIS (see §1.1).

---

### 4. Goal babbling and intrinsically motivated goal exploration

#### 4.1 Goal babbling permits direct learning of inverse kinematics [read: abstract]
- **Citation:** Matthias Rolf, Jochen J. Steil, Michael Gienger. *IEEE Trans. Autonomous Mental Development* 2(3):216–229, 2010.
- **Method:**
  1. Instead of random motor babbling, the learner picks *goals in task space* (hand position).
  2. It tries to reach them with its current inverse model plus exploratory noise, along paths toward the goal.
  3. It trains the inverse model on what it actually achieved (an early form of hindsight relabelling), weighting samples by efficiency and directness.
- **Results:** it learns inverse kinematics for redundant arms (up to 50 DoF planar arms, the Honda humanoid) where motor babbling fails because of non-convex redundancy. It is orders of magnitude more sample-efficient.
- **Relevance to our project:** the canonical argument for exploring in **hand-space goals** rather than joint space for our redundant 7-DoF arm. Reaching emerges quickly this way.

#### 4.2 SAGG-RIAC: Active learning of inverse models with intrinsically motivated goal exploration in robots [known]
- **Citation:** Adrien Baranes, Pierre-Yves Oudeyer. *Robotics and Autonomous Systems* 61(1):49–73, 2013.
- **Method:** the goal space is recursively split into regions. Each region's interest is its **competence progress**, the absolute derivative of goal-reaching competence over a sliding window. Goals are sampled preferentially from high-progress regions, plus a random fraction. Low-level reaching uses a local inverse model with micro-exploration.
- **Results:** learning inverse models for high-DoF arms, a quadruped and a fishing-rod task is much faster than with random goal or motor babbling. The learner avoids unreachable and too-easy regions automatically.
- **Relevance to our project:** **learning-progress goal sampling** is the cheapest automatic curriculum. It handles the many unreachable goals (behind the arm, inside the pedestal) without hand engineering.

#### 4.3 IMGEP: Intrinsically Motivated Goal Exploration Processes with Automatic Curriculum Learning [read]
- **Citation:** Sébastien Forestier, Rémy Portelas, Yoan Mollard, Pierre-Yves Oudeyer. *JMLR* 23, 2022. arXiv:1708.02190. (First version 2017; companion paper: Péré et al., "Unsupervised learning of goal spaces for IMGEPs", ICLR 2018.)
- **Method:**
  - Goals are parameterised fitness functions over outcome spaces.
  - **Modular, object-centred goal spaces:** one per object (hand, tool, toy, etc.).
  - **Learning progress (LP)** per module: r_i = f_g(τ) − f_g(τ′), where τ′ is the nearest previous attempt. A bandit chooses the module: 20% random, 80% ∝ LP.
  - **Meta-policy:** a kd-tree nearest neighbour over (context, outcome) pairs, followed by Gaussian mutation (σ 0.05 to 0.3) of the retrieved policy parameters. "SSPMutation" only mutates the parameters *after* the object started moving.
  - Policies are small: RBF motor primitives (5 bases per joint) or a small NN.
- **Results:**
  - Median exploration, 2D arm-tool-toy: AMB 61% vs 36% for random module selection, 11% for a flat goal space, 0% for random motor babbling.
  - A real Poppy Torso discovers how to use joysticks to move a toy, light and sound.
  - The modular agent ≈ a hand-designed curriculum.
- **Relevance to our project:** **directly applicable and very cheap.** It is population-based with no deep RL, and it runs at simulator speed on 10 cores.
  - A parameterised open-loop trajectory over 75 steps can be 23 joints × 5 RBF bases = 115 parameters.
  - Goal modules: palm position, fingertip/contact pattern, object position, object orientation.
  - It produces a *dataset* of diverse reach, touch and push outcomes, which can then be distilled into a closed-loop goal-conditioned policy (as in the follow-up GEP-PG, Colas et al. ICML 2018).
  - Pitfall: open-loop policies cannot adapt to the random object pose. Use the trial context (object pose, size, fixed flag) as the nearest-neighbour context, as in AMB.

#### 4.4 HER: Hindsight Experience Replay [known]
- **Citation:** Marcin Andrychowicz, Filip Wolski, Alex Ray, Jonas Schneider, Rachel Fong, Peter Welinder, Bob McGrew, Josh Tobin, Pieter Abbeel, Wojciech Zaremba. NeurIPS 2017. arXiv:1707.01495.
- **Method:** relabel transitions with goals achieved later in the same episode ("future" strategy, k = 4 relabels per transition). Use a sparse reward −1[‖φ(s′) − g‖ > ε]. Off-policy DDPG.
- **Results:** solves Fetch push, slide and pick-and-place from sparse rewards. Pick-and-place needs some demonstrations or "goals in the air" tricks.
- **Relevance to our project:** the core machinery for deep goal babbling. With φ(s) as palm position, object position or contact pattern, every trial is useful training data, even if the object never moves.

#### 4.5 RIG: Visual RL with Imagined Goals [known]
- **Citation:** Ashvin Nair, Vitchyr Pong, Murtaza Dalal, Shikhar Bahl, Steven Lin, Sergey Levine. NeurIPS 2018. arXiv:1807.04742.
- **Method:** a VAE on observations. Goals are sampled from the VAE prior. Reward = −latent distance. HER-style relabelling with sampled latent goals.
- **Relevance to our project:** with state input we can skip the VAE. The "**sample goals from a generative model of achieved states**" idea remains: sample object or hand goals from a density model fit to the replay buffer.

#### 4.6 Skew-Fit: State-Covering Self-Supervised RL [read: abstract; details known]
- **Citation:** Vitchyr H. Pong, Murtaza Dalal, Steven Lin, Ashvin Nair, Shikhar Bahl, Sergey Levine. ICML 2020. arXiv:1903.03698.
- **Method:** maximise I(S; G) = H(G) − H(G|S). Goals are sampled from the replay buffer with importance weights ∝ p_θ(s)^α, where α ∈ [−1, 0) (e.g., −1), so that rare states are up-weighted. The density model p_θ is refit iteratively. The goal distribution provably converges to uniform over reachable states.
- **Results:** state-of-the-art visual goal-reaching. A real robot learned door opening from pixels without reward.
- **Relevance to our project:** **the goal sampler we should use for object goals.** Up-weight rare object displacements and rotations so the agent keeps trying to move the object rather than only reaching. Cheap with state: a kNN or KDE density on object features.

#### 4.7 DISCERN: Unsupervised Control Through Non-Parametric Discriminative Rewards [read: abstract; details known]
- **Citation:** David Warde-Farley, Tom Van de Wiele, Tejas Kulkarni, Catalin Ionescu, Steven Hansen, Volodymyr Mnih. ICLR 2019. arXiv:1811.11359.
- **Method:** jointly learns a goal-conditioned policy and an embedding. The reward is the softmax probability that the final state matches the goal among K decoys, using cosine similarity of embeddings. This rewards the *controllable* aspects of the goal. Goals come from a diversity-maintaining buffer.
- **Relevance to our project:** useful if hand-picked goal features are too restrictive. A learned controllable-similarity reward over the interaction features (touch + object) is an alternative to hand-set ε thresholds. It is more complex. Try only after HER on hand-crafted goals works.

#### 4.8 Learning Latent Plans from Play [known]
- **Citation:** Corey Lynch, Mohi Khansari, Ted Xiao, Vikash Kumar, Jonathan Tompson, Sergey Levine, Pierre Sermanet. CoRL 2019. arXiv:1903.01973.
- **Method:** human teleoperated "play" data (no tasks). Goal-conditioned BC with hindsight relabelling of windows (Play-GCBC). Play-LMP adds a CVAE plan latent.
- **Results:** one policy performs 18 tasks at about 85% success. Play data is cheap and covers the state space better than task demos.
- **Relevance to our project:** **once exploration produces play-like data, offline goal-conditioned BC with hindsight windows** (1 to 3 s) is a robust, cheap way to turn it into reusable primitives. No RL is needed in the extraction stage, and it is much cheaper than online RL.

#### 4.9 Go-Explore (First return, then explore) [known]
- **Citation:** Adrien Ecoffet, Joost Huizinga, Joel Lehman, Kenneth O. Stanley, Jeff Clune. *Nature* 590:580–586, 2021. arXiv:2004.12919.
- **Method:** keep an archive of discretised "cells". Select promising cells (count-based), *return* to them (by simulator restore or a goal-conditioned policy), then explore from there. A policy-based variant handles stochastic settings.
- **Results:** Montezuma's Revenge and Pitfall records. A sparse-reward robot pick-and-place solved with a cell representation over the object and gripper.
- **Relevance to our project:** MuJoCo state restore makes the "return" step free. Store the simulator states at which contact or grasp first occurred, and start exploration episodes from them. This side-steps the hardest exploration step (first contact) at almost no cost. Cells: discretised (palm–object distance, number of touching patches, object height). Pitfall: policies trained from restored states may not learn to *reach* those states. Mix normal resets with restores (e.g., 50/50).

#### 4.10 Self-supervised exploration via disagreement [known]
- **Citation:** Deepak Pathak, Dhiraj Gandhi, Abhinav Gupta. ICML 2019. arXiv:1906.04161.
- **Method:** an ensemble of forward models (5) in feature space. Intrinsic reward = the variance of their predictions. It is robust to aleatoric noise. Because the reward is differentiable, the policy can be optimised through it directly.
- **Results:** a real Sawyer learns to push and interact with objects without reward.
- **Relevance to our project:** see `notes_unsupervised_rl.md` §2.3. For us, predict only the object and touch next state (TacEx-style channel weighting, §5.5), not joints.

#### 4.11 CEE-US (structured-world-model curiosity) [read]
- **Citation:** Cansu Sancaktar, Sebastian Blaes, Georg Martius. NeurIPS 2022. arXiv:2206.11403.
- **Method:** an ensemble of 5 GNN world models (object nodes, agent as global context). Reward = trace of the ensemble prediction covariance. Planning uses iCEM MPC (horizon 30, 128 samples, 10 elites, 3 iterations). State-only input. Downstream tasks are solved zero-shot by planning.
- **Results:** zero-shot stacking, pick-and-place and throwing in Fetch Construction. Policy-based Disagreement, RND and ICM baselines mostly stay in free space.
- **Cost:** about 72 h on one GPU.
- **Relevance to our project:** shows structured, object-centred uncertainty is what produces object interaction. MPC planning is too slow for our budget (sibling notes §2.5), so use the same reward with an amortised SAC policy.

---

### 5. Developmental models with touch and tactile intrinsic motivation

#### 5.1 MIMo: A Multi-Modal Infant Model [read]
- **Citation:** Dominik Mattern, Pierre Schumacher, Francisco M. López, Marcel C. Raabe, Markus R. Ernst, Arthur Aubret, Jochen Triesch. IEEE TCDS (2024). arXiv:2312.04318. Code: github.com/trieschlab/MIMo.
- **Model:** an 18-month-old infant in MuJoCo with five-fingered hands (or mittens), binocular vision, vestibular sense, proprioception, and **full-body virtual skin**.
  - Touch measures normal + friction force, with sensor density set by human two-point discrimination. Forces are spread over nearby sensors.
  - Two actuation models: a spring-damper motor, or an opposing-muscle pair per joint with co-contraction.
- **RL demos:** PPO and SAC (SB3 defaults) on reaching, standing, self-body touch and catching. PPO is more reliable on the harder tasks.
- **Relevance to our project:** the reference open-source embodiment for developmental touch studies. Its touch representation (per-body-part force aggregation) parallels our 18 touch patches. The co-authorship (Schumacher of DEP-RL) means DEP has been tried with similar bodies.

#### 5.2 Baby Sophia: self-exploration through self-touch and hand regard (BabyBench) [read]
- **Citation:** Stelios Zarifis, Ioannis Chalkiadakis, et al. arXiv:2511.09727, 2025. BabyBench @ ICDL 2025, built on MIMo.
- **Method:** PPO with **intrinsic rewards only**.
  - **Touch representation:** 17,175 tactile sensors are pooled into 68 body-part groups (mean absolute activation).
  - **Self-touch reward:**
    - novelty: larger for first-ever and first-in-episode touches of each body part, with an "anti-boredom" decay that turns repeated touching into a penalty;
    - a bonus for a hand's first contact with a part;
    - diversity milestones (5, 10, …, 30 parts);
    - a left/right balance bonus.
  - Training: 8 M steps in a two-stage curriculum: default pose for 4 M steps, then randomised joint angles.
  - **Hand regard:** a 10 k-step random motor babbling phase finds "hand" blobs by correlation with proprioception (ρ > 0.6). Rewards: hands visible plus moderate optical flow.
- **Results:** self-touch covers 32 of 34 parts. The curriculum beat fixed-only (26) and random-only (24). Hand regard collapsed to one hand (local optimum).
- **Relevance to our project:** a working template for **count-based novelty over discrete contact patterns**: which of our 18 patches touch the object, and which pairs. Episodic and global counters, plus a habituation penalty. Pitfalls:
  - heavy hand-tuning (milestones, jackpots);
  - fixation on one solution (the one-hand collapse);
  - our analogue: always touching with the same finger.

#### 5.3 Learning and acting in peripersonal space: moving, reaching and grasping [read: abstract]
- **Citation:** Jonathan Juett, Benjamin Kuipers. *Frontiers in Neurorobotics* 13:4, 2019. arXiv:1809.10788.
- **Method:** a developmental robot (Baxter) learns reach and grasp from unguided exploration. A "peripersonal space graph" is built from random arm configurations. Intrinsic motivation: when an *unusual* event happens (an object moves or is bumped), the agent tries to make it happen reliably, turning surprising outcomes into actions. A simulated palmar grasp reflex (break-beam sensor) is used.
- **Results:** reaching about 97% success, grasping about 50%.
- **Relevance to our project:** two cheap tricks.
  1. **"Reflexes" as priors:** a palmar grasp reflex that closes the fingers when palm touch exceeds a threshold mimics the infant grasp reflex. It makes grasp emerge from reaching plus touch.
  2. **Event-triggered goal adoption:** when the object moves unexpectedly, add that outcome as a goal (HER/IMGEP does this naturally).

#### 5.4 Tactile intrinsic motivation and touch-based curiosity [read: abstract; details in sibling notes §5]
- **Citations:**
  - Nikola Vulin, Sammy Christen, Stefan Stevšić, Otmar Hilliges. "Improved learning of robot manipulation tasks via tactile intrinsic motivation." *IEEE RA-L* 6(2), 2021. arXiv:2102.11051.
  - Sai Rajeswar, Cyril Ibrahim, Nitin Surya, Florian Golemo, David Vazquez, Aaron Courville, Pedro O. Pinheiro. "Touch-based / Haptics-based curiosity for sparse-reward tasks." CoRL 2021. arXiv:2104.00442.
- **Method:**
  - Vulin: intrinsic reward = the sum of contact forces between the robot's force sensors and the object, plus **contact-prioritised experience replay**.
  - ToC/HaC: reward = the error of predicting touch from vision/proprioception (cross-modal surprise). A task-free exploration phase is followed by relabelling with task rewards.
- **Results:** faster learning of push, pick-and-place and door opening than ICM and RND.
- **Relevance to our project:** a contact-force reward is the simplest dense "go touch the object" signal. Clip and log-scale it (our touch is already log(1 + F/0.05 N)). Exclude touch_platform. Contact-prioritised replay is a one-line change to a SAC buffer.

#### 5.5 TacEx: Tactile Curiosity Drives Robot Interaction [read]
- **Citation:** Klemens Iten, Alexander Proshkin, Bhavya Sukhija, Stelian Coros, Andreas Krause, Pieter Abbeel, Carmelo Sferrazza. CoRL 2026. arXiv:2609.40134.
- **Method:** MaxInfoRL (MaxInfoSAC). The policy maximises Q plus a weighted sum of per-modality epistemic uncertainties from an ensemble dynamics model (disagreement), with separate weights ω for vision, touch and latent. Setting ω_vision = ω_latent = 0 gives purely tactile curiosity.
  - Touch: a 32×64×3 force map on a simulated Panda gripper.
  - Hyperparameters: batch 256, γ 0.999, **UTD 20**, actor/critic MLPs 128×3, dynamics lr 1e-3, warm-up 500 steps.
- **Results:**
  - Tactile-weighted disagreement drives the robot to grasp and manipulate objects without task reward.
  - Vision-only disagreement rarely engages the object.
  - Tactile-only exploration focuses on one object, while adding vision spreads interest over about 2 objects.
  - The collected data trains offline pick-and-place, and TacEx also improves VLA post-training.
- **Relevance to our project:** **the most directly relevant 2026 result.** Uncertainty restricted to the tactile and object channels yields grasping without reward. Our version: an ensemble of 5 small MLPs predicting Δ(touch, contact_map, obj_pos, obj_rot) from (proprio, touch, object, action). Reward = the ensemble variance on those channels only. MaxInfoSAC auto-tunes the intrinsic weight like SAC's entropy temperature.

#### 5.6 Learning tactile skills through curious exploration [read: abstract]
- **Citation:** Leo Pape, Calogero M. Oddo, Marco Controzzi, Christian Cipriani, Alexander Förster, Maria C. Carrozza, Jürgen Schmidhuber. *Frontiers in Neurorobotics* 6:6, 2012.
- **Summary:** a biomimetic robot finger with a tactile sensor. Curiosity is defined as learning progress of predicting tactile input (Schmidhuber-style). It autonomously develops a small set of distinct motor skills that produce different tactile inputs (e.g., tapping vs sliding).
- **Relevance to our project:** an early proof that tactile prediction-progress yields discrete "touch primitives". It supports using **learning progress** rather than raw error as the touch curiosity signal, which also resists contact-chatter noise.

---

### 6. Other exploration-noise references (brief, [known])

- **gSDE:** Antonin Raffin, Jens Kober, Freek Stulp. "Smooth exploration for robotic RL." CoRL 2021 (PMLR 2022). arXiv:2005.05719.
  - State-dependent noise ε = θ_ε·x with θ_ε resampled every n steps. It gives smooth, safe exploration and is available in SB3.
  - Lattice's base.
- **Pink noise:** Onno Eberhard, Jakob Hollenstein, Cristina Pinneri, Georg Martius. "Pink noise is all you need: colored noise exploration in deep RL." ICLR 2023.
  - Replaces white Gaussian action noise in SAC/TD3/MPO with 1/f (β = 1) temporally correlated noise.
  - It is the best default across many DMC tasks. There is a `pink-noise-rl` package, and it costs nothing.
- **Colored-noise iCEM:** Pinneri et al. CoRL 2020 (used in CEE-US).

---

### 7. Recommendations for our setting

These are ordered by expected value per CPU-hour. Our sim: 75-step trials at 25 Hz control, delta position servos, 10 cores. All learners should log the env's `info` metrics (touching, n_touch, grasp, lifted, obj_disp, obj_rot_angle, palm_obj_dist) per trial for comparison. They should also save their replay buffers for offline relabelling (ExORL-style evaluation; see sibling notes).

#### 7.1 DEP as a zero-cost explorer and data generator (≈ 10 to 20 min, including tuning)

**Implementation (numpy, per env):**
- **Sensors and actions:**
  - x = qpos scaled to [−1, 1] (23-D; arm and fingers can be split into two DEP blocks or one).
  - Use `action_mode="absolute"`, y = servo target in [−1, 1]. This is the original lpzrobots servo setting, with inverse model M = I and f(ẋ) = +ẋ.
  - In delta mode, the action is a target *velocity*. Then use y as the delta command and x˙ as the sensor, or simply use absolute mode for DEP episodes.
- **Update rule:** C ← C + (dt_eff/τ)·(ẋ_t ẋ_{t−θ}ᵀ − C), with ẋ_t = x_t − x_{t−1}.
  - τ ≈ 10 to 40 steps (0.4 to 1.6 s).
  - Delay θ ≈ 1 to 3 steps.
  - Per-row normalisation C̃ = κ·C/(‖C_i‖ + 1e-12).
  - y = tanh(C̃·x + h), with ḣ = −ε_h·y (ε_h ≈ 0.001 to 0.01).
- **κ:** sweep in [0.5, 5] for position servos. DEP-RL's κ ≈ 1000 applies to its muscle-length input scaling, not to ours.
- **Carry state across trials:** C must persist across trials. Our 3 s trials are short relative to τ, so don't reset C at trial start. Optionally keep a per-env C and add tiny noise if C → 0.
- **Separate blocks:** consider separate DEP blocks for the arm (7) and fingers (16), each with its own κ. Fingers have small ranges and joint limits saturate tanh quickly.

**Use:**
- (a) **Baseline explorer:** report touch/grasp/displacement rates vs uniform random and pink noise.
- (b) **Replay pre-fill:** DEP-only data for the first ~50 k steps of any SAC learner.
- (c) **DEP-RL switching:** with p_switch ≈ 0.01 to 0.03 per step, DEP takes over for H_DEP ≈ 10 to 25 steps (0.4 to 1 s). It runs inside SAC-based intrinsic learners (§7.3, §7.4).
- (d) **SAR data source** (§7.5).

**Pitfalls:**
- DEP is object-agnostic and oscillatory. Expect bumping and pushing of nearby blocks, rarely grasps.
- It may sit in limit cycles against joint limits. Lower κ or add the bias term.
- Never sum DEP and policy actions. Switch between them (DEP-RL found summing chaotic).
- **Guidance options:**
  - *Kinaesthetic / attractor:* add a small term pulling the palm toward the object via a fixed Jacobian-transpose IK step in the arm block.
  - *Reflex:* use a "grasp reflex" (§7.6) during DEP episodes so contacts become grasps.

#### 7.2 Modular goal babbling with hindsight (IMGEP + HER + Skew-Fit). The main candidate for usable primitives (≈ 1.5 to 2 h)

**Learner:** goal-conditioned SAC (or DDPG with n-step 3 as in URLB):
- MLP 256–256 (×2 critics);
- UTD 1 to 4;
- 10 vectorised envs;
- HER "future" relabelling with k = 4, plus "episode-final" goals.

**Goal modules** (each g lives in a fixed feature subspace φ_k(s)):

| Module | Goal | Success threshold |
|---|---|---|
| M1 reach | palm position in shoulder frame (3-D); also object-relative palm position (obj_rel) | ε = 2 to 3 cm |
| M2 hand shape / contact | target contact pattern on the object (binary over 18 patches, or the 5 fingertip + palm subset) | Hamming distance ≤ 1, or require "≥ k patches including thumb" |
| M3 object position | free blocks only: Δobj_pos from spawn | ε = 2 cm; module disabled when `fixed=1` |
| M4 object orientation | 6-D rotation representation | angle ε = 15° |

**Module selection by learning progress (SAGG-RIAC / AMB):**
- LP_k = |competence over the recent 200 attempts − competence over the previous 200|.
- Choose a module with probability 0.2 uniform, else ∝ LP_k.
- Early on, M1 dominates. M2 then becomes learnable (reach and then contact). M3 and M4 follow once contacts happen. This is the automatic reach → grasp → manipulate curriculum.

**Goal sampling within a module:**
- Skew-Fit: draw candidates from achieved φ_k values in the replay buffer, weighted ∝ density^α with α = −1 (kNN density, k = 10, on ≤ 20 k recent points).
- Mix 20% "stretch" goals (achieved + small Gaussian perturbation) and, for M3, goals near the spawn (small displacement) to keep the problem solvable.

**Reward:** sparse −1[not achieved] works with HER. Add a small dense shaping only within the module's own feature space (e.g., −‖Δ‖), never across modules.

**Inputs:** obs ⊕ one-hot(module) ⊕ g (zero-pad other modules' goal slots).

**Pitfalls:**
- Unreachable object goals for welded blocks (handled by the fixed flag).
- HER on object goals: when the object never moves, every relabelled object goal is "stay still", and the agent learns not to touch.
  - Up-weight relabelled goals with nonzero displacement (Skew-Fit does this).
  - Optionally use Go-Explore restores from contact states (§4.9).
- The contact-pattern goal is discrete and noisy. Use a minimum-force threshold (GRASP_FORCE 0.05 N) and require the condition to hold over the last 3 steps.
- **Trial length:** at 75 steps, set γ ≈ 0.95 to 0.97 so the horizon matches.

**Evaluation (reward-free → primitives):** for held-out shapes, test success rates of
- "reach palm to point",
- "make contact pattern X on object",
- "move object by Δ".

These are the primitives.

#### 7.3 Touch- and object-focused curiosity (≈ 1 h), as a competitor or a bonus for §7.2

Options (pick 1 or 2):
- **(a) TacEx-style disagreement** (recommended):
  - ensemble of 5 MLPs (2×256) predicting Δ[touch(18), contact_map(36), obj_pos(3), obj_rot(6), obj_vel(6)] from [proprio ⊕ touch ⊕ object ⊕ action];
  - reward r_int = mean over output dims of the ensemble variance, normalised by a running std;
  - weight the tactile channels more (ω_touch ≥ ω_obj); exclude proprioceptive targets;
  - SAC with r = r_int (reward-free), optionally added to §7.2's goal reward with weight 0.1 to 0.5.
- **(b) Contact-pattern count novelty (Baby Sophia):**
  - hash the binary 18-patch contact vector (plus coarse object-height and displacement bins);
  - reward 1/√N_global(c) + an episodic first-visit bonus;
  - habituation: a negative reward once N_episode(c) exceeds about 20 steps.
  - Very cheap.
- **(c) Vulin force reward:**
  - r = Σ log(1 + F_patch/0.05) over hand patches on the object (excluding platform and forearm), clipped;
  - plus contact-prioritised replay (sample contact transitions with probability 0.5).

**Pitfalls:**
- Contact chatter is aleatoric noise. Prediction *error* curiosity (ICM) will chase it. Ensemble *variance* or learning progress is safer.
- Force rewards encourage pressing hard against welded blocks. Cap them and add a small action-rate penalty.
- Exclude touch_platform and arm-segment patches from rewards, so the agent doesn't learn to lie on the pedestal.

#### 7.4 Exploration noise: pink → gSDE → Lattice (ablation, ≈ 30 min)

- Replace SAC's white Gaussian sampling noise with **pink noise** (β = 1), resampled per trial of 75 steps. This is a free win for smooth arm motion.
- If time allows, implement **Lattice** in the SAC actor:
  - noise matrices on the last hidden layer (256-D);
  - α = 1, resample period T = 4 to 8 steps;
  - initial log-std 0;
  - log-likelihood with the full covariance (23×23 Cholesky per batch is cheap).
  - Expect the main benefit on the 16 coupled finger joints and in lower "energy" (action magnitude and jerk).

#### 7.5 Synergies from exploration data (SAR), minutes of compute

**Extraction:**
1. Take finger servo-target trajectories (16-D) from contact-rich segments of DEP, curiosity or goal-babbling data (n_touch ≥ 2).
2. Standardise, apply PCA, keep 4 to 6 components (check ≥ 80% variance), then apply FastICA.
3. Map the components back to joint space and normalise to [−1, 1].

**Use in a downstream goal-conditioned policy:**
- action = [arm 7 ⊕ synergy coefficients N ⊕ finger residual 16];
- finger command = φ·W_syn·a_syn + (1−φ)·a_res, with φ ≈ 0.75.

Compare with and without synergies on M2 and M3 success (grasp and move).

**Pitfalls:**
- Synergies from flailing data encode flailing. Filter to contact segments.
- KINESIS and Chiappa (2024) show that low-variance dimensions matter. Keep the residual.
- **Hand-designed alternative (DynSyn-like):** "per-finger flexion" groups (MCP + PIP + DIP together) plus thumb opposition. This gives 6 to 7 coefficients and needs no data.

#### 7.6 Developmental priors and curricula (cheap multipliers for every method)

- **Grasp reflex** (Juett & Kuipers; the infant palmar reflex). Use it for the first part of training, then anneal it.
  - When palm or ≥ 2 finger patches touch the object, add a closing bias to finger targets for 0.3 s.
  - Implement it as an env wrapper whose effect fades (p_reflex from 1 to 0 over training), so the policy internalises it.
- **Static → dynamic curriculum** (Chiappa 2024 SDS):
  - start with p_fixed ≈ 0.8 (welded blocks: no object dynamics; contact and grasp patterns are learnable);
  - anneal to 0.5, then lower;
  - spawn blocks closer to the palm's rest workspace early (narrow the WORKSPACE box), then widen.
- **Hard-negative mining over shapes** (KINESIS / PHC):
  - keep per-shape competence for M2 and M3;
  - over-sample the worst 25% of shapes;
  - in the MoE variant, fine-tune a copy of the policy on failure shapes and add a gate later.
  - Only worth it if a policy already succeeds on most shapes.
- **Go-Explore restores:** keep an archive of sim states with first contact or grasp. Start 20 to 50% of trials from them (perturbed). This greatly increases the amount of in-hand data per CPU-hour.

#### 7.7 Consolidation into primitives (if time remains, ≈ 30 min)

- **Offline goal-conditioned BC from play** (Lynch et al.) on the union of all explorers' buffers:
  - hindsight windows of 10 to 40 steps;
  - goal = φ(s_{t+H}) in each module space;
  - optionally a KL-regularised latent bottleneck (NPMP/MIMIC-style, β ≈ 1e-3 to 1e-2).
  
  This yields a single reusable controller and lets us compare which explorer produced the most useful data (ExORL-style).
- Optionally use a **MoE gate** (KINESIS) over frozen module-specialist policies (reach, grasp, push), trained with the same goal rewards.

#### 7.8 Suggested 5 h allocation

| Block | Time |
|---|---|
| DEP tuning and data | 0.25 h |
| Pink-noise SAC + contact-novelty reward (§7.3b), reward-free | 0.75 h |
| TacEx-style tactile/object disagreement SAC with DEP switching (§7.3a + §7.1c) | 1.25 h |
| Modular goal babbling with HER + LP + Skew-Fit (§7.2), with grasp reflex and static→dynamic curriculum | 2 h |
| SAR synergy extraction + short downstream comparison, or offline GCBC consolidation | 0.5 h |
| Evaluation on held-out shapes | 0.25 h |

#### 7.9 General pitfalls collected from the papers

1. **Full-state novelty or curiosity mostly explores the arm, not the object** (CEE-US, TacEx, and sibling notes). Always restrict intrinsic objectives to touch and object channels, or to goal modules.
2. **Flat goal spaces fail. Modular, object-centred goal spaces with learning progress work** (Forestier et al.).
3. **Summing DEP and policy actions is chaotic. Alternate instead** (DEP-RL).
4. **Without a curriculum, dexterous tasks give about 0% on the hard phase** (Chiappa 2024). Static before dynamic.
5. **Synergies are lossy.** Keep residual control (Chiappa 2024; KINESIS appendix; SAR blending).
6. **Reward-free from-scratch policies move unnaturally** (KINESIS goal reaching without a prior). Add action-rate/energy penalties and smooth (pink or Lattice) noise. Lattice also reduces energy by 20 to 60%.
7. **Hand-tuned bonus schemes** (Baby Sophia) **fixate on one solution.** Use habituation and learning progress, not fixed jackpots.