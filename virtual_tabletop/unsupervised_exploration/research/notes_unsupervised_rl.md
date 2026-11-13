# Unsupervised / reward-free exploration and skill discovery: literature notes

Project context: a simulated monkey right arm in MuJoCo (7 arm joints + 16 finger joints, joint-position control, contacts) learning from low-dimensional state to reach, grab and manipulate one block per trial (the block is either fixed in space or resting free on a platform). Compute: one Apple Silicon laptop (10 CPU cores, torch MPS), about 5 h of total training across all methods.

Provenance:
- **[read]**: details I checked against the arXiv abstract or HTML (method and/or hyperparameter appendix) during this review.
- **[known]**: well-established details I did not re-fetch in this review. Equations and the main hyperparameters are standard. Double-check exact numbers against the code before relying on them.

---

## 0. TL;DR

1. **Generic novelty over the full state mostly produces arm/finger flailing.** URLB's data-based methods (APT, ProtoRL) and RND do best on DMC locomotion. In manipulation, however, every paper that looked (CEE-US, CSD, MUSIC, CCGE, SkiLD) found that state novelty, curiosity and skill methods over the whole state ignore the object, because moving the robot is easier than moving the object.
2. **What made objects move without task reward:**
   - (a) restricting the intrinsic objective to **object or hand-object state** (MUSIC, CSD, RND-Dist, DUSDi/SkiLD factorisation);
   - (b) **controllability-aware or "hard-transition" weighting** (CSD, SUSD);
   - (c) **contact-based rewards** (Vulin; ToC/HaC; CCGE contact coverage);
   - (d) **structured, object-factored world-model uncertainty** (CEE-US, RaIR).
3. **Practical recipe for our budget:** an off-policy actor-critic (SAC or DDPG with n-step returns) at a moderate update-to-data ratio, running-std reward normalisation, and an intrinsic reward computed on a *small hand-crafted feature vector*: object pose/velocity, fingertip-to-object distances, a per-finger contact vector, and contact forces. Use skill methods (METRA/CSF/CSD) on that same object-centric feature space.

---

## 1. Benchmarks and surveys

### 1.1 URLB: Unsupervised Reinforcement Learning Benchmark [read]
- **Citation:** Laskin, Yarats, Liu, Lee, Zhan, Lu, Cang, Pinto, Abbeel. NeurIPS 2021 Datasets & Benchmarks. arXiv:2110.15191.
- **Method/setup:** Two phases: reward-free pretraining (up to 2M frames), then fine-tuning on 12 DMC tasks (Walker, Quadruped, Jaco reach) for 100k frames. It groups methods into three families:
  - knowledge-based (ICM, Disagreement, RND), with reward = prediction error or variance;
  - data-based (APT, ProtoRL), with reward = kNN particle entropy;
  - competence-based (SMM, DIAYN, APS), with reward = log q(z|s).
- **Hyperparameters:** DDPG base agent, hidden 1024, lr 1e-4, batch 1024, n-step 3, update every 2 env steps, gamma 0.99, buffer 1e6. Skill dimension for DIAYN is 16 (one-hot); APS uses a 10-d successor-feature space [known].
- **Results:** From states, the data-based methods (APT, ProtoRL) and RND are best. Competence-based methods lag behind, and none solve the benchmark. Longer pretraining does not reliably help: performance was flat or worse in 9 of 18 settings. The Jaco reach (manipulation) domain is the hardest for every method, and competence-based methods do especially poorly there.
- **Relevance to our project:** This is the reference implementation for 8 algorithms (rll-research/url_benchmark) and gives sensible DDPG defaults. It shows that skill methods are weak on arm reaching unless the skill space is restricted.

### 1.2 ExORL: Don't Change the Algorithm, Change the Data [known]
- **Citation:** Yarats, Brandfonbrener, Liu, Laskin, Abbeel, Lazaric, Pinto. 2022. arXiv:2201.13425.
- **Method:** Collects reward-free datasets with the URLB explorers, relabels them with task rewards, and runs offline RL (TD3, CQL).
- **Results:** Data from RND, APT and ProtoRL lets plain TD3 solve many tasks. The *diversity of exploratory data* matters more than the offline algorithm.
- **Relevance to our project:** We can evaluate each explorer by the coverage of its replay buffer (object displacement, contact diversity, grasp events) and by offline relabelling. This is cheaper than fine-tuning every method.

---

## 2. Knowledge-based (prediction-error / uncertainty) methods

### 2.1 ICM: Curiosity-driven Exploration by Self-supervised Prediction [known]
- **Citation:** Pathak, Agrawal, Efros, Darrell. ICML 2017. arXiv:1705.05363.
- **Method:** An inverse model trains features φ; a forward model predicts φ(s') from (φ(s), a).
- **Reward:** r = η/2 · ‖f(φ(s), a) − φ(s')‖².
- **Hyperparameters:** A3C; β = 0.2 weights the forward loss against the inverse loss.
- **Results:** Exploration in VizDoom and Mario.
- **Relevance to our project:** Low. With state input and contact-induced stochasticity, the forward error is dominated by aleatoric noise from contact chatter (the "noisy-TV" problem). The baseline is only useful if the forward error is restricted to object-state prediction.

### 2.2 RND: Exploration by Random Network Distillation [known]
- **Citation:** Burda, Edwards, Storkey, Klimov. ICLR 2019. arXiv:1810.12894.
- **Method:** A predictor f_θ regresses a fixed random network g on next states.
- **Reward:** r = ‖f_θ(s') − g(s')‖².
- **Key tricks:** Normalise observations with running mean/std and clip to [-5, 5]. Divide the intrinsic reward by a running std of intrinsic returns. Use separate value heads for intrinsic and extrinsic rewards, a non-episodic intrinsic return, and γ_int = 0.99. Train the predictor on a fraction of each batch (25%) to slow down novelty decay.
- **Results:** First to reach above-human scores on Montezuma's Revenge.
- **Relevance to our project:** Very cheap (two small MLPs). It is the strongest simple knowledge-based method in URLB from states. Apply it to *object and hand-object features* rather than the full proprioceptive state (see RND-Dist, §6.4).

### 2.3 Disagreement: Self-Supervised Exploration via Disagreement [known]
- **Citation:** Pathak, Gandhi, Gupta. ICML 2019. arXiv:1906.04161.
- **Method:** An ensemble of K forward models.
- **Reward:** the variance across members, r = Var_k[f_k(s, a)] (summed over dimensions). The reward is differentiable with respect to the action, which enables "differentiable exploration".
- **Results:** Robust to stochasticity (aleatoric noise averages out); a real Sawyer arm learned to push objects.
- **Hyperparameters:** K = 5 (URLB uses an ensemble of 5 MLPs, hidden 1024).
- **Relevance to our project:** A good fit, because MuJoCo contacts are noisy in practice and disagreement suppresses that noise. Predict only object state deltas plus the contact vector.

### 2.4 Plan2Explore: Planning to Explore via Self-Supervised World Models [known]
- **Citation:** Sekar, Rybkin, Daniilidis, Abbeel, Hafner, Pathak. ICML 2020. arXiv:2005.05960.
- **Method:** A Dreamer (RSSM) world model plus an ensemble of one-step latent predictors. The disagreement variance is the reward for an imagination-trained exploration policy.
- **Results:** Zero-shot/few-shot adaptation on DMC from pixels.
- **Relevance to our project:** Shows that optimising the *expected future* novelty inside a model is more sample-efficient than reacting to past novelty. Training a full Dreamer is too expensive for us. A cheap analogue is a one-step ensemble used for reward in an off-policy learner, or MPC (see CEE-US).

### 2.5 CEE-US: Curious Exploration via Structured World Models Yields Zero-Shot Object Manipulation [read]
- **Citation:** Sancaktar, Blaes, Martius. NeurIPS 2022. arXiv:2206.11403.
- **World model:** An ensemble (M = 5) of message-passing GNNs, one node per object plus a global agent node, fully connected, one message-passing step. It predicts state deltas.
- **Reward:** r = tr(Cov_m[f_m(s, a)]), the ensemble disagreement.
- **Planner:** iCEM MPC, 128 samples, horizon 30, 10 elites, 3 iterations, colored-noise exponent β = 3.5, replanning every step.
- **Hyperparameters:** MLP blocks 2×128 (Construction); 2,000 transitions per iteration; 300 iterations (about 72 h on one RTX 3060, though lifting appeared after about 30 iterations ≈ 60k steps).
- **Results:** In the Fetch-style Construction environment with 4 blocks, free play yields lifting, flipping and throwing. Zero-shot downstream success with the same MPC: stacking 2 blocks 0.91 (MLP model 0); pick & place with 4 objects 0.97. URLB baselines (Disagreement, RND, ICM with DDPG) "mostly stay in free space or at walls". Offline RL on CEE-US data beats MLP-model data.
- **Relevance to our project:** The strongest evidence that *object-factored epistemic uncertainty* yields grasping and lifting from state with no reward. The full MPC loop is slow (72 h on a GPU). For us, the takeaway is to use ensemble disagreement over *object-centric* predictions (a factored or small model) as the reward for a model-free learner. A short-horizon iCEM explorer is an optional stretch goal.

### 2.6 RaIR: Regularity as Intrinsic Reward for Free Play [read]
- **Citation:** Sancaktar, Piater, Martius. NeurIPS 2023. arXiv:2312.01473.
- **Method:** A relational descriptor ϕ(s_i, s_j) is computed for every object pair (e.g. absolute relative x-y position, discretised to bins of size b). The reward is the negative Shannon entropy of the histogram of descriptors: r = Σ_x p(x) log p(x). It rewards symmetric or regular configurations.
- **Combined reward:** r = r_RaIR + λ · r_Dis, with λ = 0.1 (Construction) and λ = 0.02 (Quadruped). Planned with iCEM on CEE-US GNN ensembles.
- **Results:** Zero-shot tower building improves (Singletower-3: 0.75 vs 0.40 for CEE-US). Pure RaIR (λ = 0) causes too few interactions, so the disagreement term is necessary.
- **Relevance to our project:** Low. We have a single object, so pairwise regularity among objects is degenerate. The idea of "regularity" could be applied to hand-object relations (e.g. symmetric finger placement on block faces), but that is speculative. Main lesson: structure-seeking rewards need an exploration term alongside them.

---

## 3. Data-based (state-entropy maximisation) methods

### 3.1 APT: Behavior From the Void: Unsupervised Active Pre-Training [known]
- **Citation:** Liu, Abbeel. NeurIPS 2021. arXiv:2103.04551.
- **Method:** A particle-based (kNN) estimate of state entropy computed within a minibatch.
- **Reward:** r(s) = log(1 + (1/k) Σ_{j∈kNN(s)} ‖h(s) − h(s_j)‖), with k = 12 and averaging over the neighbours. h is a contrastively learned embedding (from pixels); for states, URLB uses the raw state or a learned forward/inverse embedding.
- **Learner:** DDPG/DrQ.
- **Results:** Strongest URLB family overall.
- **Relevance to our project:** Very simple and needs no extra network if computed on a hand-designed feature vector. Caveat: entropy over all 23 joint angles rewards flailing. Use object pose plus fingertip-object relative vectors.

### 3.2 ProtoRL: Reinforcement Learning with Prototypical Representations [known]
- **Citation:** Yarats, Fergus, Lazaric, Pinto. ICML 2021. arXiv:2102.11271.
- **Method:** SwAV-style prototypes give a representation. The kNN entropy reward is computed against a queue of past embeddings (a coverage estimate across time rather than within a batch).
- **Hyperparameters:** 512 prototypes, k = 3, queue 2048.
- **Relevance to our project:** Low as a separate method. The useful idea is to compute kNN against a *memory of past states*, not just the current batch, which estimates coverage across the whole run.

### 3.3 SMM: Efficient Exploration via State Marginal Matching [known]
- **Citation:** Lee, Eysenbach, Parisotto, Xing, Levine, Salakhutdinov. 2019. arXiv:1906.05274.
- **Method:** Matches the state marginal to a target p*(s) (uniform by default).
- **Reward:** r = log p*(s) − log q(s) (a VAE density), plus a skill-mixture term (log d(z|s) − log p(z)).
- **Results:** Fetch block-pushing, where the target places mass on object positions.
- **Relevance to our project:** The idea of a *target distribution over object positions* (e.g. uniform over the platform, or lifted heights) is an easy way to inject a prior.

### 3.4 Particle-based entropy estimators in general [known]
- **Kozachenko-Leonenko estimator:** Ĥ ∝ Σ_i log ‖s_i − s_i^{kNN}‖ + const.
- **APT's change:** log(1 + mean distance to the k neighbours), with k = 10–12, for stability.
- **Practical pitfalls:** The reward scale is set by the feature scale, so normalise features per dimension (running std). Batch-level kNN shrinks as the policy concentrates, so use a large batch (≥1024) or a FIFO memory of about 10k states. Reward normalisation is needed when mixing with other terms.

---

## 4. Competence-based (mutual-information / skill-discovery) methods

### 4.1 DIAYN: Diversity Is All You Need [known]
- **Citation:** Eysenbach, Gupta, Ibarz, Levine. ICLR 2019. arXiv:1802.06070.
- **Objective:** maximise I(S; Z) + H(A|S, Z) with a discrete skill z (e.g. 50 skills).
- **Reward:** r = log q_φ(z|s) − log p(z).
- **Learner:** SAC, α = 0.1, hidden 300×2.
- **Weakness:** Converges to static, easily discriminable poses with poor coverage. Under the I(S; Z) objective, moving the arm to distinct places is enough, so the object is ignored (shown by CSD, MUSIC and DUSDi).
- **Relevance to our project:** Use it as a baseline, and only with the discriminator input restricted to object or hand-object features.

### 4.2 DADS: Dynamics-Aware Unsupervised Discovery of Skills [known]
- **Citation:** Sharma, Gu, Levine, Kumar, Hausman. ICLR 2020. arXiv:1907.01657.
- **Objective:** maximise I(S'; Z | S) using a skill-dynamics model q(s'|s, z).
- **Reward:** r = log q(s'|s, z) − log (1/L) Σ_i q(s'|s, z_i), with L = 500 prior samples.
- **Results:** Predictable skills usable for model-based (MPC) composition.
- **Learner:** On-policy-ish SAC.
- **Relevance to our project:** Medium-low. It is expensive and URLB/METRA show weak coverage. Predictable skills are attractive for later MPC, though.

### 4.3 APS: Active Pretraining with Successor Features [known]
- **Citation:** Liu, Abbeel. ICML 2021. arXiv:2108.13956.
- **Reward:** r = r_APT(s) + log q(s|z), where the second term is a successor-feature-style term φ(s)ᵀz with z on the unit sphere.
- **Results:** Combines entropy and skills; good in URLB on Walker, weaker on Jaco.
- **Relevance to our project:** Shows that "exploration + skill" sums help. It is an easy add-on to APT.

### 4.4 CIC: Contrastive Intrinsic Control [known]
- **Citation:** Laskin, Liu, Peng, Yarats, Rajeswaran, Abbeel. NeurIPS 2022. arXiv:2202.00161.
- **Representation:** A contrastive (CPC/NCE) loss between transition embeddings g(s, s') and skill embeddings, with temperature 0.5.
- **Reward:** kNN particle entropy over the *transition embedding* (APT-style, k = 12). The discriminator is used for representation only, not as reward.
- **Hyperparameters:** Skill dim 64 (continuous uniform [0, 1]^64), hidden 1024, DDPG as in URLB.
- **Results:** State of the art on URLB states at publication (+79% interquartile mean over the next best), including the best result on Jaco.
- **Relevance to our project:** A strong, simple, well-documented method that drops into a URLB-style codebase. Feed the embedding networks object-centric features.

### 4.5 LSD: Lipschitz-constrained Unsupervised Skill Discovery [known]
- **Citation:** Park, Choi, Kim, Lee, Kim. ICLR 2022. arXiv:2202.00914.
- **Reward:** r = (φ(s') − φ(s))ᵀz, with φ spectrally normalised to be 1-Lipschitz in *Euclidean state distance*. This forces large state changes.
- **Results:** Larger coverage than DIAYN/DADS on Ant and Humanoid.
- **Relevance to our project:** Superseded by CSD and METRA. The Euclidean metric means "big arm motions".

### 4.6 CSD: Controllability-Aware Unsupervised Skill Discovery [read]
- **Citation:** Park, Lee, Lee, Abbeel. ICML 2023. arXiv:2302.05103.
- **Distance:** d_CSD(s, s') = (s' − μ_θ(s))ᵀ Σ_θ(s)⁻¹ (s' − μ_θ(s)). It is proportional to −log q_θ(s'|s), where q_θ is a Gaussian density model fitted to current skill transitions, so rare (hard-to-achieve) transitions count as "far".
- **Constraint:** ‖φ(s) − φ(s')‖ ≤ d_CSD(s, s'), enforced by dual gradient descent with λ (init 3000, slack ε = 1e-6).
- **Reward:** r = (φ(s') − φ(s))ᵀz, optimised with SAC.
- **Hyperparameters (manipulation):** 2-hidden-layer 256 MLPs, lr 1e-3, batch 256, γ = 0.98, 10 gradient steps per episode, skill dim 2–3 (continuous) or 16 discrete. Fetch runs use 40k epochs × 2 episodes × 50 steps ≈ 4M steps.
- **Results:** In FetchPush, Slide and PickAndPlace, CSD is the *only* method among CSD, LSD, DIAYN and DADS that learns to move the object without supervision; the other three mostly move the arm. In Kitchen, CSD learns about 10/13 tasks vs LSD 2, DIAYN 4, DADS 0. Disagreement gives more chaotic behaviour and lower coverage.
- **Relevance to our project:** Highly relevant: it directly addresses "arm motion is easy, object motion is hard". Note that Fetch had object state *in the observation*, and that took about 4M steps; we must cut that heavily.

### 4.7 METRA: Scalable Unsupervised RL with Metric-Aware Abstraction [read]
- **Citation:** Park, Rybkin, Levine. ICLR 2024. arXiv:2310.08887.
- **Objective:** maximise E[Σ_t (φ(s_{t+1}) − φ(s_t))ᵀz] subject to ‖φ(s) − φ(s')‖₂ ≤ 1 for adjacent states. This is equivalent to bounding latent distances by temporal distance.
- **Dual updates:** φ maximises (φ(s') − φ(s))ᵀz + λ·min(ε, 1 − ‖φ(s) − φ(s')‖²); λ minimises λ·min(ε, 1 − ‖·‖²).
- **Reward:** r = (φ(s') − φ(s))ᵀz.
- **Skill prior:** continuous isotropic Gaussian, or zero-centred one-hot.
- **Hyperparameters:** SAC; skill dim 2 (Ant, Humanoid), 4 (Quadruped), 16 discrete (Cheetah), 24 discrete (Kitchen); update-to-data ratio 1/4 to 1/16; [known] 1024-unit 2-layer MLPs, lr 1e-4, batch 256, 8 parallel workers, 8 trajectories per epoch.
- **Results:** Best state coverage in most domains. It is the first method to get locomotion from pixels on Quadruped and Humanoid. Zero-shot goal reaching beats LEXA, and hierarchical control is strong.
- **Limitations stated by the authors:** Symmetric metric; linear latent behaviour; low sample efficiency (designed for wall-clock).
- **Relevance to our project:** The temporal-distance metric means the latent space covers the directions that take longest to traverse. For a free block those directions are *object displacement*, which is good. However, it needs millions of steps. Use a small skill dim (2–4) and object-centric inputs to φ.

### 4.8 CSF: Contrastive Successor Features ("Can a MISL Fly?") [read]
- **Citation:** Zheng, Tuyls, Peng, Eysenbach. 2024/2025 (arXiv v2 March 2025; ICLR 2025 [known]). arXiv:2412.08021.
- **Representation loss:** InfoNCE with an inner-product critic, (φ(s') − φ(s))ᵀz, against in-batch negatives z' (256). The negative log-sum-exp term is scaled by ξ = 5.
- **Reward:** r = (φ(s') − φ(s))ᵀz, with z ~ Unif(S^{d−1}).
- **Policy learning:** A vector-valued successor-feature critic ψ(s, a, z) ∈ R^d is trained with TD toward φ(s') − φ(s) + γψ̄(s', a', z). The policy maximises ψᵀz. There is no dual variable or Lagrangian.
- **Ablations:** The inner-product critic is crucial; MLP or kernel critics fail. The information-bottleneck reward (dropping the negative term) is crucial.
- **Hyperparameters:** lr 1e-4, hidden 1024 (one hidden layer), batch 256, EMA τ = 5e-3, SAC with automatic entropy tuning, 8 workers × 8 trajectories per round, 50–200 gradient steps per round, horizon 200 (50 for Kitchen). Skill dim: Ant 2, Cheetah 2, Quadruped 4, Humanoid 8, Kitchen 4, Robobin 9.
- **Results:** Matches METRA on coverage, zero-shot goal reaching and hierarchical control, with fewer moving parts and more stability.
- **Follow-up:** Reizinger et al. 2025 (arXiv:2507.14748) prove CSF recovers ground-truth state features up to a linear map.
- **Relevance to our project:** The simplest modern distance-style skill method. It is the preferred skill-discovery implementation for us (no dual λ to tune).

### 4.9 DUSDi: Disentangled Unsupervised Skill Discovery for Efficient Hierarchical RL [read]
- **Citation:** Hu, Wang, Stone. NeurIPS 2024. arXiv:2410.11251.
- **Setup:** A factored state S = (S¹, …, S^N); the skill z = (z¹, …, z^N) has one k-way discrete component per factor.
- **Objective:** Σ_i [I(S^i; z^i) − λ I(S^{−i}; z^i)], with λ = 0.1, using DIAYN-style discriminators per factor.
- **Value factorisation:** One Q-head per reward term (summed for the policy), which reduces variance.
- **Learner:** SAC; a downstream PPO high-level policy picks skills every 50 steps.
- **Results:** DCI disentanglement 0.86 vs 0.016 (DIAYN). Beats DIAYN, CIC, CSD and METRA on multi-factor downstream tasks.
- **Relevance to our project:** Medium. We have natural factors: arm/wrist pose, hand aperture/finger config, object position, object orientation. A DUSDi-style factorised discriminator with an *object factor* forces some skill components to control the object. The value-factorisation trick (separate critics per reward term) is broadly useful when summing intrinsic rewards.

### 4.10 SkiLD: Unsupervised Skill Discovery Guided by Factor Interactions [read]
- **Citation:** Wang, Hu, Chuck, Chen, Martín-Martín, Zhang, Niekum, Stone. NeurIPS 2024 [known] (arXiv Oct 2024). arXiv:2410.18416.
- **Skill:** z = (target local-dependency graph g, diversity index b). Dependencies are detected with pointwise conditional MI from a learned attention dynamics model.
- **Reward:** r = 1[induced graph = g] · (1 + λ log q(b|s, g)), with λ = 0.5 and hindsight relabelling. The graph-selection policy is rewarded with 1/√count(g).
- **Learners:** TD3/Rainbow skill policy; PPO graph selector.
- **Results:** The only method that achieves complex multi-object interactions (soak rag, cut peach) in Mini-behavior and iGibson. DIAYN and CSD mostly manipulate single objects.
- **Relevance to our project:** The concept translates directly: *reward inducing specific interactions* (e.g. "finger i in contact with the object"; "object velocity depends on hand"). Our contact vector gives these dependency graphs for free, with no learned causal model. A cheap version is "contact-pattern skills": z = target set of fingers in contact.

### 4.11 SUSD: Structured Unsupervised Skill Discovery through State Factorization [read: abstract/summaries]
- **Citation:** Hosseini, Soleymani Baghshah. ICLR 2026. arXiv:2602.01619.
- **Method:** A distance-maximising (METRA/CSD-style) skill objective per state factor. The factor-wise rewards are weighted by curiosity, w_i ∝ −log q(s'_i | s), from a density model, so under-explored or hard-to-control factors get more weight.
- **Results:** Beats LSD, CSD, METRA, DIAYN and DUSDi on factorised environments (Multi-Particle, Kitchen).
- **Relevance to our project:** Supports the recipe "per-factor METRA/CSF reward + upweight the object factor by its novelty".

### 4.12 Choreographer: Learning and Adapting Skills in Imagination [read: abstract]
- **Citation:** Mazzaglia, Verbelen, Dhoedt, Lacoste, Rajeswar. ICLR 2023. arXiv:2211.13350.
- **Method:** A DreamerV2-style world model. Exploration is decoupled from skill learning: a VQ-VAE codebook over latent states defines skills (reach codebook codes), learned in imagination. A meta-controller evaluates skills in imagination for adaptation.
- **Results:** Best URLB results from states and pixels at the time; good sparse-goal finding in Meta-World.
- **Relevance to our project:** The conceptual takeaway is "explore first (data-based), then fit skills as goal-reaching to clustered states". The world-model training is too costly for a 5-hour budget.

### 4.13 MUSIC: Mutual Information State Intrinsic Control [read]
- **Citation:** Zhao, Gao, Abbeel, Tresp, Xu. ICLR 2021 (spotlight). arXiv:2103.08107.
- **Method:** Split the state into *agent state* Sᵃ (gripper position) and *surrounding state* Sˢ (object position), and maximise I(Sᵃ; Sˢ). The MI is estimated with MINE: a statistics network T_φ is trained with E_joint[T] − log E_marg[e^T], where marginal samples come from shuffling agent states in time within a trajectory.
- **Per-transition reward:** r = 0.5 Σ_{i=t,t+1} T(sˢ_i, sᵃ_i) − log(0.5 Σ_i e^{T(sˢ_i, s̄ᵃ_i)}), scaled by α (5000) and clipped.
- **Hyperparameters:** DDPG/SAC, 3×256 MLP, lr 1e-3, batch 256, 16 workers.
- **Results:** With *no task reward*, the agent learns reaching, pushing and **picking up** the block in Fetch Pick&Place (first time reported). As an add-on it boosts DISCERN success on Push from 21% to 95% and on Pick&Place from 4% to 49%. Combining it with DIAYN gives diverse manipulation skills.
- **Relevance to our project:** Very high and cheap. Our agent/surround split is natural: hand and fingertip positions vs block pose. The reward is non-zero only when the object moves *with* the hand. That is exactly grasp-and-move, and it is zero for a fixed block, which is a limitation there. Use contact/force features as Sˢ for the fixed block.

### 4.14 Empowerment-based manipulation [read: abstract]
- **Citation:** Dai, Xu, Hofmann, Williams. "An Empowerment-based Solution to Robotic Manipulation Tasks with Sparse Rewards." RSS 2021 / Autonomous Robots 2023. arXiv:2010.07986.
- **Method:** Intrinsic reward = estimated empowerment max I(A; S'_object | S), i.e. the MI between actions and future *object* states, with a neural (MINE-like) estimator. It needs novelty-driven exploration early to bootstrap the estimator. Combined with a diversity term.
- **Results:** Beats other intrinsic methods on object lifting and pick-and-place with sparse rewards.
- **Relevance to our project:** Conceptually ideal ("states where my actions affect the object"), but estimator-heavy. A cheap proxy is the disagreement of an *action-conditioned* object-delta predictor, or MUSIC.

### 4.15 Asymmetric self-play for manipulation skill discovery [read: abstract]
- **Citation:** Jansonnie, Wu, Perez, Peters. "Unsupervised Skill Discovery for Robotic Manipulation through Automatic Task Generation." IEEE-RAS Humanoids 2024. arXiv:2410.04855.
- **Method:** Asymmetric self-play (Alice proposes goal states by acting, Bob reaches them) generates tasks. The behaviours are embedded as Multiplicative Compositional Policy primitives and composed by HRL.
- **Results:** Skills are more object-interactive than skill-learning baselines; transferred to a real robot.
- **Relevance to our project:** Medium-low. Self-play doubles the policy cost, but "Alice moves the block, Bob reproduces the block pose" is a powerful later stage.

---

## 5. Touch / contact / haptics as intrinsic signal

### 5.1 Vulin et al., Tactile intrinsic motivation [read: abstract; details partly known]
- **Citation:** Vulin, Christen, Stevšić, Hilliges. "Improved Learning of Robot Manipulation Tasks via Tactile Intrinsic Motivation." IEEE RA-L 6(2):2194–2201, 2021. arXiv:2102.11051.
- **Method:** The intrinsic reward is proportional to the summed contact forces between finger force sensors and the object (clipped/scaled) [known]. Contact-Prioritized Experience Replay (CPER) preferentially samples episodes and transitions with contact. Built on DDPG + HER [known].
- **Results:** Faster learning on Pick-and-Place, Push and Slide (sparse goals) than HER, PER, and curiosity baselines.
- **Relevance to our project:** Cheap and directly implementable: MuJoCo gives finger-object contact forces through `mj_contactForce` or touch sensors. The reward must be *bounded and saturating*, otherwise the agent learns to squeeze. Contact-prioritised replay is a trivially cheap add-on for any of our learners.

### 5.2 ToC / HaC: Touch-based and Haptics-based Curiosity [read]
- **Citation:** Rajeswar, Ibrahim, Surya, Golemo, Vazquez, Courville, Pinheiro.
  - "Touch-based Curiosity for Sparse-Reward Tasks." arXiv:2104.00442 (2021).
  - Published as "Haptics-based Curiosity for Sparse-reward Tasks," CoRL 2021, PMLR 164:395–405 (proceedings dated 2022).
- **Reward:** r = (1−λ)·‖ĥ_t − h_t‖ + λ·‖ẑ_{t+1} − z_{t+1}‖. The first term is the *cross-modal* error of predicting force/torque (touch) from vision; the second is a forward-dynamics error. λ is swept (≈0.5 balances).
- **Hyperparameters:** SAC, lr 3e-5, batch 128, reward scale 100, 2×128 MLP, 1M steps.
- **Results:** Beats SAC, ICM and RND on Push, Open-door and Pick-up (MiniTouch, Panda arm). The touch-only variant gives about 4× more interactions, but touch-only can lead to "passive constant contact" with little object movement, so the dynamics term is needed.
- **Relevance to our project:** The cross-modal idea maps to state as "predict the contact vector or force from proprioception + object pose". Contacts that geometry does not predict (new contact configurations) are surprising. Pitfall: force signals are non-smooth, so predict *binary* contacts or log-forces.

### 5.3 CCGE: Contact Coverage-Guided Exploration for General-Purpose Dexterous Manipulation [read]
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

### 5.4 Gentle manipulation with curiosity [known]
- **Citation:** Huang, Hao, Gal, et al. "Learning gentle object manipulation with curiosity-driven deep RL." 2019. arXiv:1903.08542.
- **Method:** Penalises impact force ("pain") and uses a *surprise about pain* curiosity term. Without it, agents learn to hit objects or avoid them entirely.
- **Relevance to our project:** Pitfall reference. Force-based rewards need a penalty or saturation on large forces, otherwise the arm learns to slam the block (and the fixed block gives arbitrarily high forces).

### 5.5 RND-Dist: Curiosity-driven learning of joint locomotion and manipulation [read: abstract]
- **Citation:** Schwarke, Klemm, van der Boon, Bjelonic, Hutter. CoRL 2023, PMLR 229:2594–2610 (no arXiv found).
- **Method:** RND applied *only to a low-dimensional task-relevant state* (e.g. hand-object distance, door angle) rather than full observations, with PPO and a single sparse +1 task reward.
- **Results:** Learns door opening and package moving on a real wheeled-legged robot.
- **Relevance to our project:** Practical confirmation that **RND on a curated low-dimensional "curiosity state"** works where full-state RND fails. This is the CCGE baseline "RND-Dist" (65% vs 91%).

---

## 6. Other relevant pieces (brief)

- **LEXA, "Discovering and Achieving Goals via World Models"** (Mendonca, Rybkin, Daniilidis, Hafner, Pathak; NeurIPS 2021; arXiv:2110.09514) [known]. An explorer (Plan2Explore) plus an achiever (goal-conditioned, temporal-distance reward) trained in imagination; tested on Robobin and Kitchen. *Relevance:* explore-then-goal-reach structure; METRA and CSF benchmark against it.
- **Go-Explore** (Ecoffet et al.; arXiv:1901.10995) [known]. Archive of cells; return to a cell, then explore. *Relevance:* with a simulator we can reset to archived states (e.g. "block grasped" states) to amplify rare events. Cheap and very effective in MuJoCo via `mj_setState`, if resets from saved states are allowed.
- **#Exploration** (Tang et al.; NeurIPS 2017; arXiv:1611.04717) [known]. SimHash counts with r = β/√n(φ(s)). *Relevance:* for our low-dimensional object/contact features, explicit discretised counts are simpler than RND and non-stationarity-free.
- **Constrained Ensemble Exploration for Unsupervised Skill Discovery** (Bai et al.; ICML 2024; arXiv:2405.16030) [read: title/abstract only]. An ensemble of skills, each maximising state entropy within a partitioned region. Not examined further.
- **Reizinger et al. 2025** (arXiv:2507.14748). Identifiability theory for CSF-style representations.

---

## 7. Practical lessons for manipulation from state with intrinsic rewards

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

## 8. Recommendations (ranked) for our project

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

### Ideas specific to contact/tactile and object state

- **Contact-set novelty:** count distinct (finger-set, face-set) patterns (2⁶ × faces) with 1/√n bonuses. A *new combination* pays more than repeated single-finger touches. This is a SkiLD-like "interaction graph" with no learned causal model.
- **Opposition/grasp bonus inside novelty:** novelty computed only on states with ≥2 contacts on opposing faces, or a count bin "stable grasp" defined by contact persistence over ≥5 steps.
- **Cross-modal contact surprise (HaC on state):** a small classifier predicts the contact vector at t+1 from (proprio, x_t, a). The reward is the BCE error, or ensemble disagreement over the contact logits. Use *binary* contacts to avoid the non-smooth force problem.
- **Object-controllability (empowerment proxy):** r = ‖E[Δ block pose | a] − E[Δ block pose | a']‖ over sampled actions, or simply the action-conditioned disagreement on block motion. This is zero for unreachable or resting blocks and large when the hand holds the block.
- **Fixed-block variant:** the block cannot move, so replace "object state" with the net wrench applied to the block (force/torque sensor via `mj_contactForce` summed in the block frame, squashed). Novelty over the wrench direction (6-D, normalised) gives "push or pull in all directions" plus "squeeze" exploration.
- **Saturation and safety:** cap the force reward with tanh(F/F₀), with F₀ ≈ 0.5 × the block's weight-equivalent. Add a small penalty for joint-limit hitting and for very large contact forces, so curiosity does not become slamming (Huang et al.).
- **Replay:** contact-prioritised replay (Vulin): sample 50% of each batch from transitions with ≥1 contact once those exist.
- **Optional curriculum via resets:** archive states with new contact cells or block-lifted states, and restart 20–30% of episodes from them (Go-Explore). This multiplies the effective sample efficiency of all methods above.
