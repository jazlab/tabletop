# Skeletal / musculoskeletal agents, embodied self-organisation, goal babbling and touch: literature notes

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

## 0. TL;DR

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

## 1. Musculoskeletal and skeletal motor control (Mathis lab, MyoSuite)

### 1.1 KINESIS: Motion Imitation for Human Musculoskeletal Locomotion [read]
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

### 1.2 DEP-RL: Embodied Exploration for RL in Overactuated and Musculoskeletal Systems [read]
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

### 1.3 Lattice: Latent Exploration for Reinforcement Learning [read]
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

### 1.4 Acquiring musculoskeletal skills with curriculum-based RL (Baoding balls) [read]
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

### 1.5 SAR: Synergistic Action Representations [read]
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

### 1.6 DynSyn: Dynamical Synergistic Representation [read: abstract]
- **Citation:** Kaibo He, Chenhui Zuo, Chengtian Ma, Yanan Sui. ICML 2024. arXiv:2407.11472.
- **Method:** builds actuator groups ("synergies") from the system's *dynamical structure*, by clustering actuators that produce similar joint motions when excited in simulation. The policy then adapts the synergy weights per state and task.
- **Results:** state-of-the-art sample efficiency on musculoskeletal models up to 700 muscles. It is the only method that worked on FullBody-Gait. KINESIS reports its EMG correlation is worse than KINESIS.
- **Relevance to our project:** low to moderate. A model-based way to get synergies *without* any play data. You could cluster our 16 finger joints by their effect on fingertip positions. Hand-defining "finger flexion" groups (all phalanges of a finger flex together) is a cheap analogue.

### 1.7 MyoDex: A Generalizable Prior for Dexterous Manipulation [read]
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

### 1.8 MyoSuite and MyoChallenge (2022, 2023) [read: retrospect and abstract]
- **Citation:**
  - Vittorio Caggiano, Huawei Wang, Guillaume Durandau, Massimo Sartori, Vikash Kumar. "MyoSuite: A contact-rich simulation suite for musculoskeletal motor control." L4DC 2022. arXiv:2205.13600 [known].
  - Caggiano et al. "MyoChallenge 2022: Learning contact-rich manipulation using a musculoskeletal hand." PMLR 220 (NeurIPS 2022 competition track) [read: PDF listing].
  - MyoChallenge 2023 retrospect (MyoSuite blog).
- **Summary:**
  - The 2022 winners (Mathis lab) used curriculum + PPO-LSTM.
  - The 2023 manipulation-track winners ("Team Lattice", EPFL) relocated unknown objects with the full MyoArm (63 muscles + hand) and reached 34% success.
  - Across top entries, the common recipe was PPO with large batch, curricula, reward shaping, and structured exploration.
- **Relevance to our project:** the MyoChallenge 2023 "relocate" task is the closest benchmark to ours: arm + hand, one object, random shape and pose. Even with shaped rewards and very large compute, 34% was the top. Expect reward-free exploration in 5 h to yield touch, push and occasional grasp/lift, not reliable relocation.

### 1.9 Mathis-lab representation work (proprioception / sensorimotor models) [read: abstract]
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

## 2. Self-organised (playful) behaviour: DEP and "playful machines"

### 2.1 DEP: A novel plasticity rule can explain the development of sensorimotor intelligence [read]
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

### 2.2 Playful Machines: Robotics Implementation of Self-Organised Behaviour [known]
- **Citation:** Ralf Der, Georg Martius. *The Playful Machine: Theoretical Foundation and Practical Realization of Self-Organizing Robots.* Springer (Cognitive Systems Monographs 15), 2012. Software: lpzrobots.
- **Summary:** homeokinesis (the predecessor of DEP). The controller adapts to maximise sensitivity (time-loop error minimisation) while keeping predictability. This yields playful, embodiment-specific behaviours (rolling, climbing, crawling) in many lpzrobots bodies. Martius, Der and Ay 2013 (PLoS ONE) derived a related rule from maximising predictive information (TiPI).
- **Relevance to our project:** conceptual background. The DEP rule is the simpler, more robust descendant. Use DEP rather than homeokinesis.

---

## 3. Motor priors from imitation and hierarchical control (DeepMind / Ölveczky lineage)

### 3.1 Neural Probabilistic Motor Primitives (NPMP) [read: abstract; details known]
- **Citation:** Josh Merel, Leonard Hasenclever, Alexandre Galashov, Arun Ahuja, Vu Pham, Greg Wayne, Yee Whye Teh, Nicolas Heess. ICLR 2019. arXiv:1811.11711.
- **Method:**
  - Train thousands of single-clip tracking experts. Distil them *offline* into an encoder-decoder:
    - the encoder q(z_t | z_{t−1}, future reference frames) feeds an AR(1)-prior-regularised latent z (KL weight β);
    - the decoder π(a | s_proprio, z) is the reusable low-level controller.
  - "Linear-feedback policy cloning" (LFPC) augments behavioural cloning with Jacobian-based local feedback to make distillation data-efficient.
  - **Downstream:** a new high-level policy outputs z (latent about 60-D [known]).
- **Results:** one-shot imitation of unseen motions. The reusable decoder yields naturalistic movement in go-to-target and similar tasks.
- **Relevance to our project:** a template for consolidating many exploration behaviours (DEP episodes, goal-reaching trajectories) into one **latent-conditioned motor controller** that later goal-directed or intrinsic agents command. That is beyond a 5 h budget as a full pipeline. A small version (a VAE-bottleneck BC policy over goal-reaching trajectories) is feasible.

### 3.2 Hierarchical motor control in mammals and machines [known]
- **Citation:** Josh Merel, Matthew Botvinick, Greg Wayne. *Nature Communications* 10:5489, 2019.
- **Summary:** a review arguing for:
  - low-level controllers that are reusable, partially autonomous and sensory-driven;
  - high-level controllers that issue abstract commands;
  - motor learning that should produce reusable "motor primitives" (spinal / brainstem-like).
  
  It draws on NPMP, the virtual rodent, and neuroscience of basal ganglia and motor cortex.
- **Relevance to our project:** it frames our goal. "Reasonable primitives" should be a goal- or latent-conditioned low-level controller (reach-to-point, close-hand-on-contact) that a higher level can sequence.

### 3.3 Deep neuroethology of a virtual rodent [read: abstract]
- **Citation:** Josh Merel, Diego Aldarondo, Jesse Marshall, Yuval Tassa, Greg Wayne, Bence Ölveczky. ICLR 2020. arXiv:1911.09451.
- **Summary:** a MuJoCo rat with vision + proprioception, trained multi-task by distributed off-policy RL on four tasks (gaps, maze forage, escape bowl, two-tap timing).
  - Network analysis finds task-specific "strategy" representations and task-invariant kinematic representations.
  - Neural sequences and population dynamics resemble rodent motor circuits.
- **Relevance to our project:** shows that *analysing* emergent representations (e.g., which units encode contact or grasp phase) is a scientific outcome in its own right. Plan to log hidden activations of our explorers.

### 3.4 A virtual rodent predicts the structure of neural activity across behaviours (MIMIC) [read: summaries and MIMIC-MJX]
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

### 3.5 MoCapAct [read: abstract; details known]
- **Citation:** Nolan Wagener, Andrey Kolobov, Felipe Vieira Frujeri, Ricky Loynd, Ching-An Cheng, Matthew Hausknecht. NeurIPS 2022 Datasets & Benchmarks. arXiv:2208.07363.
- **Summary:**
  - Releases clip-tracking experts (about 2,500 clip snippets, more than 3 h of CMU MoCap) for the dm_control CMU humanoid, plus rollouts.
  - Distils them into an NPMP-style multi-clip policy whose low level is reused for downstream tasks, and a GPT for motion completion.
  - Training the experts reportedly took very large CPU budgets [known].
- **Relevance to our project:** low directly (no monkey hand MoCap). It shows the "experts → distilled latent controller" pattern is reproducible with open code, if we later want to distil our explorers.

### 3.6 PHC: Perpetual Humanoid Control (reference of KINESIS) [known]
- **Citation:** Zhengyi Luo, Jinkun Cao, Alexander Winkler, Kris Kitani, Weipeng Xu. ICCV 2023. arXiv:2305.06456.
- **Summary:** a progressive multiplicative control policy (PMCP). New network "primitives" are allocated for hard sequences mined from failures, then combined with a composer. It also learns fail-state recovery.
- **Relevance to our project:** the source of the hard-negative mining + MoE scheme used by KINESIS (see §1.1).

---

## 4. Goal babbling and intrinsically motivated goal exploration

### 4.1 Goal babbling permits direct learning of inverse kinematics [read: abstract]
- **Citation:** Matthias Rolf, Jochen J. Steil, Michael Gienger. *IEEE Trans. Autonomous Mental Development* 2(3):216–229, 2010.
- **Method:**
  1. Instead of random motor babbling, the learner picks *goals in task space* (hand position).
  2. It tries to reach them with its current inverse model plus exploratory noise, along paths toward the goal.
  3. It trains the inverse model on what it actually achieved (an early form of hindsight relabelling), weighting samples by efficiency and directness.
- **Results:** it learns inverse kinematics for redundant arms (up to 50 DoF planar arms, the Honda humanoid) where motor babbling fails because of non-convex redundancy. It is orders of magnitude more sample-efficient.
- **Relevance to our project:** the canonical argument for exploring in **hand-space goals** rather than joint space for our redundant 7-DoF arm. Reaching emerges quickly this way.

### 4.2 SAGG-RIAC: Active learning of inverse models with intrinsically motivated goal exploration in robots [known]
- **Citation:** Adrien Baranes, Pierre-Yves Oudeyer. *Robotics and Autonomous Systems* 61(1):49–73, 2013.
- **Method:** the goal space is recursively split into regions. Each region's interest is its **competence progress**, the absolute derivative of goal-reaching competence over a sliding window. Goals are sampled preferentially from high-progress regions, plus a random fraction. Low-level reaching uses a local inverse model with micro-exploration.
- **Results:** learning inverse models for high-DoF arms, a quadruped and a fishing-rod task is much faster than with random goal or motor babbling. The learner avoids unreachable and too-easy regions automatically.
- **Relevance to our project:** **learning-progress goal sampling** is the cheapest automatic curriculum. It handles the many unreachable goals (behind the arm, inside the pedestal) without hand engineering.

### 4.3 IMGEP: Intrinsically Motivated Goal Exploration Processes with Automatic Curriculum Learning [read]
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

### 4.4 HER: Hindsight Experience Replay [known]
- **Citation:** Marcin Andrychowicz, Filip Wolski, Alex Ray, Jonas Schneider, Rachel Fong, Peter Welinder, Bob McGrew, Josh Tobin, Pieter Abbeel, Wojciech Zaremba. NeurIPS 2017. arXiv:1707.01495.
- **Method:** relabel transitions with goals achieved later in the same episode ("future" strategy, k = 4 relabels per transition). Use a sparse reward −1[‖φ(s′) − g‖ > ε]. Off-policy DDPG.
- **Results:** solves Fetch push, slide and pick-and-place from sparse rewards. Pick-and-place needs some demonstrations or "goals in the air" tricks.
- **Relevance to our project:** the core machinery for deep goal babbling. With φ(s) as palm position, object position or contact pattern, every trial is useful training data, even if the object never moves.

### 4.5 RIG: Visual RL with Imagined Goals [known]
- **Citation:** Ashvin Nair, Vitchyr Pong, Murtaza Dalal, Shikhar Bahl, Steven Lin, Sergey Levine. NeurIPS 2018. arXiv:1807.04742.
- **Method:** a VAE on observations. Goals are sampled from the VAE prior. Reward = −latent distance. HER-style relabelling with sampled latent goals.
- **Relevance to our project:** with state input we can skip the VAE. The "**sample goals from a generative model of achieved states**" idea remains: sample object or hand goals from a density model fit to the replay buffer.

### 4.6 Skew-Fit: State-Covering Self-Supervised RL [read: abstract; details known]
- **Citation:** Vitchyr H. Pong, Murtaza Dalal, Steven Lin, Ashvin Nair, Shikhar Bahl, Sergey Levine. ICML 2020. arXiv:1903.03698.
- **Method:** maximise I(S; G) = H(G) − H(G|S). Goals are sampled from the replay buffer with importance weights ∝ p_θ(s)^α, where α ∈ [−1, 0) (e.g., −1), so that rare states are up-weighted. The density model p_θ is refit iteratively. The goal distribution provably converges to uniform over reachable states.
- **Results:** state-of-the-art visual goal-reaching. A real robot learned door opening from pixels without reward.
- **Relevance to our project:** **the goal sampler we should use for object goals.** Up-weight rare object displacements and rotations so the agent keeps trying to move the object rather than only reaching. Cheap with state: a kNN or KDE density on object features.

### 4.7 DISCERN: Unsupervised Control Through Non-Parametric Discriminative Rewards [read: abstract; details known]
- **Citation:** David Warde-Farley, Tom Van de Wiele, Tejas Kulkarni, Catalin Ionescu, Steven Hansen, Volodymyr Mnih. ICLR 2019. arXiv:1811.11359.
- **Method:** jointly learns a goal-conditioned policy and an embedding. The reward is the softmax probability that the final state matches the goal among K decoys, using cosine similarity of embeddings. This rewards the *controllable* aspects of the goal. Goals come from a diversity-maintaining buffer.
- **Relevance to our project:** useful if hand-picked goal features are too restrictive. A learned controllable-similarity reward over the interaction features (touch + object) is an alternative to hand-set ε thresholds. It is more complex. Try only after HER on hand-crafted goals works.

### 4.8 Learning Latent Plans from Play [known]
- **Citation:** Corey Lynch, Mohi Khansari, Ted Xiao, Vikash Kumar, Jonathan Tompson, Sergey Levine, Pierre Sermanet. CoRL 2019. arXiv:1903.01973.
- **Method:** human teleoperated "play" data (no tasks). Goal-conditioned BC with hindsight relabelling of windows (Play-GCBC). Play-LMP adds a CVAE plan latent.
- **Results:** one policy performs 18 tasks at about 85% success. Play data is cheap and covers the state space better than task demos.
- **Relevance to our project:** **once exploration produces play-like data, offline goal-conditioned BC with hindsight windows** (1 to 3 s) is a robust, cheap way to turn it into reusable primitives. No RL is needed in the extraction stage, and it is much cheaper than online RL.

### 4.9 Go-Explore (First return, then explore) [known]
- **Citation:** Adrien Ecoffet, Joost Huizinga, Joel Lehman, Kenneth O. Stanley, Jeff Clune. *Nature* 590:580–586, 2021. arXiv:2004.12919.
- **Method:** keep an archive of discretised "cells". Select promising cells (count-based), *return* to them (by simulator restore or a goal-conditioned policy), then explore from there. A policy-based variant handles stochastic settings.
- **Results:** Montezuma's Revenge and Pitfall records. A sparse-reward robot pick-and-place solved with a cell representation over the object and gripper.
- **Relevance to our project:** MuJoCo state restore makes the "return" step free. Store the simulator states at which contact or grasp first occurred, and start exploration episodes from them. This side-steps the hardest exploration step (first contact) at almost no cost. Cells: discretised (palm–object distance, number of touching patches, object height). Pitfall: policies trained from restored states may not learn to *reach* those states. Mix normal resets with restores (e.g., 50/50).

### 4.10 Self-supervised exploration via disagreement [known]
- **Citation:** Deepak Pathak, Dhiraj Gandhi, Abhinav Gupta. ICML 2019. arXiv:1906.04161.
- **Method:** an ensemble of forward models (5) in feature space. Intrinsic reward = the variance of their predictions. It is robust to aleatoric noise. Because the reward is differentiable, the policy can be optimised through it directly.
- **Results:** a real Sawyer learns to push and interact with objects without reward.
- **Relevance to our project:** see `notes_unsupervised_rl.md` §2.3. For us, predict only the object and touch next state (TacEx-style channel weighting, §5.5), not joints.

### 4.11 CEE-US (structured-world-model curiosity) [read]
- **Citation:** Cansu Sancaktar, Sebastian Blaes, Georg Martius. NeurIPS 2022. arXiv:2206.11403.
- **Method:** an ensemble of 5 GNN world models (object nodes, agent as global context). Reward = trace of the ensemble prediction covariance. Planning uses iCEM MPC (horizon 30, 128 samples, 10 elites, 3 iterations). State-only input. Downstream tasks are solved zero-shot by planning.
- **Results:** zero-shot stacking, pick-and-place and throwing in Fetch Construction. Policy-based Disagreement, RND and ICM baselines mostly stay in free space.
- **Cost:** about 72 h on one GPU.
- **Relevance to our project:** shows structured, object-centred uncertainty is what produces object interaction. MPC planning is too slow for our budget (sibling notes §2.5), so use the same reward with an amortised SAC policy.

---

## 5. Developmental models with touch and tactile intrinsic motivation

### 5.1 MIMo: A Multi-Modal Infant Model [read]
- **Citation:** Dominik Mattern, Pierre Schumacher, Francisco M. López, Marcel C. Raabe, Markus R. Ernst, Arthur Aubret, Jochen Triesch. IEEE TCDS (2024). arXiv:2312.04318. Code: github.com/trieschlab/MIMo.
- **Model:** an 18-month-old infant in MuJoCo with five-fingered hands (or mittens), binocular vision, vestibular sense, proprioception, and **full-body virtual skin**.
  - Touch measures normal + friction force, with sensor density set by human two-point discrimination. Forces are spread over nearby sensors.
  - Two actuation models: a spring-damper motor, or an opposing-muscle pair per joint with co-contraction.
- **RL demos:** PPO and SAC (SB3 defaults) on reaching, standing, self-body touch and catching. PPO is more reliable on the harder tasks.
- **Relevance to our project:** the reference open-source embodiment for developmental touch studies. Its touch representation (per-body-part force aggregation) parallels our 18 touch patches. The co-authorship (Schumacher of DEP-RL) means DEP has been tried with similar bodies.

### 5.2 Baby Sophia: self-exploration through self-touch and hand regard (BabyBench) [read]
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

### 5.3 Learning and acting in peripersonal space: moving, reaching and grasping [read: abstract]
- **Citation:** Jonathan Juett, Benjamin Kuipers. *Frontiers in Neurorobotics* 13:4, 2019. arXiv:1809.10788.
- **Method:** a developmental robot (Baxter) learns reach and grasp from unguided exploration. A "peripersonal space graph" is built from random arm configurations. Intrinsic motivation: when an *unusual* event happens (an object moves or is bumped), the agent tries to make it happen reliably, turning surprising outcomes into actions. A simulated palmar grasp reflex (break-beam sensor) is used.
- **Results:** reaching about 97% success, grasping about 50%.
- **Relevance to our project:** two cheap tricks.
  1. **"Reflexes" as priors:** a palmar grasp reflex that closes the fingers when palm touch exceeds a threshold mimics the infant grasp reflex. It makes grasp emerge from reaching plus touch.
  2. **Event-triggered goal adoption:** when the object moves unexpectedly, add that outcome as a goal (HER/IMGEP does this naturally).

### 5.4 Tactile intrinsic motivation and touch-based curiosity [read: abstract; details in sibling notes §5]
- **Citations:**
  - Nikola Vulin, Sammy Christen, Stefan Stevšić, Otmar Hilliges. "Improved learning of robot manipulation tasks via tactile intrinsic motivation." *IEEE RA-L* 6(2), 2021. arXiv:2102.11051.
  - Sai Rajeswar, Cyril Ibrahim, Nitin Surya, Florian Golemo, David Vazquez, Aaron Courville, Pedro O. Pinheiro. "Touch-based / Haptics-based curiosity for sparse-reward tasks." CoRL 2021. arXiv:2104.00442.
- **Method:**
  - Vulin: intrinsic reward = the sum of contact forces between the robot's force sensors and the object, plus **contact-prioritised experience replay**.
  - ToC/HaC: reward = the error of predicting touch from vision/proprioception (cross-modal surprise). A task-free exploration phase is followed by relabelling with task rewards.
- **Results:** faster learning of push, pick-and-place and door opening than ICM and RND.
- **Relevance to our project:** a contact-force reward is the simplest dense "go touch the object" signal. Clip and log-scale it (our touch is already log(1 + F/0.05 N)). Exclude touch_platform. Contact-prioritised replay is a one-line change to a SAC buffer.

### 5.5 TacEx: Tactile Curiosity Drives Robot Interaction [read]
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

### 5.6 Learning tactile skills through curious exploration [read: abstract]
- **Citation:** Leo Pape, Calogero M. Oddo, Marco Controzzi, Christian Cipriani, Alexander Förster, Maria C. Carrozza, Jürgen Schmidhuber. *Frontiers in Neurorobotics* 6:6, 2012.
- **Summary:** a biomimetic robot finger with a tactile sensor. Curiosity is defined as learning progress of predicting tactile input (Schmidhuber-style). It autonomously develops a small set of distinct motor skills that produce different tactile inputs (e.g., tapping vs sliding).
- **Relevance to our project:** an early proof that tactile prediction-progress yields discrete "touch primitives". It supports using **learning progress** rather than raw error as the touch curiosity signal, which also resists contact-chatter noise.

---

## 6. Other exploration-noise references (brief, [known])

- **gSDE:** Antonin Raffin, Jens Kober, Freek Stulp. "Smooth exploration for robotic RL." CoRL 2021 (PMLR 2022). arXiv:2005.05719.
  - State-dependent noise ε = θ_ε·x with θ_ε resampled every n steps. It gives smooth, safe exploration and is available in SB3.
  - Lattice's base.
- **Pink noise:** Onno Eberhard, Jakob Hollenstein, Cristina Pinneri, Georg Martius. "Pink noise is all you need: colored noise exploration in deep RL." ICLR 2023.
  - Replaces white Gaussian action noise in SAC/TD3/MPO with 1/f (β = 1) temporally correlated noise.
  - It is the best default across many DMC tasks. There is a `pink-noise-rl` package, and it costs nothing.
- **Colored-noise iCEM:** Pinneri et al. CoRL 2020 (used in CEE-US).

---

## 7. Recommendations for our setting

These are ordered by expected value per CPU-hour. Our sim: 75-step trials at 25 Hz control, delta position servos, 10 cores. All learners should log the env's `info` metrics (touching, n_touch, grasp, lifted, obj_disp, obj_rot_angle, palm_obj_dist) per trial for comparison. They should also save their replay buffers for offline relabelling (ExORL-style evaluation; see sibling notes).

### 7.1 DEP as a zero-cost explorer and data generator (≈ 10 to 20 min, including tuning)

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

### 7.2 Modular goal babbling with hindsight (IMGEP + HER + Skew-Fit). The main candidate for usable primitives (≈ 1.5 to 2 h)

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

### 7.3 Touch- and object-focused curiosity (≈ 1 h), as a competitor or a bonus for §7.2

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

### 7.4 Exploration noise: pink → gSDE → Lattice (ablation, ≈ 30 min)

- Replace SAC's white Gaussian sampling noise with **pink noise** (β = 1), resampled per trial of 75 steps. This is a free win for smooth arm motion.
- If time allows, implement **Lattice** in the SAC actor:
  - noise matrices on the last hidden layer (256-D);
  - α = 1, resample period T = 4 to 8 steps;
  - initial log-std 0;
  - log-likelihood with the full covariance (23×23 Cholesky per batch is cheap).
  - Expect the main benefit on the 16 coupled finger joints and in lower "energy" (action magnitude and jerk).

### 7.5 Synergies from exploration data (SAR), minutes of compute

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

### 7.6 Developmental priors and curricula (cheap multipliers for every method)

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

### 7.7 Consolidation into primitives (if time remains, ≈ 30 min)

- **Offline goal-conditioned BC from play** (Lynch et al.) on the union of all explorers' buffers:
  - hindsight windows of 10 to 40 steps;
  - goal = φ(s_{t+H}) in each module space;
  - optionally a KL-regularised latent bottleneck (NPMP/MIMIC-style, β ≈ 1e-3 to 1e-2).
  
  This yields a single reusable controller and lets us compare which explorer produced the most useful data (ExORL-style).
- Optionally use a **MoE gate** (KINESIS) over frozen module-specialist policies (reach, grasp, push), trained with the same goal rewards.

### 7.8 Suggested 5 h allocation

| Block | Time |
|---|---|
| DEP tuning and data | 0.25 h |
| Pink-noise SAC + contact-novelty reward (§7.3b), reward-free | 0.75 h |
| TacEx-style tactile/object disagreement SAC with DEP switching (§7.3a + §7.1c) | 1.25 h |
| Modular goal babbling with HER + LP + Skew-Fit (§7.2), with grasp reflex and static→dynamic curriculum | 2 h |
| SAR synergy extraction + short downstream comparison, or offline GCBC consolidation | 0.5 h |
| Evaluation on held-out shapes | 0.25 h |

### 7.9 General pitfalls collected from the papers

1. **Full-state novelty or curiosity mostly explores the arm, not the object** (CEE-US, TacEx, and sibling notes). Always restrict intrinsic objectives to touch and object channels, or to goal modules.
2. **Flat goal spaces fail. Modular, object-centred goal spaces with learning progress work** (Forestier et al.).
3. **Summing DEP and policy actions is chaotic. Alternate instead** (DEP-RL).
4. **Without a curriculum, dexterous tasks give about 0% on the hard phase** (Chiappa 2024). Static before dynamic.
5. **Synergies are lossy.** Keep residual control (Chiappa 2024; KINESIS appendix; SAR blending).
6. **Reward-free from-scratch policies move unnaturally** (KINESIS goal reaching without a prior). Add action-rate/energy penalties and smooth (pink or Lattice) noise. Lattice also reduces energy by 20 to 60%.
7. **Hand-tuned bonus schemes** (Baby Sophia) **fixate on one solution.** Use habituation and learning progress, not fixed jackpots.
