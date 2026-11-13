# Part 1: Related work

Version 0's literature review (version_0/report/01_literature_review.pdf,
about 65 papers) covered reward-free exploration and skeletal agents.
Version 1 adds supervision from a hand-crafted controller in three ways.
This part summarises the methods each one builds on.

## Learning from demonstrations in off-policy RL (method A)

- **DDPGfD** (Večerík et al. 2017) keeps demonstration transitions in
  the replay buffer permanently and mixes them into every batch. This
  lets sparse-reward insertion tasks be solved from about 20
  demonstrations.
- **Overcoming exploration with demonstrations** (Nair et al. 2018) adds
  a behaviour-cloning loss on the demonstration samples. The loss is
  **Q-filtered**: a demonstration action is imitated only where the
  critic rates it above the policy's own action, so the policy can
  overtake the demonstrator. With HER, this solved block stacking that
  pure RL could not.
- **TD3+BC** (Fujimoto & Gu 2021) scales the BC term against the mean
  |Q| of the batch, so a single weight works across reward scales.
  Version 1 uses the same normalisation.
- **RLPD** (Ball et al. 2023) shows that symmetric sampling, with a fixed
  fraction of every batch from prior data, is a strong default for online
  RL with offline data. Version 1 uses 25%.

→ **A**: scripted demonstrations in a separate replay, 25% of each batch,
plus a Q-filtered BC term that fades from λ = 1 to 0 over the first half
of training.

## Reverse curricula and restarts from demonstration states (method B)

- **Reverse curriculum generation** (Florensa et al. 2017) starts
  episodes near the goal and gradually moves the start states backwards,
  so a sparse success signal is always within reach.
- **Learning Montezuma's Revenge from a single demonstration** (Salimans
  & Chen 2018) restarts episodes from states along a demonstration,
  moving the restart point from the end towards the beginning as the
  agent succeeds.
- **Go-Explore** (Ecoffet et al. 2021) restarts from archived states.
  Version 0 tried archived contact states and found a train/test shift
  when half of all trials started mid-contact.

→ **B**: each env records its own successful scripted trials. Restarts
come from snapshots at time index ≥ t_min, with t_min moving from 80% of
the trajectory (block already lifted) to 0, and the restart probability
falling from 0.5 to 0.1. Late in training most trials start from rest,
like the test trials.

## Residual policy learning (method C)

- **Residual policy learning** (Silver et al. 2018) and **residual RL**
  (Johannink et al. 2019) learn a correction `a = a_base + a_π` on top of
  a hand-designed controller. The policy starts from competent behaviour
  and only learns what the controller gets wrong. Silver et al. also give
  the policy the base action as input.

→ **C**: `a = clip(a_script + 0.5 · a_π)`, with the script's action
appended to the observation.

## Sparse task bonus (method D)

Adding a sparse success term alongside an intrinsic reward is the usual
way to steer curiosity toward a task, for example ToC/HaC (Rajeswar et
al. 2022) and CCGE (Liu et al. 2026), both covered in version 0's review.

→ **D**: a bonus of 0.5–1.0 per step while the block is lifted. This
is the only reward term that encodes the task.

## Natural movement

KINESIS (Simos, Chiappa & Mathis 2025) and Lattice (Chiappa et al. 2023)
report that policies trained from scratch move unnaturally unless effort
or smoothness is penalised, or a motor prior is used. Real fingers couple
the distal and proximal interphalangeal joints through the tendons, a
structural constraint that synergy approaches (SAR, Berg et al. 2023)
partly recover from data.

→ **Version 1 hand**: anatomical extension limits, DIP–PIP coupling, and
small effort and jerk costs on every run.
