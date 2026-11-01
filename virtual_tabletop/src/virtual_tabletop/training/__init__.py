"""Training RL agents on the presentation tasks (:mod:`virtual_tabletop.rl`).

Needs the optional ``rl`` extra (stable-baselines3, tensorboard):
``uv sync --extra rl``.

- :mod:`.wrappers`: ``DeltaJointAction`` (normalised, rate-limited joint
  steps) and ``ReachShaping`` (potential-based reward for approaching the
  object);
- :mod:`.config`: ``RunConfig`` (one YAML file per run, in
  ``config/training/``) and ``make_env``;
- :mod:`.evaluate`: success metrics for trained, scripted and idle policies
  (``vtt-rl-eval``), and videos;
- :mod:`.train`: PPO training (``vtt-rl-train``) with periodic evaluation;
- :mod:`.plots`: learning curves for a run.

Runs are written to ``<cache>/runs/<name>/``. How-to:
``documentation/5_train_rl_agents.md``.
"""
