"""Tests for the RL training wrappers, config and a tiny training run."""

import numpy as np
import pytest
from virtual_tabletop.training.config import (
    TRAINING_CONFIGS,
    RunConfig,
    controlled_joints,
    make_env,
)
from virtual_tabletop.training.wrappers import ReachShaping


@pytest.fixture(scope="module")
def config() -> RunConfig:
    return RunConfig.load(TRAINING_CONFIGS / "touch_right_arm.yaml")


def test_config_controls_the_right_arm_only(config):
    joints = controlled_joints(config.env.controlled_arms)
    assert len(joints) == 7
    assert all(j.startswith("right/") for j in joints)
    env = make_env(config, seed=0)
    assert env.action_space.shape == (7,)
    assert (
        env.action_space.low.min() == -1 and env.action_space.high.max() == 1
    )


def test_delta_actions_are_rate_limited_steps(config):
    env = make_env(config, seed=0, shaping=False)
    env.reset(seed=0)
    base = env.unwrapped
    while base.phase != "response":  # the arm is locked before
        env.step(np.zeros(7))
    start = base.last_target.copy()
    env.step(np.ones(7))
    step = base.last_target - start
    expected = config.wrappers.max_speed * base.dt
    assert np.all(step <= expected + 1e-9)
    assert np.max(step) == pytest.approx(expected)


def test_shaping_is_potential_based(config):
    """Over a trial, shaping adds up to gamma-discounted potential changes:
    with gamma = 1 it telescopes to phi(end) - phi(start) = 0 - 0."""
    env = make_env(config, seed=0)
    shaping = env.env
    assert isinstance(shaping, ReachShaping)
    shaping.gamma = 1.0
    env.reset(seed=0)
    total, rng = 0.0, np.random.default_rng(0)
    done = False
    while not done:
        _, _, terminated, truncated, info = env.step(rng.uniform(-1, 1, 7))
        total += info["shaping"]
        done = terminated or truncated or info["trial_ended"]
    assert total == pytest.approx(0.0, abs=1e-9)


@pytest.mark.slow
def test_tiny_training_run_writes_a_loadable_policy(config, tmp_path):
    pytest.importorskip("stable_baselines3")
    from virtual_tabletop.training.evaluate import run_episodes, trained_policy
    from virtual_tabletop.training.train import train

    config.ppo.total_timesteps = 512
    config.ppo.n_envs = 2
    config.ppo.n_steps = 128
    config.ppo.batch_size = 64
    config.eval_every = 10**9
    config.eval_trials = 2
    run = train(config, tmp_path)
    for name in ("model.zip", "obs_normalizer.pkl", "evals.csv"):
        assert (run / name).exists()
    metrics = run_episodes(
        make_env(config, shaping=False), trained_policy(run), n_trials=2
    )
    assert metrics["trials"] == 2
