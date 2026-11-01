"""Tests for the minimal RL presentation environment."""

import pytest
from virtual_tabletop.assets import build
from virtual_tabletop.policies import ReachPolicy
from virtual_tabletop.rl.env import PresentationEnv


def _run(env, policy=None):
    obs, _ = env.reset(seed=0)
    policy = policy or ReachPolicy(env)
    total, done, phases = 0.0, False, []
    while not done:
        obs, reward, terminated, truncated, info = env.step(policy(obs))
        assert env.observation_space.contains(obs)
        total += reward
        phases.append((info["trial"], info["phase"]))
        done = terminated or truncated
    return total, phases


def test_rl_model_has_no_robots_or_grid():
    model = build.load_rl_model()
    names = {model.body(b).name for b in range(model.nbody)}
    assert not any(n.startswith(("left_", "right_")) for n in names)
    assert model.nmocap == len(build.RL_STIMULI)
    assert model.geom("smartglass").id >= 0
    assert {"monkey/right/hand", "monkey/left/hand"} <= names


def test_trials_follow_phases_and_end_at_timeout_without_reward():
    env = PresentationEnv(num_trials=3, seed=0)
    total, phases = _run(env)
    assert total == 0.0
    cfg = env.config
    trial = env.intertrial + cfg.stimulus_duration + cfg.delay_duration
    trial += cfg.response_timeout
    assert env.time == pytest.approx(3 * trial)
    order = [p for t, p in dict.fromkeys(phases)]
    assert order[:4] == ["intertrial", "stimulus", "delay", "response"]
    # Objects alternate between the two robot arms' object lists.
    groups = [t.spec.group_name for t in env.trials]
    assert groups == [
        "left_manipulator",
        "right_manipulator",
        "left_manipulator",
    ]
    # The scripted reach touches each object (with the released arm).
    assert all(t.touched_by == t.spec.arm for t in env.trials)
    assert not any(t.rewarded for t in env.trials)


def test_reward_on_touch_ends_trials_early():
    env = PresentationEnv(num_trials=3, reward_on_touch=True, seed=0)
    total, _ = _run(env)
    assert total == 3.0
    assert all(t.rewarded for t in env.trials)
    cfg = env.config
    full = env.intertrial + cfg.stimulus_duration + cfg.delay_duration
    full += cfg.response_timeout
    assert env.time < 3 * full


def test_idle_monkey_never_touches():
    env = PresentationEnv(num_trials=2, reward_on_touch=True, seed=0)
    total, _ = _run(env, policy=lambda obs: env.data.qpos[env._qadr].copy())
    assert total == 0.0
    assert all(t.touched_at is None for t in env.trials)


def test_glass_follows_phases():
    env = PresentationEnv(num_trials=1, seed=0)
    env.reset(seed=0)
    still = env.data.qpos[env._qadr].copy()
    seen = {}
    done = False
    while not done:
        _, _, done, _, info = env.step(still)
        seen.setdefault(info["phase"], env.glass_revealed)
    assert seen["intertrial"] is False
    assert seen["stimulus"] is True
