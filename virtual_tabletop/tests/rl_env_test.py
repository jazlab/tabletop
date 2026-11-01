"""Tests for the RL presentation env: trials, the three tasks, control."""

import numpy as np
import pytest
from virtual_tabletop.rl.env import PresentationEnv
from virtual_tabletop.rl.policies import ReachPolicy, scripted_policy


def _run(env, policy=None):
    obs, _ = env.reset(seed=0)
    policy = policy or scripted_policy(env)
    total, done, phases = 0.0, False, []
    while not done:
        obs, reward, terminated, truncated, info = env.step(policy(obs))
        assert env.observation_space.contains(obs)
        total += reward
        phases.append((info["trial"], info["phase"]))
        done = terminated or truncated
    return total, phases


def _trial_length(env) -> float:
    return sum(env.schedule.durations.values())


def test_trials_follow_phases_and_end_at_timeout_without_reward():
    env = PresentationEnv(num_trials=3, seed=0)
    total, phases = _run(env)
    assert total == 0.0
    assert env.time == pytest.approx(3 * _trial_length(env))
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


def test_touch_task_rewards_touches_and_ends_trials_early():
    env = PresentationEnv(num_trials=3, reward="touch", seed=0)
    total, _ = _run(env)
    assert total == 3.0
    assert all(t.rewarded for t in env.trials)
    assert env.time < 3 * _trial_length(env)


def test_idle_monkey_never_touches():
    env = PresentationEnv(num_trials=2, reward="touch", seed=0)
    total, _ = _run(env, policy=lambda obs: env.joint_positions)
    assert total == 0.0
    assert all(t.touched_at is None for t in env.trials)


def test_stimulus_visibility_follows_phases():
    env = PresentationEnv(num_trials=1, seed=0)
    env.reset(seed=0)
    seen, done = {}, False
    while not done:
        _, _, done, _, info = env.step(env.joint_positions)
        seen.setdefault(info["phase"], env.stimulus_visible)
    assert seen["intertrial"] is False
    assert seen["stimulus"] is True


@pytest.mark.slow
def test_button_task_rewards_presses_not_touches():
    env = PresentationEnv(
        num_trials=4, reward="button", response_timeout=3.0, seed=0
    )
    total, _ = _run(env)
    assert total == 4.0
    for trial in env.trials:
        assert trial.rewarded and trial.pressed_by == trial.spec.arm
    # Holding still (no press) isn't rewarded.
    env.reset(seed=0)
    while env.phase != "response":
        env.step(env.joint_positions)
    _, reward, *_ = env.step(env.joint_positions)
    assert reward == 0.0


@pytest.mark.slow
def test_physics_keeps_the_hand_out_of_the_object():
    """Contact forces stop bones at the object's surface (and the button)."""
    env = PresentationEnv(
        num_trials=2, reward="button", response_timeout=3.0, seed=0
    )
    obs, _ = env.reset(seed=0)
    policy = scripted_policy(env)
    deepest, done = 0.0, False
    while not done:
        obs, _, terminated, truncated, _ = env.step(policy(obs))
        done = terminated or truncated
        if env.phase == "response":
            d = env.data
            for i in range(d.ncon):
                deepest = max(deepest, -d.contact[i].dist)
    assert all(t.rewarded for t in env.trials)
    assert deepest < 0.005


def test_kinematic_control_sets_angles_directly():
    env = PresentationEnv(
        num_trials=1, control="kinematic", lock_arms=False, seed=0
    )
    env.reset(seed=0)
    target = env.joint_positions
    target[env.joint_names.index("monkey/right/elbow_flexion")] = 1.0
    env.step(target)
    assert env.joint_positions == pytest.approx(target)


def test_observation_layout_names_every_entry():
    env = PresentationEnv(num_trials=1, seed=0)
    obs, _ = env.reset(seed=0)
    layout = env.observation_layout
    assert sum(s.stop - s.start for s in layout.values()) == obs.size
    np.testing.assert_allclose(
        obs[layout["object_position"]], env.scene.object_position, atol=1e-6
    )
    np.testing.assert_allclose(
        obs[layout["fingertip_right"]],
        env.scene.hand_position("right"),
        atol=1e-6,
    )
    assert obs[layout["released_left"]] == 0.0  # intertrial


def test_invalid_options_are_rejected():
    with pytest.raises(ValueError):
        PresentationEnv(reward="grab")
    with pytest.raises(ValueError):
        PresentationEnv(control="torque")


def test_scripted_policy_matches_the_task():
    assert type(scripted_policy(PresentationEnv(reward="touch"))) is (
        ReachPolicy
    )


def test_locked_arms_stay_at_rest_until_released():
    env = PresentationEnv(
        num_trials=1, control="kinematic", arms=["right"], seed=0
    )
    env.reset(seed=0)
    elbow = env.joint_names.index("monkey/right/elbow_flexion")
    reach = env.joint_positions
    reach[elbow] = 1.0
    env.step(reach)
    while env.phase != "response":
        assert env.joint_positions[elbow] == 0.0  # locked
        env.step(reach)
    assert env.joint_positions[elbow] == pytest.approx(1.0)  # released


def test_random_order_samples_poses_and_honours_arm_override():
    env = PresentationEnv(
        num_trials=60, trial_order="random", arms=["right"], seed=0
    )
    env.reset(seed=0)
    specs = []
    for _ in range(60):
        specs.append(env.trial.spec)
        env.schedule.finish(env.time, None)
        if not env.schedule.last:
            env.schedule.start(env.time)
    assert {s.arm for s in specs} == {"right"}
    assert len({s.object_pose.position for s in specs}) == 3
    assert len({s.object_id for s in specs}) > 10
