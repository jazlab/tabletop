"""Tests for the toy environments."""

import gymnasium as gym
import mujoco
import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env
from mujoco_toys import ArmBlocksEnv, PlaypenEnv
from mujoco_toys.envs.arm_blocks import JOINTS
from mujoco_toys.objects import SLOT_FEATURES

ENVS = (PlaypenEnv, ArmBlocksEnv)


@pytest.mark.parametrize("cls", ENVS)
def test_passes_the_gymnasium_checker(cls):
    check_env(cls(seed=0), skip_render_check=True)


@pytest.mark.parametrize(
    "name", ["MujocoToys/Playpen-v0", "MujocoToys/ArmBlocks-v0"]
)
def test_registered_with_gymnasium(name):
    env = gym.make(name)
    obs, _ = env.reset(seed=0)
    assert env.observation_space.contains(obs)


@pytest.mark.parametrize("cls", ENVS)
def test_reward_is_always_zero(cls):
    env = cls(seed=0)
    env.reset(seed=0)
    rewards = {env.step(env.action_space.sample())[1] for _ in range(100)}
    assert rewards == {0.0}


@pytest.mark.parametrize("cls", ENVS)
def test_seeding_is_deterministic(cls):
    actions = np.random.default_rng(1).uniform(
        -1, 1, (60, cls(seed=0).action_space.shape[0])
    )
    runs = []
    for _ in range(2):
        env = cls()
        obs, _ = env.reset(seed=7)
        trace = [obs]
        for a in actions:
            trace.append(env.step(a)[0])
        runs.append(np.array(trace))
    np.testing.assert_array_equal(runs[0], runs[1])


@pytest.mark.parametrize("cls", ENVS)
def test_episodes_draw_new_objects(cls):
    env = cls()
    env.reset(seed=1)
    first = [s.size for s in env.shapes]
    env.reset(seed=2)
    assert first != [s.size for s in env.shapes]


def test_spawning_fills_slots_then_recycles():
    env = PlaypenEnv(n_objects=2, max_objects=5, spawn_rate=50.0, seed=0)
    env.reset(seed=0)
    assert env.pool.active.sum() == 2
    spawned, counts = [], []
    for _ in range(20):
        info = env.step(np.zeros(2))[4]
        spawned += info["spawned"]
        counts.append(info["active_objects"])
    assert max(counts) == 5
    assert len(spawned) > 5  # kept spawning by recycling slots
    # Every active object is inside the arena.
    pos = env.pool.positions()[env.pool.active]
    assert np.all(np.abs(pos[:, :2]) < env.arena_half)


def test_parked_slots_are_zero_in_the_observation():
    env = PlaypenEnv(n_objects=3, max_objects=8, seed=0)
    obs, _ = env.reset(seed=0)
    layout = env.observation_layout
    assert obs.size == layout["obj7"].stop
    for i in range(8):
        part = obs[layout[f"obj{i}"]]
        assert part.size == SLOT_FEATURES
        assert (part[0] == 1.0) == env.pool.active[i]
        if not env.pool.active[i]:
            assert not part.any()


def test_the_ball_pushes_objects():
    env = PlaypenEnv(
        n_objects=1,
        max_objects=1,
        object_kinds=("box",),
        object_scale=(0.1, 0.1),
        seed=0,
    )
    env.reset(seed=0)
    # Put the agent at the left and the object right in front of it.
    env.data.qpos[:2] = (-0.8, 0.0)
    env.pool.place(0, (-0.45, 0.0, env.shapes[0].rest_height), 0.0)
    mujoco.mj_forward(env.model, env.data)
    start = env.pool.positions()[0].copy()
    touched = False
    for _ in range(40):
        info = env.step(np.array([1.0, 0.0]))[4]
        touched |= 0 in info["touching"]
    assert touched
    assert env.pool.positions()[0][0] > start[0] + 0.2


def test_arm_actions_span_the_joint_ranges():
    """Regression: joint ranges are in radians (MjSpec defaults to degrees)."""
    env = ArmBlocksEnv(seed=0)
    env.reset(seed=0)
    for (name, _, (low, high)), (lo, hi) in zip(
        JOINTS, env.model.actuator_ctrlrange, strict=True
    ):
        assert (lo, hi) == pytest.approx((low, high), abs=1e-6), name
    for _ in range(60):
        env.step(np.array([0.5, 0.0, 0.0]))
    yaw = env.data.qpos[env.model.joint("yaw").qposadr[0]]
    assert yaw == pytest.approx(np.pi / 2, abs=0.05)


def test_towers_stand_until_hit():
    env = ArmBlocksEnv(towers=1, tower_height=3, seed=3)
    env.reset(seed=3)
    heights = sorted(env.pool.positions()[env.pool.active][:, 2])
    # Hold the rest posture: nothing should fall over by itself.
    rest = np.zeros(3)  # centre of each range
    for _ in range(50):
        env.step(rest)
    after = sorted(env.pool.positions()[env.pool.active][:, 2])
    assert after == pytest.approx(heights, abs=0.01)
    assert max(after) > 0.15  # the tower is still standing


def test_pixel_and_dict_observations():
    env = PlaypenEnv(obs_mode="pixels", image_size=(64, 80), seed=0)
    obs, _ = env.reset(seed=0)
    assert obs.shape == (64, 80, 3) and obs.dtype == np.uint8
    assert obs.std() > 0  # not blank
    env = ArmBlocksEnv(obs_mode="both", image_size=(48, 48), seed=0)
    obs, _ = env.reset(seed=0)
    assert env.observation_space.contains(obs)
