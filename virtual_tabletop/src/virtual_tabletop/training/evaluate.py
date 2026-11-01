"""Evaluate policies on a run's task: success rate, time to respond, video.

::

    uv run vtt-rl-eval RUN_DIR [--trials 200] [--baselines] [--video out.mp4]

Metrics are always on the task's own (unshaped) reward, over ``--trials``
trials with a fixed seed, so trained and baseline policies see the same
trial sequence:

- **success rate**: rewarded trials / trials;
- **time to respond**: from the response window opening to the rewarded
  touch or press (rewarded trials only);
- breakdowns by object geometry and presentation pose.

Baselines (``--baselines``): the scripted policy for the task
(:func:`~virtual_tabletop.rl.policies.scripted_policy`, an IK reacher or the
button-press planner) and an idle monkey. Writes ``eval.json`` in the run.
"""

import argparse
import json
from collections import defaultdict
from collections.abc import Callable
from pathlib import Path

import mujoco
import numpy as np

from virtual_tabletop.rl.policies import scripted_policy
from virtual_tabletop.rl.scene import object_geometry
from virtual_tabletop.training.config import (
    RunConfig,
    make_base_env,
    make_env,
)

#: Seed for evaluation trial sequences (distinct from training seeds).
EVAL_SEED = 12345


def trial_metrics(env) -> dict:
    """Summarise ``env``'s completed trials (the unwrapped env)."""
    trials = env.trials
    times = [
        (t.pressed_at if env.reward == "button" else t.touched_at)
        - (
            t.start
            + sum(
                env.schedule.durations[p]
                for p in ("intertrial", "stimulus", "delay")
            )
        )
        for t in trials
        if t.rewarded
    ]
    groups: dict[str, list[bool]] = defaultdict(list)
    for t in trials:
        groups[f"geometry:{object_geometry(t.spec.object_id)}"].append(
            t.rewarded
        )
        x = t.spec.object_pose.position[0]
        groups[f"pose_x:{x:.3f}"].append(t.rewarded)
    return {
        "trials": len(trials),
        "success_rate": float(np.mean([t.rewarded for t in trials])),
        "time_to_respond_mean": float(np.mean(times)) if times else None,
        "time_to_respond_median": float(np.median(times)) if times else None,
        "times_to_respond": [round(float(x), 3) for x in times],
        "by_group": {k: float(np.mean(v)) for k, v in sorted(groups.items())},
    }


def run_episodes(
    env, act: Callable, n_trials: int, seed: int = EVAL_SEED, frames=None
) -> dict:
    """Run ``act(obs)`` on ``env`` for ``n_trials`` trials; metrics.

    ``env`` may be wrapped; trials are counted on the unwrapped env.
    ``frames`` (a ``(renderer, list)`` pair) collects video frames.
    """
    base = env.unwrapped
    base.schedule.num_trials = n_trials
    obs, _ = env.reset(seed=seed)
    done, k = False, 0
    while not done:
        obs, _, terminated, truncated, _ = env.step(act(obs))
        done = terminated or truncated
        if frames is not None and k % 2 == 0:  # 25 fps at dt = 0.02
            renderer, out = frames
            renderer.update_scene(base.data, camera=base.camera)
            out.append(renderer.render())
        k += 1
    return trial_metrics(base)


def trained_policy(run_dir: Path, deterministic: bool = True):
    """Load a run's final policy and observation normaliser.

    Returns ``act(obs) -> action`` for the run's (delta-action) env.
    """
    import pickle

    from stable_baselines3 import PPO

    model = PPO.load(run_dir / "model.zip", device="cpu")
    with open(run_dir / "obs_normalizer.pkl", "rb") as f:
        normalizer = pickle.load(f)

    def act(obs):
        action, _ = model.predict(
            normalizer.normalize_obs(obs), deterministic=deterministic
        )
        return action

    return act


def evaluate_run(
    run_dir: Path, n_trials: int, baselines: bool, video: Path | None
) -> dict:
    config = RunConfig.load(run_dir / "config.yaml")
    results = {}
    env = make_env(config, shaping=False)
    frames = None
    if video is not None:
        frames = (mujoco.Renderer(env.unwrapped.model, 480, 640), [])
    results["trained"] = run_episodes(
        env, trained_policy(run_dir), n_trials, frames=frames
    )
    if frames is not None:
        import imageio.v2 as imageio

        imageio.mimsave(video, frames[1], fps=25, macro_block_size=8)
    if baselines:
        results.update(evaluate_baselines(config, n_trials))
    (run_dir / "eval.json").write_text(json.dumps(results, indent=2))
    return results


def evaluate_baselines(config: RunConfig, n_trials: int) -> dict:
    """The scripted and idle policies on the run's task env."""
    results = {}
    base = make_base_env(config.env)
    policy = scripted_policy(base)
    results["scripted"] = run_episodes(base, policy, n_trials)
    base = make_base_env(config.env)
    results["idle"] = run_episodes(
        base, lambda obs: base.joint_positions, n_trials
    )
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("run", type=Path, help="run directory")
    parser.add_argument("--trials", type=int, default=200)
    parser.add_argument("--baselines", action="store_true")
    parser.add_argument(
        "--video", type=Path, help="video of the trained policy"
    )
    args = parser.parse_args()
    results = evaluate_run(args.run, args.trials, args.baselines, args.video)
    for name, r in results.items():
        t = r["time_to_respond_mean"]
        print(
            f"{name:8s} success {100 * r['success_rate']:5.1f}%"
            f"  time to respond "
            + ("  -  " if t is None else f"{t:.2f} s")
            + f"  ({r['trials']} trials)"
        )


if __name__ == "__main__":
    main()
