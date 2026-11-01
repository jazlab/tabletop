"""Watch the RL presentation task played by a scripted (or idle) monkey.

Interactive (macOS re-launches itself under ``mjpython``)::

    uv run vtt-rl-demo [--rig] [--reward-on-touch] [--policy reach|still]

Video instead::

    uv run vtt-rl-demo --video rl_demo.mp4 [--trials 6] [--camera overview]

Keys in the viewer: space pause/resume, R restart, =/- faster/slower.
"""

import argparse
import os
import sys
import time
from pathlib import Path

import mujoco

from virtual_tabletop.policies import ReachPolicy
from virtual_tabletop.rl.env import PresentationEnv

KEY_SPACE, KEY_R, KEY_EQUAL, KEY_MINUS = 32, 82, 61, 45


def make_policy(env: PresentationEnv, name: str):
    if name == "reach":
        return ReachPolicy(env)
    if name == "still":
        return lambda obs: env.data.qpos[env._qadr].copy()
    raise ValueError(f"Unknown policy {name}")


def render_video(env, policy_name: str, output: Path, fps: int = 30) -> Path:
    """Play one episode and write it to ``output``."""
    import imageio.v2 as imageio

    obs, _ = env.reset()
    policy = make_policy(env, policy_name)
    renderer = mujoco.Renderer(env.model, 540, 960)
    every = max(1, round(1.0 / (fps * env.dt)))
    done, k = False, 0
    with imageio.get_writer(output, fps=fps, macro_block_size=8) as writer:
        while not done:
            obs, _, terminated, truncated, _ = env.step(policy(obs))
            done = terminated or truncated
            if k % every == 0:
                renderer.update_scene(env.data, camera=env.camera)
                writer.append_data(renderer.render())
            k += 1
    renderer.close()
    return output


def run_viewer(env, policy_name: str) -> None:
    import mujoco.viewer

    state = {"paused": False, "speed": 1.0, "restart": False}

    def on_key(key: int) -> None:
        if key == KEY_SPACE:
            state["paused"] = not state["paused"]
        elif key == KEY_R:
            state["restart"] = True
        elif key == KEY_EQUAL:
            state["speed"] = min(state["speed"] * 2, 16.0)
        elif key == KEY_MINUS:
            state["speed"] = max(state["speed"] / 2, 0.125)

    obs, _ = env.reset()
    policy = make_policy(env, policy_name)
    with mujoco.viewer.launch_passive(
        env.model, env.data, key_callback=on_key
    ) as v:
        v.cam.lookat[:] = (0.95, 0.25, 0.65)
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 1.4, -60, -20
        last, budget, done = time.monotonic(), 0.0, False
        while v.is_running():
            now = time.monotonic()
            if not state["paused"]:
                budget += (now - last) * state["speed"]
            last = now
            if done:
                time.sleep(1.0)  # show the end of the episode briefly
                last = time.monotonic()
            # Change the shared model/data only while holding the viewer
            # lock, but never call viewer methods (set_texts, sync) inside
            # it: they take the same lock and would deadlock the viewer.
            with v.lock():
                if state["restart"] or done:
                    obs, _ = env.reset()
                    policy = make_policy(env, policy_name)
                    state["restart"], done, budget = False, False, 0.0
                while budget >= env.dt and not done:
                    obs, _, terminated, truncated, _ = env.step(policy(obs))
                    done = terminated or truncated
                    budget -= env.dt
                status = env.status_text() + f"  x{state['speed']:g}"
            v.set_texts(
                (
                    mujoco.mjtFontScale.mjFONTSCALE_150,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    status,
                    "",
                )
            )
            v.sync()
            time.sleep(1 / 60)


def _ensure_mjpython() -> None:
    """On macOS, re-run under mjpython (MuJoCo's viewer needs it)."""
    if sys.platform != "darwin" or os.environ.get("VTT_UNDER_MJPYTHON"):
        return
    mjpython = Path(sys.executable).with_name("mjpython")
    os.environ["VTT_UNDER_MJPYTHON"] = "1"
    os.execv(
        mjpython,
        [str(mjpython), "-m", "virtual_tabletop.rl.demo", *sys.argv[1:]],
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--task", default="foraging_ordered")
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument(
        "--policy", choices=["reach", "still"], default="reach"
    )
    parser.add_argument("--reward-on-touch", action="store_true")
    parser.add_argument(
        "--rig", action="store_true", help="show the rig frame"
    )
    parser.add_argument("--camera", default="subject")
    parser.add_argument("--video", type=Path, help="render to this file")
    args = parser.parse_args()
    if not args.video:
        _ensure_mjpython()
    env = PresentationEnv(
        args.task,
        num_trials=args.trials,
        reward_on_touch=args.reward_on_touch,
        include_rig=args.rig,
        camera=args.camera,
    )
    if args.video:
        print(render_video(env, args.policy, args.video))
        rewarded = sum(t.rewarded for t in env.trials)
        touched = sum(t.touched_at is not None for t in env.trials)
        print(
            f"{len(env.trials)} trials, {touched} touched, {rewarded} rewarded"
        )
    else:
        run_viewer(env, args.policy)


if __name__ == "__main__":
    main()
