"""Watch an RL task played by a scripted (or idle) monkey.

Interactive (macOS re-launches itself under ``mjpython``)::

    uv run vtt-rl-demo [--reward none|touch|button] [--rig]
        [--policy scripted|still] [--response-timeout S]

Video instead::

    uv run vtt-rl-demo --video rl_demo.mp4 [--trials 6] [--camera overview]

Keys in the viewer: space pause/resume, R restart, =/- faster/slower.
"""

import argparse
from pathlib import Path

import mujoco

from virtual_tabletop import viewer
from virtual_tabletop.rl.env import REWARDS, PresentationEnv
from virtual_tabletop.rl.policies import scripted_policy

#: Seconds the viewer lingers on the end of an episode before restarting.
END_PAUSE = 1.0


def make_policy(env: PresentationEnv, name: str):
    if name == "scripted":
        return scripted_policy(env)
    if name == "still":
        return lambda obs: env.joint_positions
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
    playback = viewer.Playback()
    state = {"restart": True, "budget": 0.0, "linger": 0.0, "done": False}

    def on_key(key: int) -> None:
        if not playback.on_key(key) and key == viewer.KEY_R:
            state["restart"] = True

    def update(wall_dt: float) -> str:
        if state["done"]:  # show the end of the episode briefly
            state["linger"] -= wall_dt
            if state["linger"] > 0:
                return env.status_text()
            state["restart"] = True
        if state["restart"]:
            state["obs"], _ = env.reset()
            state["policy"] = make_policy(env, policy_name)
            state.update(restart=False, done=False, budget=0.0)
        state["budget"] += playback.advance(wall_dt)
        while state["budget"] >= env.dt and not state["done"]:
            state["obs"], _, terminated, truncated, _ = env.step(
                state["policy"](state["obs"])
            )
            state["done"] = terminated or truncated
            state["budget"] -= env.dt
        if state["done"]:
            state["linger"] = END_PAUSE
        return env.status_text() + f"  x{playback.speed:g}"

    def camera(cam: mujoco.MjvCamera) -> None:
        cam.lookat[:] = (0.95, 0.25, 0.65)
        cam.distance, cam.azimuth, cam.elevation = 1.4, -60, -20

    viewer.run_passive(
        env.model, env.data, update, on_key=on_key, setup_camera=camera
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--task", default="foraging_ordered")
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument(
        "--policy", choices=["scripted", "still"], default="scripted"
    )
    parser.add_argument(
        "--reward",
        choices=REWARDS,
        default="none",
        help="the task: no reward, reward on touch, or on button press",
    )
    parser.add_argument(
        "--response-timeout",
        type=float,
        help="response window (s); default: the task config's, or 3 s for "
        "the button task (the scripted press needs about 1.5 s)",
    )
    parser.add_argument(
        "--rig", action="store_true", help="show the rig frame"
    )
    parser.add_argument("--camera", default="subject")
    parser.add_argument("--video", type=Path, help="render to this file")
    args = parser.parse_args()
    if not args.video:
        viewer.ensure_mjpython("virtual_tabletop.rl.demo")
    env = PresentationEnv(
        args.task,
        num_trials=args.trials,
        reward=args.reward,
        response_timeout=args.response_timeout
        or (3.0 if args.reward == "button" else None),
        include_rig=args.rig,
        camera=args.camera,
    )
    if args.video:
        print(render_video(env, args.policy, args.video))
        trials = env.trials
        touched = sum(t.touched_at is not None for t in trials)
        pressed = sum(t.pressed_at is not None for t in trials)
        rewarded = sum(t.rewarded for t in trials)
        print(
            f"{len(trials)} trials, {touched} touched, {pressed} pressed,"
            f" {rewarded} rewarded"
        )
    else:
        run_viewer(env, args.policy)


if __name__ == "__main__":
    main()
