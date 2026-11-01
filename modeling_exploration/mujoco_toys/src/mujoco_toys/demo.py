"""Watch a toy environment with a simple policy (``toys-demo``).

::

    uv run toys-demo playpen --video playpen.mp4 [--policy random|wander]
    uv run toys-demo arm --video arm.mp4 --seconds 20 --camera top
    uv run toys-demo playpen --still playpen.png

Policies: ``random`` (uniform random actions every step: jittery);
``wander`` (smooth random actions: an Ornstein-Uhlenbeck process, which
moves the agent around the way exploration noise does). Options for the
environment: ``--spawn-rate`` (objects per second), ``--drop`` (spawn
height), ``--objects`` (at the start), ``--seed``.
"""

import argparse
from pathlib import Path

import numpy as np

from mujoco_toys.envs import ArmBlocksEnv, PlaypenEnv

ENVS = {"playpen": PlaypenEnv, "arm": ArmBlocksEnv}


class Wander:
    """Smooth random actions (Ornstein-Uhlenbeck noise, clipped)."""

    def __init__(self, n: int, rng, theta: float = 0.15, sigma: float = 0.35):
        self.x = np.zeros(n)
        self.rng, self.theta, self.sigma = rng, theta, sigma

    def __call__(self, obs) -> np.ndarray:
        self.x += -self.theta * self.x + self.sigma * self.rng.normal(
            size=self.x.size
        )
        return np.clip(self.x, -1, 1)


def make_policy(name: str, env, rng):
    if name == "random":
        return lambda obs: env.action_space.sample()
    if name == "wander":
        return Wander(env.action_space.shape[0], rng)
    raise ValueError(name)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("env", choices=sorted(ENVS))
    parser.add_argument(
        "--policy", choices=["random", "wander"], default="wander"
    )
    parser.add_argument("--video", type=Path)
    parser.add_argument("--still", type=Path)
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--camera", default="overview")
    parser.add_argument("--objects", type=int, default=None)
    parser.add_argument("--spawn-rate", type=float, default=0.0)
    parser.add_argument("--drop", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    kwargs = dict(
        spawn_rate=args.spawn_rate,
        drop_height=args.drop,
        episode_seconds=None,
        seed=args.seed,
        camera=args.camera,
    )
    if args.objects is not None:
        kwargs["n_objects"] = args.objects
    env = ENVS[args.env](**kwargs)
    obs, _ = env.reset(seed=args.seed)
    if args.still:
        from PIL import Image

        Image.fromarray(env.render_pixels(args.camera, (600, 800))).save(
            args.still
        )
        print(args.still)
    if not args.video:
        return
    import imageio.v2 as imageio

    policy = make_policy(args.policy, env, np.random.default_rng(args.seed))
    fps = round(1 / env.control_dt)
    touches = set()
    with imageio.get_writer(args.video, fps=fps, macro_block_size=8) as out:
        for _ in range(int(args.seconds / env.control_dt)):
            obs, _, _, _, info = env.step(policy(obs))
            touches.update(info["touching"])
            out.append_data(env.render_pixels(args.camera, (480, 640)))
    print(args.video)
    print(
        f"objects touched: {len(touches)}; "
        f"in the scene at the end: {info['active_objects']}"
    )


if __name__ == "__main__":
    main()
