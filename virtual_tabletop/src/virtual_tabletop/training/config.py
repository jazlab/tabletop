"""Training-run configuration and the env factory.

One YAML file per run in ``config/training/`` (see ``touch_right_arm.yaml``);
its keys are :class:`RunConfig`'s fields, grouped as ``env``, ``wrappers``
and ``ppo``.
"""

from dataclasses import asdict, dataclass, field
from pathlib import Path

import gymnasium as gym
import yaml

from virtual_tabletop import paths
from virtual_tabletop.assets import subject
from virtual_tabletop.assets.rl_model import load_rl_model
from virtual_tabletop.rl.env import PresentationEnv
from virtual_tabletop.training.wrappers import DeltaJointAction, ReachShaping

TRAINING_CONFIGS = paths.CONFIG_DIR / "training"
RUNS_DIR = paths.CACHE_DIR / "runs"


@dataclass
class EnvConfig:
    """:class:`~virtual_tabletop.rl.env.PresentationEnv` options.

    ``controlled_arms`` lists the arms whose 7 arm joints the agent
    controls (fingers stay straight; other arms hold at rest).
    ``released_arms`` overrides which arm(s) trials release.
    """

    task: str = "foraging_ordered"
    reward: str = "touch"
    num_trials: int = 5
    response_timeout: float | None = 3.0
    trial_order: str = "random"
    released_arms: list[str] | None = None
    controlled_arms: list[str] = field(default_factory=lambda: ["right"])
    control: str = "position"


@dataclass
class WrapperConfig:
    """Training wrappers: delta actions and reach shaping (0 = off)."""

    max_speed: float = 3.0
    shaping_scale: float = 5.0


@dataclass
class PPOConfig:
    """stable-baselines3 PPO hyperparameters (and run length)."""

    total_timesteps: int = 2_000_000
    n_envs: int = 8
    n_steps: int = 1024
    batch_size: int = 512
    n_epochs: int = 10
    learning_rate: float = 3e-4
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_range: float = 0.2
    ent_coef: float = 0.0
    net_arch: list[int] = field(default_factory=lambda: [128, 128])
    log_std_init: float = -0.5


@dataclass
class RunConfig:
    """A training run.

    Attributes:
        name: Run name (output directory under ``<cache>/runs/``).
        seed: Base seed (env ``i`` uses ``seed + i``).
        eval_every: Evaluate every this many env steps (total).
        eval_trials: Trials per evaluation.
    """

    name: str = "touch_right_arm"
    seed: int = 0
    eval_every: int = 100_000
    eval_trials: int = 100
    env: EnvConfig = field(default_factory=EnvConfig)
    wrappers: WrapperConfig = field(default_factory=WrapperConfig)
    ppo: PPOConfig = field(default_factory=PPOConfig)

    @classmethod
    def load(cls, path: Path) -> "RunConfig":
        raw = yaml.safe_load(Path(path).read_text()) or {}
        return cls(
            **{
                k: v
                for k, v in raw.items()
                if k not in ("env", "wrappers", "ppo")
            },
            env=EnvConfig(**raw.get("env", {})),
            wrappers=WrapperConfig(**raw.get("wrappers", {})),
            ppo=PPOConfig(**raw.get("ppo", {})),
        )

    def save(self, path: Path) -> None:
        path.write_text(yaml.safe_dump(asdict(self), sort_keys=False))

    @property
    def run_dir(self) -> Path:
        return RUNS_DIR / self.name


def controlled_joints(arms: list[str]) -> list[str]:
    """The arm joints (no fingers) of ``arms``, as env joint names
    (``right/elbow_flexion``)."""
    model = load_rl_model()
    return [
        name.removeprefix(subject.SUBJECT_PREFIX)
        for side in arms
        for name in subject.arm_joint_names(model, side)
    ]


def make_base_env(config: EnvConfig, seed: int | None = None, **kwargs):
    """The task env (no training wrappers), as configured."""
    return PresentationEnv(
        config.task,
        num_trials=config.num_trials,
        response_timeout=config.response_timeout,
        arms=config.released_arms,
        trial_order=config.trial_order,
        reward=config.reward,
        joints=controlled_joints(config.controlled_arms),
        control=config.control,
        seed=seed,
        **kwargs,
    )


def make_env(
    config: RunConfig, seed: int | None = None, shaping: bool = True, **kwargs
) -> gym.Env:
    """The training env: task env + delta actions (+ reach shaping)."""
    env = make_base_env(config.env, seed, **kwargs)
    if shaping and config.wrappers.shaping_scale > 0:
        env = ReachShaping(
            env, config.wrappers.shaping_scale, gamma=config.ppo.gamma
        )
    return DeltaJointAction(env, config.wrappers.max_speed)
