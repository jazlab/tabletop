"""Train a PPO agent on an RL task (``vtt-rl-train``).

::

    uv run vtt-rl-train config/training/touch_right_arm.yaml [--timesteps N]

Runs ``n_envs`` envs in parallel processes (``SubprocVecEnv``), normalises
observations and rewards (``VecNormalize``), and trains stable-baselines3
PPO. Every ``eval_every`` steps the current (deterministic) policy is
evaluated on the unshaped task. Writes to ``<cache>/runs/<name>/``:

- ``config.yaml``: the run's configuration;
- ``progress.csv``: training trial success rate per PPO rollout;
- ``evals.csv``: evaluation success rate and time to respond over training;
- ``tb/``: TensorBoard logs (``uv run tensorboard --logdir <run>/tb``);
- ``model.zip``, ``obs_normalizer.pkl``: the final policy and the
  observation normalisation it expects;
- ``learning_curves.png``: success rate and time to respond over training;
- ``eval.json``, ``policy.mp4``: final evaluation (with baselines) and a
  video, as from ``vtt-rl-eval``.
"""

import argparse
import csv
import functools
import pickle
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from virtual_tabletop.training.config import RunConfig, make_env
from virtual_tabletop.training.evaluate import evaluate_run, run_episodes
from virtual_tabletop.training.plots import plot_run


@dataclass
class ObsNormalizer:
    """The observation normalisation a trained policy expects (a frozen
    copy of ``VecNormalize``'s running statistics)."""

    mean: np.ndarray
    var: np.ndarray
    clip: float
    epsilon: float = 1e-8

    def normalize_obs(self, obs: np.ndarray) -> np.ndarray:
        scaled = (obs - self.mean) / np.sqrt(self.var + self.epsilon)
        return np.clip(scaled, -self.clip, self.clip).astype(np.float32)

    @classmethod
    def from_vec_normalize(cls, venv) -> "ObsNormalizer":
        return cls(
            venv.obs_rms.mean.copy(),
            venv.obs_rms.var.copy(),
            venv.clip_obs,
            venv.epsilon,
        )


def _env_fn(config: RunConfig, seed: int):
    """Module-level env factory (picklable for subprocess envs)."""
    return make_env(config, seed)


def _callbacks(config: RunConfig, run_dir: Path):
    from stable_baselines3.common.callbacks import BaseCallback

    class TrialStats(BaseCallback):
        """Training success rate (rewarded / ended trials) per rollout."""

        def _on_training_start(self):
            self._file = open(run_dir / "progress.csv", "w", newline="")
            self._csv = csv.writer(self._file)
            self._csv.writerow(["timesteps", "trials", "success_rate"])
            self._ended = self._rewarded = 0

        def _on_step(self) -> bool:
            for info in self.locals["infos"]:
                if info.get("trial_ended"):
                    self._ended += 1
                    self._rewarded += bool(info.get("trial_rewarded"))
            return True

        def _on_rollout_end(self):
            if self._ended:
                rate = self._rewarded / self._ended
                self.logger.record("train/trial_success_rate", rate)
                self._csv.writerow([self.num_timesteps, self._ended, rate])
                self._file.flush()
            self._ended = self._rewarded = 0

        def _on_training_end(self):
            self._file.close()

    class PeriodicEval(BaseCallback):
        """Evaluate the deterministic policy on the unshaped task."""

        def _on_training_start(self):
            self._env = make_env(config, shaping=False)
            self._next = 0
            self._start = time.monotonic()
            self._file = open(run_dir / "evals.csv", "w", newline="")
            self._csv = csv.writer(self._file)
            self._csv.writerow(
                ["timesteps", "success_rate", "time_to_respond", "wall_s"]
            )

        def _evaluate(self):
            venv = self.model.get_env()

            def act(obs):
                action, _ = self.model.predict(
                    venv.normalize_obs(obs), deterministic=True
                )
                return action

            r = run_episodes(self._env, act, config.eval_trials)
            wall = time.monotonic() - self._start
            self.logger.record("eval/success_rate", r["success_rate"])
            if r["time_to_respond_mean"] is not None:
                self.logger.record(
                    "eval/time_to_respond", r["time_to_respond_mean"]
                )
            self._csv.writerow(
                [
                    self.num_timesteps,
                    r["success_rate"],
                    r["time_to_respond_mean"],
                    round(wall, 1),
                ]
            )
            self._file.flush()
            print(
                f"{self.num_timesteps:>9,d} steps  {wall / 60:5.1f} min  "
                f"eval success {100 * r['success_rate']:5.1f}%",
                flush=True,
            )

        def _on_step(self) -> bool:
            if self.num_timesteps >= self._next:
                self._evaluate()
                self._next += config.eval_every
            return True

        def _on_training_end(self):
            self._evaluate()
            self._file.close()

    return [TrialStats(), PeriodicEval()]


def train(config: RunConfig, run_dir: Path | None = None) -> Path:
    """Train PPO as configured; return the run directory."""
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CallbackList
    from stable_baselines3.common.vec_env import (
        SubprocVecEnv,
        VecMonitor,
        VecNormalize,
    )

    run_dir = run_dir or config.run_dir
    run_dir.mkdir(parents=True, exist_ok=True)
    config.save(run_dir / "config.yaml")
    ppo = config.ppo
    venv = SubprocVecEnv(
        [
            functools.partial(_env_fn, config, config.seed + i)
            for i in range(ppo.n_envs)
        ],
        start_method="spawn",
    )
    venv = VecNormalize(
        VecMonitor(venv), norm_obs=True, norm_reward=True, gamma=ppo.gamma
    )
    model = PPO(
        "MlpPolicy",
        venv,
        n_steps=ppo.n_steps,
        batch_size=ppo.batch_size,
        n_epochs=ppo.n_epochs,
        learning_rate=ppo.learning_rate,
        gamma=ppo.gamma,
        gae_lambda=ppo.gae_lambda,
        clip_range=ppo.clip_range,
        ent_coef=ppo.ent_coef,
        policy_kwargs=dict(
            net_arch=dict(pi=ppo.net_arch, vf=ppo.net_arch),
            log_std_init=ppo.log_std_init,
        ),
        tensorboard_log=str(run_dir / "tb"),
        seed=config.seed,
        device="cpu",
    )
    try:
        model.learn(
            ppo.total_timesteps,
            callback=CallbackList(_callbacks(config, run_dir)),
            tb_log_name="ppo",
        )
    finally:
        model.save(run_dir / "model.zip")
        with open(run_dir / "obs_normalizer.pkl", "wb") as f:
            pickle.dump(ObsNormalizer.from_vec_normalize(venv), f)
        venv.close()
    return run_dir


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("config", type=Path, help="run config YAML")
    parser.add_argument("--timesteps", type=int, help="override the length")
    parser.add_argument("--name", help="override the run name")
    parser.add_argument(
        "--no-final-eval",
        action="store_true",
        help="skip the final evaluation, baselines and video",
    )
    args = parser.parse_args()
    config = RunConfig.load(args.config)
    if args.timesteps:
        config.ppo.total_timesteps = args.timesteps
    if args.name:
        config.name = args.name
    run_dir = train(config)
    print(f"Run: {run_dir}")
    print(plot_run(run_dir))
    if not args.no_final_eval:
        results = evaluate_run(
            run_dir, 200, baselines=True, video=run_dir / "policy.mp4"
        )
        for name, r in results.items():
            print(f"{name:8s} success {100 * r['success_rate']:5.1f}%")


if __name__ == "__main__":
    main()
