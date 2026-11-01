# Train RL agents

Train a PPO agent on an RL task (`documentation/3_rl_tasks.md`), evaluate
it against scripted and idle baselines, and watch it. Runs natively on the
Mac (CPU), no Docker or ROS.

## Setup (once)

```bash
cd ~/Desktop/tabletop/tabletop_current/tabletop/virtual_tabletop
uv sync --all-extras        # includes the `rl` extra: stable-baselines3, tensorboard
```

## Train

```bash
uv run vtt-rl-train config/training/touch_right_arm.yaml
```

Prints the evaluation success rate every `eval_every` env steps, then a
final evaluation (200 trials) of the trained policy and the baselines.
Everything goes to `.cache/runs/<name>/` (see below). Options:
`--timesteps N` (override the length), `--name NAME` (another output
directory), `--no-final-eval`.

Watch training live:

```bash
uv run tensorboard --logdir .cache/runs/touch_right_arm/tb
```

(`train/trial_success_rate`, `eval/success_rate`, `eval/time_to_respond`,
`rollout/ep_rew_mean`, PPO losses.)

## Evaluate and watch

```bash
uv run vtt-rl-eval .cache/runs/touch_right_arm --baselines --video agent.mp4
```

Success rate and time to respond for the trained policy (and, with
`--baselines`, the scripted policy and an idle monkey) on the same 200
trials (fixed seed), plus breakdowns by object geometry and pose; writes
`eval.json` in the run.

## The first run's setup (`config/training/touch_right_arm.yaml`)

| Choice | Setting | Why |
|---|---|---|
| Task | `touch`: reward 1 when the released hand touches the object; the trial then ends | the simplest response |
| Arm | the right arm's 7 arm joints; fingers straight; left arm at rest; every trial releases the right arm | a small action space first |
| Arm locks | arms held at rest until released (as on the rig) | no hovering at the object before the window opens |
| Trials | random objects and poses (3 poses), 5 trials per episode, 3 s response window | variety; time to learn (the rig's 0.5 s later) |
| Actions | joint-angle steps in [−1, 1], 1 = 3 rad/s (`DeltaJointAction`) | smooth exploration around the current posture |
| Shaping | potential-based: `5 × (decrease in fingertip-to-object distance, m)` during the response window (`ReachShaping`) | random postures almost never touch; doesn't change the optimal policy |
| Physics | position servos, contact forces | the hand can't pass through the object |
| Learner | stable-baselines3 PPO, 8 parallel envs, observation and reward normalisation, 128×128 MLPs | robust default for continuous control; parallelises on CPU |

## The button task

Three button-task configs (1 M steps each, about 6 minutes):

| Config | Shaping | Result (200 trials) |
|---|---|---|
| `button_right_arm.yaml` | straight toward the button (`shaping_target: button`) | 16.5%: every drum, no cup (the hand presses against the cup's outer wall) |
| `button_via_opening_right_arm.yaml` | through the opening: entry 1 cm above the rim, 2 cm funnel (`button_via_opening`) | 0% (the hand stalls above the near rim) |
| `button_via_opening_v2_right_arm.yaml` | entry 4 cm above the rim, 2.5 cm funnel, drum straight (`approach_margin`, `funnel_radius`, `approach_default`) | 5% (drums at one pose) |

The scripted press-path policy scores 100%. Entering the cup is the open
problem; see the report (`gitignore/reports/rl_report.pdf`) for the
analysis and next steps (demonstrations, a cup curriculum, longer runs).

Other experiments: copy the YAML, change keys (every key is documented in
`src/virtual_tabletop/training/config.py`), give it a new `name`.

## Run outputs (`.cache/runs/<name>/`)

| File | What |
|---|---|
| `config.yaml` | the run's full configuration |
| `progress.csv` | training trial success rate per PPO update |
| `evals.csv` | evaluation success rate and time to respond during training |
| `tb/` | TensorBoard logs |
| `model.zip`, `obs_normalizer.pkl` | the final policy, and the observation normalisation it expects |
| `eval.json`, `policy.mp4` | the final evaluation (with baselines) and a video |

## Use a trained policy in Python

```python
from pathlib import Path

from virtual_tabletop.training.config import RunConfig, make_env
from virtual_tabletop.training.evaluate import run_episodes, trained_policy

run = ".cache/runs/touch_right_arm"
config = RunConfig.load(f"{run}/config.yaml")
env = make_env(config, shaping=False)
print(run_episodes(env, trained_policy(Path(run)), n_trials=50))
```
