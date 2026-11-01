# Virtual TableTop documentation

| Doc | What |
|---|---|
| [architecture.md](architecture.md) | How the package is organised: the two models, replay vs RL, design tools, generated files |
| [configuration.md](configuration.md) | Every file in `config/`, each key and who reads it |
| [1_run_foraging_in_sim_and_view_in_rviz.md](1_run_foraging_in_sim_and_view_in_rviz.md) | Run the rig's foraging task in the Docker ROS sim on a Mac and watch it in RViz |
| [2_record_n_trials_and_replay_in_mujoco.md](2_record_n_trials_and_replay_in_mujoco.md) | Record N sim trials to a ROS bag, then replay it in MuJoCo |
| [3_rl_tasks.md](3_rl_tasks.md) | The RL tasks (none / touch / button): the demo and the Python env |
| [4_explore_the_monkey_arm.md](4_explore_the_monkey_arm.md) | Joint-control design tools: arm explorer and arm report |
| [5_train_rl_agents.md](5_train_rl_agents.md) | Train PPO agents on the RL tasks, evaluate them against baselines, watch them |

How-tos 1–2 need Docker and the ROS stack; 3–5 run natively (no ROS).
