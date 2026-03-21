"""Cursor control task."""

import numpy as np
from matplotlib import pyplot as plt
import sys; sys.path.append("..")
import cursor_control as cursor_control_lib

_N_PLOT = 5
_N_STEPS = 20


def main():
    environment = cursor_control_lib.CursorControlEnvironment(
        batch_size=_N_PLOT,
        feedback_window=0.4,
        bounds=(-1, 1),
    )
    agent = cursor_control_lib.ExplorationAgent(
        environment=environment,
        scale=0.5,
        smooth_tau=0.85,
        seq_length=_N_STEPS,
    )
    action_sequence = agent.action_sequence()
    observations = [environment.reset()]
    for action in action_sequence:
        observations.append(environment.step(action))
    observations = np.array(observations)
    # obs_object = observations[:, :, :2]
    # obs_hand = observations[:, :, 2:4]
    # obs_feedback = observations[:, :, 4]
    # obs_action = observations[:, :, 5:]
    obs_hand = observations[:, :, 0:2]
    obs_action = observations[:, :, 2:4]

    # Turn feedback into color
    # feedback_color = np.where(obs_feedback > 0, "blue", "cyan")

    # Plot
    fig, axes = plt.subplots(1, _N_PLOT, figsize=(3 * _N_PLOT, 3))
    for i in range(_N_PLOT):
        axes[i].set_title(f"Sample {i}")
        # axes[i].scatter(obs_object[:, i, 0], obs_object[:, i, 1], c='red', alpha=1.0, s=50)
        # axes[i].scatter(obs_hand[:, i, 0], obs_hand[:, i, 1], c=feedback_color[:, i], alpha=1.0, s=50)
        axes[i].scatter(obs_hand[:, i, 0], obs_hand[:, i, 1], c="blue", alpha=1.0, s=50)
        axes[i].quiver(
            obs_hand[:, i, 0], obs_hand[:, i, 1],
            obs_action[:, i, 0], obs_action[:, i, 1],
            angles='xy', scale_units='xy', scale=1.5, color='green', alpha=1.0,
        )
        axes[i].set_xlim(-1, 1)
        axes[i].set_ylim(-1, 1)
        axes[i].set_aspect('equal')
        axes[i].set_xticks([-1, 0, 1])
        axes[i].set_yticks([-1, 0, 1])
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
