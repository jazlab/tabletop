"""Cursor control task."""

import numpy as np
from matplotlib import pyplot as plt
import sys; sys.path.append("..")
import cursor_control as cursor_control_lib

_N_PLOT = 8
_N_STEPS = 1


def _plot_observation(ax, agent, observation, action=None):
    components = agent.environment.observation_to_components(observation)
    hand = components["hand"]
    ax.scatter(hand[0], hand[1], c="blue", alpha=1.0, s=50)
    if action is not None:
        ax.quiver(
            hand[0], hand[1],
            action[0], action[1],
            angles='xy', scale_units='xy', scale=1.5, color='green', alpha=1.0,
        )


def main():
    environment = cursor_control_lib.CursorControlEnvironment(
        batch_size=_N_PLOT,
        feedback_window=0.4,
        bounds=(-1, 1),
    )
    agent = cursor_control_lib.ExplorationAgent(
        environment=environment,
        scale=1.0,
        smooth_tau=0.9,
        seq_length=_N_STEPS,
        burn_in_steps=0,
    )
    obs_start, obs_end, action_start, duration = agent()

    # Plot
    fig, axes = plt.subplots(1, _N_PLOT, figsize=(3 * _N_PLOT, 3))
    for i in range(_N_PLOT):
        axes[i].set_title(f"Sample {i}, duration {duration[i]}")
        _plot_observation(axes[i], agent, obs_start[i], action=action_start[i])
        _plot_observation(axes[i], agent, obs_end[i])
        axes[i].set_xlim(-1.05, 1.05)
        axes[i].set_ylim(-1.05, 1.05)
        axes[i].set_aspect('equal')
        axes[i].set_xticks([-1, 0, 1])
        axes[i].set_yticks([-1, 0, 1])
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
