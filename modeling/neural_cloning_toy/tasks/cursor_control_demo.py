"""Cursor control task."""

import numpy as np
from matplotlib import pyplot as plt
import sys; sys.path.append("..")
import cursor_control as cursor_control_lib

_N_PLOT = 5
_N_STEPS = 10


class Agent():

    def __init__(self, task):
        self._task = task

    def forward(self, state, goal):
        return self._task.get_target_action(state, goal)


def main():
    task = cursor_control_lib.CursorControl(
        area_start=((-1, -1), (1, 1)),
        area_goal=((-1, -1), (1, 1)),
        area_start_test=((-1, -1), (0, 0)),
        area_goal_test=((-1, -1), (0, 0)),
        max_action_magnitude=0.2,
    )
    agent = Agent(task)
    for mode, test in zip(["train", "test"], [False, True]):
        states, goal, _, target_actions = task.rollout(agent, batch_size=_N_PLOT, test=test, n_steps=_N_STEPS)
        goal = goal.detach().cpu().numpy()
        states = states.detach().cpu().numpy()
        target_actions = target_actions.detach().cpu().numpy()
        fig, axes = plt.subplots(1, _N_PLOT, figsize=(3 * _N_PLOT, 3))
        for i in range(_N_PLOT):
            axes[i].scatter(states[:, i, 0], states[:, i, 1], c='blue', alpha=0.5, s=5)
            axes[i].quiver(
                states[:, i, 0], states[:, i, 1],
                target_actions[:, i, 0], target_actions[:, i, 1],
                angles='xy', scale_units='xy', scale=1.5, color='green', alpha=0.5
            )
            axes[i].scatter(states[0, i, 0], states[0, i, 1], c='blue', s=20)
            axes[i].scatter(goal[i, 0], goal[i, 1], c='red', s=20)
            axes[i].set_xlim(-1, 1)
            axes[i].set_ylim(-1, 1)
            axes[i].set_aspect('equal')
            axes[i].set_xticks([-1, 0, 1])
            axes[i].set_yticks([-1, 0, 1])
        fig.tight_layout()
        fig.suptitle(f"{mode} samples")
    plt.show()


if __name__ == "__main__":
    main()
