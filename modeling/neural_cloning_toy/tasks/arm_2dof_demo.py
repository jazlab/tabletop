"""Cursor control task."""

import numpy as np
from matplotlib import pyplot as plt
import sys; sys.path.append("..")
import arm_2dof as arm_2dof_lib

_N_PLOT = 5
_N_STEPS = 10


class Agent():

    def __init__(self, task):
        self._task = task

    def forward(self, state, goal):
        return self._task.get_target_action(state, goal)
    

def main():
    samples = [
        (2, 1.5, 1),
        (1, 3, 3.5),
        (0.5, 0.5, 0.5),
    ]
    fig, axes = plt.subplots(1, len(samples), figsize=(3 * len(samples), 3))
    for (d, segment_0, segment_1), ax in zip(samples, axes):
        px, py, theta_0, theta_1 = arm_2dof_lib.compute_vertex(d, segment_0, segment_1)
        ax.scatter(0, 0, c='r')
        ax.scatter(d, 0, c='b')
        ax.scatter(px, py, c='g')
        title = f"theta_0={theta_0:.2f}, theta_1={theta_1:.2f}"
        ax.set_title(title)
        ax.set_aspect('equal')
    fig.tight_layout()
    plt.show()



def main():
    task = arm_2dof_lib.Arm2DOF(
        segment_lengths=(1.5, 1),
        goal_theta=(0.5 * np.pi, 2 * np.pi),
        goal_theta_test=(0, 0.5 * np.pi),
        max_action_magnitude=0.2,
    )
    agent = Agent(task)
    for mode, test in zip(["train", "test"], [False, True]):
        states, goal, _, target_actions = task.rollout(agent, batch_size=_N_PLOT, test=test, n_steps=_N_STEPS)
        states_joint, states_end = task.pose_to_coords(states)
        goal_joint, goal_end = task.pose_to_coords(goal)
        states = states.detach().cpu().numpy()
        goal = goal.detach().cpu().numpy()
        target_actions = target_actions.detach().cpu().numpy()
        fig, axes = plt.subplots(1, _N_PLOT, figsize=(3 * _N_PLOT, 3))
        for i in range(_N_PLOT):
            # Plot segments
            alphas = np.linspace(0, 1, _N_STEPS + 2)[1:]
            for j in range(_N_STEPS + 1):
                axes[i].plot(*zip([0, 0], states_joint[j, i]), c='cyan', alpha=alphas[j])
                axes[i].plot(*zip(states_joint[j, i], states_end[j, i]), c='blue', alpha=alphas[j])
            for i_goal in range(2):
                axes[i].plot(*zip([0, 0], goal_joint[i, i_goal]), c='magenta', alpha=0.5)
                axes[i].plot(*zip(goal_joint[i, i_goal], goal_end[i, i_goal]), c='red', alpha=0.5)
            
            # Scatter hand points
            axes[i].scatter(states_end[-1, i, 0], states_end[-1, i, 1], c='blue', s=40)
            axes[i].scatter(goal_end[i, 0, 0], goal_end[i, 0, 1], c='red', s=20)

            axes[i].set_xlim(-2.5, 2.5)
            axes[i].set_ylim(-2.5, 2.5)
            axes[i].set_aspect('equal')
            axes[i].set_xticks([-2, -1, 0, 1, 2])
            axes[i].set_yticks([-2, -1, 0, 1, 2])
            axes[i].scatter(0, 0, c='k', marker='+', s=20)
        fig.tight_layout()
        fig.suptitle(f"{mode} samples")
    plt.show()


if __name__ == "__main__":
    main()
