"""Arm 2 DOF task."""

import numpy as np
from tasks import base
import torch


def compute_vertex(d, segment_0, segment_1):
    """
    Returns third vertex (px, py) of the triangle with vertices (0, 0), (0, d)
    and side lengths segment_0 and segment_1.

    Returns None if no such triangle exists.
    """
    # Sanity checks
    if d > segment_0 + segment_1:
        return None
    if d < abs(segment_0 - segment_1):
        return None
    if d == 0:
        return None

    # Compute vertex
    px = (segment_0**2 - segment_1**2 + d**2) / (2 * d)
    py = np.sqrt(segment_0**2 - px**2)

    # Compute thetas
    theta_0 = np.arctan2(py, px)
    theta_1 = np.arctan2(d - px, py) + (1.5 * np.pi - theta_0)

    return (px, py, theta_0, theta_1)


class Arm2DOF(base.BaseTask):
    """Arm 2 DOF task."""

    def __init__(self,
                 segment_lengths=(1, 1),
                 goal_theta=(0.5 * np.pi, 2 * np.pi),
                 goal_theta_test=(0, 0.5 * np.pi),
                 max_action_magnitude: float=0.1):
        self._segment_lengths = segment_lengths
        self._goal_theta = goal_theta
        self._goal_theta_test = goal_theta_test
        self._max_action_magnitude = max_action_magnitude

    def _inverse_kinematics(self, goal_dist, goal_theta):
        """Compute poses for goal."""
        poses = []
        for d, theta in zip(goal_dist, goal_theta):
            pose_oriented_0 = compute_vertex(d, self._segment_lengths[0], self._segment_lengths[1])
            if pose_oriented_0 is None:
                raise ValueError("Invalid pose")
            _, _, theta_0_0, theta_0_1 = pose_oriented_0
            thetas_oriented_0 = np.array([theta_0_0, theta_0_1])
            thetas_oriented_1 = np.array([-1 * theta_0_0, -1 * theta_0_1])
            thetas_0 = thetas_oriented_0 + np.array([theta, 0])
            thetas_1 = thetas_oriented_1 + np.array([theta, 0])
            poses.append([thetas_0, thetas_1])
        return np.array(poses)
    
    def _theta_diff(self, theta_0, theta_1):
        """Compute minimum angular delta to get from theta_0 to theta_1"""
        diff = (theta_1 - theta_0) % (2 * np.pi)
        diff_negative = (diff > np.pi).float()
        diff_signed = diff_negative * (diff - 2 * np.pi) + (1 - diff_negative) * diff
        return diff_signed

    def get_target_action(self, state: torch.Tensor, goal: torch.Tensor) -> torch.Tensor:
        """Get the target action."""
        goal_0 = goal[:, 0]
        goal_1 = goal[:, 1]
        goal_0_diff = self._theta_diff(state, goal_0)
        goal_1_diff = self._theta_diff(state, goal_1)
        goal_0_dist = torch.sum(torch.abs(goal_0_diff), dim=-1)
        goal_1_dist = torch.sum(torch.abs(goal_1_diff), dim=-1)
        mask = (goal_0_dist < goal_1_dist).float()
        target_action = mask[:, None] * goal_0_diff + (1 - mask)[:, None] * goal_1_diff
        target_action = torch.clamp(target_action, min=-self._max_action_magnitude, max=self._max_action_magnitude)
        return target_action

    def step(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        """Step the environment."""
        new_state = state + action
        return new_state

    def get_batch(self, batch_size: int, test: bool):
        """Get a batch of data."""

        # Sample goal
        max_reach = self._segment_lengths[0] + self._segment_lengths[1]
        min_reach = abs(self._segment_lengths[0] - self._segment_lengths[1])
        goal_dist = np.random.uniform(low=min_reach, high=max_reach, size=(batch_size,))
        if test:
            goal_theta = np.random.uniform(low=self._goal_theta[0], high=self._goal_theta[1], size=(batch_size,))
        else:
            goal_theta = np.random.uniform(low=self._goal_theta_test[0], high=self._goal_theta_test[1], size=(batch_size,))
        goal_poses = self._inverse_kinematics(goal_dist, goal_theta)
        goal_poses = torch.from_numpy(goal_poses).float()

        # Sample start
        start_pose = np.random.uniform(low=0, high=max_reach, size=(batch_size, 2))
        start_pose = torch.from_numpy(start_pose).float()

        return start_pose, goal_poses

    def pose_to_coords(self, pose):
        """Convert a batched pose (theta_0, theta_1) to Cartesian coordinates (x, y)."""
        joint_position = self._segment_lengths[0] * torch.stack([torch.cos(pose[..., 0]), torch.sin(pose[..., 0])], dim=-1)
        absolute_theta_1 = pose[..., 1] + pose[..., 0]
        coords_1 = self._segment_lengths[1] * torch.stack([torch.cos(absolute_theta_1), torch.sin(absolute_theta_1)], dim=-1)
        end_position = joint_position + coords_1
        return joint_position, end_position
    
    def get_observation(self, state, goal):
        _, goal_end_position = self.pose_to_coords(goal)
        observation = torch.cat([state, goal_end_position[:, 0]], dim=-1)
        return observation
    
    @property
    def observation_size(self):
        return 4

    @property
    def action_size(self):
        return 2
