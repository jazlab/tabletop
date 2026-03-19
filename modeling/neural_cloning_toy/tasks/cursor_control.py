"""Cursor control task."""

import numpy as np
from tasks import base
import torch


class Sampler2D():
    """Samples from difference of uniform distributions."""

    def __init__(self, area_keep, area_discard=None):
        self._area_keep = area_keep
        self._area_discard = area_discard

    def _should_discard(self, sample):
        if self._area_discard is None:
            return False
        should_discard = (
            self._area_discard[0][0] <= sample[0] <= self._area_discard[1][0] and
            self._area_discard[0][1] <= sample[1] <= self._area_discard[1][1]
        )
        return should_discard

    def sample(self):
        sample = np.random.uniform(self._area_keep[0], self._area_keep[1])
        if self._should_discard(sample):
            return self.sample()
        return sample
    
    def __call__(self, batch_size):
        batch = np.array([self.sample() for _ in range(batch_size)])
        batch_torch = torch.from_numpy(batch).float()
        return batch_torch


class CursorControl(base.BaseTask):
    """Cursor control task."""

    def __init__(self,
                 area_start=((-1, -1), (1, 1)),
                 area_goal=((-1, -1), (1, 1)),
                 area_start_test=((-1, -1), (0, 0)),
                 area_goal_test=((-1, -1), (0, 0)),
                 max_action_magnitude: float=0.1):
        self._sampler_start_train = Sampler2D(area_keep=area_start, area_discard=area_start_test)
        self._sampler_goal_train = Sampler2D(area_keep=area_goal, area_discard=area_goal_test)
        self._sampler_start_test = Sampler2D(area_keep=area_start_test)
        self._sampler_goal_test = Sampler2D(area_keep=area_goal_test)
        self._max_action_magnitude = max_action_magnitude

    def get_target_action(self, state: torch.Tensor, goal: torch.Tensor) -> torch.Tensor:
        """Get the target action."""
        target_action = goal - state
        target_action_norm = torch.norm(target_action, dim=-1, keepdim=True)
        target_norm = torch.min(target_action_norm, torch.tensor(self._max_action_magnitude))
        target_action_normalized = target_action / torch.clamp(target_action_norm, min=1e-8)
        target_action = target_action_normalized * target_norm
        return target_action

    def step(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        """Step the environment."""
        new_state = state + action
        return new_state

    def get_batch(self, batch_size: int, test: bool):
        """Get a batch of data."""
        if test:
            batch_start = self._sampler_start_test(batch_size=batch_size)
            batch_goal = self._sampler_goal_test(batch_size=batch_size)
        else:
            batch_start = self._sampler_start_train(batch_size=batch_size)
            batch_goal = self._sampler_goal_train(batch_size=batch_size)
        return batch_start, batch_goal
    
    def get_observation(self, state, goal):
        return torch.cat([state, goal], dim=-1)
    
    @property
    def observation_size(self) -> int:
        """Get the size of the observation."""
        return 4

    @property
    def action_size(self) -> int:
        """Get the size of the action space."""
        return 2
