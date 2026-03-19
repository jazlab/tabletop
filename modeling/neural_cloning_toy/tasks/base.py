"""Base task."""

import abc
import torch
import numpy as np


class BaseTask(abc.ABC):
    """Base task."""

    @abc.abstractmethod
    def get_target_action(self, state: torch.Tensor, goal: torch.Tensor) -> torch.Tensor:
        """Get the target action."""
        pass

    @abc.abstractmethod
    def step(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        """Step the environment."""
        pass

    @abc.abstractmethod
    def get_batch(self, batch_size: int, test: bool):
        """Get a batch of data."""
        pass

    @abc.abstractmethod
    def get_observation(self, state: torch.Tensor, goal: torch.Tensor) -> torch.Tensor:
        """Convert state and goal to observation."""
        pass

    @abc.abstractmethod
    def observation_size(self) -> int:
        """Get the size of the observation."""
        pass

    @abc.abstractmethod
    def action_size(self) -> int:
        """Get the size of the action space."""
        pass

    def rollout(self, agent, state: torch.Tensor, goal: torch.Tensor, n_steps: int) -> torch.Tensor:
        """Roll out predictor model."""
        states = [state]
        actions = [agent.forward(state, goal)]
        target_actions = [self.get_target_action(state, goal)]
        for _ in range(n_steps):
            states.append(self.step(states[-1], actions[-1]))
            actions.append(agent.forward(states[-1], goal))
            target_actions.append(self.get_target_action(states[-1], goal))
        states = torch.stack(states, dim=0)
        actions = torch.stack(actions, dim=0)
        target_actions = torch.stack(target_actions, dim=0)
        return states, goal, actions, target_actions