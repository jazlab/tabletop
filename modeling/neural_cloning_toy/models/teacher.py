"""Teacher model."""

import torch
from models import mlp as mlp_lib


class Teacher(torch.nn.Module):
    """Teacher model."""

    def __init__(
        self,
        task,
        hidden_features,
        action_scale=1.0,
    ):
        """Create Teacher model."""
        super(Teacher, self).__init__()
        self._task = task
        self._action_scale = action_scale
        self._net = mlp_lib.MLP(
            in_features=task.observation_size,
            hidden_features=hidden_features,
            out_features=task.action_size,
        )

    def forward(self, state, goal):
        """Apply net to input."""
        observation = self._task.get_observation(state, goal)
        net_output = self._net(observation)
        action = self._action_scale * torch.nn.functional.tanh(net_output)
        return action

    def cache(self, write_dir):
        """Cache model outputs."""
        torch.save(self.state_dict(), write_dir / "model_snapshot.pth")

    @property
    def task(self):
        """Return task."""
        return self._task

    @property
    def net(self):
        """Return network."""
        return self._net
    
    @property
    def action_scale(self):
        """Return action scale."""
        return self._action_scale
