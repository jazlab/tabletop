"""Dataset class for models."""

import torch


class Dataset:

    def __init__(
        self,
        agent,
        snr_logit_range=(-10, 10),
    ):
        """Dataset constructor."""
        self._agent = agent
        self._snr_logit_range = snr_logit_range

    def _sample_snr(self, shape):
        """Sample uniform SNR from the specified range."""
        range_0, range_1 = self._snr_logit_range
        snr_logit = torch.rand(shape, dtype=torch.float32) * (range_1 - range_0) + range_0
        snr = torch.sigmoid(snr_logit)
        return snr

    def get_batch(self):
        """Get a batch of data."""
        obs_start, obs_end, action_start, duration = self._agent()
        # Shape (batch_size, observation_dim)
        obs_start = torch.tensor(obs_start, dtype=torch.float32)
        # Shape (batch_size, observation_dim)
        obs_end = torch.tensor(obs_end, dtype=torch.float32)
        # Shape (batch_size, action_dim)
        action_start = torch.tensor(action_start, dtype=torch.float32)
        # Shape (batch_size,)
        duration = torch.tensor(duration, dtype=torch.float32)

        # Combine start and end state
        state = torch.cat([obs_start, action_start, obs_end], dim=1)

        # Sample uniform SNR from the specified range
        snr = self._sample_snr(shape=state.shape)

        return state, snr, duration
    
    @property
    def agent(self):
        return self._agent
    
    @property
    def environment(self):
        return self._agent.environment
    
    @property
    def state_dim(self):
        return 2 * self._agent.observation_dim + self._agent.action_dim
