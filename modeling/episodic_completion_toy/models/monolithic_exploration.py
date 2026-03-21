"""MonolithicExploration model."""

import torch
from models import mlp as mlp_lib

_EPSILON = 1e-8


class MonolithicExploration(torch.nn.Module):
    """MonolithicExploration model."""

    def __init__(
        self,
        dataset,
        hidden_features,
        n_rnn_steps,
        tau,
    ):
        """Create MonolithicExploration model."""
        super(MonolithicExploration, self).__init__()
        self._dataset = dataset
        self._n_rnn_steps = n_rnn_steps
        self._tau = tau
        self._net = mlp_lib.MLP(
            in_features=1 + 2 * self._dataset.state_dim,
            hidden_features=hidden_features,
            out_features=2 * self._dataset.state_dim,
        )

    def forward(self, state, snr, duration):
        """Apply net to input."""
        states_per_step = [state]
        snr_per_step = [snr]
        for _ in range(self._n_rnn_steps):
            net_inputs = torch.cat([states_per_step[-1], snr_per_step[-1], duration[:, None]], dim=-1)
            net_outputs = self._net(net_inputs)
            state_term = net_outputs[:, :self._dataset.state_dim]
            snr_term = torch.sigmoid(net_outputs[:, self._dataset.state_dim:])
            tau_state = self._tau * (1 - snr)
            new_state = (1 - tau_state) * states_per_step[-1] + tau_state * state_term
            new_snr = (1 - tau_state) * snr_per_step[-1] + tau_state * snr_term
            states_per_step.append(new_state)
            snr_per_step.append(new_snr)

        states_per_step = torch.stack(states_per_step)
        snr_per_step = torch.stack(snr_per_step)
        return states_per_step, snr_per_step
    
    def loss(self):
        state, snr, duration = self._dataset.get_batch()
        state_noised = snr * state + (1 - snr) * torch.randn_like(state)
        model_state_per_step, model_snr_per_step = self.forward(state_noised, snr, duration)

        # Compute loss
        model_var_per_step = 1 - model_snr_per_step
        log_term = torch.log(model_var_per_step + _EPSILON)
        mse_term = (model_state_per_step - state[None]) ** 2 / (2 * model_var_per_step ** 2 + _EPSILON)
        loss_batch = log_term + mse_term
        loss = torch.mean(loss_batch)

        return loss

    def cache(self, write_dir):
        """Cache model outputs."""
        torch.save(self.state_dict(), write_dir / "model_snapshot.pth")

    @property
    def dataset(self):
        """Return dataset."""
        return self._dataset

    @property
    def net(self):
        """Return network."""
        return self._net
