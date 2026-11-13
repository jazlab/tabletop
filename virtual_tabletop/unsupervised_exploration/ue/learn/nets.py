"""MLPs for the actor, critics and reward models."""

import math

import torch
import torch.nn as nn
import torch.nn.functional as F


def mlp(inp, out, hidden=(256, 256), ln=True, act=nn.ELU):
    layers, d = [], inp
    for i, h in enumerate(hidden):
        layers.append(nn.Linear(d, h))
        if ln and i == 0:
            layers += [nn.LayerNorm(h), nn.Tanh()]
        else:
            layers.append(act())
        d = h
    layers.append(nn.Linear(d, out))
    return nn.Sequential(*layers)


class Actor(nn.Module):
    def __init__(self, inp, act_dim, hidden):
        super().__init__()
        self.net = mlp(inp, 2 * act_dim, hidden)

    def forward(self, x):
        mu, log_std = self.net(x).chunk(2, -1)
        log_std = torch.tanh(log_std)
        log_std = -5 + 0.5 * (2 - -5) * (log_std + 1)  # [-5, 2]
        return mu, log_std

    def sample(self, x, deterministic=False, eps=None):
        mu, log_std = self(x)
        if deterministic:
            return torch.tanh(mu), None
        std = log_std.exp()
        if eps is None:
            eps = torch.randn_like(mu)
        u = mu + std * eps
        a = torch.tanh(u)
        logp = (-0.5 * eps.pow(2) - log_std - 0.5 * math.log(2 * math.pi)
                ).sum(-1)
        logp = logp - (2 * (math.log(2) - u - F.softplus(-2 * u))).sum(-1)
        return a, logp


class Critic(nn.Module):
    def __init__(self, inp, act_dim, hidden, n=2):
        super().__init__()
        self.qs = nn.ModuleList([mlp(inp + act_dim, 1, hidden)
                                 for _ in range(n)])

    def forward(self, x, a):
        xa = torch.cat([x, a], -1)
        return torch.cat([q(xa) for q in self.qs], -1)
