"""Weighted mixtures of reward modules."""


import inspect

import torch.nn as nn

from .base import RewardModule


class Mix(RewardModule):
    """Weighted sum of normalised rewards of several modules."""

    def __init__(self, spec, parts, **kw):
        super().__init__(spec, normalize=False)
        self.parts = nn.ModuleList([p for p, _ in parts])
        self.weights = [w for _, w in parts]
        self.z_dim = max(p.z_dim for p in self.parts)
        self.supervised = any(p.supervised for p in self.parts)

    def sample_z(self, n, obs=None):
        for p in self.parts:
            if p.z_dim:
                if "obs" in inspect.signature(p.sample_z).parameters:
                    return p.sample_z(n, obs=obs)
                return p.sample_z(n)
        return None

    def observe(self, obs, next_obs, done, z=None):
        for p in self.parts:
            if "z" in inspect.signature(p.observe).parameters:
                p.observe(obs, next_obs, done, z=z)
            else:
                p.observe(obs, next_obs, done)

    def relabel(self, buf, t, e, z):
        for p in self.parts:
            if hasattr(p, "relabel"):
                return p.relabel(buf, t, e, z)
        return z

    @property
    def goal_module(self):
        for p in self.parts:
            if hasattr(p, "MODULES"):
                return p
        return None

    def reward(self, obs, act, next_obs, z, info=None):
        return sum(w * p.reward(obs, act, next_obs, z, info=info)
                   for p, w in zip(self.parts, self.weights))

    def reward_terms(self, obs, act, next_obs, z, info=None):
        """Each part's weighted reward (for logging)."""
        return {type(p).__name__: w * p.reward(obs, act, next_obs, z,
                                               info=info)
                for p, w in zip(self.parts, self.weights)}

    def update(self, obs, act, next_obs, z):
        out = {}
        for p in self.parts:
            out.update(p.update(obs, act, next_obs, z))
        return out

    def stats(self):
        out = {}
        for p in self.parts:
            if hasattr(p, "stats"):
                out.update(p.stats())
        return out
