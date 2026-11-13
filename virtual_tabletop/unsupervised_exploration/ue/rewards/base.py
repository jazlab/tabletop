"""Shared pieces of the reward modules: running normalisers, a batched
ensemble MLP, and the :class:`RewardModule` base class (zero reward)."""

import math

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from ..learn.nets import mlp  # noqa: F401  (re-exported)


class RunningNorm(nn.Module):
    """Running mean / std of a feature vector (updated from batches)."""

    def __init__(self, dim, clip=5.0):
        super().__init__()
        self.register_buffer("mean", torch.zeros(dim))
        self.register_buffer("var", torch.ones(dim))
        self.register_buffer("count", torch.tensor(1e-4))
        self.clip = clip

    @torch.no_grad()
    def update(self, x):
        bm, bv, n = x.mean(0), x.var(0, unbiased=False), x.shape[0]
        delta = bm - self.mean
        tot = self.count + n
        self.mean += delta * n / tot
        self.var = (self.var * self.count + bv * n
                    + delta ** 2 * self.count * n / tot) / tot
        self.count = tot

    def forward(self, x):
        return ((x - self.mean) / (self.var.sqrt() + 1e-6)).clamp(
            -self.clip, self.clip)


class EnsembleMLP(nn.Module):
    """``k`` independent MLPs evaluated together with batched matmuls.

    Input ``[N, in]`` (shared) or ``[k, N, in]``; output ``[k, N, out]``.
    """

    def __init__(self, k, inp, out, hidden=(256, 256)):
        super().__init__()
        dims = [inp, *hidden, out]
        self.W = nn.ParameterList()
        self.b = nn.ParameterList()
        for a, c in zip(dims[:-1], dims[1:]):
            w = torch.empty(k, a, c)
            for i in range(k):
                nn.init.kaiming_uniform_(w[i].T, a=math.sqrt(5))
            self.W.append(nn.Parameter(w))
            self.b.append(nn.Parameter(torch.zeros(k, 1, c)))
        self.k = k

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(0).expand(self.k, -1, -1)
        for i, (w, b) in enumerate(zip(self.W, self.b)):
            x = torch.baddbmm(b, x, w)
            if i < len(self.W) - 1:
                x = F.elu(x)
        return x


class ScalarNorm:
    """Running std of a reward stream, for scale-free mixing."""

    def __init__(self):
        self.var, self.count = 1.0, 1e-4

    def update(self, r):
        r = r.detach()
        bv, n = float(r.var()) if r.numel() > 1 else 0.0, r.numel()
        tot = self.count + n
        self.var = (self.var * self.count + bv * n) / tot
        self.count = min(tot, 1e6)  # keep adapting

    def __call__(self, r):
        return r / math.sqrt(self.var + 1e-8)


class RewardModule(nn.Module):
    """Base class: zero reward, no skills."""

    z_dim = 0
    supervised = False

    def __init__(self, spec, feats="interaction", normalize=True,
                 clip=5.0, lr=3e-4):
        super().__init__()
        self.spec = spec
        groups = [feats] if isinstance(feats, str) else feats
        self.feat_idx = torch.as_tensor(spec["group_index"](groups)
                                        if feats != "all" else
                                        np.arange(spec["obs_dim"]))
        self.feat_dim = len(self.feat_idx)
        self.do_norm, self.clip, self.lr = normalize, clip, lr
        self.rnorm = ScalarNorm()

    def f(self, obs):
        return obs[..., self.feat_idx]

    # Hooks -----------------------------------------------------------------

    def sample_z(self, n):
        return None

    def observe(self, obs, next_obs, done):
        """Per env step (numpy, all envs): update counts etc."""

    def raw_reward(self, obs, act, next_obs, z):
        return torch.zeros(obs.shape[0])

    #: Modules that read the env's stored ``info`` (next-state values,
    #: a dict of tensors keyed like ``ue.sim.vec.INFO_KEYS``) set this.
    uses_info = False

    def reward(self, obs, act, next_obs, z, info=None):
        with torch.no_grad():
            if self.uses_info:
                r = self.raw_reward(obs, act, next_obs, z, info=info)
            else:
                r = self.raw_reward(obs, act, next_obs, z)
        if self.do_norm:
            self.rnorm.update(r)
            r = self.rnorm(r).clamp(-self.clip, self.clip)
        return r

    def update(self, obs, act, next_obs, z):
        return {}

    def make_opt(self):
        params = [p for p in self.parameters() if p.requires_grad]
        self.opt = torch.optim.Adam(params, self.lr) if params else None


class NoReward(RewardModule):
    pass
