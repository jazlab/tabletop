"""Skill discovery: :class:`CSF` (Zheng et al. 2025) and :class:`DIAYN`
(Eysenbach et al. 2019)."""

import math

import numpy as np
import torch
import torch.nn.functional as F

from ..learn.nets import mlp
from .base import RewardModule, RunningNorm
from .prediction import RND


class CSF(RewardModule):
    """Contrastive successor features (skills; METRA-like).

    phi: features -> R^d; skills z ~ uniform on the unit sphere, one per
    trial. phi is trained with InfoNCE on (phi(s') - phi(s)) . z against
    in-batch negative skills (scaled by ``xi``); the reward is the
    information-bottleneck term (phi(s') - phi(s)) . z. The policy and
    critics are z-conditioned (plain SAC rather than the vector successor
    critic: simpler at our budget).
    """

    def __init__(self, spec, z_dim=4, xi=5.0, hidden=(256, 256), **kw):
        kw.setdefault("normalize", False)
        super().__init__(spec, **kw)
        self.z_dim, self.xi = z_dim, xi
        self.norm = RunningNorm(self.feat_dim)
        self.phi = mlp(self.feat_dim, z_dim, hidden)
        self.make_opt()

    def sample_z(self, n):
        z = np.random.randn(n, self.z_dim)
        return (z / np.linalg.norm(z, axis=1, keepdims=True)).astype(
            np.float32)

    def _dphi(self, obs, next_obs):
        return (self.phi(self.norm(self.f(next_obs)))
                - self.phi(self.norm(self.f(obs))))

    def raw_reward(self, obs, act, next_obs, z):
        return (self._dphi(obs, next_obs) * z).sum(-1)

    def update(self, obs, act, next_obs, z):
        self.norm.update(self.f(next_obs))
        dphi = self._dphi(obs, next_obs)
        logits = dphi @ z.T  # [B, B]
        pos = logits.diag()
        neg = torch.logsumexp(logits, 1) - math.log(logits.shape[1])
        loss = -(pos - self.xi * neg).mean()
        self.opt.zero_grad()
        loss.backward()
        self.opt.step()
        return {"csf_pos": pos.mean().item(), "csf_neg": neg.mean().item()}


class DIAYN(RewardModule):
    """Discrete skills: r = log q(z|s') - log p(z), + RND bonus."""

    def __init__(self, spec, n_skills=16, hidden=(256, 256), w_rnd=0.5,
                 **kw):
        kw.setdefault("feats", "skill")
        super().__init__(spec, **kw)
        self.z_dim = n_skills
        self.norm = RunningNorm(self.feat_dim)
        self.disc = mlp(self.feat_dim, n_skills, hidden)
        self.make_opt()
        self.rnd = RND(spec, feats="interaction") \
            if w_rnd else None
        self.w_rnd = w_rnd

    def sample_z(self, n):
        return np.eye(self.z_dim, dtype=np.float32)[
            np.random.randint(0, self.z_dim, n)]

    def raw_reward(self, obs, act, next_obs, z):
        logq = F.log_softmax(self.disc(self.norm(self.f(next_obs))), -1)
        r = (logq * z).sum(-1) + math.log(self.z_dim)
        return r

    def reward(self, obs, act, next_obs, z, info=None):
        r = super().reward(obs, act, next_obs, z)
        if self.rnd is not None:
            r = r + self.w_rnd * self.rnd.reward(obs, act, next_obs, z)
        return r

    def update(self, obs, act, next_obs, z):
        self.norm.update(self.f(next_obs))
        logits = self.disc(self.norm(self.f(next_obs)))
        loss = F.cross_entropy(logits, z.argmax(-1))
        self.opt.zero_grad()
        loss.backward()
        self.opt.step()
        acc = (logits.argmax(-1) == z.argmax(-1)).float().mean().item()
        out = {"diayn_loss": loss.item(), "diayn_acc": acc}
        if self.rnd is not None:
            out.update(self.rnd.update(obs, act, next_obs, z))
        return out
