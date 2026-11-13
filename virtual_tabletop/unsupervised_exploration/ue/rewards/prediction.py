"""Knowledge-based objectives: prediction error and ensemble uncertainty.

- :class:`RND` (Burda et al. 2019);
- :class:`Disagreement` (Pathak et al. 2019; object-centric as in CEE-US);
- :class:`TouchSurprise` (haptics-based curiosity, Rajeswar et al. 2022).
"""


import numpy as np
import torch
import torch.nn.functional as F

from ..learn.nets import mlp
from .base import EnsembleMLP, RewardModule, RunningNorm


class RND(RewardModule):
    def __init__(self, spec, out_dim=64, hidden=(256, 256), **kw):
        super().__init__(spec, **kw)
        self.norm = RunningNorm(self.feat_dim)
        self.target = mlp(self.feat_dim, out_dim, hidden, ln=False)
        for p in self.target.parameters():
            p.requires_grad_(False)
        self.pred = mlp(self.feat_dim, out_dim, hidden, ln=False)
        self.make_opt()

    def raw_reward(self, obs, act, next_obs, z):
        x = self.norm(self.f(next_obs))
        return (self.pred(x) - self.target(x)).pow(2).mean(-1)

    def update(self, obs, act, next_obs, z):
        self.norm.update(self.f(next_obs))
        x = self.norm(self.f(next_obs))
        loss = (self.pred(x) - self.target(x)).pow(2).mean()
        self.opt.zero_grad()
        loss.backward()
        self.opt.step()
        return {"rnd_loss": loss.item()}


class Disagreement(RewardModule):
    """Ensemble of forward models; reward = variance of their predictions.

    Each model predicts the change of the (normalised) feature vector from
    the full observation and the action.
    """

    def __init__(self, spec, k=5, hidden=(128, 128), **kw):
        super().__init__(spec, **kw)
        self.k = k
        self.in_norm = RunningNorm(spec["obs_dim"])
        self.out_norm = RunningNorm(self.feat_dim)
        self.models = EnsembleMLP(k, spec["obs_dim"] + spec["act_dim"],
                                  self.feat_dim, hidden)
        self.make_opt()

    def _preds(self, obs, act):
        x = torch.cat([self.in_norm(obs), act], -1)
        return self.models(x)

    def _target(self, obs, next_obs):
        return (self.out_norm(self.f(next_obs))
                - self.out_norm(self.f(obs)))

    def raw_reward(self, obs, act, next_obs, z):
        return self._preds(obs, act).var(0).mean(-1)

    def update(self, obs, act, next_obs, z):
        self.in_norm.update(obs)
        self.out_norm.update(self.f(next_obs))
        preds = self._preds(obs, act)
        tgt = self._target(obs, next_obs)
        # Bootstrap masks: each model trains on a random half.
        mask = (torch.rand(self.k, obs.shape[0], 1) < 0.5).float()
        loss = (((preds - tgt) ** 2) * mask).sum() / (
            mask.sum() * tgt.shape[-1])
        self.opt.zero_grad()
        loss.backward()
        self.opt.step()
        return {"dis_loss": loss.item()}


class TouchSurprise(RewardModule):
    """Ensemble disagreement on the *next contact pattern*.

    Five classifiers predict which (hand unit, object region) cells of the
    contact map and which touch patches will be active at t+1, from the
    full observation and action (proprioception + object state + touch).
    Reward: mean predictive variance of the Bernoulli outputs (epistemic
    uncertainty about touch), plus ``w_obj`` times object-motion
    disagreement (as Rajeswar et al. pair touch with dynamics so the agent
    does not settle into constant passive contact).
    """

    def __init__(self, spec, k=5, hidden=(128, 128), w_obj=0.5, **kw):
        super().__init__(spec, **kw)
        lay = spec["layout"]
        self.c_idx = torch.as_tensor(np.r_[
            np.arange(lay["contact_map"].start, lay["contact_map"].stop),
            np.arange(lay["touch"].start, lay["touch"].stop)])
        self.o_idx = torch.as_tensor(np.r_[
            np.arange(lay["obj_pos"].start, lay["obj_pos"].stop),
            np.arange(lay["obj_drot"].start, lay["obj_drot"].stop)])
        self.k, self.w_obj = k, w_obj
        self.in_norm = RunningNorm(spec["obs_dim"])
        nc, no = len(self.c_idx), len(self.o_idx)
        self.nc = nc
        self.models = EnsembleMLP(k, spec["obs_dim"] + spec["act_dim"],
                                  nc + no, hidden)
        self.make_opt()

    def _out(self, obs, act):
        x = torch.cat([self.in_norm(obs), act], -1)
        return self.models(x)

    def _labels(self, obs, next_obs):
        c = (next_obs[..., self.c_idx] > 0.2).float()
        do = (next_obs[..., self.o_idx] - obs[..., self.o_idx]) * 10
        return c, do

    def raw_reward(self, obs, act, next_obs, z):
        out = self._out(obs, act)
        p = torch.sigmoid(out[..., :self.nc])
        r = p.var(0).mean(-1) * 10
        if self.w_obj:
            r = r + self.w_obj * out[..., self.nc:].var(0).mean(-1)
        return r

    def update(self, obs, act, next_obs, z):
        self.in_norm.update(obs)
        out = self._out(obs, act)
        c, do = self._labels(obs, next_obs)
        mask = (torch.rand(self.k, obs.shape[0], 1) < 0.5).float()
        bce = F.binary_cross_entropy_with_logits(
            out[..., :self.nc], c.expand_as(out[..., :self.nc]),
            reduction="none")
        mse = (out[..., self.nc:] - do) ** 2
        loss = ((bce.mean(-1, keepdim=True) + mse.mean(-1, keepdim=True))
                * mask).sum() / mask.sum()
        self.opt.zero_grad()
        loss.backward()
        self.opt.step()
        return {"touch_loss": loss.item(),
                "contact_rate": c.mean().item()}
