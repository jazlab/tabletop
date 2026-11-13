"""Competence-based: hand-object mutual information (MUSIC,
Zhao et al. 2021)."""

import math

import numpy as np
import torch

from ..learn.nets import mlp
from .base import RewardModule


class MUSIC(RewardModule):
    """Mutual information between hand state and object state.

    S_a = palm position + fingertips; S_s = object displacement, rotation
    since spawn, and wrench (the wrench carries the signal for fixed
    objects). As in MUSIC (Zhao et al. 2021) the reward is a per-sample
    MI lower bound, but with a separable critic T(a, s) = f(a) . g(s)
    (InfoNCE/CPC form), so every other sample in the batch is a negative
    at the cost of one matrix product:

        r = T(a', s') - log mean_j exp T(a', s_j)

    High when the object's state is predictable from the hand's, i.e. when
    they move together (or, for fixed objects, when pushes are consistent).
    """

    def __init__(self, spec, hidden=(256, 256), emb=32, clip_min=None,
                 s_parts=("obj_pos", "obj_drot", "wrench"), **kw):
        super().__init__(spec, **kw)
        lay = spec["layout"]
        # MUSIC clips its reward. Without a lower clip, the pointwise MI of
        # novel hand-object states (e.g. a first touch) is negative, and the
        # agent learns to avoid touching (seen in r2_music_contact_noclip).
        self.clip_min = clip_min

        def rng(*names):
            return torch.as_tensor(np.concatenate([
                np.arange(lay[n].start, lay[n].stop) for n in names]))

        self.a_idx = rng("palm_pos", "tips")
        # v3 uses ("obj_vel", "wrench"): zero unless something acts on the
        # block. Displacement/rotation since spawn carry tiny per-trial
        # settling offsets that identify the trial, and with negatives from
        # other trials the critic learns "hand states typical of this
        # trial" instead of hand-object coupling (MUSIC itself shuffles
        # within a trajectory, which avoids this).
        self.s_idx = rng(*s_parts)
        self.fa = mlp(len(self.a_idx), emb, hidden)
        self.gs = mlp(len(self.s_idx), emb, hidden)
        self.make_opt()

    def _scores(self, x):
        a = self.fa(x[:, self.a_idx])
        s = self.gs(x[:, self.s_idx])
        return s @ a.T  # [B(s), B(a)]

    def raw_reward(self, obs, act, next_obs, z):
        # Pointwise MI: contrast the true object state against the other
        # object states for the same hand state (column-wise). When object
        # states do not vary (nothing touched yet) this is exactly 0;
        # contrasting over hand states instead rewards an arbitrary hand
        # region (a bug in the first version).
        sc = self._scores(next_obs)
        r = sc.diag() - (torch.logsumexp(sc, 0) - math.log(sc.shape[0]))
        if self.clip_min is not None:
            r = r.clamp(min=self.clip_min)
        return r

    def update(self, obs, act, next_obs, z):
        sc = self._scores(next_obs)
        n = sc.shape[0]
        mi_s = (sc.diag() - (torch.logsumexp(sc, 0) - math.log(n))).mean()
        mi_a = (sc.diag() - (torch.logsumexp(sc, 1) - math.log(n))).mean()
        mi = 0.5 * (mi_s + mi_a)
        loss = -mi
        self.opt.zero_grad()
        loss.backward()
        self.opt.step()
        return {"music_mi": mi.item()}
