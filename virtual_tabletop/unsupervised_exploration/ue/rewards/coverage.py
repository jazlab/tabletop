"""Data-based objectives: state entropy and contact coverage.

- :class:`APT` (Liu & Abbeel 2021): kNN particle entropy;
- :class:`ContactCount` (CCGE-lite, after Liu et al. 2026).
"""

import math

import numpy as np
import torch

from .base import RewardModule, RunningNorm


class APT(RewardModule):
    """kNN particle entropy: r = log(1 + mean dist to k nearest in batch)."""

    def __init__(self, spec, k=12, **kw):
        super().__init__(spec, **kw)
        self.k = k
        self.norm = RunningNorm(self.feat_dim)
        self._mem = None

    def raw_reward(self, obs, act, next_obs, z):
        x = self.norm(self.f(next_obs))
        mem = self._mem if self._mem is not None else x
        d = torch.cdist(x, mem)
        kd = d.topk(self.k + 1, largest=False).values[:, 1:]
        return torch.log1p(kd.mean(-1))

    def update(self, obs, act, next_obs, z):
        self.norm.update(self.f(next_obs))
        self._mem = self.norm(self.f(next_obs)).detach()
        return {}


def obj_state_bin(obs, lay):
    """Coarse object state: fixed (2) x height above spawn (3) x up-face
    (6: which object axis points up, and its sign)."""
    fixed = obs[:, lay["fixed"].start] > 0.5
    h = obs[:, lay["obj_pos"].start + 2] / 5.0  # metres
    hb = np.digitize(h, [0.01, 0.04])
    rot = obs[:, lay["obj_rot"]]  # R[:,0], R[:,1] (row-major pairs)
    c0, c1 = rot[:, :3], rot[:, 3:6]
    c2 = np.cross(c0, c1)
    up = np.stack([c0[:, 2], c1[:, 2], c2[:, 2]], 1)  # world z in obj axes
    ax = np.abs(up).argmax(1)
    sg = up[np.arange(len(up)), ax] > 0
    face = 2 * ax + sg
    return (fixed.astype(int) * 3 + hb) * 6 + face  # in [0, 36)


class ContactCount(RewardModule):
    """Contact-coverage counts (CCGE-lite).

    Cells: (object-state bin, hand unit, object region), 36 x 6 x 6. Each
    env step adds one count to every active cell (contact-map entry > 0.2),
    and one to the step's *contact set* (which cells are active together,
    hashed, per object-state bin).

    r = mean over active cells of 1/sqrt(C + 1)            (coverage)
        + w_set / sqrt(C_set + 1)  if any contact          (new combos)
        + w_reach * mean_u exp(-d_u / 2 cm) * g_u           (reach)

    where d_u is hand unit u's distance to the object surface and g_u the
    mean coverage bonus of u's cells in the current object state: hands
    are drawn toward the object with the hand units that have touched it
    least.
    """

    def __init__(self, spec, w_set=0.5, w_reach=0.1, **kw):
        super().__init__(spec, **kw)
        self.lay = spec["layout"]
        self.counts = np.zeros((36, 6, 6))
        self.set_counts = {}
        self.w_set, self.w_reach = w_set, w_reach

    def _cells(self, obs):
        cm = obs[:, self.lay["contact_map"]].reshape(-1, 6, 6) > 0.2
        return obj_state_bin(obs, self.lay), cm

    def _set_keys(self, s, cm):
        bits = np.packbits(cm.reshape(len(cm), -1), axis=1)
        return [(int(si), b.tobytes()) for si, b in zip(s, bits)]

    def observe(self, obs, next_obs, done):
        s, cm = self._cells(next_obs)
        for i in range(len(s)):
            self.counts[s[i]] += cm[i]
            if cm[i].any():
                key = self._set_keys(s[i:i + 1], cm[i:i + 1])[0]
                self.set_counts[key] = self.set_counts.get(key, 0) + 1

    def raw_reward(self, obs, act, next_obs, z):
        x = next_obs.numpy()
        s, cm = self._cells(x)
        g = 1.0 / np.sqrt(self.counts[s] + 1.0)  # [B, 6, 6]
        n_on = cm.sum((1, 2))
        cov = (g * cm).sum((1, 2)) / np.maximum(n_on, 1)
        r = cov.copy()
        if self.w_set:
            keys = self._set_keys(s, cm)
            setb = np.array([1.0 / math.sqrt(self.set_counts.get(k, 0) + 1)
                             for k in keys])
            r += self.w_set * setb * (n_on > 0)
        if self.w_reach:
            d = x[:, self.lay["surf_dist"]] / 10.0  # metres, [B, 6]
            r += self.w_reach * (np.exp(-d / 0.02) * g.mean(2)).mean(1)
        return torch.as_tensor(r, dtype=torch.float32)

    def stats(self):
        return {"cells_visited": int((self.counts > 0).sum()),
                "contact_sets": len(self.set_counts)}
