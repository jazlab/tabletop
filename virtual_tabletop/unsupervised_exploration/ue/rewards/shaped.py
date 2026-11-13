"""Supervised reference reward (task knowledge)."""


import torch

from .base import RewardModule


class Shaped(RewardModule):
    """Supervised: dense reach + contact + grasp + lift reward.

    r = (1 - tanh(10 d))           d: mean palm/thumb/index surface distance
        + 0.5 * thumb-and-finger contact on the object
        + 0.5 * opposing-region contact (any two units on opposite faces)
        + 3 * clip(height gain / 5 cm, 0, 1)   for free objects
    """

    supervised = True

    def __init__(self, spec, **kw):
        kw.setdefault("normalize", False)
        super().__init__(spec, **kw)
        self.lay = spec["layout"]

    def raw_reward(self, obs, act, next_obs, z):
        x, L = next_obs, self.lay
        d = x[:, L["surf_dist"]][:, :3].mean(-1) / 10
        cm = (x[:, L["contact_map"]].reshape(-1, 6, 6) > 0.2).float()
        thumb = cm[:, 1].amax(-1)
        fingers = cm[:, 2:].amax((-1, -2))
        palm = cm[:, 0].amax(-1)
        regions = cm.amax(1)  # [B, 6] any unit on region
        opposed = (regions[:, 0::2] * regions[:, 1::2]).amax(-1)
        h = (x[:, L["obj_pos"].start + 2] / 5).clamp(0, 0.05) / 0.05
        free = 1 - x[:, L["fixed"].start]
        return ((1 - torch.tanh(10 * d))
                + 0.5 * thumb * torch.maximum(fingers, palm)
                + 0.5 * opposed + 3 * h * free)
