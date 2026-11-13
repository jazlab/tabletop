"""Rewards read from the env's stored ``info`` (version 1).

- :class:`Smoothness`: costs on action size and change ("effort" and
  "jerk", computed by the env on the executed action). These are the
  naturalness regularisers.
- :class:`LiftBonus`: sparse reward while a free block is lifted (method
  D; **supervised**: task knowledge about lifting).
"""

from .base import RewardModule


class Smoothness(RewardModule):
    """r = -(w_effort * mean(a^2) + w_jerk * mean((a_t - a_{t-1})^2))."""

    uses_info = True

    def __init__(self, spec, w_effort=0.1, w_jerk=0.2, **kw):
        kw.setdefault("normalize", False)
        super().__init__(spec, **kw)
        self.w_effort, self.w_jerk = w_effort, w_jerk

    def raw_reward(self, obs, act, next_obs, z, info=None):
        return -(self.w_effort * info["effort"] + self.w_jerk * info["jerk"])


class LiftBonus(RewardModule):
    """r = 1[lifted] * (0.5 + 10 * min(height gain, 5 cm)): 0.5-1.0 per step
    while a free block is lifted. "Lifted" is the env's definition: more
    than 2 cm above its spawn height, off the pedestal, touched by the
    hand."""

    uses_info = True
    supervised = True

    def __init__(self, spec, **kw):
        kw.setdefault("normalize", False)
        super().__init__(spec, **kw)

    def raw_reward(self, obs, act, next_obs, z, info=None):
        h = info["obj_height"].clamp(0.0, 0.05)
        return info["lifted"] * (0.5 + 10.0 * h)


__all__ = ["Smoothness", "LiftBonus"]
