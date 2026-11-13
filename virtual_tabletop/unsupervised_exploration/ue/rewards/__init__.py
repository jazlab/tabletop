"""Intrinsic reward modules (the exploration objectives).

Every module sees only the observation (and action), never ``info``, and is
used by the same SAC learner (:mod:`ue.learn.sac`). Rewards are recomputed from
the module's current state for every sampled batch.

Feature groups (``feats``; see :class:`ue.sim.env.PlayEnv`): ``interaction``
(touch, contact map, wrench, surface distances, object pose and velocity;
no joint angles) is the default lens, following CEE-US, CSD, MUSIC and
CCGE: novelty over the whole body state makes the 23-joint arm flail
instead of engaging the object. ``all`` gives the naive full-state version.

Purely unsupervised (no task knowledge beyond the choice of features):

- :class:`NoReward`      — max-entropy SAC with zero reward;
- :class:`RND`           — random network distillation (Burda et al. 2019);
- :class:`Disagreement`  — ensemble forward-model variance (Pathak et al.
  2019; object-centric as in CEE-US);
- :class:`TouchSurprise` — ensemble disagreement on next *contacts*
  (haptics-based curiosity, Rajeswar et al. 2022, on state);
- :class:`APT`           — kNN particle state entropy (Liu & Abbeel 2021);
- :class:`ContactCount`  — contact-coverage counts over (object state,
  hand unit, object region) plus contact-set novelty and a reaching term
  toward rarely-touched hand units (CCGE, Liu et al. 2026);
- :class:`MUSIC`         — mutual information between hand state and object
  state (Zhao et al. 2021), optionally bootstrapped with contact counts;
- :class:`CSF`           — skills by contrastive successor features
  (Zheng et al. 2025), a METRA-style objective;
- :class:`DIAYN`         — discrete skills by discriminability (Eysenbach
  et al. 2019), plus an RND bonus (as in APS-style mixtures).

Supervised (task knowledge; for comparison):

- :class:`Shaped`        — dense reach + grasp + lift reward.

Mixtures: any module can be combined with others through :class:`Mix`.
"""

from .base import NoReward, RewardModule
from .coverage import APT, ContactCount, obj_state_bin
from .goals import GoalBabbling
from .mix import Mix
from .mutual_info import MUSIC
from .prediction import RND, Disagreement, TouchSurprise
from .regularizers import LiftBonus, Smoothness
from .shaped import Shaped
from .skills import CSF, DIAYN

MODULES = {
    "none": NoReward, "rnd": RND, "disagreement": Disagreement,
    "touch": TouchSurprise, "apt": APT, "contact": ContactCount,
    "music": MUSIC, "csf": CSF, "diayn": DIAYN, "shaped": Shaped,
    "goals": GoalBabbling, "smooth": Smoothness, "lift": LiftBonus,
}


def build(spec, method: str, kwargs: dict | None = None, mix=None):
    """``method`` names a module; ``mix`` is a list of (name, weight,
    kwargs) to combine instead."""
    if mix:
        parts = [(MODULES[n](spec, **(kw or {})), w) for n, w, kw in mix]
        return Mix(spec, parts)
    return MODULES[method](spec, **(kwargs or {}))


__all__ = ["MODULES", "build", "RewardModule", "obj_state_bin"]
