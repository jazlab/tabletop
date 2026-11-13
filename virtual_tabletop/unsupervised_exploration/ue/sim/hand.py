"""Hand versions: joint-range overrides and finger coupling.

The arm file (``monkey_arm/right_arm_fingers_v0.xml``) lets every finger
segment bend backwards (hyperextend) by 60°, and the thumb segments by
30–60°. Learned policies use these postures heavily: version 0 RND held
finger joints past −20° in about 26% of finger-joint steps, which looks
double-jointed. Version 1 restricts extension to macaque-like values,
only in this project's model; the shared arm file and the PPO environment
are unchanged.

Sign convention, measured on the model: for every ``*_flexion`` joint,
positive angles curl the segment toward the palm (flexion) and negative
angles bend it backwards (hyperextension). For ``thumb_d0_abduction``,
positive moves the thumb toward the palm side (opposition).

Flexion limits are kept as in the arm file: digits 90° at the knuckle (d0,
MCP) and 60° at the middle and end joints (d1, d2). Only the extension
side is changed.

Coupling (optional, ``couple_dip``): in real fingers the end joint (DIP,
d2) flexes together with the middle joint (PIP, d1) through the tendons.
With coupling, the d2 servo target of the index, middle, ring and pinky
follows ``ratio * d1 target``. Those four d2 joints are then not separate
actions, so there are 19 actions instead of 23. They are still physical
joints, and a d2 can deviate from its target under contact, like a
compliant finger.
"""

from dataclasses import dataclass, field

import numpy as np

DIGITS = ("index", "middle", "ring", "pinky")


@dataclass(frozen=True)
class HandSpec:
    """A hand version.

    Attributes:
        name: Version name (``v0``, ``v1``, ...).
        ranges_deg: Joint-name suffix -> (lower, upper) in degrees, applied
            on top of the arm file's ranges.
        couplings: (follower, leader, ratio): the follower's servo target
            is ``ratio * leader target`` and it is not an action.
    """

    name: str
    ranges_deg: dict = field(default_factory=dict)
    couplings: tuple = ()


def _v1_ranges():
    r = {}
    for f in DIGITS:
        r[f"{f}_d0_flexion"] = (-20.0, 90.0)   # MCP: modest extension
        r[f"{f}_d1_flexion"] = (-5.0, 60.0)    # PIP: ~no hyperextension
        r[f"{f}_d2_flexion"] = (-5.0, 60.0)    # DIP: ~no hyperextension
    # Thumb base (CMC): a large natural range in primates, including
    # extension, so the flexion range is kept as in the arm file; the
    # scripted pinch uses its full extension as a thumb "post". Abduction
    # loses only its far dorsal end.
    r["thumb_d0_abduction"] = (-15.0, 90.0)
    r["thumb_d0_flexion"] = (-30.0, 30.0)
    r["thumb_d1_flexion"] = (-10.0, 60.0)   # thumb MCP
    r["thumb_d2_flexion"] = (-10.0, 60.0)   # thumb IP
    return r


DIP_RATIO = 2.0 / 3.0

HANDS = {
    "v0": HandSpec("v0"),
    # Restricted extension, independent joints (23 actions).
    "v1_ranges": HandSpec("v1_ranges", _v1_ranges()),
    # Restricted extension + DIP-PIP coupling (19 actions). The version 1
    # default.
    "v1": HandSpec("v1", _v1_ranges(), tuple(
        (f"{f}_d2_flexion", f"{f}_d1_flexion", DIP_RATIO) for f in DIGITS)),
}


def get_hand(name: str) -> HandSpec:
    if name not in HANDS:
        raise ValueError(f"unknown hand {name!r}; one of {list(HANDS)}")
    return HANDS[name]


def apply_ranges(spec, hand: HandSpec, prefix: str) -> None:
    """Override joint ranges in an (uncompiled) ``MjSpec``. Servo control
    ranges follow, since the servos inherit their joint's range.

    The arm is attached from an MJCF file whose angles are in degrees, and
    its joints keep that unit inside the combined spec, so the overrides
    are written in degrees. :func:`check_ranges` verifies the compiled
    result.
    """
    for j in spec.joints:
        suffix = j.name.removeprefix(prefix)
        if suffix in hand.ranges_deg:
            j.range = list(hand.ranges_deg[suffix])


def check_ranges(model, hand: HandSpec, prefix: str) -> None:
    """Assert that the compiled joint ranges match the hand spec."""
    for suffix, (lo, hi) in hand.ranges_deg.items():
        got = np.degrees(model.jnt_range[model.joint(prefix + suffix).id])
        if not np.allclose(got, (lo, hi), atol=1e-6):
            raise AssertionError(f"{suffix}: compiled range {got}, "
                                 f"expected {(lo, hi)}")


def coupling_indices(joint_names: list[str], hand: HandSpec):
    """Action layout for a hand.

    Returns:
        action_idx: indices (into the 23 joints) driven by the action;
        followers: list of (follower index, leader index, ratio).
    """
    short = [n.rsplit("/", 1)[-1] for n in joint_names]
    followers = [(short.index(f), short.index(lead), r)
                 for f, lead, r in hand.couplings]
    drop = {f for f, _, _ in followers}
    action_idx = np.array([i for i in range(len(short)) if i not in drop])
    return action_idx, followers
