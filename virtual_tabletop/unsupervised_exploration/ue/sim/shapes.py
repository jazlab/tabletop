"""Procedural block shapes for the play environment.

Every shape is a list of *convex parts* (vertex arrays, metres, in the
shape's own frame, roughly centred on the origin). Most shapes are a single
convex polyhedron; a few (L, T, cross, step) are unions of boxes, which
MuJoCo handles as several convex geoms on one body.

Families (``FAMILIES``):

- ``box``: cubes, slabs and bars (random aspect ratios);
- ``prism``: n-gonal prisms, n in {3, 4, 5, 6, 8}, regular or squashed;
- ``cylinder``: 14-gon prisms (a near-cylinder; rolls when on its side);
- ``pyramid``: square or triangular base, apex possibly offset;
- ``frustum``: truncated pyramids (n-gon base, smaller top);
- ``wedge``: right-triangular prism (ramp);
- ``platonic``: tetrahedron, octahedron, dodecahedron, icosahedron
  (randomly stretched along one axis);
- ``bipyramid``: two n-gonal pyramids glued base-to-base;
- ``random_hull``: convex hull of 8-14 random points (an irregular
  polyhedron);
- ``compound``: L, T, cross or step made of 2-3 boxes (non-convex).

Sizes: the largest extent is drawn in [``min_size``, ``max_size``]
(default 3-7 cm: from a small cube to a block that fills the hand).
"""

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import ConvexHull

FAMILIES = (
    "box",
    "prism",
    "cylinder",
    "pyramid",
    "frustum",
    "wedge",
    "platonic",
    "bipyramid",
    "random_hull",
    "compound",
)


@dataclass
class Shape:
    name: str
    family: str
    parts: list[np.ndarray]  # each (k, 3) convex vertex set
    meta: dict = field(default_factory=dict)

    @property
    def vertices(self) -> np.ndarray:
        return np.concatenate(self.parts)

    @property
    def extent(self) -> np.ndarray:
        v = self.vertices
        return v.max(0) - v.min(0)


# --- primitives ---------------------------------------------------------


def _box(half) -> np.ndarray:
    hx, hy, hz = half
    return np.array(
        [[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1)
         for sz in (-1, 1)],
        float,
    )


def _ngon(n, rx, ry, phase=0.0) -> np.ndarray:
    a = phase + 2 * np.pi * np.arange(n) / n
    return np.stack([rx * np.cos(a), ry * np.sin(a)], 1)


def _prism(n, rx, ry, h, phase=0.0) -> np.ndarray:
    p = _ngon(n, rx, ry, phase)
    return np.concatenate(
        [np.c_[p, np.full(n, -h / 2)], np.c_[p, np.full(n, h / 2)]]
    )


def _frustum(n, r_bottom, r_top, h, phase=0.0, top_offset=(0, 0)):
    b = _ngon(n, r_bottom, r_bottom, phase)
    t = _ngon(n, r_top, r_top, phase) + np.asarray(top_offset)
    return np.concatenate(
        [np.c_[b, np.full(n, -h / 2)], np.c_[t, np.full(n, h / 2)]]
    )


def _platonic(kind: str) -> np.ndarray:
    phi = (1 + 5**0.5) / 2
    if kind == "tetrahedron":
        v = [[1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]]
    elif kind == "octahedron":
        v = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1],
             [0, 0, -1]]
    elif kind == "icosahedron":
        v = []
        for a in (-1, 1):
            for b in (-phi, phi):
                v += [[0, a, b], [a, b, 0], [b, 0, a]]
    elif kind == "dodecahedron":
        v = [[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]
        for a in (-1, 1):
            for b in (-1, 1):
                v += [[0, a / phi, b * phi], [a / phi, b * phi, 0],
                      [b * phi, 0, a / phi]]
    else:
        raise ValueError(kind)
    v = np.array(v, float)
    return v / np.abs(v).max()


def _normalise(parts, size, rng=None) -> list[np.ndarray]:
    """Scale so the largest extent is ``size``; centre the bounding box."""
    v = np.concatenate(parts)
    centre = (v.max(0) + v.min(0)) / 2
    scale = size / (v.max(0) - v.min(0)).max()
    return [(p - centre) * scale for p in parts]


# --- families -------------------------------------------------------------


def make_shape(family: str, rng: np.random.Generator, size: float,
               name: str) -> Shape:
    meta = {}
    if family == "box":
        kind = rng.choice(["cube", "slab", "bar", "brick"])
        ratios = {
            "cube": [1, 1, 1],
            "slab": [1, rng.uniform(0.7, 1), rng.uniform(0.25, 0.45)],
            "bar": [1, rng.uniform(0.25, 0.45), rng.uniform(0.25, 0.45)],
            "brick": [1, rng.uniform(0.45, 0.75), rng.uniform(0.35, 0.6)],
        }[kind]
        parts = [_box(np.array(ratios) / 2)]
        meta["kind"] = str(kind)
    elif family == "prism":
        n = int(rng.choice([3, 4, 5, 6, 8]))
        squash = rng.uniform(0.6, 1.0)
        parts = [_prism(n, 1, squash, rng.uniform(0.5, 2.0),
                        rng.uniform(0, np.pi))]
        meta["n"] = n
    elif family == "cylinder":
        parts = [_prism(14, 1, 1, rng.uniform(0.6, 2.4))]
    elif family == "pyramid":
        n = int(rng.choice([3, 4]))
        h = rng.uniform(0.7, 1.6)
        off = rng.uniform(-0.3, 0.3, 2)
        base = _ngon(n, 1, 1, np.pi / n)
        parts = [np.concatenate([np.c_[base, np.zeros(n)],
                                 [[off[0], off[1], h]]])]
        meta["n"] = n
    elif family == "frustum":
        n = int(rng.choice([4, 5, 6, 8]))
        parts = [_frustum(n, 1, rng.uniform(0.4, 0.8), rng.uniform(0.6, 1.6),
                          rng.uniform(0, np.pi), rng.uniform(-0.15, 0.15, 2))]
        meta["n"] = n
    elif family == "wedge":
        w, h = rng.uniform(0.4, 0.9), rng.uniform(0.3, 0.8)
        tri = np.array([[0, 0], [1, 0], [0, h]])
        parts = [np.concatenate([np.c_[tri[:, 0], np.full(3, -w / 2),
                                       tri[:, 1]],
                                 np.c_[tri[:, 0], np.full(3, w / 2),
                                       tri[:, 1]]])]
    elif family == "platonic":
        kind = str(rng.choice(["tetrahedron", "octahedron", "dodecahedron",
                               "icosahedron"]))
        v = _platonic(kind) * np.array([1, 1, rng.uniform(0.7, 1.4)])
        parts = [v]
        meta["kind"] = kind
    elif family == "bipyramid":
        n = int(rng.choice([3, 4, 5, 6]))
        ring = _ngon(n, 1, 1)
        h1, h2 = rng.uniform(0.6, 1.3, 2)
        parts = [np.concatenate([np.c_[ring, np.zeros(n)], [[0, 0, h1]],
                                 [[0, 0, -h2]]])]
        meta["n"] = n
    elif family == "random_hull":
        k = int(rng.integers(8, 15))
        pts = rng.normal(size=(k, 3)) * rng.uniform(0.5, 1.0, 3)
        hull = ConvexHull(pts)
        parts = [pts[hull.vertices]]
    elif family == "compound":
        kind = str(rng.choice(["L", "T", "cross", "step"]))
        t = rng.uniform(0.25, 0.4)  # bar thickness relative to length
        d = rng.uniform(0.25, 0.5)  # depth
        if kind == "L":
            parts = [_box([0.5, t / 2, d / 2]) + [0, 0, 0],
                     _box([t / 2, 0.35, d / 2]) + [0.5 - t / 2, 0.35 + t / 2
                                                   - 0.0, 0]]
        elif kind == "T":
            parts = [_box([0.5, t / 2, d / 2]),
                     _box([t / 2, 0.35, d / 2]) + [0, -0.35 - t / 2, 0]]
        elif kind == "cross":
            parts = [_box([0.5, t / 2, d / 2]), _box([t / 2, 0.5, d / 2])]
        else:  # step: two stacked boxes, offset
            parts = [_box([0.5, d / 2, t / 2]),
                     _box([0.25, d / 2, t / 2]) + [-0.25, 0, t]]
        meta["kind"] = kind
    else:
        raise ValueError(family)
    return Shape(name, family, _normalise(parts, size), meta)


def make_library(n: int, seed: int, min_size: float = 0.03,
                 max_size: float = 0.07) -> list[Shape]:
    """``n`` shapes, cycling through the families (shuffled) for balance."""
    rng = np.random.default_rng(seed)
    families = []
    while len(families) < n:
        families += list(rng.permutation(FAMILIES))
    shapes = []
    for i, fam in enumerate(families[:n]):
        size = rng.uniform(min_size, max_size)
        shapes.append(make_shape(str(fam), rng, size, f"s{i:02d}_{fam}"))
    return shapes


#: Version 1 size classes: largest extent (m) of each copy of a base shape.
V1_SIZES = (0.03, 0.04, 0.05)


def make_library_v1(n_base: int, seed: int,
                    sizes: tuple = V1_SIZES,
                    jitter: float = 0.05) -> list[Shape]:
    """Version 1 library: ``n_base`` base shapes, each in every size class.

    Each base shape (family, proportions) appears at every size in
    ``sizes`` (largest extent, ±``jitter`` relative), so the same block
    comes small, medium and large. Version 0 drew one size per shape from
    3–7 cm. Here the largest blocks are 5 cm, which fits the monkey hand's
    thumb-to-finger aperture (about 4.7 cm).
    """
    rng = np.random.default_rng(seed)
    families = []
    while len(families) < n_base:
        families += list(rng.permutation(FAMILIES))
    shapes = []
    for i, fam in enumerate(families[:n_base]):
        base = make_shape(str(fam), rng, 1.0, f"b{i:02d}_{fam}")
        for k, size in enumerate(sizes):
            s = size * (1 + rng.uniform(-jitter, jitter))
            shapes.append(Shape(
                f"{base.name}_{'sml'[k] if len(sizes) == 3 else k}",
                base.family, [p * s for p in base.parts],
                dict(base.meta, size_class=k, size=round(s, 4))))
    return shapes


#: Shape-set versions: ``(builder, n)``; the env's ``n_shapes`` is ignored
#: for v1, which has ``n_base * len(V1_SIZES)`` shapes.
SHAPE_SETS = ("v0", "v1")
