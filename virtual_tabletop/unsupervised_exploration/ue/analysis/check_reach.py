"""Check the spawn workspace is within reach of the palm centre.

Samples random arm postures (fingers straight), records palm-centre
positions, and for a grid over the spawn box reports the distance to the
nearest reachable palm position (and the fraction within 2 cm).
"""
import mujoco
import numpy as np
from scipy.spatial import cKDTree

from ue.sim.env import WORKSPACE
from ue.sim.model import PlayModel

pm = PlayModel(4, 0)
m = pm.model
d = mujoco.MjData(m)
rng = np.random.default_rng(0)
P = []
for _ in range(60000):
    q = rng.uniform(pm.jnt_range[:7, 0], pm.jnt_range[:7, 1])
    d.qpos[pm.qadr[:7]] = q
    mujoco.mj_kinematics(m, d)
    P.append(d.geom_xpos[pm.palm_geom].copy())
P = np.array(P)
tree = cKDTree(P)
ws = WORKSPACE
for label, zr in (("fixed", ws["z_fixed"]), ("platform+3cm",
                  (ws["z_platform"][0] + 0.03, ws["z_platform"][1] + 0.03))):
    g = np.stack(np.meshgrid(np.linspace(*ws["x"], 7),
                             np.linspace(*ws["y"], 5),
                             np.linspace(*zr, 5)), -1).reshape(-1, 3)
    dist, _ = tree.query(g)
    print(label, "max dist %.3f, frac<2cm %.2f"
          % (dist.max(), (dist < 0.02).mean()))
    bad = g[dist > 0.02]
    if len(bad):
        print("  unreachable examples", np.round(bad[:8], 3))
