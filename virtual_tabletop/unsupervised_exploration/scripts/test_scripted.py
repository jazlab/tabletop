"""Run the scripted grasp on N trials; print metrics; save frames/video.

Usage: test_scripted.py OUT_PREFIX [N] [KWARGS] [P_FIXED] [N_VIDEO] [SEED]
[HAND] [SHAPES]

Defaults: N=24, KWARGS={} (passed to ``ScriptedGrasp``), P_FIXED=0,
N_VIDEO=4, SEED=3, HAND=v1, SHAPES=v1.

Prints per trial (shape, size class, steps with hand touch / grasp /
pinch / lifted, lifted at the last step, max object height in cm) and the
rates; saves ``OUT_PREFIX`` ``scripted.mp4`` (the first ``N_VIDEO``
trials, camera "close") and ``scripted_frames.png`` (5 frames from each
of the first 4 trials).
"""
import sys

import imageio
import numpy as np
from ue.sim.env import PlayEnv
from ue.teach.scripted import ScriptedGrasp

SP = sys.argv[1]
n = int(sys.argv[2]) if len(sys.argv) > 2 else 24
kw = eval(sys.argv[3]) if len(sys.argv) > 3 else {}
p_fixed = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
n_video = int(sys.argv[5]) if len(sys.argv) > 5 else 4
seed = int(sys.argv[6]) if len(sys.argv) > 6 else 3
hand = sys.argv[7] if len(sys.argv) > 7 else "v1"
shapes = sys.argv[8] if len(sys.argv) > 8 else "v1"
env = PlayEnv(seed=seed, render_size=(360, 480), p_fixed=p_fixed,
              hand=hand, shapes=shapes)
pol = ScriptedGrasp(env, **kw)
frames, res = [], []
for ep in range(n):
    obs, info = env.reset()
    agg = dict(touch=0, grasp=0, pinch=0, lift=0, maxh=0.0)
    for k in range(env.max_steps):
        obs, _, _, tr, info = env.step(pol(obs))
        agg["touch"] += info["hand_touching"]
        agg["grasp"] += info["grasp"]
        agg["pinch"] += info["pinch"]
        agg["lift"] += info["lifted"]
        agg["maxh"] = max(agg["maxh"], info["obj_height"])
        if ep < n_video and k % 3 == 0:
            frames.append(env.render("close"))
    size = env.pm.shapes[env.shape_index].meta.get("size_class", -1)
    res.append((info["shape"], size, agg["touch"], agg["grasp"],
                agg["pinch"], agg["lift"], int(info["lifted"]),
                round(agg["maxh"] * 100, 1)))
for r in res:
    print(r)
print("touch rate", np.mean([r[2] > 0 for r in res]),
      "grasp rate", np.mean([r[3] > 0 for r in res]),
      "pinch rate", np.mean([r[4] > 0 for r in res]),
      "lift rate", np.mean([r[5] > 0 for r in res]),
      "held at end", np.mean([r[6] for r in res]))
imageio.mimsave(SP + "scripted.mp4", frames, fps=25, macro_block_size=1)
n_strip = min(4, n_video, n)
if n_strip:
    imageio.imwrite(SP + "scripted_frames.png", np.concatenate(
        [np.concatenate(frames[i * 25:i * 25 + 25:5], 1)
         for i in range(n_strip)], 0)[::2, ::2])
