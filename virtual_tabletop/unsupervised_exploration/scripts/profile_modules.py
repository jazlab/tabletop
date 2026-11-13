"""Time reward + update of each intrinsic module on a random batch."""
import time

import torch
from ue.learn.sac import SAC
from ue.rewards import build
from ue.sim.env import PlayEnv

torch.set_num_threads(1)
env = PlayEnv(seed=0, n_shapes=2)
D, A = env.observation_space.shape[0], env.action_space.shape[0]
spec = dict(obs_dim=D, act_dim=A, layout=env.observation_layout,
            group_index=env.group_index)
B, n = 512, 3
obs = torch.randn(B * n, D) * 0.3
act = torch.rand(B * n, A) * 2 - 1
nxt = obs + 0.01 * torch.randn_like(obs)
for name in ["contact", "rnd", "disagreement", "touch", "apt", "music",
             "csf", "diayn", "goals"]:
    mod = build(spec, name, {})
    z = torch.as_tensor(mod.sample_z(B * n)) if mod.z_dim else None
    zb = z[:B] if z is not None else None
    for _ in range(2):
        mod.reward(obs, act, nxt, z)
        mod.update(obs[:B], act[:B], nxt[:B], zb)
    t = time.time()
    for _ in range(10):
        mod.reward(obs, act, nxt, z)
    tr = (time.time() - t) / 10
    t = time.time()
    for _ in range(10):
        mod.update(obs[:B], act[:B], nxt[:B], zb)
    tu = (time.time() - t) / 10
    print(f"{name:13s} reward {tr*1000:6.1f} ms  update {tu*1000:6.1f} ms")
agent = SAC(D, A, 0)
x = obs[:B]
r = torch.zeros(B, n)
v = torch.ones(B, n)
t = time.time()
for _ in range(10):
    agent.update(x, act[:B], r, v, x, torch.ones(B) * 0.9)
print(f"SAC update {(time.time()-t)/10*1000:.1f} ms")
