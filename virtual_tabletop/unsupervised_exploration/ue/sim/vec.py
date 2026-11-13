"""A small subprocess vector env for :class:`~ue.env.PlayEnv`.

``n_workers`` processes each step ``envs_per_worker`` envs in sequence.
Episodes auto-reset in the same step: the returned ``obs`` is the first
observation of the new trial, and ``final_obs`` holds the last observation
of the finished one (for bootstrapping / storing the transition).

Each env gets its own seed and shape-library seed (``seed * 1000 + i``), so
the agent sees ``32 * n_envs`` distinct shapes during training. Evaluation
uses held-out library seeds (see :mod:`ue.evaluate`).
"""

import multiprocessing as mp
import os

import numpy as np

INFO_KEYS = ("touching", "hand_touching", "n_touch", "pinch", "grasp",
             "lifted", "obj_disp", "obj_height", "obj_rot_angle",
             "palm_obj_dist", "min_surf_dist", "force", "fixed", "fell",
             "unstable", "effort", "jerk")


def _worker(conn, env_kwargs_list):
    os.environ["OMP_NUM_THREADS"] = "1"
    from ue.sim.env import PlayEnv

    envs = [PlayEnv(**kw) for kw in env_kwargs_list]
    while True:
        cmd, data = conn.recv()
        if cmd == "reset":
            obs = [e.reset()[0] for e in envs]
            conn.send(np.stack(obs))
        elif cmd == "step":
            out_obs, final, dones, infos = [], [], [], []
            for e, a in zip(envs, data):
                o, _, term, trunc, info = e.step(a)
                done = term or trunc
                infos.append([float(info[k]) for k in INFO_KEYS])
                final.append(o)
                if done:
                    o, _ = e.reset()
                out_obs.append(o)
                dones.append(done)
            conn.send((np.stack(out_obs), np.stack(final),
                       np.array(dones), np.array(infos, np.float32)))
        elif cmd == "set_attr":
            for e in envs:
                setattr(e, data[0], data[1])
            conn.send(True)
        elif cmd == "close":
            conn.close()
            return


class VecPlayEnv:
    def __init__(self, n_envs: int = 8, n_workers: int = 2, seed: int = 0,
                 **env_kwargs):
        assert n_envs % n_workers == 0
        self.n_envs = n_envs
        per = n_envs // n_workers
        ctx = mp.get_context("spawn")
        self.conns, self.procs = [], []
        for w in range(n_workers):
            kws = [dict(env_kwargs, seed=seed * 1000 + w * per + i)
                   for i in range(per)]
            parent, child = ctx.Pipe()
            p = ctx.Process(target=_worker, args=(child, kws), daemon=True)
            p.start()
            self.conns.append(parent)
            self.procs.append(p)
        self.per = per
        from ue.sim.env import STRUCTURAL, PlayEnv
        probe = PlayEnv(seed=0, n_shapes=2,
                        **{k: env_kwargs[k] for k in STRUCTURAL
                           if k in env_kwargs})
        self.probe = probe
        self.observation_layout = probe.observation_layout
        self.obs_dim = probe.observation_space.shape[0]
        self.act_dim = probe.action_space.shape[0]
        self.max_steps = probe.max_steps
        self.groups = probe.groups
        self.group_index = probe.group_index
        self.info_keys = INFO_KEYS

    def reset(self):
        for c in self.conns:
            c.send(("reset", None))
        return np.concatenate([c.recv() for c in self.conns])

    def step(self, actions):
        for i, c in enumerate(self.conns):
            c.send(("step", actions[i * self.per:(i + 1) * self.per]))
        res = [c.recv() for c in self.conns]
        obs = np.concatenate([r[0] for r in res])
        final = np.concatenate([r[1] for r in res])
        dones = np.concatenate([r[2] for r in res])
        infos = np.concatenate([r[3] for r in res])
        return obs, final, dones, infos

    def set_attr(self, name, value):
        for c in self.conns:
            c.send(("set_attr", (name, value)))
        for c in self.conns:
            c.recv()

    def close(self):
        for c in self.conns:
            try:
                c.send(("close", None))
            except (BrokenPipeError, OSError):
                pass
        for p in self.procs:
            p.join(timeout=2)
