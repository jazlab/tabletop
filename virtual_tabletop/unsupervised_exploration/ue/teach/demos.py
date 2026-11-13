"""Demonstration data from the scripted controller (version 1 method A).

usage:
    python -m ue.teach.demos OUT.npz [--trials 1500] [--workers 3]
        [--hand v1] [--shapes v1] [--seed0 5000]

Runs the scripted controller (:mod:`ue.teach.scripted`) on
training-type shape libraries. Seeds start at ``seed0`` = 5000, disjoint
from the evaluation libraries (9000+) and from training env seeds (below
2000). Free and fixed blocks alternate. A trial is kept if it succeeds:
free blocks lifted, fixed blocks with an opposed grasp.

Transitions are stored trial-major as ``[T, n_trials, ...]`` arrays:
``obs``, ``act`` (in the env's action space, e.g. 19-D for hand v1:
the scripted controller's 23-D action without the coupled DIP joints),
``next_obs``, ``done`` and ``info`` (``ue.sim.vec.INFO_KEYS`` order).
:func:`load_replay` turns them into a :class:`~ue.learn.replay.Replay`
whose columns are trials, so n-step chains stay inside one trial.
"""

import argparse
import multiprocessing as mp
import os

import numpy as np


def _run(args):
    """Worker: ``n`` trials from library ``seed``; returns kept trials."""
    seed, n, hand, shapes = args
    os.environ["OMP_NUM_THREADS"] = "1"
    from ..sim.env import PlayEnv
    from ..sim.vec import INFO_KEYS
    from .scripted import ScriptedGrasp

    env = PlayEnv(seed=seed, shape_seed=seed, hand=hand, shapes=shapes)
    kept, stats = [], []
    for i in range(n):
        env.p_fixed = float(i % 2 == 1)
        obs, _ = env.reset()
        ctrl = ScriptedGrasp(env)
        rec = {k: [] for k in ("obs", "act", "next_obs", "done", "info")}
        lifted = grasped = unstable = False
        for t in range(env.max_steps):
            full = np.clip(env.full_action(ctrl(obs)), -1, 1)
            act = full[env.action_idx]
            nxt, _, _, trunc, info = env.step(act)
            rec["obs"].append(obs)
            rec["act"].append(act)
            rec["next_obs"].append(nxt)
            rec["done"].append(trunc)
            rec["info"].append([float(info[k]) for k in INFO_KEYS])
            lifted |= info["lifted"]
            grasped |= info["grasp"]
            unstable |= info["unstable"]
            obs = nxt
            if trunc:
                break
        ok = (not unstable and len(rec["obs"]) == env.max_steps
              and (lifted if not env.fixed else grasped))
        stats.append((env.fixed, lifted, grasped, ok))
        if ok:
            kept.append({k: np.asarray(v, np.float32)
                         for k, v in rec.items()})
    return kept, stats


def generate(out, trials=1500, workers=3, hand="v1", shapes="v1",
             seed0=5000, per_lib=25):
    jobs = [(seed0 + j, per_lib, hand, shapes)
            for j in range(int(np.ceil(trials / per_lib)))]
    with mp.get_context("spawn").Pool(workers) as pool:
        res = pool.map(_run, jobs)
    kept = [k for r in res for k in r[0]]
    stats = np.array([s for r in res for s in r[1]], bool)
    data = {k: np.stack([t[k] for t in kept], 1) for k in kept[0]}
    free, fixed = stats[~stats[:, 0]], stats[stats[:, 0]]
    summary = dict(
        trials=len(stats), kept=len(kept),
        free_lift=float(free[:, 1].mean()) if len(free) else 0.0,
        fixed_grasp=float(fixed[:, 2].mean()) if len(fixed) else 0.0)
    np.savez_compressed(out, **data, **{f"stat_{k}": v
                                        for k, v in summary.items()})
    return summary


def load_replay(path, z_dim=0):
    """A :class:`~ue.learn.replay.Replay` holding the demonstrations
    (columns = trials; full)."""
    from ..learn.replay import Replay

    d = np.load(path)
    T, n = d["obs"].shape[:2]
    buf = Replay(T, n, d["obs"].shape[-1], d["act"].shape[-1], z_dim,
                 d["info"].shape[-1])
    buf.obs[:], buf.act[:], buf.next_obs[:] = d["obs"], d["act"], \
        d["next_obs"]
    buf.done[:], buf.info[:] = d["done"].astype(bool), d["info"]
    buf.t, buf.full = 0, True
    return buf


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("out")
    p.add_argument("--trials", type=int, default=1500)
    p.add_argument("--workers", type=int, default=3)
    p.add_argument("--hand", default="v1")
    p.add_argument("--shapes", default="v1")
    p.add_argument("--seed0", type=int, default=5000)
    a = p.parse_args(argv)
    print(generate(a.out, a.trials, a.workers, a.hand, a.shapes, a.seed0))


if __name__ == "__main__":
    main()
