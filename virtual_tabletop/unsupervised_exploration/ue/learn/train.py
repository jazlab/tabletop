"""Train one exploration method: SAC + an intrinsic reward module.

usage: python -m ue.learn.train version_<v>/configs/<...>.yaml
           [--steps N] [--name NAME] [--seed S] [--version V]

Config keys (defaults in ``DEFAULTS``):

- ``method`` (a key of :data:`ue.intrinsic.MODULES`) and
  ``method_kwargs``, or ``mix``: list of ``[name, weight, kwargs]``;
- ``steps`` (env steps, all envs), ``n_envs``, ``n_workers``;
- ``utd`` (gradient updates per env step), ``batch``, ``hidden``,
  ``gamma``, ``n_step``, ``lr``, ``warmup`` (random-action env steps);
- ``env``: kwargs of :class:`ue.sim.env.PlayEnv` (e.g. ``p_fixed``);
- ``schedule``: env attributes annealed linearly over training, e.g.
  ``{p_fixed: [0.8, 0.5], reflex: [1.0, 0.0, 0.6]}`` (start, end, and the
  fraction of training at which ``end`` is reached): the static -> dynamic
  curriculum and the fading grasp reflex.

Outputs in ``version_<v>/runs/<name>/``: ``config.yaml``,
``progress.csv`` (per-trial
metrics averaged over each logging window, learner stats), ``agent.pt``
and ``module.pt`` checkpoints (also ``agent_<step>.pt`` snapshots).
"""

import argparse
import csv
import inspect
import os
import pickle
import time
from pathlib import Path

import numpy as np
import torch
import yaml

from .. import paths
from ..rewards import build
from ..sim.vec import INFO_KEYS, VecPlayEnv
from .noise import pink_noise
from .replay import Replay
from .sac import SAC

DEFAULTS = dict(
    name=None, method="none", method_kwargs={}, mix=None, seed=0,
    steps=600_000, n_envs=8, n_workers=1, utd=0.25, batch=512,
    hidden=[256, 256], gamma=0.97, n_step=3, lr=3e-4, warmup=8000,
    buffer=600_000, log_every=10_000, snapshot_every=200_000,
    target_entropy=None, init_alpha=0.05, fixed_alpha=None, env={},
    torch_threads=1, schedule={}, noise="white", dep_switch=None,
    demos=None,
)


def scheduled(spec, frac):
    """``[start, end]`` or ``[start, end, end_frac]``: linear in training
    progress ``frac``, reaching ``end`` at ``end_frac`` (default 1)."""
    start, end = spec[0], spec[1]
    stop = spec[2] if len(spec) > 2 else 1.0
    return start + (end - start) * min(frac / stop, 1.0)


class TrialStats:
    """Per-trial aggregates of the env's info (evaluation metrics)."""

    def __init__(self, n_envs):
        self.k = {k: i for i, k in enumerate(INFO_KEYS)}
        self.n = n_envs
        self._reset(np.arange(n_envs))
        self.done_trials = []

    def _reset(self, idx):
        if not hasattr(self, "acc"):
            self.acc = {k: np.zeros(self.n) for k in
                        ("touch", "grasp", "pinch", "lift", "disp", "rot",
                         "ntouch", "len", "mind")}
        for v in self.acc.values():
            v[idx] = 0
        self.acc["mind"][idx] = 1.0

    def add(self, info, dones):
        k, a = self.k, self.acc
        a["touch"] += info[:, k["hand_touching"]]
        a["grasp"] += info[:, k["grasp"]]
        a["pinch"] += info[:, k["pinch"]]
        a["lift"] += info[:, k["lifted"]]
        a["ntouch"] += info[:, k["n_touch"]]
        a["disp"] = np.maximum(a["disp"], info[:, k["obj_disp"]])
        a["rot"] = np.maximum(a["rot"], info[:, k["obj_rot_angle"]])
        a["mind"] = np.minimum(a["mind"], info[:, k["min_surf_dist"]])
        a["len"] += 1
        for i in np.flatnonzero(dones):
            L = a["len"][i]
            self.done_trials.append(dict(
                fixed=info[i, k["fixed"]],
                touch_frac=a["touch"][i] / L, any_touch=a["touch"][i] > 0,
                any_grasp=a["grasp"][i] > 0, grasp_frac=a["grasp"][i] / L,
                any_pinch=a["pinch"][i] > 0, any_lift=a["lift"][i] > 0,
                lift_frac=a["lift"][i] / L, max_disp=a["disp"][i],
                max_rot=a["rot"][i], mean_ntouch=a["ntouch"][i] / L,
                min_dist=a["mind"][i]))
            self._reset([i])

    def flush(self):
        t, self.done_trials = self.done_trials, []
        if not t:
            return {}
        out = {f"tr_{k}": float(np.mean([x[k] for x in t])) for k in t[0]}
        free = [x for x in t if x["fixed"] < 0.5]
        if free:
            out["tr_free_lift"] = float(np.mean([x["any_lift"]
                                                 for x in free]))
            out["tr_free_disp"] = float(np.mean([x["max_disp"]
                                                 for x in free]))
        out["trials"] = len(t)
        return out


def sample_z(module, n, obs):
    """Skills/goals for new trials (goal modules also see the new obs)."""
    if not module.z_dim:
        return None
    if "obs" in inspect.signature(module.sample_z).parameters:
        return module.sample_z(n, obs=obs)
    return module.sample_z(n)


def observe(module, obs, next_obs, dones, z):
    if "z" in inspect.signature(module.observe).parameters:
        module.observe(obs, next_obs, dones, z=z)
    else:
        module.observe(obs, next_obs, dones)


def save_extra(module, path):
    """Pickle a module's non-tensor state (counts, goal archives)."""
    keep = {}
    for m in getattr(module, "parts", [module]):
        for k in ("counts", "set_counts", "archive", "n_arch", "outcomes"):
            if hasattr(m, k):
                keep[f"{type(m).__name__}.{k}"] = getattr(m, k)
    pickle.dump(keep, open(path, "wb"))


def load_extra(module, path):
    if not path.exists():
        return
    keep = pickle.load(open(path, "rb"))
    for m in getattr(module, "parts", [module]):
        for key, v in keep.items():
            cls, k = key.split(".", 1)
            if cls == type(m).__name__:
                setattr(m, k, v)


def env_spec(vec):
    return dict(obs_dim=vec.obs_dim, act_dim=vec.act_dim,
                layout=vec.observation_layout, group_index=vec.group_index)


def sample_batch(buf, agent, module, B, info_keys=INFO_KEYS):
    n = agent.n_step
    t, e = buf.sample_idx(B, n)
    ts, es, valid = buf.chain(t, e, n)
    obs = torch.as_tensor(buf.obs[ts, es])  # [B, n, D]
    act = torch.as_tensor(buf.act[ts, es])
    nxt = torch.as_tensor(buf.next_obs[ts, es])
    z = torch.as_tensor(buf.z[t, e]) if module.z_dim else None
    if z is not None and hasattr(module, "relabel"):
        z = module.relabel(buf, t, e, z)
    D = obs.shape[-1]
    zz = (z[:, None].expand(-1, n, -1).reshape(B * n, -1)
          if z is not None else None)
    inf = torch.as_tensor(buf.info[ts, es]).reshape(B * n, -1)
    info = {k: inf[:, i] for i, k in enumerate(info_keys)}
    r = module.reward(obs.reshape(B * n, D), act.reshape(B * n, -1),
                      nxt.reshape(B * n, D), zz, info=info).reshape(B, n)
    valid = torch.as_tensor(valid, dtype=torch.float32)
    last = valid.sum(1).long() - 1
    x_next = nxt[torch.arange(B), last]
    disc = agent.gamma ** (last + 1).float()
    x0, a0, n0 = obs[:, 0], act[:, 0], nxt[:, 0]
    if z is not None:
        x_in = torch.cat([x0, z], -1)
        xn_in = torch.cat([x_next, z], -1)
    else:
        x_in, xn_in = x0, x_next
    return dict(x=x_in, a=a0, r=r, valid=valid, xn=xn_in, disc=disc,
                obs=x0, next_obs=n0, z=z)


def reward_term_stats(buf, module, n=512):
    """Mean and spread of each part of a mixed reward on recent-ish
    replay samples (for choosing weights)."""
    t, e = buf.sample_idx(n, 1)
    inf = torch.as_tensor(buf.info[t, e])
    info = {k: inf[:, i] for i, k in enumerate(INFO_KEYS)}
    z = torch.as_tensor(buf.z[t, e]) if module.z_dim else None
    with torch.no_grad():
        terms = module.reward_terms(torch.as_tensor(buf.obs[t, e]),
                                    torch.as_tensor(buf.act[t, e]),
                                    torch.as_tensor(buf.next_obs[t, e]),
                                    z, info)
    out = {}
    for k, r in terms.items():
        out[f"rterm_{k}_mean"] = round(r.mean().item(), 4)
        out[f"rterm_{k}_std"] = round(r.std().item(), 4)
    return out


def sample_mixed(buf, demo, agent, module, B, demo_frac):
    """A batch with ``demo_frac`` of its samples from the demonstration
    replay (method A); ``is_demo`` marks them."""
    if demo is None or demo_frac <= 0:
        b = sample_batch(buf, agent, module, B)
        b["is_demo"] = torch.zeros(B, dtype=torch.bool)
        return b
    nd = int(round(B * demo_frac))
    b1 = sample_batch(buf, agent, module, B - nd)
    b2 = sample_batch(demo, agent, module, nd)
    out = {k: (None if b1[k] is None else torch.cat([b1[k], b2[k]]))
           for k in b1}
    out["is_demo"] = torch.cat([torch.zeros(B - nd, dtype=torch.bool),
                                torch.ones(nd, dtype=torch.bool)])
    return out


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("config")
    p.add_argument("--steps", type=int)
    p.add_argument("--name")
    p.add_argument("--seed", type=int)
    p.add_argument("--version", help="version whose runs/ gets the output "
                   "(default: the version directory holding the config)")
    args = p.parse_args(argv)
    cfg = dict(DEFAULTS)
    cfg.update(yaml.safe_load(open(args.config)))
    if args.steps:
        cfg["steps"] = args.steps
    if args.name:
        cfg["name"] = args.name
    if args.seed is not None:
        cfg["seed"] = args.seed
    cfg["name"] = cfg["name"] or Path(args.config).stem
    vdir = (paths.version_dir(args.version) if args.version
            else paths.version_of(args.config))
    if vdir is None:
        raise SystemExit("config is outside a version_* directory; "
                         "pass --version")
    out = vdir / "runs" / cfg["name"]
    out.mkdir(parents=True, exist_ok=True)
    yaml.safe_dump(cfg, open(out / "config.yaml", "w"))
    torch.set_num_threads(cfg["torch_threads"])
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    np.random.seed(cfg["seed"])
    torch.manual_seed(cfg["seed"])

    vec = VecPlayEnv(cfg["n_envs"], cfg["n_workers"], seed=cfg["seed"] + 1,
                     **cfg["env"])
    spec = env_spec(vec)
    module = build(spec, cfg["method"], cfg["method_kwargs"], cfg["mix"])
    agent = SAC(vec.obs_dim, vec.act_dim, module.z_dim,
                hidden=tuple(cfg["hidden"]), lr=cfg["lr"],
                gamma=cfg["gamma"], n_step=cfg["n_step"],
                batch=cfg["batch"], init_alpha=cfg["init_alpha"],
                target_entropy=cfg["target_entropy"],
                fixed_alpha=cfg["fixed_alpha"])
    N = cfg["n_envs"]
    buf = Replay(cfg["buffer"] // N, N, vec.obs_dim, vec.act_dim,
                 module.z_dim, len(INFO_KEYS))
    stats = TrialStats(N)
    # Demonstrations (method A): a replay of scripted trials, mixed into
    # every batch, plus a fading Q-filtered behaviour-cloning term.
    demo = None
    if cfg["demos"]:
        from ..teach.demos import load_replay
        demo = load_replay(paths.PROJECT / cfg["demos"]["path"],
                           module.z_dim)
        if demo.obs.shape[-1] != vec.obs_dim:
            raise SystemExit("demo obs dim does not match the env")
    obs = vec.reset()
    z = sample_z(module, N, obs)
    step, upd_acc, n_upd = 0, 0.0, 0
    learn_log, t0 = {}, time.time()
    log_f = open(out / "progress.csv", "w", newline="")
    writer = None
    next_log, next_snap = cfg["log_every"], cfg["snapshot_every"]
    rew_hist = []
    # Exploration noise: white (SAC default) or pink, one coloured
    # sequence per env per trial.
    T = vec.max_steps
    pink = cfg["noise"] == "pink"
    noise_seq = pink_noise(N, T, vec.act_dim) if pink else None
    t_in_trial = np.zeros(N, int)
    # DEP-RL switching: with prob p per step DEP drives an env for h steps.
    dep, dep_left = None, np.zeros(N, int)
    if cfg["dep_switch"]:
        from ..teach.dep import VecDEP
        probe = vec.probe
        dep = VecDEP(N, vec.observation_layout, probe.lo, probe.hi,
                     probe.max_speed, probe.dt, seed=cfg["seed"])
    n_dep_steps = 0
    while step < cfg["steps"]:
        if step < cfg["warmup"]:
            act = np.random.uniform(-1, 1, (N, vec.act_dim)).astype(
                np.float32)
        else:
            eps = (noise_seq[np.arange(N), np.minimum(t_in_trial, T - 1)]
                   if pink else None)
            act = agent.act(obs, z, eps=eps)
        if dep is not None:
            for i in range(N):
                if dep_left[i] == 0 and np.random.rand() < \
                        cfg["dep_switch"]["p"]:
                    dep_left[i] = cfg["dep_switch"]["h"]
                if dep_left[i] > 0:
                    a_dep = dep.act(i, obs[i])[vec.probe.action_idx]
                    if step >= cfg["warmup"]:
                        act[i] = a_dep
                        n_dep_steps += 1
                    dep_left[i] -= 1
        nobs, final, dones, info = vec.step(act)
        t_in_trial += 1
        if dones.any():
            t_in_trial[dones] = 0
            dep_left[dones] = 0
            if pink:
                noise_seq[dones] = pink_noise(int(dones.sum()), T,
                                              vec.act_dim)
            if dep is not None:
                for i in np.flatnonzero(dones):
                    dep.reset(i)
        buf.add(obs, act, final, dones, z, info)
        observe(module, obs, final, dones, z)
        stats.add(info, dones)
        if module.z_dim and dones.any():
            z = z.copy()
            z[dones] = sample_z(module, int(dones.sum()), nobs[dones])
        obs = nobs
        step += N
        if step >= cfg["warmup"] // 2:
            upd_acc += cfg["utd"] * N
            while upd_acc >= 1:
                upd_acc -= 1
                frac = step / cfg["steps"]
                b = sample_mixed(buf, demo, agent, module, cfg["batch"],
                                 cfg["demos"]["frac"] if demo else 0.0)
                mlog = module.update(b["obs"], b["a"], b["next_obs"],
                                     b["z"])
                bc = None
                if demo is not None and cfg["demos"].get("bc"):
                    bc = {"mask": b["is_demo"],
                          "lam": float(scheduled(cfg["demos"]["bc"],
                                                 frac))}
                if step >= cfg["warmup"]:
                    alog = agent.update(b["x"], b["a"], b["r"], b["valid"],
                                        b["xn"], b["disc"], bc=bc)
                    if bc is not None:
                        alog["bc_lam"] = bc["lam"]
                    learn_log = {**alog, **mlog}
                    rew_hist.append(b["r"][:, 0].mean().item())
                n_upd += 1
        if step % 20_000 < N and cfg["schedule"]:
            frac = step / cfg["steps"]
            for attr, spec in cfg["schedule"].items():
                vec.set_attr(attr, float(scheduled(spec, frac)))
        if step >= next_log:
            next_log += cfg["log_every"]
            row = {"step": step, "time": round(time.time() - t0, 1),
                   "sps": round(step / (time.time() - t0), 1),
                   "updates": n_upd, "dep_steps": n_dep_steps,
                   "r_int": float(np.mean(rew_hist)) if rew_hist else 0.0}
            rew_hist = []
            row.update(stats.flush())
            row.update({k: round(v, 5) for k, v in learn_log.items()})
            if hasattr(module, "stats"):
                row.update(module.stats())
            if hasattr(module, "reward_terms") and buf.size > 4096:
                row.update(reward_term_stats(buf, module))
            if writer is None:
                keys = list(row.keys())
                writer = csv.DictWriter(log_f, fieldnames=keys,
                                        extrasaction="ignore")
                writer.writeheader()
            writer.writerow(row)
            log_f.flush()
            print(" ".join(f"{k}={v:.3g}" if isinstance(v, float) else
                           f"{k}={v}" for k, v in row.items()), flush=True)
        if step >= next_snap:
            next_snap += cfg["snapshot_every"]
            torch.save(agent.state_dict(), out / f"agent_{step}.pt")
            save_extra(module, out / "module_extra.pkl")
    torch.save(agent.state_dict(), out / "agent.pt")
    torch.save(module.state_dict(), out / "module.pt")
    save_extra(module, out / "module_extra.pkl")
    vec.close()
    print("done", out)


if __name__ == "__main__":
    main()
