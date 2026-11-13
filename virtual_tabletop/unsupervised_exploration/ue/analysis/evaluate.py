"""Evaluate a trained explorer (or a baseline) on held-out shapes.

usage:
    python -m ue.analysis.evaluate version_<v>/runs/<name> [--trials 120]
        [--video]
    python -m ue.analysis.evaluate random|still|dep|scripted --version V

Trials use shape libraries never seen in training (library and spawn seeds
9000+), half fixed and half free. The policy is stochastic by default
(the exploratory behaviour); ``--deterministic`` uses the mean action.
For skill methods each trial gets a skill: CSF a direction on the sphere,
DIAYN the skills in turn.

Per-trial metrics (from the env's ``info``; never seen by the learners):
any touch / pinch / grasp / lift, fraction of time touching, grasping and
lifted, max object displacement and rotation, min hand-surface distance,
mean number of touching patches. Coverage over all trials: distinct
contact cells (object-state bin x hand unit x object region, as in the
contact-count method), distinct contact sets, and 2-cm voxels of object
displacement (free trials).

Writes ``eval[_det].json`` (summary, per-trial rows) in the run directory
(or ``version_<v>/runs/baseline_<name>/``) and, with ``--video``,
``video.mp4``.
"""

import argparse
import json
from pathlib import Path

import imageio
import numpy as np
import torch
import yaml
from PIL import Image, ImageDraw

from .. import paths
from ..learn.sac import SAC
from ..learn.train import DEFAULTS, load_extra
from ..rewards import build, obj_state_bin
from ..sim.env import STRUCTURAL, PlayEnv
from ..sim.hand import HANDS

EVAL_SEED = 9000


class Policy:
    """Wraps a run's actor (and its skill sampler)."""

    def __init__(self, run: Path, env=None, deterministic=False,
                 snapshot=None):
        cfg = dict(DEFAULTS)
        cfg.update(yaml.safe_load(open(run / "config.yaml")))
        # Structural env options the policy was trained with (evaluation
        # envs must match them).
        self.env_kwargs = {k: cfg["env"][k] for k in STRUCTURAL
                           if k in cfg["env"]}
        if env is None or self.env_kwargs:
            env = PlayEnv(seed=0, n_shapes=2, **self.env_kwargs)
        spec = dict(obs_dim=env.observation_space.shape[0],
                    act_dim=env.action_space.shape[0],
                    layout=env.observation_layout,
                    group_index=env.group_index)
        self.module = build(spec, cfg["method"], cfg["method_kwargs"],
                            cfg["mix"])
        self.agent = SAC(spec["obs_dim"], spec["act_dim"],
                         self.module.z_dim, hidden=tuple(cfg["hidden"]))
        sd = torch.load(run / (snapshot or "agent.pt"), weights_only=False)
        self.agent.load_state_dict(sd)
        if (run / "module.pt").exists():
            self.module.load_state_dict(torch.load(run / "module.pt",
                                                   weights_only=False))
        load_extra(self.module, run / "module_extra.pkl")
        self.det = deterministic
        self.z = None
        self.cfg = cfg
        self.n_skill = 0

    def new_trial(self, i, obs=None):
        zd = self.module.z_dim
        if not zd:
            self.z = None
        elif self.cfg["method"] == "diayn":
            self.z = np.eye(zd, dtype=np.float32)[i % zd][None]
        elif self.cfg["method"] == "csf":
            # The 2d axis directions +-e_k, in turn (comparable skills).
            k = i % (2 * zd)
            z = np.zeros((1, zd), np.float32)
            z[0, k // 2] = 1.0 if k % 2 == 0 else -1.0
            self.z = z
        elif self.cfg["method"] == "goals" or getattr(
                self.module, "goal_module", None) is not None:
            self.z = self.module.sample_z(1, obs=obs[None])
        else:
            self.z = self.module.sample_z(1)
        return None if self.z is None else self.z[0]

    def __call__(self, obs, env):
        return self.agent.act(obs[None], self.z, self.det)[0]


class Baseline:
    def __init__(self, kind, env, hand="v0", shapes="v0"):
        self.kind = kind
        self.ctrl = None
        self.env_kwargs = {"hand": hand, "shapes": shapes}

    def new_trial(self, i, obs=None):
        return None

    def __call__(self, obs, env):
        if self.kind == "random":
            return np.random.uniform(-1, 1, env.action_space.shape)
        if self.kind == "still":
            return np.zeros(env.action_space.shape)
        if self.ctrl is None or self.ctrl.env is not env:
            if self.kind == "dep":
                from ..teach.dep import DEPController
                old = self.ctrl
                self.ctrl = DEPController(env)
                if old is not None:  # DEP keeps its couplings
                    self.ctrl.arm, self.ctrl.fingers = old.arm, old.fingers
            elif self.kind == "scripted":
                if self.env_kwargs.get("hand", "v0") == "v0":
                    # Frozen copy: reproduces the version 0 baseline.
                    from ..teach.scripted_v0 import ScriptedGraspV0 as SG
                else:
                    from ..teach.scripted import ScriptedGrasp as SG
                self.ctrl = SG(env)
        return self.ctrl(obs)


def label(frame, text):
    im = Image.fromarray(frame)
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, im.width, 16], fill=(0, 0, 0))
    d.text((4, 2), text, fill=(255, 255, 255))
    return np.asarray(im)


def evaluate(policy, n_trials=120, video=None, n_video=12, camera="close",
             size=(240, 320), seed=EVAL_SEED, envs_per_lib=10, cols=3):
    rows = []
    lay = None
    cells, sets, voxels = set(), set(), set()
    frames_by_trial = []
    env = None
    for i in range(n_trials):
        if i % envs_per_lib == 0:
            if env is not None:
                env.close()
            lib = seed + i // envs_per_lib
            env = PlayEnv(seed=lib, shape_seed=lib, render_size=size,
                          p_fixed=0.5,
                          **getattr(policy, "env_kwargs", {}))
            lay = env.observation_layout
        env.p_fixed = float(i % 2 == 0)  # alternate fixed / free
        obs, info = env.reset()
        z = policy.new_trial(i, obs)
        acc = dict(touch=0, pinch=0, grasp=0, lift=0, ntouch=0, disp=0.0,
                   rot=0.0, mind=1.0, height=0.0, force=0.0, fell=False,
                   first_touch=None, units=np.zeros(6), regions=np.zeros(6),
                   palm_path=0.0, carry=0.0, turn=0.0, lift_events=0,
                   hyper=0.0, jerk=0.0, effort=0.0, res=[])
        palm_prev = env.palm_centre
        obj_prev, quat_prev, was_lifted = None, None, False
        # Natural (hand v1) lower limits of the hand joints, to measure how
        # far any hand version bends past them.
        nat = HANDS["v1"].ranges_deg
        hand_idx = [i for i, n in enumerate(env.pm.joint_names)
                    if n.rsplit("/", 1)[-1] in nat]
        nat_lo = np.radians([nat[env.pm.joint_names[i].rsplit("/", 1)[-1]]
                             [0] for i in hand_idx]) - np.radians(2)
        frames = []
        gm_obj = getattr(policy, "module", None)
        gm_obj = getattr(gm_obj, "goal_module", None) or gm_obj
        gmod = getattr(gm_obj, "MODULES", None)
        if gmod is not None and z is not None:
            gm = gmod[int(np.argmax(z[:4]))]
            gdist = []
        else:
            gm = None
        for k in range(env.max_steps):
            obs, _, _, trunc, info = env.step(policy(obs, env))
            if gm is not None:
                mod = gm_obj
                a = mod.achieved(obs, gm)
                gdist.append(float(mod._dist(a, z[mod.slot[gm]], gm)))
            acc["touch"] += info["hand_touching"]
            acc["pinch"] += info["pinch"]
            acc["grasp"] += info["grasp"]
            acc["lift"] += info["lifted"]
            acc["ntouch"] += info["n_touch"]
            acc["disp"] = max(acc["disp"], info["obj_disp"])
            acc["rot"] = max(acc["rot"], info["obj_rot_angle"])
            acc["mind"] = min(acc["mind"], info["min_surf_dist"])
            acc["height"] = max(acc["height"], info["obj_height"])
            acc["force"] = max(acc["force"], info["force"])
            acc["fell"] = acc["fell"] or bool(info["fell"])
            # Manipulation while lifted: carry distance, in-hand rotation,
            # number of separate lifts.
            oc = env.object_centre
            q = env.data.xquat[env.pm.object_body].copy()
            if info["lifted"]:
                if was_lifted:
                    acc["carry"] += float(np.linalg.norm(oc - obj_prev))
                    acc["turn"] += 2 * np.arccos(min(1.0, abs(float(
                        np.dot(q, quat_prev)))))
                else:
                    acc["lift_events"] += 1
            obj_prev, quat_prev, was_lifted = oc, q, bool(info["lifted"])
            # Naturalness: hand joints bent past their natural (v1) limit
            # by more than 2 deg; jerk.
            qh = env.data.qpos[env.pm.qadr[hand_idx]]
            acc["hyper"] += float((qh < nat_lo).mean())
            acc["jerk"] += info.get("jerk", 0.0)
            acc["effort"] += info.get("effort", 0.0)
            if "res_ratio" in info:
                acc["res"].append(info["res_ratio"])
            if info["hand_touching"] and acc["first_touch"] is None:
                acc["first_touch"] = (k + 1) * env.dt
            x = obs[None]
            s = obj_state_bin(x, lay)[0]
            cm = x[0, lay["contact_map"]].reshape(6, 6) > 0.2
            acc["units"] += cm.any(1)
            acc["regions"] += cm.any(0)
            pc = env.palm_centre
            acc["palm_path"] += float(np.linalg.norm(pc - palm_prev))
            palm_prev = pc
            for u, r in zip(*np.nonzero(cm)):
                cells.add((int(s), int(u), int(r)))
            if cm.any():
                sets.add((int(s), np.packbits(cm.ravel()).tobytes()))
            if not env.fixed:
                d = env.object_centre - env.spawn_centre
                voxels.add(tuple(np.floor(d / 0.02).astype(int)))
            if video is not None and i < n_video:
                f = env.render(camera)
                tag = (f"trial {i} {'fixed' if env.fixed else 'free'} "
                       f"{info['shape']}"
                       + (" TOUCH" if info["hand_touching"] else "")
                       + (" GRASP" if info["grasp"] else "")
                       + (" LIFT" if info["lifted"] else ""))
                frames.append(label(f, tag))
        if frames:
            frames_by_trial.append(frames)
        L = env.max_steps
        rows.append(dict(
            trial=i, fixed=bool(env.fixed), shape=info["shape"],
            z=None if z is None else np.asarray(z).round(3).tolist(),
            any_touch=acc["touch"] > 0, any_pinch=acc["pinch"] > 0,
            any_grasp=acc["grasp"] > 0, any_lift=acc["lift"] > 0,
            touch_frac=acc["touch"] / L, grasp_frac=acc["grasp"] / L,
            lift_frac=acc["lift"] / L, mean_ntouch=acc["ntouch"] / L,
            max_disp=acc["disp"], max_rot=acc["rot"], min_dist=acc["mind"],
            max_height=acc["height"], max_force=acc["force"],
            fell=acc["fell"], first_touch=acc["first_touch"],
            unit_use=(acc["units"] / L).round(3).tolist(),
            region_use=(acc["regions"] / L).round(3).tolist(),
            palm_path=acc["palm_path"], held_end=bool(info["lifted"]),
            carry=acc["carry"], turn_lifted=acc["turn"],
            lift_events=acc["lift_events"], hyper_frac=acc["hyper"] / L,
            jerk=acc["jerk"] / L, effort=acc["effort"] / L,
            res_ratio=float(np.mean(acc["res"])) if acc["res"] else None))
        if gm is not None:
            eps = gm_obj.eps[gm]
            # Distance at the start (to tell trivial goals apart).
            rows[-1].update(goal_module=gm, goal_d_final=gdist[-1],
                            goal_d_min=min(gdist), goal_d_start=gdist[0],
                            goal_success=gdist[-1] < eps,
                            goal_trivial=gdist[0] < eps)
    env.close()
    summary = summarize(rows)
    if rows and "goal_module" in rows[0]:
        gs = {}
        for m in sorted({r["goal_module"] for r in rows}):
            sel = [r for r in rows if r["goal_module"] == m]
            hard = [r for r in sel if not r["goal_trivial"]]
            gs[m] = dict(n=len(sel), success=float(np.mean(
                [r["goal_success"] for r in sel])),
                n_nontrivial=len(hard),
                success_nontrivial=float(np.mean(
                    [r["goal_success"] for r in hard])) if hard else None,
                d_start=float(np.mean([r["goal_d_start"] for r in sel])),
                d_final=float(np.mean([r["goal_d_final"] for r in sel])))
        summary["goals"] = gs
    summary.update(contact_cells=len(cells), contact_sets=len(sets),
                   disp_voxels=len(voxels))
    if video is not None and frames_by_trial:
        write_grid_video(frames_by_trial, video, cols=cols)
    return summary, rows


def summarize(rows):
    keys = ["any_touch", "any_pinch", "any_grasp", "any_lift", "touch_frac",
            "grasp_frac", "lift_frac", "mean_ntouch", "max_disp", "max_rot",
            "min_dist", "max_force", "fell"]
    # Version 1 metrics (absent from version 0 evaluations).
    keys += [k for k in ("held_end", "carry", "turn_lifted", "lift_events",
                         "hyper_frac", "jerk", "effort", "max_height")
             if rows and k in rows[0]]
    out = {}
    for group, sel in (("all", rows), ("fixed", [r for r in rows
                                                 if r["fixed"]]),
                       ("free", [r for r in rows if not r["fixed"]])):
        if not sel:
            continue
        out[group] = {k: float(np.mean([r[k] for r in sel])) for k in keys}
        out[group]["n"] = len(sel)
        rr = [r["res_ratio"] for r in sel if r.get("res_ratio") is not None]
        if rr:
            out[group]["res_ratio"] = float(np.mean(rr))
        ft = [r["first_touch"] for r in sel if r["first_touch"] is not None]
        out[group]["time_to_touch"] = float(np.mean(ft)) if ft else None
        # Displacement while staying on the pedestal (non-destructive).
        kept = [r["max_disp"] for r in sel if not r["fell"]]
        out[group]["disp_kept"] = float(np.mean(kept)) if kept else 0.0
    return out


def write_grid_video(trials, path, cols=3, fps=25):
    """Tile trials (lists of frames) into a grid video, 3 at a time."""
    h, w = trials[0][0].shape[:2]
    rows = int(np.ceil(len(trials) / cols))
    frames = []
    # Play rows of trials in sequence: each "page" shows `cols` trials.
    for r in range(rows):
        page = trials[r * cols:(r + 1) * cols]
        T = max(len(t) for t in page)
        for k in range(T):
            tiles = [t[min(k, len(t) - 1)] for t in page]
            while len(tiles) < cols:
                tiles.append(np.zeros((h, w, 3), np.uint8))
            frames.append(np.concatenate(tiles, 1))
    imageio.mimsave(path, frames, fps=fps, macro_block_size=1)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("target")
    p.add_argument("--trials", type=int, default=120)
    p.add_argument("--video", action="store_true")
    p.add_argument("--n-video", type=int, default=12)
    p.add_argument("--deterministic", action="store_true")
    p.add_argument("--snapshot")
    p.add_argument("--tag", default="")
    p.add_argument("--size", type=int, nargs=2, default=(240, 320),
                   help="video tile height width")
    p.add_argument("--camera", default="close")
    p.add_argument("--cols", type=int, default=3)
    p.add_argument("--version", default="1",
                   help="version for baseline outputs (runs/baseline_*)")
    p.add_argument("--shapes", default=None,
                   help="shape set for baselines (default: v0 for version "
                   "0, else v1)")
    p.add_argument("--hand", default=None,
                   help="hand for baselines (default: v0 for version 0, "
                   "else v1)")
    args = p.parse_args(argv)
    torch.set_num_threads(1)
    probe = PlayEnv(seed=0, n_shapes=2)
    if args.target in ("random", "still", "dep", "scripted"):
        out = paths.runs_dir(args.version) / f"baseline_{args.target}"
        out.mkdir(parents=True, exist_ok=True)
        v0 = str(args.version) in ("0", "v0")
        hand = args.hand or ("v0" if v0 else "v1")
        shapes = args.shapes or ("v0" if v0 else "v1")
        policy = Baseline(args.target, probe, hand, shapes)
    else:
        out = Path(args.target)
        policy = Policy(out, None, args.deterministic, args.snapshot)
    np.random.seed(0)
    torch.manual_seed(0)
    suffix = ("_det" if args.deterministic else "") + args.tag
    video = out / f"video{suffix}.mp4" if args.video else None
    summary, rows = evaluate(policy, args.trials, video, args.n_video,
                             camera=args.camera, size=tuple(args.size),
                             cols=args.cols)
    json.dump({"summary": summary, "trials": rows},
              open(out / f"eval{suffix}.json", "w"), indent=1,
              default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
