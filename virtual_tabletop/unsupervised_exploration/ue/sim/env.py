"""PlayEnv: the right monkey arm and one block per trial, no reward.

Each episode is one *trial* (``trial_time`` s, default 3 s): a new block
appears, the arm starts at rest (hanging; small random joint noise), and
the agent moves the arm with joint control. Then the next trial re-spawns a
different block. There is no task reward (``reward`` is always 0); learners
supply their own intrinsic rewards from the observation / ``info``.

Trials (sampled independently):

- a shape from the library (:mod:`.shapes`; 32 per model, the library seed
  differs per env so many more shapes are seen across workers), a random
  colour;
- ``fixed`` (prob ``p_fixed``): held in space by a weld, at a random 3-D
  position and a uniformly random orientation; a thin rod is drawn under it;
- otherwise free, resting on a pedestal ("platform") whose top height and
  position are random; the block is dropped from a random orientation onto
  it and settled for 0.4 s before the trial starts (so it lies on one of its
  stable faces).

Positions are in a box in front of the right shoulder, all within reach
(checked by :mod:`scripts.check_reach`).

Action: 23 joint commands in [-1, 1] (7 arm + 16 finger joints). Default
``action_mode="delta"``: each moves its joint's position-servo target by
``±max_speed * dt`` (rad). ``"absolute"``: the action maps linearly onto
the joint range.

Observation (float32 vector; ``observation_layout`` names the slices):

========================  ====  =============================================
``qpos``                  23    joint angles, scaled to [-1, 1] by range
``qvel``                  23    joint velocities / 10
``target``                23    current servo targets, scaled like qpos
``palm_pos``              3     palm centre relative to the shoulder (×5)
``tips``                  15    5 fingertips relative to the palm (×10)
``touch``                 18    per-patch contact force with the object:
                                palm, 15 finger segments, forearm, upper arm
                                (log(1 + F / 0.05 N))
``touch_platform``        1     total force on the pedestal (same scale)
``contact_map``           36    6 hand units (palm, thumb, index, middle,
                                ring, pinky) x 6 object regions (+-x, +-y,
                                +-z faces of its bounding box), tanh(F/0.2N)
``wrench``                6     net force (tanh(F/0.5N)) and torque
                                (tanh(T/0.01Nm)) on the object, object frame
``surf_dist``             6     palm and fingertip distance to the object
                                surface (×10, capped at 1)
``obj_rel``               3     object centre - palm centre (×5)
``obj_pos``               3     object centre relative to the spawn (×5)
``obj_rot``               6     first two columns of its rotation matrix
``obj_drot``              3     rotation since spawn (rotation vector, rad)
``obj_vel``               6     linear (×2) and angular (×0.2) velocity
``obj_size``              3     bounding-box extent (×10)
``fixed``                 1     1 if the object is held in space
``time``                  1     fraction of the trial elapsed
========================  ====  =============================================

Groups (for learners that use subsets): ``proprio`` = qpos, qvel, target,
palm_pos, tips; ``touch`` = touch, touch_platform, contact_map, wrench;
``object`` = obj_* and fixed; ``interaction`` = touch, contact_map,
wrench, surf_dist, obj_rel, obj_pos, obj_drot, obj_vel (no joint angles:
the input of the object-centric intrinsic rewards); ``skill`` = the same
without obj_rel (only quantities the agent changes).

``info`` carries evaluation quantities (never used as reward by the purely
unsupervised learners): ``touching`` (hand on object), ``n_touch`` (hand
patches on the object), ``grasp`` (thumb + another finger / palm on the
object with forces > ``GRASP_FORCE``), ``lifted`` (free object > 2 cm above
its spawn height and not touching the pedestal), ``obj_disp`` (m),
``obj_rot_angle`` (rad from spawn), ``palm_obj_dist``.
"""

import gymnasium as gym
import mujoco
import numpy as np

from .hand import coupling_indices
from .model import PARKED, PLATFORM_HALF, PlayModel

PATCH_SCALE = 0.05  # N
GRASP_FORCE = 0.05  # N per patch
LIFT_HEIGHT = 0.02  # m

# Workspace (world frame; the shoulder is at (1.02, 0.21, 0.80) facing +y).
WORKSPACE = {
    "x": (0.94, 1.12),
    "y": (0.33, 0.42),
    "z_fixed": (0.62, 0.82),
    "z_platform": (0.58, 0.72),  # pedestal top height
}

FINGER_PATCHES = {
    "thumb": [1, 2, 3],
    "others": list(range(4, 16)),
    "palm": [0],
}

PALETTE = np.array([
    [0.90, 0.20, 0.20], [0.20, 0.60, 0.90], [0.95, 0.75, 0.15],
    [0.30, 0.75, 0.35], [0.65, 0.35, 0.85], [0.95, 0.50, 0.15],
    [0.15, 0.80, 0.80], [0.90, 0.40, 0.65], [0.55, 0.55, 0.55],
    [0.35, 0.25, 0.15],
])


def _quat_random(rng):
    u = rng.normal(size=4)
    return u / np.linalg.norm(u)


def _quat_mul(a, b):
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, a, b)
    return out


def _quat_angle(a, b):
    dot = abs(float(np.dot(a, b)))
    return 2 * np.arccos(min(1.0, dot))


#: Env options that change the model, the action space or the observation.
#: A policy must be evaluated with the same values it was trained with.
STRUCTURAL = ("hand", "shapes", "residual_scale")


class PlayEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"]}

    def __init__(
        self,
        seed: int = 0,
        shape_seed: int | None = None,
        n_shapes: int = 32,
        trial_time: float = 3.0,
        dt: float = 0.04,
        max_speed: float = 2.5,
        action_mode: str = "delta",
        p_fixed: float = 0.5,
        init_noise: float = 0.1,
        reflex: float = 0.0,
        restart_p: float = 0.0,
        restart_free_only: bool = False,
        hand: str = "v0",
        shapes: str = "v0",
        residual_scale: float = 0.0,
        demo_trials: int = 0,
        demo_restart_p: float = 0.0,
        demo_tmin: float = 0.0,
        render_size=(480, 640),
        camera: str = "persp",
    ):
        self.pm = PlayModel(n_shapes, seed if shape_seed is None
                            else shape_seed, hand=hand, shapes=shapes)
        self.hand = self.pm.hand
        self.model, self.data = self.pm.model, mujoco.MjData(self.pm.model)
        self.rng = np.random.default_rng(seed)
        self.trial_time, self.dt = trial_time, dt
        self.max_steps = int(round(trial_time / dt))
        self.substeps = max(1, round(dt / self.model.opt.timestep))
        self.max_speed, self.action_mode = max_speed, action_mode
        self.p_fixed, self.init_noise = p_fixed, init_noise
        # Grasp reflex strength in [0, 1] (a developmental prior; 0 = off):
        # while the palm or >= 2 hand units touch the object, finger
        # flexion targets are pushed toward closing at reflex * 3 rad/s.
        self.reflex = reflex
        names = [n.rsplit("/", 1)[-1] for n in self.pm.joint_names]
        self._flex_idx = np.array([i for i, n in enumerate(names)
                                   if i >= 7 and "flexion" in n])
        self._thumb_abd = names.index("thumb_d0_abduction")
        self._reflex_on = False
        # Go-Explore-style restarts (a curriculum prior, not a reward):
        # states where >= 2 hand units touch the object are archived, and a
        # new trial starts from one of them with probability restart_p.
        self.restart_p = restart_p
        self.restart_free_only = restart_free_only
        self._archive = []
        self.restarted = False
        self.render_size, self.camera = render_size, camera
        self._renderer = None
        pm = self.pm
        self.n_joints = len(pm.joint_names)
        self.lo, self.hi = pm.jnt_range[:, 0], pm.jnt_range[:, 1]
        self.rest = np.clip(0.0, self.lo, self.hi)
        # Joints driven by the action (all 23, or 19 with DIP coupling) and
        # coupled followers (target = ratio * leader target).
        self.action_idx, self.followers = coupling_indices(pm.joint_names,
                                                           self.hand)
        self.action_space = gym.spaces.Box(-1, 1, (len(self.action_idx),),
                                           np.float32)
        n_act = len(self.action_idx)
        # Residual mode (version 1 method C): the scripted controller acts
        # every step and the policy adds a correction:
        # executed = clip(a_script + residual_scale * a_policy). The
        # script's action is appended to the observation.
        self.residual_scale = residual_scale
        self.parts = list(self.PARTS)
        if residual_scale > 0:
            self.parts.append(("base_action", n_act))
        self._script = None
        self._base_action = np.zeros(n_act)
        self._prev_exec = np.zeros(n_act)
        # Reverse curriculum from demonstrations (version 1 method B): at
        # the first reset the env records ``demo_trials`` scripted trials
        # on its own shape library and keeps the state snapshots of the
        # successful ones. A trial then starts, with probability
        # ``demo_restart_p``, from a snapshot at a time index >=
        # ``demo_tmin`` x (trajectory length). Both are annealed by the
        # training loop (``schedule``).
        self.demo_trials = demo_trials
        self.demo_restart_p, self.demo_tmin = demo_restart_p, demo_tmin
        self._demo_trajs = None
        self.observation_layout = self._layout()
        n = max(s.stop for s in self.observation_layout.values())
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (n,),
                                                np.float32)
        self.groups = {
            "proprio": ["qpos", "qvel", "target", "palm_pos", "tips"],
            "touch": ["touch", "touch_platform", "contact_map", "wrench"],
            "object": ["obj_rel", "obj_pos", "obj_rot", "obj_vel",
                       "obj_size", "fixed"],
            # Interaction features for intrinsic rewards (no joints, and
            # no absolute object orientation, which is random per trial).
            "interaction": ["touch", "contact_map", "wrench", "surf_dist",
                            "obj_rel", "obj_pos", "obj_drot", "obj_vel"],
            # Only quantities the agent changes (no spawn-dependent
            # offsets): for skill discriminators.
            "skill": ["touch", "contact_map", "wrench", "surf_dist",
                      "obj_pos", "obj_drot", "obj_vel"],
        }
        self.shape_index = 0
        self.fixed = False
        self.steps = 0

    # --- layout ----------------------------------------------------------

    PARTS = [("qpos", 23), ("qvel", 23), ("target", 23), ("palm_pos", 3),
             ("tips", 15), ("touch", 18), ("touch_platform", 1),
             ("contact_map", 36), ("wrench", 6), ("surf_dist", 6),
             ("obj_rel", 3), ("obj_pos", 3), ("obj_rot", 6), ("obj_drot", 3),
             ("obj_vel", 6),
             ("obj_size", 3), ("fixed", 1), ("time", 1)]

    def _layout(self):
        out, i = {}, 0
        for name, size in getattr(self, "parts", self.PARTS):
            out[name] = slice(i, i + size)
            i += size
        return out

    def group_index(self, groups) -> np.ndarray:
        idx = []
        for g in groups:
            names = self.groups.get(g, [g])
            for name in names:
                s = self.observation_layout[name]
                idx += list(range(s.start, s.stop))
        return np.array(idx)

    # --- trial setup -------------------------------------------------------

    def _set_shape(self, k: int, rgba):
        m, pm = self.model, self.pm
        all_g = pm.all_object_geoms
        m.geom_contype[all_g] = 0
        m.geom_conaffinity[all_g] = 0
        m.geom_rgba[all_g, 3] = 0.0
        for g in pm.shape_geoms[k]:
            m.geom_contype[g] = 3
            m.geom_conaffinity[g] = 3
            m.geom_rgba[g] = rgba
        ine = pm.inertials[k]
        b = pm.object_body
        # Broadphase filters on body-level bits compiled from the geoms.
        m.body_contype[b] = 3
        m.body_conaffinity[b] = 3
        m.body_mass[b] = ine["mass"]
        m.body_ipos[b] = ine["ipos"]
        m.body_iquat[b] = ine["iquat"]
        m.body_inertia[b] = ine["inertia"]
        self.shape_index = k
        verts = pm.shapes[k].vertices
        self._verts = verts
        self._centre_local = (verts.max(0) + verts.min(0)) / 2
        self._extent = verts.max(0) - verts.min(0)

    def _set_object_pose(self, pos, quat):
        d, pm = self.data, self.pm
        d.qpos[pm.object_qadr:pm.object_qadr + 3] = pos
        d.qpos[pm.object_qadr + 3:pm.object_qadr + 7] = quat
        d.qvel[pm.object_dofadr:pm.object_dofadr + 6] = 0

    def _arm_rest(self):
        d, pm = self.data, self.pm
        q = self.rest.copy()
        if self.init_noise:
            q = np.clip(q + self.rng.normal(0, self.init_noise, q.shape),
                        self.lo, self.hi)
        for f, lead, ratio in self.followers:  # coupled joints start coupled
            q[f] = np.clip(ratio * q[lead], self.lo[f], self.hi[f])
        d.qpos[pm.qadr] = q
        d.qvel[pm.dofadr] = 0
        d.ctrl[pm.actuators] = q
        self.target = q.copy()

    def _spawn(self):
        m, d, pm, rng = self.model, self.data, self.pm, self.rng
        ws = WORKSPACE
        k = int(rng.integers(len(pm.shapes)))
        rgb = PALETTE[rng.integers(len(PALETTE))]
        rgb = np.clip(rgb + rng.normal(0, 0.06, 3), 0, 1)
        self._set_shape(k, (*rgb, 1.0))
        self.fixed = bool(rng.random() < self.p_fixed)
        xy = np.array([rng.uniform(*ws["x"]), rng.uniform(*ws["y"])])
        quat = _quat_random(rng)
        mujoco.mj_resetData(m, d)
        self._arm_rest()
        if self.fixed:
            z = rng.uniform(*ws["z_fixed"])
            centre = np.array([*xy, z])
            # Body origin so that the bbox centre lands at ``centre``.
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, quat)
            pos = centre - R.reshape(3, 3) @ self._centre_local
            self._set_object_pose(pos, quat)
            m.eq_data[pm.weld, :3] = 0
            m.eq_data[pm.weld, 3:6] = pos
            m.eq_data[pm.weld, 6:10] = quat
            d.eq_active[pm.weld] = 1
            d.mocap_pos[pm.platform_mocap] = PARKED + [0, 0.5, 0]
            # Rod from below up to the object's lowest point.
            R = R.reshape(3, 3)
            low = (self._verts @ R.T + pos)[:, 2].min()
            d.mocap_pos[pm.holder_mocap] = [*xy, low]
            mujoco.mj_forward(m, d)
        else:
            d.eq_active[pm.weld] = 0
            d.mocap_pos[pm.holder_mocap] = PARKED + [0, 1, 0]
            top = rng.uniform(*ws["z_platform"])
            d.mocap_pos[pm.platform_mocap] = [*xy, top - PLATFORM_HALF[2]]
            for _ in range(5):
                R = np.zeros(9)
                mujoco.mju_quat2Mat(R, quat)
                R = R.reshape(3, 3)
                low = (self._verts @ R.T)[:, 2].min()
                pos = np.array([*xy, top - low + 0.002])
                self._set_object_pose(pos, quat)
                mujoco.mj_forward(m, d)
                mujoco.mj_step(m, d, int(0.4 / m.opt.timestep))
                p = d.qpos[pm.object_qadr:pm.object_qadr + 3]
                if p[2] > top - 0.01 and abs(p[0] - xy[0]) < 0.05 and \
                        abs(p[1] - xy[1]) < 0.05:
                    break
                quat = _quat_random(rng)
            # The arm is reset again after settling (it may have drifted).
            self._arm_rest()
            d.qvel[pm.object_dofadr:pm.object_dofadr + 6] = 0
            mujoco.mj_forward(m, d)
        self.platform_top = (None if self.fixed
                             else d.mocap_pos[pm.platform_mocap][2]
                             + PLATFORM_HALF[2])
        self.spawn_centre = self.object_centre.copy()
        self.spawn_quat = d.xquat[pm.object_body].copy()

    # --- state ------------------------------------------------------------

    @property
    def object_centre(self) -> np.ndarray:
        d, b = self.data, self.pm.object_body
        return d.xpos[b] + d.xmat[b].reshape(3, 3) @ self._centre_local

    @property
    def palm_centre(self) -> np.ndarray:
        return self.data.geom_xpos[self.pm.palm_geom].copy()

    def _contacts(self):
        """Contact quantities between the arm, object and pedestal.

        Returns a dict: ``touch`` per-patch normal force on the object (18);
        ``plat`` total arm force on the pedestal; ``obj_plat`` whether the
        object rests on the pedestal; ``cmap`` (6 hand units x 6 object
        regions) normal force; ``wrench`` net force/torque the arm applies
        to the object (object frame, about its centre); ``unit_dir`` summed
        unit contact normals per hand unit (object frame) for opposition.
        """
        m, d, pm = self.model, self.data, self.pm
        touch = np.zeros(18)
        cmap = np.zeros((6, 6))
        unit_dir = np.zeros((6, 3))
        force = np.zeros(3)
        torque = np.zeros(3)
        plat = 0.0
        obj_plat = False
        f6 = np.zeros(6)
        obj_body = pm.object_body
        plat_body = m.geom_bodyid[pm.platform_geom]
        b = obj_body
        R = d.xmat[b].reshape(3, 3)
        oc = self.object_centre
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = c.geom1, c.geom2
            b1, b2 = m.geom_bodyid[g1], m.geom_bodyid[g2]
            a1, a2 = pm.arm_geom_mask[g1], pm.arm_geom_mask[g2]
            if a1 == a2:
                if (b1 == obj_body and b2 == plat_body) or (
                        b2 == obj_body and b1 == plat_body):
                    obj_plat = True
                continue
            arm_g, other = (g1, g2) if a1 else (g2, g1)
            mujoco.mj_contactForce(m, d, i, f6)
            fn = abs(f6[0])
            ob = m.geom_bodyid[other]
            if ob == obj_body:
                p = pm.geom_patch[arm_g]
                if p >= 0:
                    touch[p] += fn
                frame = c.frame.reshape(3, 3)
                fw = frame.T @ f6[:3]
                # The force acts on geom2 along +normal (geom1 -> geom2).
                f_obj = fw if other == g2 else -fw
                force += f_obj
                r = c.pos - oc
                torque += np.cross(r, f_obj)
                if 0 <= p < 16:
                    unit = 0 if p == 0 else 1 + (p - 1) // 3
                    local = R.T @ r
                    ax = int(np.argmax(np.abs(local / np.maximum(
                        self._extent / 2, 1e-3))))
                    region = 2 * ax + int(local[ax] > 0)
                    cmap[unit, region] += fn
                    nrm = R.T @ (frame[0] if other == g2 else -frame[0])
                    unit_dir[unit] += nrm * min(fn, 1.0)
            elif other == pm.platform_geom:
                plat += fn
        return {"touch": touch, "plat": plat, "obj_plat": obj_plat,
                "cmap": cmap, "wrench": np.r_[R.T @ force, R.T @ torque],
                "unit_dir": unit_dir}

    def _drot(self, quat):
        """Rotation since spawn as a world-frame rotation vector (rad)."""
        q = np.zeros(4)
        inv = self.spawn_quat * np.array([1, -1, -1, -1])
        mujoco.mju_mulQuat(q, quat, inv)
        v = np.zeros(3)
        mujoco.mju_quat2Vel(v, q, 1.0)
        return v

    def _surface_dists(self):
        """Distance from the palm and 5 fingertip geoms to the object."""
        m, d, pm = self.model, self.data, self.pm
        out = np.full(6, 0.1)
        ft = np.zeros(6)
        geoms = pm.shape_geoms[self.shape_index]
        for j, g in enumerate([pm.palm_geom, *pm.tip_geoms]):
            for og in geoms:
                dist = mujoco.mj_geomDistance(m, d, g, og, 0.1, ft)
                out[j] = min(out[j], dist)
        return np.maximum(out, 0.0)

    def _obs_and_info(self):
        d, pm = self.data, self.pm
        con = self._contacts()
        touch, plat, obj_plat = con["touch"], con["plat"], con["obj_plat"]
        self._last_cmap = np.tanh(con["cmap"] / 0.2)
        palm = self.palm_centre
        tips = d.geom_xpos[pm.tip_geoms]
        oc = self.object_centre
        b = pm.object_body
        R = d.xmat[b].reshape(3, 3)
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, d, mujoco.mjtObj.mjOBJ_BODY, b,
                                 vel, 0)
        q = d.qpos[pm.qadr]
        mid, half = (self.hi + self.lo) / 2, (self.hi - self.lo) / 2
        sd = self._surface_dists()
        wrench = con["wrench"]
        parts = {
            "qpos": (q - mid) / half,
            "qvel": d.qvel[pm.dofadr] / 10.0,
            "target": (self.target - mid) / half,
            "palm_pos": (palm - pm.shoulder) * 5,
            "tips": ((tips - palm) * 10).ravel(),
            "touch": np.log1p(touch / PATCH_SCALE),
            "touch_platform": [np.log1p(plat / PATCH_SCALE)],
            "contact_map": np.tanh(con["cmap"] / 0.2).ravel(),
            "wrench": np.r_[np.tanh(wrench[:3] / 0.5),
                            np.tanh(wrench[3:] / 0.01)],
            "surf_dist": sd * 10,
            "obj_rel": (oc - palm) * 5,
            "obj_pos": (oc - self.spawn_centre) * 5,
            "obj_rot": R[:, :2].T.ravel(),
            "obj_drot": self._drot(d.xquat[b]),
            "obj_vel": np.r_[vel[3:] * 2, vel[:3] * 0.2],
            "obj_size": self._extent * 10,
            "fixed": [float(self.fixed)],
            "time": [self.steps / self.max_steps],
        }
        parts["base_action"] = self._base_action
        obs = np.concatenate([np.asarray(parts[n], float).ravel()
                              for n, _ in self.parts]).astype(np.float32)
        on = touch > GRASP_FORCE
        thumb = on[FINGER_PATCHES["thumb"]].any()
        other = on[FINGER_PATCHES["others"]].any() or on[0]
        # Opposition: two hand units pushing on the object from opposite
        # sides (unit contact normals with a negative dot product).
        ud = con["unit_dir"]
        nrm = np.linalg.norm(ud, axis=1)
        act = np.flatnonzero(nrm > 0.02)
        opposed = False
        for i in act:
            for j in act:
                if i < j and ud[i] @ ud[j] / (nrm[i] * nrm[j]) < -0.3:
                    opposed = True
        disp = float(np.linalg.norm(oc - self.spawn_centre))
        lifted = (not self.fixed and not obj_plat
                  and oc[2] > self.spawn_centre[2] + LIFT_HEIGHT
                  and on[:16].any())
        info = {
            "touching": bool(on.any()),
            "hand_touching": bool(on[:16].any()),
            "n_touch": int(on.sum()),
            "pinch": bool(thumb and other),
            "grasp": bool(opposed),
            "lifted": bool(lifted),
            "obj_disp": disp,
            "obj_height": float(oc[2] - self.spawn_centre[2]),
            "obj_rot_angle": _quat_angle(d.xquat[b], self.spawn_quat),
            "palm_obj_dist": float(np.linalg.norm(oc - palm)),
            "min_surf_dist": float(sd.min()),
            "force": float(np.linalg.norm(wrench[:3])),
            "fixed": self.fixed,
            "shape": pm.shapes[self.shape_index].family,
            "fell": (not self.fixed and oc[2] < self.platform_top - 0.05),
        }
        return obs, info

    # --- gym API ------------------------------------------------------------

    def get_state(self):
        d, pm = self.data, self.pm
        return dict(
            shape=self.shape_index, rgba=self.model.geom_rgba[
                pm.shape_geoms[self.shape_index][0]].copy(),
            fixed=self.fixed, platform_top=self.platform_top,
            spawn_centre=self.spawn_centre.copy(),
            spawn_quat=self.spawn_quat.copy(), target=self.target.copy(),
            qpos=d.qpos.copy(), qvel=d.qvel.copy(), ctrl=d.ctrl.copy(),
            mocap_pos=d.mocap_pos.copy(), mocap_quat=d.mocap_quat.copy(),
            eq_active=d.eq_active.copy(),
            eq_data=self.model.eq_data[pm.weld].copy())

    def set_state(self, st):
        m, d, pm = self.model, self.data, self.pm
        mujoco.mj_resetData(m, d)
        self._set_shape(st["shape"], st["rgba"])
        self.fixed, self.platform_top = st["fixed"], st["platform_top"]
        self.spawn_centre = st["spawn_centre"].copy()
        self.spawn_quat = st["spawn_quat"].copy()
        self.target = st["target"].copy()
        for k in ("qpos", "qvel", "ctrl", "mocap_pos", "mocap_quat",
                  "eq_active"):
            getattr(d, k)[:] = st[k]
        m.eq_data[pm.weld] = st["eq_data"]
        mujoco.mj_forward(m, d)

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.restarted = False
        if self.demo_trials and self._demo_trajs is None:
            self._make_demo_archive()
        if self._archive and self.rng.random() < self.restart_p:
            self.set_state(self._archive[self.rng.integers(
                len(self._archive))])
            self.restarted = True
        elif self._demo_trajs and self.rng.random() < self.demo_restart_p:
            traj = self._demo_trajs[self.rng.integers(len(self._demo_trajs))]
            t0 = int(self.demo_tmin * (len(traj) - 1))
            self.set_state(traj[self.rng.integers(t0, len(traj))])
            self.restarted = True
        else:
            self._spawn()
        self.steps = 0
        self._reflex_on = False
        self._prev_exec = np.zeros(len(self.action_idx))
        obs, info = self._obs_and_info()
        if self.residual_scale > 0:
            from ..teach.scripted import ScriptedGrasp
            self._script = ScriptedGrasp(self)
            obs = self._with_base_action(obs)
        return obs, info

    def _with_base_action(self, obs):
        """Ask the script for its next action; put it in the obs."""
        a = np.clip(self.full_action(self._script(obs)), -1, 1)
        self._base_action = a[self.action_idx]
        obs = obs.copy()
        obs[self.observation_layout["base_action"]] = self._base_action
        return obs

    def _make_demo_archive(self):
        """Record ``demo_trials`` scripted trials on this env's own shapes
        and keep the state snapshots of the successful ones (free blocks:
        lifted; fixed blocks: opposed grasp). If fewer than 5 lifting
        trajectories exist, free trials with a grasp are kept too."""
        from ..teach.scripted import ScriptedGrasp
        lifts, grasps = [], []
        for _ in range(self.demo_trials):
            self._spawn()
            self.steps, self._prev_exec = 0, np.zeros(len(self.action_idx))
            ctrl = ScriptedGrasp(self)
            obs, _ = self._obs_and_info()
            traj, lifted, grasped = [], False, False
            for _ in range(self.max_steps):
                traj.append(self.get_state())
                obs, _, _, trunc, info = self.step(ctrl(obs))
                lifted |= info["lifted"]
                grasped |= info["grasp"]
                if trunc:
                    break
            if lifted or (self.fixed and grasped):
                lifts.append(traj)
            elif grasped:
                grasps.append(traj)
        self._demo_trajs = lifts + (grasps if len(lifts) < 5 else [])
        self.demo_stats = {"trials": self.demo_trials,
                           "kept": len(self._demo_trajs),
                           "lift_or_fixed_grasp": len(lifts)}

    def full_action(self, action) -> np.ndarray:
        """A per-joint (23) action from the policy's action. Controllers
        that act on all 23 joints (scripted, DEP) may pass 23 values; the
        coupled followers' entries are then ignored."""
        action = np.asarray(action, float)
        if len(action) == self.n_joints:
            return action
        full = np.zeros(self.n_joints)
        full[self.action_idx] = action
        return full

    def _apply_coupling(self):
        for f, lead, ratio in self.followers:
            self.target[f] = np.clip(ratio * self.target[lead],
                                     self.lo[f], self.hi[f])

    def step(self, action):
        action = np.asarray(action, float)
        if self.residual_scale > 0:
            res = self.residual_scale * np.clip(action, -1, 1)
            action = np.clip(self._base_action + res, -1, 1)
            res_ratio = (np.abs(res).mean()
                         / (np.abs(self._base_action).mean() + 1e-6))
        a = np.clip(self.full_action(action), -1, 1)
        # Smoothness signals on the executed action (action space).
        exe = a[self.action_idx]
        effort = float(np.mean(exe ** 2))
        jerk = float(np.mean((exe - self._prev_exec) ** 2))
        self._prev_exec = exe
        if self.action_mode == "delta":
            self.target = np.clip(self.target + a * self.max_speed * self.dt,
                                  self.lo, self.hi)
        else:
            mid, half = (self.hi + self.lo) / 2, (self.hi - self.lo) / 2
            self.target = mid + a * half
        if self.reflex > 0 and self._reflex_on:
            step = self.reflex * 3.0 * self.dt
            idx = self._flex_idx
            self.target[idx] = np.minimum(self.target[idx] + step,
                                          self.hi[idx] * 0.9)
            j = self._thumb_abd
            self.target[j] = min(self.target[j] + step, self.hi[j] * 0.9)
        self._apply_coupling()
        self.data.ctrl[self.pm.actuators] = self.target
        n_bad = self.data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number
        mujoco.mj_step(self.model, self.data, self.substeps)
        self.steps += 1
        obs, info = self._obs_and_info()
        # MuJoCo resets the state silently on a numerical blow-up (rare:
        # 3 times in version 0's ~20M steps); flag it and end the trial.
        unstable = (self.data.warning[mujoco.mjtWarning.mjWARN_BADQACC]
                    .number > n_bad)
        info["unstable"] = bool(unstable)
        info["effort"], info["jerk"] = effort, jerk
        if self.residual_scale > 0:
            info["res_ratio"] = float(res_ratio)
            obs = self._with_base_action(obs)
        units = (self._last_cmap > 0.2).any(1)
        self._reflex_on = bool(units[0] or units.sum() >= 2)
        if (self.restart_p > 0 and units.sum() >= 2 and not self.restarted
                and self.steps % 5 == 0 and not info["fell"]
                and not (self.restart_free_only and self.fixed)):
            if len(self._archive) >= 300:
                self._archive.pop(self.rng.integers(len(self._archive)))
            self._archive.append(self.get_state())
        truncated = self.steps >= self.max_steps or unstable
        return obs, 0.0, False, truncated, info

    def render(self, camera=None):
        if self._renderer is None:
            h, w = self.render_size
            self._renderer = mujoco.Renderer(self.model, h, w)
        self._renderer.update_scene(self.data, camera=camera or self.camera)
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
