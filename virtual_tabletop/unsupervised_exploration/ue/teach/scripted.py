"""A scripted reach-grasp-lift controller (sanity check and reference).

Top-down thumb-finger pinch. The monkey hand is small (fingers ~7 cm from
the palm origin; curled, they reach only ~3 cm below the palm), so a
whole-hand power grasp does not fit most blocks. Instead the abducted,
extended thumb hangs ~2.3 cm below the palm as a fixed post, and the
fingertips sweep the block horizontally against it, at the thumb tip's
height (the pinch axis is the finger heading).

Finger sweep: a table of fingertip positions over the knuckle (MCP) and
middle-joint (PIP) flexion of each finger, with the end joint at 2/3 of
the middle joint (the v1 hand's DIP coupling; also used on v0), is built
once. During the approach the fingers are pre-curled so their tips are at
the thumb tip's height, beyond the block's far face; closing moves the
tips along a straight horizontal line toward and past the thumb (the
overshoot is the squeeze). The fingertips therefore never dig below the
thumb, which lets the pinch be low on flat blocks without the fingers
striking the pedestal. No joint is driven past its v1 range: the fingers
never hyperextend.

Planning (at the first step of each trial, on a kinematic model copy):
candidate grasps are a finger heading (yaw), a downward tilt of the
fingers (pitch; tilting widens the horizontal thumb-to-fingertip
aperture) and a grip depth below the block's top (bounded below by a
clearance above the pedestal). Each is scored on the block's geometry
alone: overflow of the aperture by the part of the block above the grip
(the thumb and fingertips must pass it on the way down), the width at the
grip, the slope of the gripped faces (faces that widen downward squeeze
the block out of the grip), the sideways offset of the centre of mass
from the pinch line, and a preference for gripping ~1.5 cm below the top.
The arm IK (damped least squares from a few seed postures, 60 iterations)
is solved once per (heading, tilt) to penalise unreachable poses (the
reachable headings depend strongly on the block's position), then
exactly for the winner. Descent and lift follow straight vertical
waypoint paths (a joint-space move between distant postures swings the
thumb tip sideways onto the block).

Measured (``PlayEnv(hand="v1", shapes="v1")``, seeds 100-102 x 30 free
trials): lifted at some step 81%, still lifted at the last step 74%,
opposed grasp 88%, pinch 99%, fell 1% (the earlier version,
``scripted_v0.py``: 68% / 56% / 79% / 82% / 7%). Fixed blocks
(``p_fixed=1``, seeds 100-101 x 30): opposed grasp 85%, pinch 92%.
Remaining failures: steep-faced pointed blocks (pyramids, irregular
hulls) squeezed out of the pinch during the lift, and blocks wider than
the ~4.5 cm aperture across every reachable heading.

Phases (joint-space moves; each ends when the servo targets reach their
goal and the joints have settled, or on a timeout):

0. fold: shoulder abduction and elbow flexion first, so the hanging hand
   rises laterally, clear of the pedestal (until the palm is above the
   block's top);
1. hover: ``hover`` m above the pinch pose, thumb abducted and extended
   (a fixed post), fingertips pre-curled at its height;
2. descend: to the pinch pose;
3. close: sweep the fingertips toward the thumb, then squeeze for
   ``hold`` s;
4. lift: raise the pinch pose by ``lift`` m, slowly; the fingers keep
   squeezing.

Outputs delta actions in [-1, 1] for :class:`~ue.sim.env.PlayEnv` (23
values; the env ignores coupled joints). Not used by any learner: it
shows the hand *can* grip and lift these blocks, and gives reference
values for the evaluation metrics. ``ue/teach/scripted_v0.py`` is the
earlier version (fingers flexed at fixed joint goals, one grip depth).
"""

import mujoco
import numpy as np

PALM = "monkey/right/palm"
DIGITS = ("index", "middle", "ring", "pinky")
DIP_RATIO = 2.0 / 3.0


class ScriptedGrasp:
    """Plan-then-execute top-down pinch controller.

    Args:
        env: the :class:`~ue.sim.env.PlayEnv` to control.
        hover: approach height above the pinch pose (m).
        lift: lift height (m).
        depths: candidate grip depths below the block's top (m).
        best_depth: preferred grip depth (m).
        floor_clear: minimum height of the thumb and fingertip segment
            centres above the pedestal (m; the tips reach ~3 mm lower).
        margin: clearance between the thumb / fingertips and the block
            during the approach (m).
        open_extra: extra opening of the fingertips beyond the far face
            during the approach (m).
        overshoot: closed fingertip goal past the thumb tip (m; sets the
            squeeze).
        sweep_dz: height of the fingertip sweep relative to the thumb tip
            (m).
        yaws: candidate finger headings (deg, world).
        pitches: candidate downward tilts of the fingers (deg).
        w_width: score weight of the block's width at the grip.
        ik_tol: IK residual above which a candidate is penalised.
        fold_clear: the fold ends when the palm is this high above the
            block's top (m).
        fold_abduction: minimum shoulder abduction (rad) while folding up.
        fold_elbow: minimum elbow flexion (rad) while folding up.
        fold_speed: speed fraction of the other arm joints while folding.
        lift_speed: speed fraction of the arm joints while lifting.
        close_time: duration of the fingertip sweep (s).
        settle: joint error (rad) under which a phase counts as reached.
        timeouts: maximum duration (s) of each phase but the lift.
        hold: time (s) the closed fingers squeeze before lifting.
    """

    FOLD, HOVER, DESCEND, CLOSE, LIFT = range(5)

    def __init__(self, env, hover=0.035, lift=0.08,
                 depths=(0.008, 0.012, 0.016, 0.02),
                 best_depth=0.016, floor_clear=0.004, margin=0.004,
                 open_extra=0.006, overshoot=0.012, sweep_dz=-0.003,
                 yaws=tuple(range(30, 191, 10)), pitches=(0, 10, 20, 30),
                 ik_tol=0.003, w_width=0.2, fold_clear=0.04,
                 fold_abduction=1.0, fold_elbow=2.0,
                 fold_speed=0.4, lift_speed=0.4, close_time=0.3,
                 settle=0.05, timeouts=(0.8, 0.6, 0.4, 0.7), hold=0.15):
        self.env = env
        self.hover, self.lift = hover, lift
        self.depths, self.best_depth = depths, best_depth
        self.floor_clear, self.margin = floor_clear, margin
        self.open_extra, self.overshoot = open_extra, overshoot
        self.sweep_dz = sweep_dz
        self.yaws = np.radians(np.asarray(yaws, float))
        self.pitches = np.radians(np.asarray(pitches, float))
        self.ik_tol, self.w_width = ik_tol, w_width
        self.fold_clear, self.fold_abduction = fold_clear, fold_abduction
        self.fold_elbow, self.fold_speed = fold_elbow, fold_speed
        self.lift_speed, self.close_time = lift_speed, close_time
        self.settle, self.timeouts, self.hold = settle, timeouts, hold
        self.names = [n.split("/")[-1] for n in env.pm.joint_names]
        m = env.model
        self.palm_body = m.body(PALM).id
        self.kin = mujoco.MjData(m)  # kinematic scratch copy for IK
        self.jp = np.zeros((3, m.nv))
        self.jr = np.zeros((3, m.nv))
        # Seeds: forward reaching postures (the hanging arm is a backward
        # local minimum of the IK).
        self.seeds = np.array([
            [0.56, 0.25, 0.2, 1.66, 0.41, 0.0, 0.0],
            [0.8, 0.0, 0.3, 1.8, 0.9, 0.0, 0.0],
            [0.0, -0.25, 0.0, 1.9, 1.6, 0.2, -0.5],
            [1.2, 0.2, 0.5, 1.6, 0.3, -0.3, 0.3],
        ])
        self._build_hand_tables()
        self.plan = None

    # --- hand kinematics -------------------------------------------------

    def _thumb_goal(self):
        """Hand joint goals with the thumb as a post, fingers straight."""
        env = self.env
        goal = np.zeros(env.n_joints - 7)
        for i, n in enumerate(self.names[7:]):
            if n == "thumb_d0_abduction":
                goal[i] = env.hi[7 + i]
            elif n == "thumb_d0_flexion":
                goal[i] = env.lo[7 + i]
        return goal

    def _build_hand_tables(self):
        """Fingertip positions (palm frame) over MCP x PIP flexion."""
        env, m, d, pm = self.env, self.env.model, self.kin, self.env.pm
        idx = {n: 7 + i for i, n in enumerate(self.names[7:])}
        self.jidx = np.array([[idx[f"{f}_d{k}_flexion"] for k in range(3)]
                              for f in DIGITS])
        j0, j1 = self.jidx[0, 0], self.jidx[0, 1]
        a = np.linspace(max(env.lo[j0], 0.0), env.hi[j0], 37)
        b = np.linspace(max(env.lo[j1], 0.0), env.hi[j1], 25)
        aa, bb = [x.ravel() for x in np.meshgrid(a, b, indexing="ij")]
        self.grid = np.stack([aa, bb], 1)
        hand = self._thumb_goal()
        tips = np.zeros((len(aa), 5, 3))
        for k, (x, y) in enumerate(self.grid):
            q = hand.copy()
            for f in range(4):
                j = self.jidx[f] - 7
                q[j] = [x, y, np.clip(DIP_RATIO * y, env.lo[j[2] + 7],
                                      env.hi[j[2] + 7])]
            d.qpos[pm.qadr[7:]] = q
            mujoco.mj_kinematics(m, d)
            R = d.xmat[self.palm_body].reshape(3, 3)
            tips[k] = (d.geom_xpos[pm.tip_geoms] - d.xpos[self.palm_body]) @ R
        self.thumb_tip = tips[0, 0]
        self.finger_tips = tips[:, 1:]   # (grid, 4 digits, 3)

    @staticmethod
    def _hv(p, pitch):
        """Palm-frame points -> (along heading, up) at a finger tilt."""
        c, s = np.cos(pitch), np.sin(pitch)
        y, z = p[..., 1], p[..., 2]
        return np.stack([-c * y + s * z, s * y + c * z], -1)

    def _aperture(self, pitch):
        """Horizontal reach of the index/middle tips past the thumb tip, at
        the thumb tip's height."""
        th = self._hv(self.thumb_tip, pitch)
        hv = self._hv(self.finger_tips[:, :2], pitch)
        ok = np.abs(hv[..., 1] - th[1] - self.sweep_dz) < 0.003
        reach = np.where(ok, hv[..., 0], -np.inf).max(0)
        return float(reach.min() - th[0])

    def _finger_pose(self, h, pitch):
        """Finger joint goals putting each fingertip at ``h`` m along the
        heading from the thumb tip, at the sweep height."""
        th = self._hv(self.thumb_tip, pitch)
        tgt = np.array([th[0] + h, th[1] + self.sweep_dz])
        hv = self._hv(self.finger_tips, pitch)        # (grid, 4, 2)
        k = np.argmin(((hv - tgt) ** 2).sum(-1), 0)   # per digit
        return self.grid[k]                           # (4, 2)

    def _hand_goal(self, s):
        """Hand joint goals at sweep progress ``s`` (0 open, 1 closed)."""
        p = self.plan
        h = p["h_open"] + s * (p["h_close"] - p["h_open"])
        goal = self._thumb_goal()
        env = self.env
        for f, (x, y) in enumerate(self._finger_pose(h, p["pitch"])):
            j = self.jidx[f]
            goal[j - 7] = [x, y, np.clip(DIP_RATIO * y, env.lo[j[2]],
                                         env.hi[j[2]])]
        return goal

    # --- block geometry --------------------------------------------------

    def _parts(self):
        env, d = self.env, self.env.data
        b = env.pm.object_body
        R, x = d.xmat[b].reshape(3, 3), d.xpos[b]
        return [p @ R.T + x for p in env.pm.shapes[env.shape_index].parts]

    @staticmethod
    def _above(parts, z0):
        """Points spanning the parts above height ``z0``: vertices above
        plus the plane's crossings of all vertex pairs (their convex hull
        per part is exactly the part above the plane)."""
        out = []
        for p in parts:
            hi, lo = p[p[:, 2] >= z0], p[p[:, 2] < z0]
            out.append(hi)
            if len(hi) and len(lo):
                a, c = hi[:, None], lo[None]
                t = (a[..., 2] - z0) / (a[..., 2] - c[..., 2])
                out.append((a + t[..., None] * (c - a)).reshape(-1, 3))
        return np.concatenate(out)

    @staticmethod
    def _section(parts, z0):
        """Points whose hull is the block's cross-section at ``z0``."""
        out = []
        for p in parts:
            hi, lo = p[p[:, 2] >= z0], p[p[:, 2] < z0]
            if len(hi) and len(lo):
                a, c = hi[:, None], lo[None]
                t = (a[..., 2] - z0) / (a[..., 2] - c[..., 2])
                out.append((a + t[..., None] * (c - a)).reshape(-1, 3))
        return np.concatenate(out) if out else np.zeros((0, 3))

    # --- planning --------------------------------------------------------

    def _ik(self, pos, rot, q0, iters=150):
        """Arm joints putting the thumb tip at ``pos``, palm at ``rot``.

        Args:
            pos: world target of the palm-frame point ``self._pp``.
            rot: 3x3 target palm orientation (columns = palm axes).
            q0: seed for the 7 arm joints.
            iters: damped least-squares iterations.

        Returns:
            (q, error): joints and a combined position (m) and orientation
            (rad, scaled by 0.05 m) residual.
        """
        env, m, d, pm = self.env, self.env.model, self.kin, self.env.pm
        lo, hi = env.lo[:7], env.hi[:7]
        cols = pm.dofadr[:7]
        q = q0.copy()
        for _ in range(iters):
            d.qpos[pm.qadr[:7]] = q
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            R = d.xmat[self.palm_body].reshape(3, 3)
            p = d.xpos[self.palm_body] + R @ self._pp
            r_err = 0.5 * sum(np.cross(R[:, i], rot[:, i]) for i in range(3))
            e = np.r_[pos - p, 0.05 * r_err]
            if e @ e < 1e-10:
                break
            mujoco.mj_jac(m, d, self.jp, self.jr, p, self.palm_body)
            J = np.r_[self.jp[:, cols], 0.05 * self.jr[:, cols]]
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
            q = np.clip(q + dq, lo, hi)
        return q, float(np.linalg.norm(e))

    @staticmethod
    def _palm_rot(f, pitch):
        """Palm orientation: fingers along heading ``f`` tilted down by
        ``pitch`` (rad), palm facing down (and back)."""
        z = np.array([0, 0, 1.0])
        fy = -(np.cos(pitch) * f - np.sin(pitch) * z)   # palm y
        fz = np.cos(pitch) * z + np.sin(pitch) * f      # palm z
        return np.column_stack([np.cross(fy, fz), fy, fz])

    def _candidates(self):
        """Geometric grasp candidates, best first.

        Returns:
            list of (score, dict(yaw, pitch, depth, thumb, width)).
        """
        env = self.env
        parts = self._parts()
        z_top = max(p[:, 2].max() for p in parts)
        z_bot = min(p[:, 2].min() for p in parts)
        z_min = (env.platform_top if env.platform_top is not None
                 else z_bot) + self.floor_clear - min(self.sweep_dz, 0.0)
        com = env.data.xipos[env.pm.object_body]
        depths = sorted({round(min(dp, z_top - z_min), 4)
                         for dp in self.depths})
        depths = [dp for dp in depths if dp > 0.002] or [
            max((z_top - z_bot) / 2, 0.002)]
        aps = [self._aperture(p) for p in self.pitches]
        dz = 0.003
        out = []
        for depth in depths:
            zg = z_top - depth
            top = self._above(parts, zg)
            sec = self._section(parts, zg)
            lo, hi = self._section(parts, zg - dz), self._section(parts,
                                                                  zg + dz)
            if not len(sec):
                continue
            if not len(lo):
                lo = sec
            if not len(hi):
                hi = sec
            for yaw in self.yaws:
                f = np.array([np.cos(yaw), np.sin(yaw), 0.0])
                g = np.array([-f[1], f[0], 0.0])
                pf = top @ f
                width_top = pf.max() - pf.min()
                sf = sec @ f
                width = sf.max() - sf.min()
                # Face slope: positive when the block widens downward.
                slope = ((np.ptp(lo @ f) - np.ptp(hi @ f)) / (4 * dz))
                sg = sec @ g
                gc = np.clip(com @ g, sg.min() + 0.004, sg.max() - 0.004) \
                    if np.ptp(sg) > 0.008 else (sg.max() + sg.min()) / 2
                off_g = abs(com @ g - gc)
                thumb = (f * (pf.min() - self.margin) + g * gc
                         + np.array([0, 0, zg]))
                for pitch, ap in zip(self.pitches, aps):
                    over = max(width_top + 2 * self.margin - ap, 0.0)
                    score = (4 * over + self.w_width * width
                             + 0.01 * max(slope - 0.2, 0.0)
                             + 0.3 * off_g
                             + 0.3 * abs(depth - self.best_depth)
                             + 0.01 * pitch)
                    out.append((score, dict(
                        yaw=yaw, pitch=pitch, depth=depth, thumb=thumb,
                        width=width, width_top=width_top, slope=slope,
                        aperture=ap, f=f, z_top=z_top)))
        out.sort(key=lambda c: c[0])
        return out

    def _make_plan(self):
        """Choose a grasp; solve hover / pinch / lift postures."""
        cands = self._candidates()
        self._pp = self.thumb_tip
        # Reachability per (heading, tilt): IK at the best candidate's
        # thumb position (the depth changes it little).
        reach = {}
        for _, c in cands:
            key = (c["yaw"], c["pitch"])
            if key not in reach:
                rot = self._palm_rot(c["f"], c["pitch"])
                reach[key] = min((self._ik(c["thumb"], rot, s, iters=60)
                                  for s in self.seeds), key=lambda s: s[1])
        best = None
        for score, c in cands:
            q0, err = reach[(c["yaw"], c["pitch"])]
            total = score + 3 * max(err - self.ik_tol, 0)
            if best is None or total < best[0]:
                best = (total, c, q0)
        _, c, q0 = best
        rot = self._palm_rot(c["f"], c["pitch"])
        q, err = self._ik(c["thumb"], rot, q0)
        up = np.array([0, 0, 1.0])
        q_hover, _ = self._ik(c["thumb"] + self.hover * up, rot, q)
        # Straight vertical paths (joint-space moves between distant
        # postures swing the thumb tip sideways onto the block).
        down, qk = [], q_hover
        for k in range(1, 5):
            qk, _ = self._ik(c["thumb"] + self.hover * (1 - k / 4) * up,
                             rot, qk, iters=40)
            down.append(qk)
        down[-1] = q
        lift, qk = [], q
        for k in range(1, 9):
            qk, _ = self._ik(c["thumb"] + self.lift * k / 8 * up, rot, qk,
                             iters=40)
            lift.append(qk)
        self.plan = dict(c, err=err, centre=c["thumb"],
                         h_open=c["width_top"] + 2 * self.margin
                         + self.open_extra,
                         h_close=-self.overshoot,
                         q=[[q_hover], [q_hover], down, [q], lift])
        self.phase, self.t_phase, self.s = self.FOLD, 0.0, 0.0
        self.wp = 0

    # --- execution -------------------------------------------------------

    def __call__(self, obs):
        """Delta action for the current step (plans at a trial's start)."""
        env = self.env
        if env.steps == 0 or self.plan is None:
            self._make_plan()
        dt, ph = env.dt, self.phase
        if ph >= self.CLOSE:
            self.s = min(1.0, self.s + dt / self.close_time)
        hand = (self._thumb_goal() if ph == self.FOLD
                else self._hand_goal(self.s))
        path = self.plan["q"][ph]
        goal = np.r_[path[min(self.wp, len(path) - 1)], hand]
        speed = np.ones(env.n_joints)
        if ph == self.FOLD:
            # Abduct and flex the elbow first (the hand rises laterally,
            # clear of the pedestal), the other arm joints slowly toward
            # the hover posture.
            goal[0] = max(goal[0], self.fold_abduction)
            goal[3] = max(goal[3], self.fold_elbow)
            speed[:7] = self.fold_speed
            speed[[0, 3]] = 1.0
        elif ph == self.LIFT:
            speed[:7] = self.lift_speed
        a = np.clip((goal - env.target) / (env.max_speed * dt), -speed,
                    speed)
        # Phase transitions: targets at the goal and joints settled, or a
        # timeout.
        self.t_phase += dt
        q = env.data.qpos[env.pm.qadr]
        at_goal = np.abs(goal - env.target).max() < 1e-3
        reached = at_goal and np.abs(goal - q)[:7].max() < self.settle
        if np.abs(goal - env.target)[:7].max() < 0.02:
            self.wp += 1   # next waypoint of a straight path
        reached = reached and self.wp >= len(path)
        if ph == self.FOLD:
            reached = env.palm_centre[2] > self.plan["z_top"] + self.fold_clear
        elif ph == self.CLOSE:
            reached = (self.s >= 1.0 and at_goal
                       and self.t_phase > self.close_time + self.hold)
        if ph < self.LIFT and (reached or self.t_phase > self.timeouts[ph]):
            self.phase += 1
            self.t_phase, self.wp = 0.0, 0
        return a
