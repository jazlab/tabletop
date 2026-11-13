"""Version 0 scripted controller, frozen (reproduces version_0 results).

The maintained controller is :mod:`ue.teach.scripted`. This copy is
unchanged apart from the class name ``ScriptedGraspV0``.

Original description: a scripted reach-grasp-lift controller (sanity
check and reference).

Top-down thumb-finger pinch. The monkey hand is small (fingers ~7 cm from
the palm origin; curled, they reach only ~3 cm below the palm), so a
whole-hand power grasp does not fit most blocks. Instead the abducted,
extended thumb hangs ~2.3 cm below the palm as a fixed post, and the
flexing index / middle fingers sweep the top part of the block against
it (the pinch axis is the palm frame's y, along the fingers).

Planning (at the first step of each trial, on a kinematic copy of the
model): for each candidate finger heading (yaw) and downward tilt of the
fingers (pitch; tilting turns the ~4.7 cm thumb-to-fingertip aperture
horizontal, for wider blocks), the width of the block's top part along the
heading is measured and the arm IK is solved (damped least squares from a
few seed postures) for the pinch pose: thumb tip just outside the near
face, ``grip_depth`` below the top. The narrowest heading that fits the
aperture and is reachable wins. Measured over 40 trials of
``PlayEnv(seed=3, p_fixed=0.0)``: ~60% lifted, ~68% opposed grasps, ~83%
pinches; misses are mostly blocks wider than the aperture at the top
(upright 5-6 cm cylinders, frusta) and pointed tops (pyramids) that slip
out upward.

Phases (joint-space moves; each ends when the servo targets reach their
goal and the joints have settled, or on a timeout):

0. fold: shoulder abduction and elbow flexion first, so the hanging hand
   rises laterally, clear of the pedestal (until the palm is above the
   block's top);
1. hover: ``hover`` m above the pinch pose, fingers straight, thumb
   abducted and extended (a fixed post);
2. descend: to the pinch pose;
3. close: flex the fingers (the arm holds) and squeeze for ``hold`` s;
4. lift: raise the pinch pose by ``lift`` m, slowly.

Outputs delta actions in [-1, 1] for :class:`~ue.env.PlayEnv`. Not used by
any learner: it shows the hand *can* grip and lift these blocks, and gives
reference values for the evaluation metrics.
"""

import mujoco
import numpy as np

PALM = "monkey/right/palm"


class ScriptedGraspV0:
    """Plan-then-execute top-down pinch controller.

    Args:
        env: the :class:`~ue.env.PlayEnv` to control.
        hover: approach height above the pinch pose (m).
        lift: lift height (m).
        top_extra: the block's width is measured over its top
            grip depth + ``top_extra`` (m).
        grip_depth: depth of the thumb tip below the block's top (m).
        floor_clear: minimum height of the thumb tip above the pedestal
            (m; reduces the grip depth on low blocks).
        margin: clearance between the thumb tip and the near face (m).
        lateral: sideways placement of the block's centre between the
            thumb tip (0) and the index/middle tips (1).
        yaws: candidate finger headings (deg, world).
        pitches: candidate downward tilts of the fingers (deg): tilting
            turns the thumb-to-fingertip aperture (~4.7 cm) horizontal,
            for wider blocks.
        open_flex: finger d0 flexion during the approach (rad).
        close_flex: (d0, d1, d2) finger flexion goals when closed, as
            fractions of the upper joint limit.
        fold_abduction: minimum shoulder abduction (rad) while folding up.
        fold_elbow: minimum elbow flexion (rad) while folding up.
        fold_speed: speed fraction of the other arm joints while folding.
        lift_speed: speed fraction of the arm joints while lifting.
        settle: joint error (rad) under which a phase counts as reached.
        timeouts: maximum duration (s) of each phase but the lift.
        hold: time (s) the closed fingers squeeze before lifting.
    """

    FOLD, HOVER, DESCEND, CLOSE, LIFT = range(5)

    def __init__(self, env, hover=0.04, lift=0.08, top_extra=0.005,
                 grip_depth=0.022, floor_clear=0.007, margin=0.004,
                 lateral=0.5, yaws=tuple(range(60, 161, 10)),
                 pitches=(0, 20, 35), open_flex=0.0,
                 close_flex=(1.0, 1.0, 0.8), fold_abduction=1.0,
                 fold_elbow=2.0, fold_speed=0.4, lift_speed=0.4,
                 settle=0.05, timeouts=(0.8, 0.8, 0.4, 0.6), hold=0.15):
        self.env = env
        self.hover, self.lift, self.top_extra = hover, lift, top_extra
        self.grip_depth, self.margin = grip_depth, margin
        self.floor_clear = floor_clear
        self.lateral = lateral
        self.yaws = np.radians(np.asarray(yaws, float))
        self.pitches = np.radians(np.asarray(pitches, float))
        self.open_flex, self.close_flex = open_flex, close_flex
        self.fold_abduction = fold_abduction
        self.fold_elbow, self.fold_speed = fold_elbow, fold_speed
        self.lift_speed = lift_speed
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
        self.plan = None

    # --- planning --------------------------------------------------------

    def _top_region(self, depth):
        """Points spanning the block's top ``depth`` m (world frame).

        For each convex part: its vertices above the slice plane plus the
        plane's crossings of all vertex pairs (a superset of the edges), so
        their convex hull is exactly the part above the plane.

        Returns:
            (points, z_top).
        """
        env, d = self.env, self.env.data
        b = env.pm.object_body
        R, x = d.xmat[b].reshape(3, 3), d.xpos[b]
        parts = [p @ R.T + x
                 for p in env.pm.shapes[env.shape_index].parts]
        z_top = max(p[:, 2].max() for p in parts)
        z0 = z_top - depth
        out = []
        for p in parts:
            hi, lo = p[p[:, 2] >= z0], p[p[:, 2] < z0]
            out.append(hi)
            if len(hi) and len(lo):
                a, c = hi[:, None], lo[None]
                t = (a[..., 2] - z0) / (a[..., 2] - c[..., 2])
                out.append((a + t[..., None] * (c - a)).reshape(-1, 3))
        return np.concatenate(out), z_top

    def _ik(self, pos, rot, q0, iters=150):
        """Arm joints putting the pinch point at ``pos``, palm at ``rot``.

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
            mujoco.mj_jac(m, d, self.jp, self.jr, p, self.palm_body)
            J = np.r_[self.jp[:, cols], 0.05 * self.jr[:, cols]]
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
            q = np.clip(q + dq, lo, hi)
        return q, float(np.linalg.norm(e))

    def _hand_points(self):
        """Thumb tip and index/middle tip (palm frame), hand open."""
        m, d, pm = self.env.model, self.kin, self.env.pm
        d.qpos[pm.qadr[7:]] = self._finger_goal(closed=False)
        mujoco.mj_kinematics(m, d)
        R = d.xmat[self.palm_body].reshape(3, 3)
        p = d.xpos[self.palm_body]
        tips = (d.geom_xpos[pm.tip_geoms] - p) @ R
        return tips[0], tips[1:3].mean(0)

    @staticmethod
    def _palm_rot(f, pitch):
        """Palm orientation: fingers along heading ``f`` tilted down by
        ``pitch`` (rad), palm facing down (and back)."""
        z = np.array([0, 0, 1.0])
        fy = -(np.cos(pitch) * f - np.sin(pitch) * z)   # palm y
        fz = np.cos(pitch) * z + np.sin(pitch) * f      # palm z
        return np.column_stack([np.cross(fy, fz), fy, fz])

    def _make_plan(self):
        """Choose a heading and tilt; solve hover / pinch / lift postures.

        The thumb tip goes ``grip_depth`` below the block's top, just
        outside its near face along the heading; the fingers then flex
        across the far face toward it.
        """
        # Grip depth: shallower on low blocks (the thumb must stay clear
        # of the pedestal).
        env = self.env
        z_top = self._top_region(0.0)[1]
        depth = self.grip_depth
        if env.platform_top is not None:
            depth = min(depth, z_top - env.platform_top - self.floor_clear)
        top, z_top = self._top_region(depth + self.top_extra)
        thumb, finger = self._hand_points()
        best = None
        for yaw in self.yaws:
            f = np.array([np.cos(yaw), np.sin(yaw), 0.0])
            g = np.array([-f[1], f[0], 0.0])
            pf, pg = top @ f, top @ g
            width = pf.max() - pf.min()
            centre = (f * (pf.max() + pf.min()) / 2
                      + g * (pg.max() + pg.min()) / 2)
            centre[2] = z_top
            for pitch in self.pitches:
                rot = self._palm_rot(f, pitch)
                aperture = rot @ (finger - thumb) @ f
                # Thumb tip to the block's top-centre (world), in the
                # palm frame: the palm-frame point placed at ``centre``.
                off = f * (width / 2 + self.margin) + [0, 0, depth]
                # Laterally, between the thumb and finger lines.
                gl = rot.T @ g
                self._pp = (thumb + gl * ((finger - thumb) @ gl)
                            * self.lateral
                            + rot.T @ off)
                sols = [self._ik(centre, rot, s) for s in self.seeds]
                q, err = min(sols, key=lambda s: s[1])
                score = (max(width + 2 * self.margin - aperture, 0)
                         + 0.2 * width + 0.01 * pitch
                         + 3 * max(err - 0.005, 0))
                if best is None or score < best[0]:
                    best = (score, yaw, pitch, width, centre, rot, q,
                            self._pp.copy(), err)
        _, yaw, pitch, width, centre, rot, q, pp, err = best
        self._pp = pp
        up = np.array([0, 0, 1.0])
        q_hover, _ = self._ik(centre + self.hover * up, rot, q)
        q_lift, _ = self._ik(centre + self.lift * up, rot, q)
        self.plan = {"yaw": yaw, "pitch": pitch, "width": width,
                     "centre": centre, "err": err,
                     "q": [q_hover, q_hover, q, q, q_lift]}
        self.phase, self.t_phase = self.FOLD, 0.0

    # --- execution -------------------------------------------------------

    def _finger_goal(self, closed):
        """Hand joint goals: thumb post, fingers slightly or fully flexed."""
        env = self.env
        goal = np.zeros(env.n_joints - 7)
        for i, n in enumerate(self.names[7:]):
            hi = env.hi[7 + i]
            if n == "thumb_d0_abduction":
                goal[i] = hi
            elif n == "thumb_d0_flexion":
                goal[i] = env.lo[7 + i]
            elif n.startswith("thumb"):
                goal[i] = 0.0
            else:
                k = int(n.split("_")[1][1])  # segment: d0, d1, d2
                if closed:
                    goal[i] = hi * self.close_flex[k]
                elif k == 0:
                    goal[i] = self.open_flex
        return goal

    def __call__(self, obs):
        """Delta action for the current step (plans at a trial's start)."""
        env = self.env
        if env.steps == 0 or self.plan is None:
            self._make_plan()
        dt, ph = env.dt, self.phase
        goal = np.r_[self.plan["q"][ph], self._finger_goal(ph >= self.CLOSE)]
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
        if ph == self.FOLD:
            reached = env.palm_centre[2] > self.plan["centre"][2] + 0.03
        elif ph == self.CLOSE:
            reached = at_goal and self.t_phase > self.hold
        if ph < self.LIFT and (reached or self.t_phase > self.timeouts[ph]):
            self.phase += 1
            self.t_phase = 0.0
        return a
