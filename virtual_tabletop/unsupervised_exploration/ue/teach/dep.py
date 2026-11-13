"""DEP: differential extrinsic plasticity (Der & Martius 2015; DEP-RL,
Schumacher et al. 2023), a self-organising controller with no reward and
no networks.

Sensors x = joint angles scaled to [-1, 1]; motor outputs y = servo targets
in [-1, 1] (converted to the env's delta actions). Per block (arm 7 joints,
fingers 16):

    y_t = tanh(kappa * C~ x_t + h)
    C  <- C + (1/tau) * (dx_t dx_{t-theta}^T - C)       (dx = x_t - x_{t-1})
    C~ = C / (||C|| + eps)                               (normalised)
    h  <- h - eps_h * y_t                                (bias decay)

Correlated joint velocities strengthen the corresponding sensor-motor
couplings, so small movements are amplified into coherent whole-limb
oscillations and swings that change as the body meets objects. C persists
across trials (it is the controller's "memory").
"""

import numpy as np


class DEPBlock:
    def __init__(self, n, kappa=2.0, tau=20.0, theta=1, eps_h=0.002,
                 rng=None):
        self.kappa, self.tau, self.theta, self.eps_h = (kappa, tau, theta,
                                                        eps_h)
        rng = rng or np.random.default_rng(0)
        self.C = rng.normal(0, 0.01, (n, n))
        self.h = np.zeros(n)
        self.hist = []

    def step(self, x):
        self.hist.append(x.copy())
        if len(self.hist) > self.theta + 2:
            self.hist.pop(0)
        if len(self.hist) >= self.theta + 2:
            dx = self.hist[-1] - self.hist[-2]
            dx_old = self.hist[-1 - self.theta] - self.hist[-2 - self.theta]
            self.C += (np.outer(dx, dx_old) - self.C) / self.tau
        Cn = self.C / (np.linalg.norm(self.C) + 1e-12)
        y = np.tanh(self.kappa * Cn @ x + self.h)
        self.h -= self.eps_h * y
        return y

    def reset_history(self):
        self.hist = []


class DEPController:
    """DEP on the arm and finger joints of :class:`ue.sim.env.PlayEnv`."""

    def __init__(self, env, kappa_arm=2.0, kappa_fingers=1.5, tau=20.0,
                 theta=1, seed=0):
        self.env = env
        rng = np.random.default_rng(seed)
        self.arm = DEPBlock(7, kappa_arm, tau, theta, rng=rng)
        self.fingers = DEPBlock(env.n_joints - 7, kappa_fingers, tau, theta,
                                rng=rng)
        self._last_steps = None

    def __call__(self, obs):
        env = self.env
        if self._last_steps is None or env.steps < self._last_steps:
            self.arm.reset_history()
            self.fingers.reset_history()
        self._last_steps = env.steps
        x = obs[env.observation_layout["qpos"]].astype(float)
        y = np.r_[self.arm.step(x[:7]), self.fingers.step(x[7:])]
        mid, half = (env.hi + env.lo) / 2, (env.hi - env.lo) / 2
        target = mid + y * half
        return np.clip((target - env.target) / (env.max_speed * env.dt),
                       -1, 1)


class VecDEP:
    """DEP for ``n`` envs computed from observations only (for DEP-RL
    switching inside a learner): joint angles (``qpos``) are the sensors,
    current servo targets (``target``) convert outputs to delta actions.
    """

    def __init__(self, n, layout, lo, hi, max_speed, dt, kappa_arm=2.0,
                 kappa_fingers=1.5, tau=20.0, seed=0):
        rng = np.random.default_rng(seed)
        self.blocks = [(DEPBlock(7, kappa_arm, tau, rng=rng),
                        DEPBlock(len(lo) - 7, kappa_fingers, tau, rng=rng))
                       for _ in range(n)]
        self.q, self.t = layout["qpos"], layout["target"]
        self.half = (hi - lo) / 2
        self.scale = max_speed * dt

    def reset(self, i):
        for b in self.blocks[i]:
            b.reset_history()

    def act(self, i, obs):
        x = obs[self.q].astype(float)
        arm, fing = self.blocks[i]
        y = np.r_[arm.step(x[:7]), fing.step(x[7:])]
        tgt = obs[self.t]
        return np.clip((y - tgt) * self.half / self.scale, -1, 1)
