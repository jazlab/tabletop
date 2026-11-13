"""Modular intrinsically motivated goal exploration (IMGEP + HER)."""


import numpy as np
import torch

from .base import RewardModule


class GoalBabbling(RewardModule):
    """Modular intrinsically motivated goal exploration (IMGEP) + HER.

    Self-generated goals in four goal spaces ("modules", Forestier et al.
    2022), each a slice of the observation:

    - ``reach``:   palm-to-object vector (obj_rel; success within 2.5 cm);
    - ``contact``: which hand units (palm, thumb, index, middle, ring,
      pinky) touch the object (success: exact pattern);
    - ``move``:    object displacement since spawn (free objects only;
      success within 2 cm);
    - ``turn``:    object rotation since spawn (rotation vector; success
      within 0.25 rad, free objects only).

    At each trial start a module is chosen with probability proportional to
    its absolute learning progress (change in success rate between the last
    two windows of ``lp_window`` attempts; 20% uniform), and a goal is drawn
    from that module's archive of achieved outcomes, preferring rare ones
    (of ``n_cand`` random archive points, the one farthest from its
    nearest neighbours; a cheap Skew-Fit). The agent is conditioned on
    ``z = [one-hot module, goal slots]``.

    Reward: ``1[success] - 0.1 * min(d / eps, 10)`` in the active module's
    space. Hindsight: a fraction ``her`` of sampled transitions get their
    goal replaced by an outcome achieved later in the same trial.

    No task reward: goals are the agent's own past outcomes.
    """

    MODULES = ("reach", "contact", "move", "turn")

    def __init__(self, spec, her=0.7, lp_window=100, n_cand=16,
                 archive=20000, p_uniform=0.2, reach_radius=None, **kw):
        kw.setdefault("normalize", False)
        super().__init__(spec, **kw)
        lay = spec["layout"]
        self.lay = lay
        self.dims = {"reach": 3, "contact": 6, "move": 3, "turn": 3}
        self.eps = {"reach": 0.125, "contact": 0.5, "move": 0.1,
                    "turn": 0.25}  # in observation units
        self.slot = {}
        i = len(self.MODULES)
        for m in self.MODULES:
            self.slot[m] = slice(i, i + self.dims[m])
            i += self.dims[m]
        self.z_dim = i
        self.her, self.lp_window, self.n_cand = her, lp_window, n_cand
        self.p_uniform = p_uniform
        # Reach goals (palm-to-object vectors) further than this from the
        # object are rescaled onto the sphere (obs units, x5 m). Without
        # it, rare-goal sampling favours far-away palm targets and contact
        # goals never become achievable.
        self.reach_radius = reach_radius
        self.cap = archive
        self.archive = {m: np.zeros((archive, self.dims[m]), np.float32)
                        for m in self.MODULES}
        self.n_arch = {m: 0 for m in self.MODULES}
        self.outcomes = {m: [] for m in self.MODULES}
        self._z_env = None

    # --- goal spaces --------------------------------------------------------

    def achieved(self, x, m):
        L = self.lay
        if m == "reach":
            return x[..., L["obj_rel"]]
        if m == "contact":
            cm = x[..., L["contact_map"]]
            cm = cm.reshape(*cm.shape[:-1], 6, 6)
            return (cm > 0.2).any(-1).astype(np.float32) if isinstance(
                cm, np.ndarray) else (cm > 0.2).any(-1).float()
        if m == "move":
            return x[..., L["obj_pos"]]
        return x[..., L["obj_drot"]]

    def _dist(self, a, g, m):
        if m == "contact":
            return (a - g).abs().sum(-1) if torch.is_tensor(a) else \
                np.abs(a - g).sum(-1)
        return (a - g).norm(dim=-1) if torch.is_tensor(a) else \
            np.linalg.norm(a - g, axis=-1)

    # --- goal selection ---------------------------------------------------

    def lp(self, m):
        o, w = self.outcomes[m], self.lp_window
        if len(o) < 2 * w:
            return 1.0  # optimistic until measured
        return abs(np.mean(o[-w:]) - np.mean(o[-2 * w:-w]))

    def _sample_goal(self, m):
        n = min(self.n_arch[m], self.cap)
        if n < 50:
            return None
        A = self.archive[m][:n]
        cand = A[np.random.randint(0, n, self.n_cand)]
        ref = A[np.random.randint(0, n, min(n, 2000))]
        d = np.linalg.norm(cand[:, None] - ref[None], axis=-1)
        d.sort(1)
        return cand[np.argmax(d[:, 1:6].mean(1))]

    def sample_z(self, n, obs=None):
        z = np.zeros((n, self.z_dim), np.float32)
        for i in range(n):
            fixed = obs is not None and obs[i, self.lay["fixed"].start] > 0.5
            mods = [m for m in self.MODULES
                    if not (fixed and m in ("move", "turn"))]
            if np.random.rand() < self.p_uniform:
                m = mods[np.random.randint(len(mods))]
            else:
                lp = np.array([self.lp(m) for m in mods]) + 1e-3
                m = mods[np.random.choice(len(mods), p=lp / lp.sum())]
            g = self._sample_goal(m)
            if g is not None and m == "reach" and self.reach_radius:
                n_ = np.linalg.norm(g)
                if n_ > self.reach_radius:
                    g = g * self.reach_radius / n_
            if g is None:  # archive empty: default goals
                g = np.zeros(self.dims[m], np.float32)
                if m == "contact":
                    g[:2] = 1  # palm + thumb
            z[i, self.MODULES.index(m)] = 1
            z[i, self.slot[m]] = g
        return z

    def observe(self, obs, next_obs, done, z=None):
        x = next_obs
        for m in self.MODULES:
            a = self.achieved(x, m)
            if m in ("move", "turn"):
                a = a[x[:, self.lay["fixed"].start] < 0.5]
            if m == "contact":
                a = a[a.any(-1)] if len(a) else a
            for v in a[::2]:
                k = self.n_arch[m] % self.cap
                self.archive[m][k] = v
                self.n_arch[m] += 1
        if z is not None and done.any():
            for i in np.flatnonzero(done):
                m = self.MODULES[int(np.argmax(z[i, :4]))]
                g = z[i, self.slot[m]]
                a = self.achieved(x[i], m)
                self.outcomes[m].append(
                    float(self._dist(a, g, m) < self.eps[m]))

    # --- reward and relabelling ------------------------------------------

    def raw_reward(self, obs, act, next_obs, z):
        r = torch.zeros(obs.shape[0])
        for j, m in enumerate(self.MODULES):
            sel = z[:, j] > 0.5
            if not sel.any():
                continue
            a = self.achieved(next_obs[sel], m)
            g = z[sel][:, self.slot[m]]
            d = self._dist(a, g, m)
            r[sel] = ((d < self.eps[m]).float()
                      - 0.1 * (d / self.eps[m]).clamp(max=10))
        return r

    def relabel(self, buf, t, e, z, horizon=80):
        """HER 'future': replace goals by later achieved outcomes."""
        B = len(t)
        sel = np.random.rand(B) < self.her
        if not sel.any():
            return z
        ts = (t[:, None] + np.arange(horizon)[None]) % buf.cap
        d = buf.done[ts, e[:, None]]
        end = np.where(d.any(1), d.argmax(1), horizon - 1)
        off = (np.random.rand(B) * (end + 1)).astype(int)
        tf = (t + off) % buf.cap
        fut = buf.next_obs[tf, e]
        z = z.clone()
        for i in np.flatnonzero(sel):
            m = self.MODULES[int(z[i, :4].argmax())]
            z[i, self.slot[m]] = torch.as_tensor(self.achieved(fut[i], m))
        return z

    def stats(self):
        out = {}
        for m in self.MODULES:
            o = self.outcomes[m]
            out[f"gb_{m}_succ"] = round(float(np.mean(o[-100:])), 3) if o \
                else 0.0
            out[f"gb_{m}_n"] = len(o)
            out[f"gb_{m}_lp"] = round(self.lp(m), 3)
        return out
