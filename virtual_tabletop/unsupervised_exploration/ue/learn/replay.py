"""Replay ring buffer of shape [T, n_envs, ...] (n-step chains are
contiguous and cut at trial boundaries)."""

import numpy as np


class Replay:
    def __init__(self, capacity, n_envs, obs_dim, act_dim, z_dim, info_dim):
        self.cap, self.n_envs = capacity, n_envs
        self.obs = np.zeros((capacity, n_envs, obs_dim), np.float32)
        self.next_obs = np.zeros((capacity, n_envs, obs_dim), np.float32)
        self.act = np.zeros((capacity, n_envs, act_dim), np.float32)
        self.z = np.zeros((capacity, n_envs, max(z_dim, 1)), np.float32)
        self.done = np.zeros((capacity, n_envs), bool)
        self.info = np.zeros((capacity, n_envs, info_dim), np.float32)
        self.t, self.full = 0, False

    def add(self, obs, act, next_obs, done, z, info):
        t = self.t
        self.obs[t], self.act[t], self.next_obs[t] = obs, act, next_obs
        self.done[t], self.info[t] = done, info
        if z is not None:
            self.z[t] = z
        self.t = (t + 1) % self.cap
        self.full = self.full or self.t == 0

    @property
    def size(self):
        return (self.cap if self.full else self.t) * self.n_envs

    def sample_idx(self, batch, n_step):
        hi = self.cap if self.full else self.t
        # Avoid chains that cross the write head.
        t = np.random.randint(0, hi - n_step, batch)
        if self.full:
            t = (self.t + t) % self.cap
        e = np.random.randint(0, self.n_envs, batch)
        return t, e

    def chain(self, t, e, n_step):
        """Indices of each sample's n-step chain and a validity mask."""
        ts = (t[:, None] + np.arange(n_step)[None]) % self.cap  # [B, n]
        es = np.repeat(e[:, None], n_step, 1)
        d = self.done[ts, es]
        # Step k is valid if no earlier step in the chain ended a trial.
        valid = np.ones_like(d)
        valid[:, 1:] = np.cumsum(d[:, :-1], 1) == 0
        return ts, es, valid
