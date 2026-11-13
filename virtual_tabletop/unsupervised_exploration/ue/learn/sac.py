"""SAC with n-step returns and pluggable intrinsic rewards.

The replay buffer is a ring of shape ``[capacity, n_envs, ...]`` (one column
per env, in time order), so n-step chains are read straight out of it and
cut at trial boundaries. Rewards are *not* stored: the reward module
(:mod:`ue.rewards`) recomputes them for each sampled batch with its
current parameters, as in URLB (Laskin et al. 2021), since intrinsic rewards
drift as their models learn.

Skill- or goal-conditioned methods give each env a latent ``z`` per trial
(``RewardModule.sample_z``); the actor and critics see ``[obs_in, z]``.
Methods may relabel ``z`` in sampled batches (hindsight goals).

Trials end by time limit only, so targets always bootstrap (the ``time``
feature is in the observation).
"""

import copy
import math

import torch
import torch.nn as nn
import torch.nn.functional as F

from .nets import Actor, Critic


class SAC:
    def __init__(self, obs_dim, act_dim, z_dim=0, hidden=(512, 512),
                 lr=3e-4, gamma=0.97, tau=0.01, n_step=3, batch=512,
                 init_alpha=0.1, target_entropy=None, device="cpu",
                 fixed_alpha=None):
        self.obs_dim, self.act_dim, self.z_dim = obs_dim, act_dim, z_dim
        self.gamma, self.tau, self.n_step, self.batch = (gamma, tau, n_step,
                                                         batch)
        inp = obs_dim + z_dim
        self.device = device
        self.actor = Actor(inp, act_dim, hidden).to(device)
        self.critic = Critic(inp, act_dim, hidden).to(device)
        self.critic_t = copy.deepcopy(self.critic)
        self.a_opt = torch.optim.Adam(self.actor.parameters(), lr)
        self.c_opt = torch.optim.Adam(self.critic.parameters(), lr)
        self.log_alpha = torch.tensor(math.log(init_alpha), device=device,
                                      requires_grad=True)
        self.al_opt = torch.optim.Adam([self.log_alpha], lr)
        self.fixed_alpha = fixed_alpha
        self.target_entropy = (-act_dim / 2 if target_entropy is None
                               else target_entropy)

    @property
    def alpha(self):
        if self.fixed_alpha is not None:
            return torch.tensor(self.fixed_alpha)
        return self.log_alpha.exp().detach()

    @torch.no_grad()
    def act(self, obs, z=None, deterministic=False, eps=None):
        x = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        if self.z_dim:
            x = torch.cat([x, torch.as_tensor(z, dtype=torch.float32,
                                              device=self.device)], -1)
        if eps is not None:
            eps = torch.as_tensor(eps, dtype=torch.float32)
        a, _ = self.actor.sample(x, deterministic, eps)
        return a.cpu().numpy()


    def update(self, x, a, rewards, valid, x_next_n, disc_n, bc=None):
        """One SAC update.

        Args:
            x: [B, obs+z] inputs at the chain start; a: actions.
            rewards: [B, n] per-step rewards; valid: [B, n] mask.
            x_next_n: [B, obs+z] input after the last valid step.
            disc_n: [B] gamma ** (number of valid steps).
            bc: optional ``{"mask": [B] bool (demonstration samples),
                "lam": float}``: adds a Q-filtered behaviour-cloning term
                on the demonstration samples (Nair et al. 2018):
                ``lam * |Q| * mean(1[Q(s,a_d) > Q(s,pi(s))] *
                ||tanh(mu(s)) - a_d||^2)``, where ``|Q|`` (the batch's mean
                absolute Q, as in TD3+BC) makes ``lam`` scale-free.
        """
        g = self.gamma ** torch.arange(rewards.shape[1], device=self.device)
        ret = (rewards * valid * g).sum(1)
        with torch.no_grad():
            a2, logp2 = self.actor.sample(x_next_n)
            q2 = self.critic_t(x_next_n, a2).min(-1).values
            target = ret + disc_n * (q2 - self.alpha * logp2)
        q = self.critic(x, a)
        c_loss = F.mse_loss(q, target[:, None].expand_as(q))
        self.c_opt.zero_grad(set_to_none=True)
        c_loss.backward()
        nn.utils.clip_grad_norm_(self.critic.parameters(), 10)
        self.c_opt.step()
        a_new, logp = self.actor.sample(x)
        q_new = self.critic(x, a_new).min(-1).values
        a_loss = (self.alpha * logp - q_new).mean()
        bc_log = {}
        if bc is not None and bc["lam"] > 0 and bc["mask"].any():
            m = bc["mask"]
            mu, _ = self.actor(x[m])
            with torch.no_grad():
                q_demo = self.critic(x[m], a[m]).min(-1).values
                better = (q_demo > q_new[m]).float()
                q_scale = q_new.abs().mean()
            err = (torch.tanh(mu) - a[m]).pow(2).sum(-1)
            bc_loss = (better * err).mean()
            a_loss = a_loss + bc["lam"] * q_scale * bc_loss
            bc_log = {"bc_loss": bc_loss.item(),
                      "bc_filter": better.mean().item(),
                      "bc_err": err.mean().item()}
        self.a_opt.zero_grad(set_to_none=True)
        a_loss.backward()
        self.a_opt.step()
        if self.fixed_alpha is None:
            al_loss = -(self.log_alpha * (logp.detach()
                                          + self.target_entropy)).mean()
            self.al_opt.zero_grad(set_to_none=True)
            al_loss.backward()
            self.al_opt.step()
        with torch.no_grad():
            for p, pt in zip(self.critic.parameters(),
                             self.critic_t.parameters()):
                pt.lerp_(p, self.tau)
        return {**bc_log,
                "critic_loss": c_loss.item(), "actor_loss": a_loss.item(),
                "alpha": float(self.alpha), "q": q.mean().item(),
                "entropy": -logp.mean().item(), "ret": ret.mean().item()}

    def state_dict(self):
        return {"actor": self.actor.state_dict(),
                "critic": self.critic.state_dict(),
                "log_alpha": self.log_alpha.detach()}

    def load_state_dict(self, sd):
        self.actor.load_state_dict(sd["actor"])
        self.critic.load_state_dict(sd["critic"])
        self.critic_t = copy.deepcopy(self.critic)
        with torch.no_grad():
            self.log_alpha.copy_(sd["log_alpha"])
