"""Small MuJoCo worlds for reward-free, intrinsically motivated agents.

Environments (Gymnasium; reward is always 0):

- :class:`~mujoco_toys.envs.PlaypenEnv` (``MujocoToys/Playpen-v0``): a
  ball agent pushing randomly spawning objects in a walled arena;
- :class:`~mujoco_toys.envs.ArmBlocksEnv` (``MujocoToys/ArmBlocks-v0``): a
  3-joint arm (base yaw, shoulder, elbow) with blocks and towers to topple.

``import mujoco_toys`` registers both with Gymnasium.
"""

import gymnasium as gym

from mujoco_toys.envs import ArmBlocksEnv, PlaypenEnv

gym.register("MujocoToys/Playpen-v0", entry_point=PlaypenEnv)
gym.register("MujocoToys/ArmBlocks-v0", entry_point=ArmBlocksEnv)

__all__ = ["ArmBlocksEnv", "PlaypenEnv"]
