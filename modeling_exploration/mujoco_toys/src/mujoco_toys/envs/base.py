"""The reward-free toy environment base class.

Subclasses build a model (``_build_spec``), say where objects may spawn
(``_spawn_spot``), what the agent's state is (``_agent_state``) and which
geoms are the agent's (``_agent_geoms``). The base class handles episodes,
object spawning, observations, rendering and diagnostics.

**Reward is always 0**: these environments are for intrinsic motivation.
``info`` carries diagnostics an intrinsic-motivation study may want to log
(never to optimise directly): agent-object contacts, which objects moved,
spawns.

Observations (``obs_mode``):

- ``"state"``: a flat float32 vector: the agent's state, then
  ``max_objects`` object slots of :data:`~mujoco_toys.objects.SLOT_FEATURES`
  numbers each (parked slots are zeros; ``observation_layout`` names the
  parts);
- ``"pixels"``: an RGB image (``image_size``) from ``camera``;
- ``"both"``: a dict with ``"state"`` and ``"pixels"``.
"""

from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np

from mujoco_toys.objects import SLOT_FEATURES, ObjectPool, Shape, add_slots

OBS_MODES = ("state", "pixels", "both")


class ToyEnv(gym.Env):
    """Base class; see the module docstring.

    Args:
        n_objects: Objects in the scene at the start of an episode.
        max_objects: Object slots (the most objects ever in the scene;
            spawning beyond it recycles the oldest object).
        spawn_rate: Expected new objects per second of simulated time (a
            Poisson process; 0 = none after the start).
        drop_height: Spawned objects appear this high above their resting
            height and fall (m; 0 = placed on the floor).
        episode_seconds: Episode length (s of simulated time); ``None`` for
            no time limit.
        control_dt: Seconds of simulation per ``step``.
        obs_mode: ``"state"``, ``"pixels"`` or ``"both"``.
        image_size: ``(height, width)`` of pixel observations and renders.
        camera: Camera for pixel observations and ``render``.
        render_mode: ``"rgb_array"`` or ``None``.
        seed: Default seed (``reset(seed=...)`` overrides it).
    """

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(
        self,
        *,
        n_objects: int = 6,
        max_objects: int = 12,
        spawn_rate: float = 0.0,
        drop_height: float = 0.0,
        episode_seconds: float | None = 30.0,
        control_dt: float = 0.04,
        obs_mode: str = "state",
        image_size: tuple[int, int] = (96, 96),
        camera: str = "overview",
        render_mode: str | None = None,
        seed: int | None = None,
    ):
        if obs_mode not in OBS_MODES:
            raise ValueError(f"obs_mode must be one of {OBS_MODES}")
        if not 0 <= n_objects <= max_objects:
            raise ValueError("need 0 <= n_objects <= max_objects")
        self.n_objects, self.max_objects = n_objects, max_objects
        self.spawn_rate, self.drop_height = spawn_rate, drop_height
        self.episode_seconds, self.control_dt = episode_seconds, control_dt
        self.obs_mode, self.image_size = obs_mode, image_size
        self.camera, self.render_mode = camera, render_mode
        self._seed = seed
        self._renderers: dict[tuple, mujoco.Renderer] = {}
        # Build once to fix the spaces (shapes vary per episode, sizes don't).
        self._new_episode_model(np.random.default_rng(seed))
        self.action_space = gym.spaces.Box(
            -1.0, 1.0, (self.model.nu,), np.float32
        )
        n_state = self._agent_state().size + max_objects * SLOT_FEATURES
        state_space = gym.spaces.Box(-np.inf, np.inf, (n_state,), np.float32)
        pixel_space = gym.spaces.Box(0, 255, (*image_size, 3), np.uint8)
        self.observation_space = {
            "state": state_space,
            "pixels": pixel_space,
            "both": gym.spaces.Dict(state=state_space, pixels=pixel_space),
        }[obs_mode]

    # --- to implement ----------------------------------------------------

    def _build_spec(self, rng) -> mujoco.MjSpec:
        raise NotImplementedError

    def _spawn_spot(self, rng) -> np.ndarray:
        raise NotImplementedError

    def _agent_state(self) -> np.ndarray:
        raise NotImplementedError

    def _agent_geoms(self) -> set[int]:
        raise NotImplementedError

    def _keep_clear(self) -> list:
        """``(x, y, radius)`` spots spawns must avoid besides objects."""
        return []

    def _reset_agent(self, rng) -> None:
        """Put the agent in its start state (after the keyframe)."""

    # --- episodes --------------------------------------------------------

    def _new_episode_model(self, rng) -> None:
        self.shapes = self._random_shapes(rng)
        spec = self._build_spec(rng)
        add_slots(spec, self.shapes)
        self.mj_spec = spec  # (``spec`` is Gymnasium's EnvSpec)
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self._substeps = max(
            1, round(self.control_dt / self.model.opt.timestep)
        )
        self.pool = ObjectPool(
            self.model,
            self.data,
            self.shapes,
            self._spawn_spot,
            self._keep_clear,
        )
        self._agent_geom_ids = self._agent_geoms()
        self._object_geom_ids = set(self.pool.geoms)

    def _random_shapes(self, rng) -> list[Shape]:
        raise NotImplementedError

    def reset(self, *, seed: int | None = None, options=None):
        super().reset(seed=seed if seed is not None else self._seed)
        rng = self.np_random
        self._new_episode_model(rng)
        mujoco.mj_resetData(self.model, self.data)
        self._reset_agent(rng)
        mujoco.mj_forward(self.model, self.data)
        for _ in range(self.n_objects):
            self.pool.spawn(rng, 0.0)
        mujoco.mj_forward(self.model, self.data)
        self.steps = 0
        self._last_positions = self.pool.positions().copy()
        return self._obs(), self._info(spawned=[])

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        self.data.ctrl[:] = self._ctrl_from_action(action)
        mujoco.mj_step(self.model, self.data, self._substeps)
        self.steps += 1
        spawned = []
        if self.spawn_rate > 0:
            for _ in range(
                self.np_random.poisson(self.spawn_rate * self.control_dt)
            ):
                i = self.pool.spawn(
                    self.np_random, self.time, self.drop_height
                )
                if i is not None:
                    spawned.append(i)
            if spawned:
                mujoco.mj_forward(self.model, self.data)
        truncated = (
            self.episode_seconds is not None
            and self.time >= self.episode_seconds - 1e-9
        )
        info = self._info(spawned)
        return self._obs(), 0.0, False, truncated, info

    def _ctrl_from_action(self, action: np.ndarray) -> np.ndarray:
        """Map actions in [-1, 1] to each actuator's control range."""
        low, high = self.model.actuator_ctrlrange.T
        return low + (action + 1.0) * 0.5 * (high - low)

    @property
    def time(self) -> float:
        return self.steps * self.control_dt

    # --- observations and diagnostics ------------------------------------

    def _state(self) -> np.ndarray:
        return np.concatenate(
            [self._agent_state(), self.pool.features().ravel()]
        ).astype(np.float32)

    def _obs(self):
        if self.obs_mode == "state":
            return self._state()
        pixels = self.render_pixels(self.camera, self.image_size)
        if self.obs_mode == "pixels":
            return pixels
        return {"state": self._state(), "pixels": pixels}

    @property
    def observation_layout(self) -> dict[str, slice]:
        """Slices of the state vector: ``agent``, then ``obj<i>``."""
        n_agent = self._agent_state().size
        layout = {"agent": slice(0, n_agent)}
        for i in range(self.max_objects):
            start = n_agent + i * SLOT_FEATURES
            layout[f"obj{i}"] = slice(start, start + SLOT_FEATURES)
        return layout

    def _info(self, spawned) -> dict:
        positions = self.pool.positions()
        moved = np.linalg.norm(positions - self._last_positions, axis=1)
        self._last_positions = positions.copy()
        touching = set()
        for c in self.data.contact[: self.data.ncon]:
            pair = {c.geom1, c.geom2}
            if pair & self._agent_geom_ids and pair & self._object_geom_ids:
                touching |= pair & self._object_geom_ids
        return {
            "time": self.time,
            "active_objects": int(self.pool.active.sum()),
            "spawned": spawned,
            "touching": sorted(self.pool.geoms.index(g) for g in touching),
            "moved": [
                int(i)
                for i in np.flatnonzero((moved > 1e-3) & self.pool.active)
            ],
        }

    # --- rendering --------------------------------------------------------

    def render_pixels(self, camera: str, size: tuple[int, int]) -> np.ndarray:
        key = (id(self.model), size)
        if key not in self._renderers:
            for old in self._renderers.values():
                old.close()
            self._renderers = {key: mujoco.Renderer(self.model, *size)}
        renderer = self._renderers[key]
        renderer.update_scene(self.data, camera=camera)
        return renderer.render()

    def render(self):
        if self.render_mode != "rgb_array":
            return None
        return self.render_pixels(self.camera, (480, 640))

    def close(self):
        for renderer in self._renderers.values():
            renderer.close()
        self._renderers = {}

    def save_xml(self, path: Path) -> Path:
        """Write the current episode's model as MJCF (for inspection)."""
        path.write_text(self.mj_spec.to_xml())
        return path
