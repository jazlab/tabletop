"""Cursor control task."""

import numpy as np


class CursorControlEnvironment():
    """Cursor control environment."""

    def __init__(self, batch_size: int, feedback_window: float=0.2, bounds=(-1, 1)):
        self._batch_size = batch_size
        self._feedback_window = feedback_window
        self._bounds = bounds
        self.reset()

    def reset(self):
        """Reset the environment."""
        self._object = np.random.rand(self._batch_size, 2) * (self._bounds[1] - self._bounds[0]) + self._bounds[0]
        self._hand = np.random.rand(self._batch_size, 2) * (self._bounds[1] - self._bounds[0]) + self._bounds[0]
        return self.get_observation()

    def step(self, action: np.ndarray) -> np.ndarray:
        """Step the environment."""
        self._hand = self._hand + action
        self._hand = np.clip(self._hand, self._bounds[0], self._bounds[1])
        observation = self.get_observation()
        return observation

    def get_observation(self) -> np.ndarray:
        """Get the current observation."""
        dist = np.linalg.norm(self._hand - self._object, axis=-1)
        feedback = (dist < self._feedback_window).astype(np.float32)
        observation = np.concatenate([self._hand, self._object, feedback[:, None]], axis=-1)
        # observation = np.concatenate([self._hand, self._object], axis=-1)
        # observation = self._hand
        return observation
    
    def observation_to_components(self, observation):
        components = {
            "hand": observation[..., 0:2],
            "object": observation[..., 2:4],
            "feedback": observation[..., 4:5],
        }
        return components

    @property
    def observation_dim(self):
        # return 2 + 2 + 1 + 2
        return 2 + 2 + 1
    
    @property
    def action_dim(self):
        return 2
    
    @property
    def batch_size(self):
        return self._batch_size


class ExplorationAgent():

    def __init__(self,
                 environment: CursorControlEnvironment,
                 scale: float,
                 smooth_tau: float,
                 seq_length: int,
                 burn_in_steps: int=5):
        self._environment = environment
        self._scale = scale
        self._smooth_tau = smooth_tau
        self._seq_length = seq_length
        self._burn_in_steps = burn_in_steps
    
    def _random_action(self):
        """Generate a random action."""
        return self._scale * (np.random.rand(self.batch_size, self._environment.action_dim) * 2 - 1)

    def action_sequence(self):
        """Generate action sequence."""
        actions = []
        current_action = np.sqrt(1 - self._smooth_tau) * self._random_action()
        for _ in range(self._seq_length):
            actions.append(current_action)
            current_action = self._smooth_tau * current_action + (1 - self._smooth_tau) * self._random_action()
        return np.array(actions)
    
    def __call__(self):
        actions = self.action_sequence()
        observations = [self.environment.reset()]
        for action in actions:
            observations.append(self.environment.step(action))
        observations = np.array(observations)

        # Sample start and end
        observation_start = observations[self._burn_in_steps]
        action_start = actions[self._burn_in_steps]
        max_duration = observations.shape[0] - self._burn_in_steps
        duration = np.random.randint(1, max_duration, size=(self.batch_size,))
        observation_end = observations[self._burn_in_steps + duration, np.arange(len(duration))]

        return observation_start, observation_end, action_start, duration
    
    @property
    def environment(self):
        return self._environment
    
    @property
    def batch_size(self):
        return self._environment.batch_size
    
    @property
    def seq_length(self):
        return self._seq_length
    
    @property
    def observation_dim(self):
        return self._environment.observation_dim

    @property
    def action_dim(self):
        return self._environment.action_dim
