"""SmoothPursuit task."""

import abc
import enum
import numpy as np
import time


class SmoothPursuitTask(abc.ABC):
    
    def __init__(self,
                 commander,
                 reward_period=1.,
                 reward_duration_ms=100):
        self._commander = commander
        self._reward_period = reward_period
        self._reward_duration_ms = reward_duration_ms
        
        # Make circular path
        self._path = []
        for i in range(100):
            theta = 2 * np.pi * i / 100
            x = 0.1 * np.cos(theta)
            y = 0.1 * np.sin(theta)
            self._path.append((x, y))
    
    def start(self):
        last_reward_time = time.time()
        for position in self._path:
            self._commander.move_to_position_sync(position) # synchronous call
            if time.time() - last_reward_time > self._reward_period:
                self._commander.reward(self._reward_duration_ms)
                last_reward_time = time.time()
    