"""Common modules."""

import enum


class State(enum.Enum):
    """Application state."""

    RUNNING = 0
    PAUSED = 1
    STOPPED = 2
