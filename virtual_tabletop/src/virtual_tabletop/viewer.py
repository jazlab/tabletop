"""MuJoCo's interactive (passive) viewer, shared by the ``vtt-*`` viewers.

- :func:`ensure_mjpython`: on macOS the passive viewer must run under
  ``mjpython``; viewers call this first to re-launch themselves that way.
- :func:`run_passive`: the viewer loop. It owns the locking rule: the model
  and data are changed only while holding the viewer's lock, and viewer
  methods (``set_texts``, ``sync``) are only called *outside* it; calling
  them inside deadlocks the window (it freezes with a spinning wheel).
- :class:`Playback`: pause and speed for viewers that play something back
  in (scaled) real time, with the shared keys.
"""

import os
import sys
import time
from collections.abc import Callable
from pathlib import Path

import mujoco

#: GLFW key codes the viewers use.
KEY_SPACE, KEY_RIGHT, KEY_LEFT = 32, 262, 263
KEY_0, KEY_EQUAL, KEY_MINUS = 48, 61, 45
KEY_H, KEY_N, KEY_O, KEY_P, KEY_R = 72, 78, 79, 80, 82


def ensure_mjpython(module: str) -> None:
    """On macOS, re-run ``python -m module`` under mjpython if needed."""
    if sys.platform != "darwin" or os.environ.get("VTT_UNDER_MJPYTHON"):
        return
    mjpython = Path(sys.executable).with_name("mjpython")
    os.environ["VTT_UNDER_MJPYTHON"] = "1"
    os.execv(mjpython, [str(mjpython), "-m", module, *sys.argv[1:]])


class Playback:
    """Pause and speed control: space pauses, ``=``/``-`` double/halve."""

    def __init__(self, speed: float = 1.0):
        self.paused = False
        self.speed = speed

    def on_key(self, key: int) -> bool:
        """Handle a playback key; return whether it was one."""
        if key == KEY_SPACE:
            self.paused = not self.paused
        elif key == KEY_EQUAL:
            self.speed = min(self.speed * 2, 16.0)
        elif key == KEY_MINUS:
            self.speed = max(self.speed / 2, 0.125)
        else:
            return False
        return True

    def advance(self, wall_dt: float) -> float:
        """Simulated time to play for ``wall_dt`` seconds of wall time."""
        return 0.0 if self.paused else wall_dt * self.speed


def run_passive(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    update: Callable[[float], str],
    *,
    on_key: Callable[[int], None] | None = None,
    setup_camera: Callable[[mujoco.MjvCamera], None] | None = None,
    fps: float = 60.0,
) -> None:
    """Run the passive viewer until its window closes.

    Args:
        update: Called every frame, **under the viewer lock**, with the wall
            time since the previous frame (s). It may change ``model`` and
            ``data``; it returns the top-left overlay text.
        on_key: Key callback (GLFW key code).
        setup_camera: Called once with the viewer's free camera.
        fps: Frame rate of the loop.
    """
    import mujoco.viewer

    with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as v:
        if setup_camera is not None:
            setup_camera(v.cam)
        last = time.monotonic()
        while v.is_running():
            now = time.monotonic()
            with v.lock():
                text = update(now - last)
            last = now
            # Outside the lock: these take it themselves.
            v.set_texts(
                (
                    mujoco.mjtFontScale.mjFONTSCALE_150,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    text,
                    "",
                )
            )
            v.sync()
            time.sleep(1 / fps)
