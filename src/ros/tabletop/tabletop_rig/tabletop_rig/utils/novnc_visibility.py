"""Pause drawing, never data collection, for off-screen DWM windows.

The layout supervisor publishes a small shared status file. Unknown or stale
status fails open so a missing supervisor cannot leave a view frozen forever.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path

STALE_AFTER = 3.0


def status_path() -> Path:
    return Path(
        os.environ.get(
            "TABLETOP_NOVNC_VISIBILITY_FILE",
            "/tabletop/.cache/tabletop/novnc-visibility.json",
        )
    )


def should_render(document: object, title: str, now: float) -> bool:
    """Only a fresh, explicit false for this exact title disables drawing."""
    if not isinstance(document, dict) or document.get("version") != 1:
        return True
    updated = document.get("updated_epoch")
    if (
        isinstance(updated, bool)
        or not isinstance(updated, (float, int))
        or not 0 <= now - updated <= STALE_AFTER
    ):
        return True
    windows = document.get("windows")
    if not isinstance(windows, dict):
        return True
    return windows.get(title) is not False


class WindowVisibility:
    """Cheap per-loop visibility check; event pumping must remain unconditional."""

    def __init__(self, title: str, path: Path | None = None) -> None:
        self.title = title
        self.path = status_path() if path is None else path
        self._mtime_ns: int | None = None
        self._document: object = None
        self._last_visible: bool | None = None

    def __call__(self) -> bool:
        try:
            mtime = self.path.stat().st_mtime_ns
            if mtime != self._mtime_ns:
                self._document = json.loads(self.path.read_text())
                self._mtime_ns = mtime
            visible = should_render(self._document, self.title, time.time())
        except (OSError, ValueError, TypeError):
            visible = True
        if visible != self._last_visible:
            print(
                f"[VIEW_RENDER] {self.title}: "
                f"{'active' if visible else 'paused (hidden)'}",
                flush=True,
            )
            self._last_visible = visible
        return visible


class VisibilityPublisher:
    """Atomic updates on workspace changes, with a one-second heartbeat."""

    def __init__(self, path: Path | None = None) -> None:
        self.path = status_path() if path is None else path
        self._previous: dict[str, bool] | None = None
        self._published = 0.0

    def publish(self, windows: dict[str, bool]) -> None:
        now = time.monotonic()
        if windows == self._previous and now - self._published < 1.0:
            return
        temporary = self.path.with_suffix(f".{os.getpid()}.tmp")
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary.write_text(
                json.dumps(
                    {
                        "version": 1,
                        "updated_epoch": time.time(),
                        "windows": windows,
                    }
                )
            )
            temporary.replace(self.path)
            self._previous = dict(windows)
            self._published = now
        except OSError:
            # Read-only/unavailable status storage disables this optimization,
            # not the GUI supervisor or the underlying data collection.
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass
