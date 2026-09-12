"""Serialize deterministic DWM workspace assignment across GUI containers."""

from __future__ import annotations

import ctypes
import fcntl
import os
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator

DEFAULT_LOCK_PATH = Path(
    os.environ.get(
        "TABLETOP_NOVNC_WORKSPACE_LOCK",
        "/tabletop/.cache/tabletop/novnc-workspace.lock",
    )
)


def switch_dwm_workspace(number: int) -> bool:
    """Send DWM's Alt+number workspace shortcut through XTest."""
    if not 1 <= number <= 9:
        raise ValueError("DWM workspace number must be between 1 and 9")
    try:
        x11 = ctypes.CDLL("libX11.so.6")
        xtst = ctypes.CDLL("libXtst.so.6")
        x11.XOpenDisplay.restype = ctypes.c_void_p
        display = x11.XOpenDisplay(None)
        if not display:
            return False
        x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        x11.XFlush.argtypes = [ctypes.c_void_p]
        x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        xtst.XTestFakeKeyEvent.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint,
            ctypes.c_int,
            ctypes.c_ulong,
        ]
        alt = x11.XKeysymToKeycode(display, 0xFFE9)
        digit = x11.XKeysymToKeycode(display, ord(str(number)))
        for keycode, pressed in (
            (alt, True),
            (digit, True),
            (digit, False),
            (alt, False),
        ):
            xtst.XTestFakeKeyEvent(display, keycode, pressed, 0)
        x11.XFlush(display)
        x11.XCloseDisplay(display)
        return True
    except OSError:
        return False


@contextmanager
def reserve_dwm_workspace(
    number: int,
    *,
    return_workspace: int = 1,
    lock_path: Path = DEFAULT_LOCK_PATH,
) -> Iterator[bool]:
    """Reserve the X desktop while a new window is mapped onto ``number``.

    DWM assigns a new window to whichever workspace is visible when the window
    first maps.  TableTop GUI processes live in separate containers, so their
    old switch/create/return sequences could interleave and put two windows on
    one workspace.  ``flock`` on the shared workspace bind mount makes that
    sequence atomic across containers.  Locks are released automatically if a
    process exits.
    """
    if not 1 <= number <= 9 or not 1 <= return_workspace <= 9:
        raise ValueError("DWM workspace numbers must be between 1 and 9")
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a+", encoding="utf-8") as lock_file:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
        switched = switch_dwm_workspace(number)
        try:
            yield switched
        finally:
            if switched and number != return_workspace:
                switch_dwm_workspace(return_workspace)
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


def window_title_exists(fragment: str) -> bool:
    """Return whether an X11 top-level window title contains ``fragment``."""
    try:
        x11 = ctypes.CDLL("libX11.so.6")
        x11.XOpenDisplay.restype = ctypes.c_void_p
        x11.XDefaultRootWindow.restype = ctypes.c_ulong
        x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        x11.XQueryTree.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),
            ctypes.POINTER(ctypes.c_uint),
        ]
        x11.XFetchName.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_char_p),
        ]
        x11.XFree.argtypes = [ctypes.c_void_p]
        x11.XCloseDisplay.argtypes = [ctypes.c_void_p]

        display = x11.XOpenDisplay(None)
        if not display:
            return False
        root = x11.XDefaultRootWindow(display)
        returned_root = ctypes.c_ulong()
        parent = ctypes.c_ulong()
        children = ctypes.POINTER(ctypes.c_ulong)()
        count = ctypes.c_uint()
        found = False
        if x11.XQueryTree(
            display,
            root,
            ctypes.byref(returned_root),
            ctypes.byref(parent),
            ctypes.byref(children),
            ctypes.byref(count),
        ):
            for index in range(count.value):
                title = ctypes.c_char_p()
                if x11.XFetchName(
                    display, children[index], ctypes.byref(title)
                ):
                    value = (
                        title.value.decode(errors="replace")
                        if title.value
                        else ""
                    )
                    if title:
                        x11.XFree(title)
                    if fragment in value:
                        found = True
                        break
        if children:
            x11.XFree(children)
        x11.XCloseDisplay(display)
        return found
    except OSError:
        return False


def wait_for_window_title(
    fragment: str,
    *,
    timeout: float = 60.0,
    poll_interval: float = 0.1,
) -> bool:
    """Wait until a matching top-level X11 window has been created."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if window_title_exists(fragment):
            return True
        time.sleep(poll_interval)
    return False
