"""Keep the standard TableTop GUI windows on deterministic DWM workspaces."""

from __future__ import annotations

import ctypes
import signal
import time
from dataclasses import dataclass

from tabletop_rig.utils.novnc_workspace import switch_dwm_workspace

WORKSPACE_TITLES = (
    ("/rig.rviz", 1),
    ("/camera_robot.rviz", 2),
    ("/left_back_rig.rviz", 3),
    ("TableTop Camera Trigger / Exposure Monitor", 4),
    ("TableTop I/O Check", 5),
    ("TableTop TTL Monitor", 6),
    ("TableTop Session Health", 7),
)


class XWindowAttributes(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_int),
        ("y", ctypes.c_int),
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("border_width", ctypes.c_int),
        ("depth", ctypes.c_int),
        ("visual", ctypes.c_void_p),
        ("root", ctypes.c_ulong),
        ("class_", ctypes.c_int),
        ("bit_gravity", ctypes.c_int),
        ("win_gravity", ctypes.c_int),
        ("backing_store", ctypes.c_int),
        ("backing_planes", ctypes.c_ulong),
        ("backing_pixel", ctypes.c_ulong),
        ("save_under", ctypes.c_int),
        ("colormap", ctypes.c_ulong),
        ("map_installed", ctypes.c_int),
        ("map_state", ctypes.c_int),
        ("all_event_masks", ctypes.c_long),
        ("your_event_mask", ctypes.c_long),
        ("do_not_propagate_mask", ctypes.c_long),
        ("override_redirect", ctypes.c_int),
        ("screen", ctypes.c_void_p),
    ]


class XMapEvent(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("serial", ctypes.c_ulong),
        ("send_event", ctypes.c_int),
        ("display", ctypes.c_void_p),
        ("event", ctypes.c_ulong),
        ("window", ctypes.c_ulong),
        ("override_redirect", ctypes.c_int),
    ]


class XEvent(ctypes.Union):
    _fields_ = [
        ("type", ctypes.c_int),
        ("xmap", XMapEvent),
        ("pad", ctypes.c_long * 24),
    ]


@dataclass(frozen=True)
class Window:
    window_id: int
    title: str
    x: int
    y: int
    width: int
    height: int

    @property
    def visible(self) -> bool:
        return self.x > -100 and self.width > 100 and self.height > 100


def workspace_for_title(title: str) -> int | None:
    for fragment, workspace in WORKSPACE_TITLES:
        if fragment in title:
            return workspace
    return None


class XDesktop:
    """Minimal Xlib/XTest bridge for DWM window discovery and retagging."""

    def __init__(self) -> None:
        self.x11 = ctypes.CDLL("libX11.so.6")
        self.xtst = ctypes.CDLL("libXtst.so.6")
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.x11.XDefaultRootWindow.restype = ctypes.c_ulong
        self.x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        self.x11.XQueryTree.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),
            ctypes.POINTER(ctypes.c_uint),
        ]
        self.x11.XFetchName.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_void_p),
        ]
        self.x11.XGetWindowAttributes.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.POINTER(XWindowAttributes),
        ]
        self.x11.XQueryPointer.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_uint),
        ]
        self.x11.XWarpPointer.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_uint,
            ctypes.c_uint,
            ctypes.c_int,
            ctypes.c_int,
        ]
        self.x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.xtst.XTestFakeKeyEvent.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint,
            ctypes.c_int,
            ctypes.c_ulong,
        ]
        self.x11.XFlush.argtypes = [ctypes.c_void_p]
        self.x11.XFree.argtypes = [ctypes.c_void_p]
        self.x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.x11.XSelectInput.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.c_long,
        ]
        self.x11.XPending.argtypes = [ctypes.c_void_p]
        self.x11.XPending.restype = ctypes.c_int
        self.x11.XNextEvent.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(XEvent),
        ]
        self.display = self.x11.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError("could not open the noVNC X display")
        self.root = self.x11.XDefaultRootWindow(self.display)
        self.x11.XSelectInput(self.display, self.root, 1 << 19)

    def close(self) -> None:
        if self.display:
            self.x11.XCloseDisplay(self.display)
            self.display = None

    def windows(self) -> list[Window]:
        returned_root = ctypes.c_ulong()
        parent = ctypes.c_ulong()
        children = ctypes.POINTER(ctypes.c_ulong)()
        count = ctypes.c_uint()
        result: list[Window] = []
        if not self.x11.XQueryTree(
            self.display,
            self.root,
            ctypes.byref(returned_root),
            ctypes.byref(parent),
            ctypes.byref(children),
            ctypes.byref(count),
        ):
            return result
        try:
            for index in range(count.value):
                window_id = children[index]
                title_pointer = ctypes.c_void_p()
                attributes = XWindowAttributes()
                if not self.x11.XFetchName(
                    self.display, window_id, ctypes.byref(title_pointer)
                ):
                    continue
                try:
                    title = (
                        ctypes.string_at(title_pointer).decode(
                            errors="replace"
                        )
                        if title_pointer.value
                        else ""
                    )
                finally:
                    if title_pointer.value:
                        self.x11.XFree(title_pointer)
                if not title or not self.x11.XGetWindowAttributes(
                    self.display, window_id, ctypes.byref(attributes)
                ):
                    continue
                if attributes.map_state != 2:
                    continue
                result.append(
                    Window(
                        int(window_id),
                        title,
                        attributes.x,
                        attributes.y,
                        attributes.width,
                        attributes.height,
                    )
                )
        finally:
            if children:
                self.x11.XFree(children)
        return result

    def mapped_window_ids(self) -> list[int]:
        """Drain newly mapped top-level window IDs from the X event queue."""
        result: list[int] = []
        while self.x11.XPending(self.display):
            event = XEvent()
            self.x11.XNextEvent(self.display, ctypes.byref(event))
            if event.type == 19:  # MapNotify
                result.append(int(event.xmap.window))
        return result

    def _fake_key(self, keysym: int, pressed: bool) -> None:
        keycode = self.x11.XKeysymToKeycode(self.display, keysym)
        self.xtst.XTestFakeKeyEvent(self.display, keycode, pressed, 0)

    def tag(self, window: Window, workspace: int) -> bool:
        """Focus a visible window and send DWM's Alt+Shift+number shortcut."""
        if not window.visible:
            return False
        root_return = ctypes.c_ulong()
        child_return = ctypes.c_ulong()
        root_x = ctypes.c_int()
        root_y = ctypes.c_int()
        win_x = ctypes.c_int()
        win_y = ctypes.c_int()
        mask = ctypes.c_uint()
        have_pointer = bool(
            self.x11.XQueryPointer(
                self.display,
                self.root,
                ctypes.byref(root_return),
                ctypes.byref(child_return),
                ctypes.byref(root_x),
                ctypes.byref(root_y),
                ctypes.byref(win_x),
                ctypes.byref(win_y),
                ctypes.byref(mask),
            )
        )
        target_x = window.x + max(1, window.width // 2)
        target_y = window.y + max(1, window.height // 2)
        self.x11.XWarpPointer(
            self.display, 0, self.root, 0, 0, 0, 0, target_x, target_y
        )
        self.x11.XFlush(self.display)
        time.sleep(0.08)
        self._fake_key(0xFFE9, True)
        self._fake_key(0xFFE1, True)
        self._fake_key(ord(str(workspace)), True)
        self._fake_key(ord(str(workspace)), False)
        self._fake_key(0xFFE1, False)
        self._fake_key(0xFFE9, False)
        if have_pointer:
            self.x11.XWarpPointer(
                self.display,
                0,
                self.root,
                0,
                0,
                0,
                0,
                root_x.value,
                root_y.value,
            )
        self.x11.XFlush(self.display)
        time.sleep(0.08)
        return True


def repair_layout(desktop: XDesktop) -> None:
    """Inspect every managed workspace and retag misplaced standard windows."""
    for current_workspace in range(1, 8):
        switch_dwm_workspace(current_workspace)
        time.sleep(0.15)
        for window in desktop.windows():
            target = workspace_for_title(window.title)
            if (
                target is not None
                and target != current_workspace
                and window.visible
            ):
                desktop.tag(window, target)
    switch_dwm_workspace(1)


def main() -> int:
    stopping = False

    def stop(*_args) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    desktop = XDesktop()
    try:
        time.sleep(2.0)
        repair_layout(desktop)
        while not stopping:
            needs_scan = False
            for window_id in desktop.mapped_window_ids():
                deadline = time.monotonic() + 1.0
                window = None
                while time.monotonic() < deadline:
                    window = next(
                        (
                            item
                            for item in desktop.windows()
                            if item.window_id == window_id
                        ),
                        None,
                    )
                    if window is not None and workspace_for_title(
                        window.title
                    ):
                        break
                    time.sleep(0.02)
                if window is None:
                    continue
                target = workspace_for_title(window.title)
                if target is not None and not desktop.tag(window, target):
                    needs_scan = True
            if needs_scan:
                repair_layout(desktop)
            time.sleep(0.02)
    finally:
        desktop.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
