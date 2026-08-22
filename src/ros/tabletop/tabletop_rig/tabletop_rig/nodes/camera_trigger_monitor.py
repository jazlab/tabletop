"""Live GUI for camera exposure events caused by the Teensy trigger output."""

import ctypes
import signal
import threading
from collections import deque
from time import monotonic

import cv2
import numpy as np
import rclpy
from flir_camera_msgs.msg import ImageMetaData

CAMERAS = (
    ("left_front_top_cam", "Left front", (255, 194, 85)),
    ("right_front_top_cam", "Right front", (105, 140, 255)),
    ("left_back_top_cam", "Left back", (143, 227, 102)),
    ("right_back_top_cam", "Right back", (255, 149, 216)),
    ("left_bottom_cam", "Left bottom", (102, 209, 255)),
    ("right_bottom_cam", "Right bottom", (220, 230, 120)),
)
ALL_CAMERAS_MASK = (1 << len(CAMERAS)) - 1
WINDOW_NAME = "TableTop Camera Trigger / Exposure Monitor"


class ExposureData:
    """Thread-safe sliding window of camera-reported exposure metadata."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.events = {name: deque(maxlen=600) for name, _, _ in CAMERAS}
        self.exposure_us = {name: 0 for name, _, _ in CAMERAS}
        self.last_received = {name: 0.0 for name, _, _ in CAMERAS}
        self.groups: dict[int, int] = {}

    def record(self, camera: str, index: int, message: ImageMetaData) -> None:
        stamp = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        with self.lock:
            self.events[camera].append(stamp)
            self.exposure_us[camera] = message.exposure_time
            self.last_received[camera] = monotonic()
            self.groups[stamp] = self.groups.get(stamp, 0) | (1 << index)
            cutoff = stamp - 3_000_000_000
            stale = [
                group_stamp
                for group_stamp in self.groups
                if group_stamp < cutoff
            ]
            for group_stamp in stale:
                del self.groups[group_stamp]

    def snapshot(self):
        with self.lock:
            return (
                {name: tuple(values) for name, values in self.events.items()},
                dict(self.exposure_us),
                dict(self.last_received),
                dict(self.groups),
            )


def _text(
    canvas: np.ndarray,
    value: str,
    position: tuple[int, int],
    scale: float,
    color: tuple[int, int, int],
    thickness: int = 1,
) -> None:
    cv2.putText(
        canvas,
        value,
        position,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def render(
    data: ExposureData, width: int = 1900, height: int = 1015
) -> np.ndarray:
    """Render one low-cost monitor frame."""
    events, exposure_us, received, groups = data.snapshot()
    canvas = np.full((height, width, 3), (33, 26, 23), dtype=np.uint8)
    _text(
        canvas,
        "120 Hz Camera Trigger / Exposure Response",
        (36, 52),
        1.0,
        (248, 244, 241),
        2,
    )
    _text(
        canvas,
        "Each tick is camera-reported exposure metadata following the shared Teensy Line0 trigger.",
        (36, 82),
        0.55,
        (195, 180, 170),
    )
    _text(
        canvas,
        "This verifies camera response and synchronization; it is not an electrical voltage probe.",
        (36, 106),
        0.55,
        (195, 180, 170),
    )

    newest = max(
        (values[-1] for values in events.values() if values), default=0
    )
    plot_left = 180
    plot_right = width - 300
    plot_width = plot_right - plot_left
    top = 160
    row_height = (height - top - 80) // len(CAMERAS)
    now = monotonic()

    if newest:
        complete = sum(
            newest - 1_000_000_000 <= stamp <= newest
            and mask == ALL_CAMERAS_MASK
            for stamp, mask in groups.items()
        )
        incomplete = sum(
            newest - 1_000_000_000 <= stamp <= newest - 50_000_000
            and mask != ALL_CAMERAS_MASK
            for stamp, mask in groups.items()
        )
        status_color = (143, 227, 102) if incomplete == 0 else (107, 107, 255)
        _text(
            canvas,
            f"Complete groups: {complete}/s    incomplete: {incomplete}",
            (width - 590, 52),
            0.65,
            status_color,
            2,
        )

    for index, (camera, label, color) in enumerate(CAMERAS):
        y = top + index * row_height + row_height // 2
        cv2.line(canvas, (plot_left, y), (plot_right, y), (89, 75, 67), 1)
        _text(canvas, label, (28, y + 7), 0.62, color, 2)

        camera_events = events[camera]
        cutoff = newest - 250_000_000
        for stamp in camera_events:
            if stamp < cutoff or not newest:
                continue
            x = plot_right - int((newest - stamp) * plot_width / 250_000_000)
            cv2.line(canvas, (x, y - 25), (x, y + 25), color, 2)

        fps = sum(stamp >= newest - 1_000_000_000 for stamp in camera_events)
        age = now - received[camera] if received[camera] else float("inf")
        connected = age < 0.5
        live_color = (143, 227, 102) if connected else (107, 107, 255)
        status = "LIVE" if connected else "NO DATA"
        _text(
            canvas,
            f"{status:7} {fps:3d} fps  {exposure_us[camera]:4d} us",
            (plot_right + 18, y + 7),
            0.55,
            live_color,
            1,
        )

    _text(
        canvas, "250 ms ago", (plot_left, height - 25), 0.48, (149, 131, 120)
    )
    _text(canvas, "now", (plot_right - 32, height - 25), 0.48, (149, 131, 120))
    return canvas


def _switch_dwm_workspace(number: int) -> bool:
    """Send Alt+number to dwm using the XTest library."""
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
    except (OSError, ValueError):
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    data = ExposureData()
    node = rclpy.create_node("camera_trigger_monitor")
    subscriptions = []
    for index, (camera, _, _) in enumerate(CAMERAS):
        subscriptions.append(
            node.create_subscription(
                ImageMetaData,
                f"/cam_sync/{camera}/meta",
                lambda message, camera=camera, index=index: data.record(
                    camera, index, message
                ),
                1,
            )
        )

    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())

    def spin() -> None:
        while not stopped.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    spin_thread = threading.Thread(
        target=spin, name="camera-meta-spin", daemon=True
    )
    spin_thread.start()

    _switch_dwm_workspace(4)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1900, 1015)
    cv2.imshow(WINDOW_NAME, render(data))
    cv2.waitKey(1)
    return_to_rig_at = monotonic() + 0.75
    returned_to_rig = False

    while not stopped.is_set() and rclpy.ok():
        cv2.imshow(WINDOW_NAME, render(data))
        # Ten redraws per second are enough to make the 250 ms event history
        # feel live while avoiding a full CPU core for an always-on dashboard.
        key = cv2.waitKey(100) & 0xFF
        if key in (ord("q"), 27):
            break
        if not returned_to_rig and monotonic() >= return_to_rig_at:
            _switch_dwm_workspace(1)
            returned_to_rig = True

    stopped.set()
    spin_thread.join(timeout=1.0)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
