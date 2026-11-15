"""Camera-only noVNC view; consumes bounded previews, never acquisition/control."""

from __future__ import annotations

import signal
import threading
from time import monotonic

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from tabletop_rig.utils.novnc_workspace import reserve_dwm_workspace

WINDOW_NAME = "TableTop Six Camera Views"
CAMERAS = (
    ("left_front_top_cam", "Left front"),
    ("left_back_top_cam", "Left back"),
    ("left_bottom_cam", "Left bottom"),
    ("right_front_top_cam", "Right front"),
    ("right_back_top_cam", "Right back"),
    ("right_bottom_cam", "Right bottom"),
)


def preview_frame(message: Image) -> np.ndarray:
    """Honor row padding and encoding; retain the message buffer via NumPy."""
    channels = {"mono8": 1, "bgr8": 3, "rgb8": 3}.get(message.encoding)
    if channels is None or message.height <= 0 or message.width <= 0:
        raise ValueError("Unsupported or empty preview image")
    if message.step < message.width * channels:
        raise ValueError("Preview row stride is too short")
    data = np.frombuffer(message.data, dtype=np.uint8)
    if data.size < message.height * message.step:
        raise ValueError("Truncated preview image")
    rows = data[: message.height * message.step].reshape(
        message.height, message.step
    )
    frame = rows[:, : message.width * channels]
    frame = frame.reshape(message.height, message.width, channels)
    if channels == 1:
        return frame[:, :, 0]
    return frame[:, :, ::-1] if message.encoding == "rgb8" else frame


class CameraGrid(Node):
    def __init__(self) -> None:
        super().__init__("camera_grid")
        self.lock = threading.Lock()
        self.frames: dict[str, tuple[np.ndarray, float]] = {}
        self.subscriptions_ = []
        for camera, _ in CAMERAS:
            self.subscriptions_.append(
                self.create_subscription(
                    Image,
                    f"/cam_preview/{camera}/image_raw",
                    lambda message, camera=camera: self.receive(
                        camera, message
                    ),
                    QoSProfile(
                        depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
                    ),
                )
            )

    def receive(self, camera: str, message: Image) -> None:
        try:
            frame = preview_frame(message)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        with self.lock:
            self.frames[camera] = (frame, monotonic())

    def snapshot(self) -> dict[str, tuple[np.ndarray, float]]:
        with self.lock:
            return dict(self.frames)


def render(
    frames: dict,
    width: int = 1900,
    height: int = 1015,
    now: float | None = None,
) -> np.ndarray:
    now = monotonic() if now is None else now
    canvas = np.full((height, width, 3), (24, 27, 31), dtype=np.uint8)
    cv2.putText(
        canvas,
        "CAMERAS",
        (24, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (230, 234, 240),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        "Live previews  /  acquisition and recording unchanged",
        (190, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (145, 153, 165),
        1,
        cv2.LINE_AA,
    )
    gap, top = 14, 55
    cell_w = (width - 4 * gap) // 3
    cell_h = (height - top - 3 * gap) // 2
    for index, (camera, label) in enumerate(CAMERAS):
        x = gap + (index % 3) * (cell_w + gap)
        y = top + gap + (index // 3) * (cell_h + gap)
        cv2.rectangle(
            canvas, (x, y), (x + cell_w, y + cell_h), (38, 42, 48), -1
        )
        cv2.putText(
            canvas,
            label,
            (x + 14, y + 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (235, 238, 242),
            1,
            cv2.LINE_AA,
        )
        entry = frames.get(camera)
        if entry is None:
            cv2.putText(
                canvas,
                "Waiting for preview",
                (x + 25, y + cell_h // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (135, 145, 158),
                1,
                cv2.LINE_AA,
            )
            continue
        frame, received = entry
        live = now - received < 2.0
        cv2.putText(
            canvas,
            "LIVE" if live else "STALE",
            (x + cell_w - 80, y + 27),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (124, 214, 136) if live else (90, 170, 245),
            1,
            cv2.LINE_AA,
        )
        scale = min(
            (cell_w - 16) / frame.shape[1], (cell_h - 52) / frame.shape[0]
        )
        image = cv2.resize(
            frame,
            (
                max(1, int(frame.shape[1] * scale)),
                max(1, int(frame.shape[0] * scale)),
            ),
        )
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        iy = y + 44 + (cell_h - 52 - image.shape[0]) // 2
        ix = x + (cell_w - image.shape[1]) // 2
        canvas[iy : iy + image.shape[0], ix : ix + image.shape[1]] = image
    return canvas


def main(args=None) -> None:
    from tabletop_rig.utils.novnc_visibility import WindowVisibility

    rclpy.init(args=args)
    node = CameraGrid()
    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())

    def spin() -> None:
        while not stopped.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    visible = WindowVisibility(WINDOW_NAME)
    try:
        with reserve_dwm_workspace(1):
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW_NAME, 1900, 1015)
            cv2.imshow(WINDOW_NAME, render(node.snapshot()))
            cv2.waitKey(1)
        while not stopped.is_set() and rclpy.ok():
            if visible():
                cv2.imshow(WINDOW_NAME, render(node.snapshot()))
            if cv2.waitKey(100) & 0xFF in (27, ord("q")):
                break
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        stopped.set()
        thread.join(timeout=2)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
