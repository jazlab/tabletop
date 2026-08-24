"""Passive noVNC workspace-6 monitor for device-observed sync TTLs."""

from __future__ import annotations

import ctypes
import os
import signal
import threading
from collections import deque
from dataclasses import dataclass, field, replace
from statistics import median
from time import monotonic

import cv2
import numpy as np
import rclpy
from flir_camera_msgs.msg import ImageMetaData
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import UInt16
from tabletop_interfaces.msg import TeensySensor
from ur_msgs.msg import IOStates

WINDOW_NAME = "TableTop TTL Monitor"
WIDTH = 1900
HEIGHT = 1015
TRACE_SECONDS = 8.0
DATA_STALE_SECONDS = 0.5
PULSE_STALE_SECONDS = 1.8
UR_MATCH_WINDOW_SECONDS = 0.075
UR_MATCHES_REQUIRED = 3
FLIR_CAMERA = os.environ.get("TABLETOP_TTL_FLIR_CAMERA", "left_back_top_cam")
FLIR_LINE = int(os.environ.get("TABLETOP_TTL_FLIR_LINE", "3"))
EYELINK_MASK = int(os.environ.get("TABLETOP_TTL_EYELINK_MASK", "8"), 0)

BG = (33, 28, 26)
PANEL = (53, 47, 44)
TEXT = (242, 239, 235)
MUTED = (164, 153, 146)
GREEN = (116, 205, 133)
AMBER = (96, 191, 239)
RED = (105, 105, 237)
BLUE = (229, 157, 90)
GREY = (105, 98, 94)
GRID = (79, 70, 66)

CHANNELS = ("teensy", "eyelink", "robot_1", "robot_2", "flir")
LABELS = {
    "teensy": "Teensy source",
    "eyelink": "EyeLink",
    "robot_1": "Robot 1 / left",
    "robot_2": "Robot 2 / right",
    "flir": "FLIR sync camera",
}
COLORS = {
    "teensy": GREEN,
    "eyelink": (226, 157, 239),
    "robot_1": (229, 157, 90),
    "robot_2": (102, 209, 255),
    "flir": (143, 227, 102),
}


@dataclass
class TTLChannel:
    received_at: float = 0.0
    initialized: bool = False
    state: bool = False
    pulse_count: int = 0
    last_rise_at: float = 0.0
    last_fall_at: float = 0.0
    last_lag_ms: float | None = None
    detail: str = ""
    edges: list[tuple[float, bool]] = field(default_factory=list)


@dataclass
class TTLSnapshot:
    channels: dict[str, TTLChannel]
    robot_inputs: dict[str, tuple[int, bool] | None]
    eyelink_online: bool = False
    reference_period_ms: float | None = None


class TTLMonitorState:
    """Thread-safe TTL state and UR input-correlation detector."""

    def __init__(
        self,
        configured_robot_inputs: dict[str, tuple[int, bool] | None]
        | None = None,
    ) -> None:
        self.lock = threading.RLock()
        self.channels = {name: TTLChannel() for name in CHANNELS}
        self.channels["teensy"].detail = "Teensy pin 0"
        self.channels["eyelink"].detail = "Input bit 3, active-low"
        self.channels["flir"].detail = f"{FLIR_CAMERA} Line{FLIR_LINE}"
        self.robot_inputs = {"robot_1": None, "robot_2": None}
        if configured_robot_inputs:
            self.robot_inputs.update(configured_robot_inputs)
        self.eyelink_online = False
        self.reference_period_ms: float | None = None
        self._reference_rises: deque[tuple[int, float]] = deque(maxlen=32)
        self._reference_sequence = 0
        self._robot_values = {"robot_1": {}, "robot_2": {}}
        self._robot_transitions = {
            "robot_1": deque(maxlen=128),
            "robot_2": deque(maxlen=128),
        }
        self._robot_candidates = {"robot_1": {}, "robot_2": {}}

    def snapshot(self) -> TTLSnapshot:
        with self.lock:
            return TTLSnapshot(
                channels={
                    name: replace(channel, edges=list(channel.edges))
                    for name, channel in self.channels.items()
                },
                robot_inputs=dict(self.robot_inputs),
                eyelink_online=self.eyelink_online,
                reference_period_ms=self.reference_period_ms,
            )

    def set_eyelink_online(self, online: bool) -> None:
        with self.lock:
            self.eyelink_online = online

    def _update_channel(
        self, name: str, state: bool, now: float, detail: str | None = None
    ) -> tuple[bool, bool]:
        channel = self.channels[name]
        channel.received_at = now
        if detail is not None:
            channel.detail = detail
        if not channel.initialized:
            channel.initialized = True
            channel.state = state
            channel.edges.append((now, state))
            return False, False
        if channel.state == state:
            return False, False
        channel.state = state
        channel.edges.append((now, state))
        cutoff = now - TRACE_SECONDS - 2.0
        channel.edges = [edge for edge in channel.edges if edge[0] >= cutoff]
        if state:
            channel.pulse_count += 1
            channel.last_rise_at = now
            if name != "teensy" and self._reference_rises:
                lag = now - self._reference_rises[-1][1]
                if abs(lag) <= 0.3:
                    channel.last_lag_ms = lag * 1000.0
            return True, True
        channel.last_fall_at = now
        return False, True

    def record_teensy(self, state: bool, now: float | None = None) -> None:
        now = monotonic() if now is None else now
        with self.lock:
            rising, _ = self._update_channel(
                "teensy", state, now, "Teensy pin 0 - 100 ms pulse"
            )
            if not rising:
                return
            self._reference_sequence += 1
            if self._reference_rises:
                self.reference_period_ms = (
                    now - self._reference_rises[-1][1]
                ) * 1000.0
            self._reference_rises.append((self._reference_sequence, now))
            for side in ("robot_1", "robot_2"):
                for when, pin, level in tuple(self._robot_transitions[side]):
                    if abs(when - now) <= UR_MATCH_WINDOW_SECONDS:
                        self._record_robot_match(
                            side,
                            pin,
                            level,
                            when,
                            self._reference_sequence,
                            now,
                        )

    def record_eyelink(self, raw_input: int, now: float | None = None) -> None:
        now = monotonic() if now is None else now
        active = not bool(raw_input & EYELINK_MASK)
        with self.lock:
            self._update_channel(
                "eyelink",
                active,
                now,
                f"Input bit 3 active-low - raw {raw_input}",
            )

    def record_flir(self, line_status: int, now: float | None = None) -> None:
        now = monotonic() if now is None else now
        active = bool(line_status & (1 << FLIR_LINE))
        with self.lock:
            self._update_channel(
                "flir",
                active,
                now,
                f"{FLIR_CAMERA} Line{FLIR_LINE} - raw {line_status}",
            )

    def record_robot(
        self,
        side: str,
        values: dict[int, bool],
        now: float | None = None,
    ) -> None:
        now = monotonic() if now is None else now
        with self.lock:
            channel = self.channels[side]
            channel.received_at = now
            previous = self._robot_values[side]
            for pin, level in values.items():
                if pin in previous and previous[pin] != level:
                    self._robot_transitions[side].append((now, pin, level))
                    if self._reference_rises:
                        sequence, reference_at = self._reference_rises[-1]
                        if abs(now - reference_at) <= UR_MATCH_WINDOW_SECONDS:
                            self._record_robot_match(
                                side,
                                pin,
                                level,
                                now,
                                sequence,
                                reference_at,
                            )
                previous[pin] = level

            selected = self.robot_inputs[side]
            if selected is None:
                channel.detail = f"Scanning {len(values)} digital inputs - no correlated input"
                return
            pin, active_level = selected
            if pin not in values:
                channel.detail = f"Configured DI {pin} is absent"
                return
            polarity = "high" if active_level else "low"
            self._update_channel(
                side,
                values[pin] == active_level,
                now,
                f"DI {pin} - active-{polarity}",
            )

    def _record_robot_match(
        self,
        side: str,
        pin: int,
        active_level: bool,
        transition_at: float,
        sequence: int,
        reference_at: float,
    ) -> None:
        if self.robot_inputs[side] is not None:
            return
        key = (pin, active_level)
        candidates = self._robot_candidates[side]
        matches = candidates.setdefault(key, {})
        matches.setdefault(sequence, transition_at - reference_at)
        eligible = [
            (candidate, lags)
            for candidate, lags in candidates.items()
            if len(lags) >= UR_MATCHES_REQUIRED
        ]
        if not eligible:
            return
        best, lags = min(
            eligible,
            key=lambda item: (
                -len(item[1]),
                median(abs(value) for value in item[1].values()),
                item[0][0],
            ),
        )
        self.robot_inputs[side] = best
        pin, level = best
        polarity = "high" if level else "low"
        channel = self.channels[side]
        channel.detail = f"DI {pin} - auto-detected active-{polarity}"
        channel.last_lag_ms = median(lags.values()) * 1000.0
        channel.initialized = False


def _optional_robot_input(prefix: str) -> tuple[int, bool] | None:
    raw = os.environ.get(f"TABLETOP_TTL_{prefix}_PIN", "").strip()
    if not raw:
        return None
    pin = int(raw)
    active_low = os.environ.get(
        f"TABLETOP_TTL_{prefix}_ACTIVE_LOW", "false"
    ).lower() in ("1", "true", "yes")
    return pin, not active_low


class TTLMonitorNode(Node):
    """ROS subscriptions behind the passive TTL dashboard."""

    def __init__(self) -> None:
        super().__init__("ttl_dashboard")
        self.state = TTLMonitorState(
            {
                "robot_1": _optional_robot_input("ROBOT1"),
                "robot_2": _optional_robot_input("ROBOT2"),
            }
        )
        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._subscriptions = [
            self.create_subscription(
                TeensySensor,
                "/teensy/sensor",
                lambda msg: self.state.record_teensy(
                    bool(msg.sync_pulse_state)
                ),
                qos,
            ),
            self.create_subscription(
                UInt16,
                "/eyelink/ttl_input",
                lambda msg: self.state.record_eyelink(int(msg.data)),
                qos,
            ),
            self.create_subscription(
                IOStates,
                "/left_io_and_status_controller/io_states",
                lambda msg: self.state.record_robot(
                    "robot_1",
                    {
                        int(item.pin): bool(item.state)
                        for item in msg.digital_in_states
                    },
                ),
                qos,
            ),
            self.create_subscription(
                IOStates,
                "/right_io_and_status_controller/io_states",
                lambda msg: self.state.record_robot(
                    "robot_2",
                    {
                        int(item.pin): bool(item.state)
                        for item in msg.digital_in_states
                    },
                ),
                qos,
            ),
            self.create_subscription(
                ImageMetaData,
                f"/cam_sync/{FLIR_CAMERA}/meta",
                lambda msg: self.state.record_flir(int(msg.line_status)),
                qos,
            ),
        ]
        self._graph_timer = self.create_timer(1.0, self._graph_tick)

    def _graph_tick(self) -> None:
        self.state.set_eyelink_online("eyelink" in set(self.get_node_names()))


def _rounded_rect(
    canvas: np.ndarray,
    x1: int,
    y1: int,
    x2: int,
    y2: int,
    color: tuple[int, int, int],
    radius: int = 18,
) -> None:
    cv2.rectangle(canvas, (x1 + radius, y1), (x2 - radius, y2), color, -1)
    cv2.rectangle(canvas, (x1, y1 + radius), (x2, y2 - radius), color, -1)
    for x, y in (
        (x1 + radius, y1 + radius),
        (x2 - radius, y1 + radius),
        (x1 + radius, y2 - radius),
        (x2 - radius, y2 - radius),
    ):
        cv2.circle(canvas, (x, y), radius, color, -1, cv2.LINE_AA)


def _text(
    canvas: np.ndarray,
    value: str,
    position: tuple[int, int],
    scale: float = 0.58,
    color: tuple[int, int, int] = TEXT,
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


def _channel_status(
    name: str,
    channel: TTLChannel,
    snapshot: TTLSnapshot,
    now: float,
) -> tuple[str, tuple[int, int, int]]:
    fresh = now - channel.received_at <= DATA_STALE_SECONDS
    if name == "eyelink" and not fresh:
        if snapshot.eyelink_online:
            return "WAITING FOR RECORDING", AMBER
        return "EYELINK OFFLINE", GREY
    if (
        name.startswith("robot")
        and fresh
        and snapshot.robot_inputs[name] is None
    ):
        return "NO CORRELATED TTL", AMBER
    if not fresh:
        return "NO DATA", GREY
    if channel.state:
        return "HIGH", GREEN
    if (
        channel.last_rise_at
        and now - channel.last_rise_at <= PULSE_STALE_SECONDS
    ):
        return "LIVE", BLUE
    return "NO RECENT PULSE", RED


def _draw_card(
    canvas: np.ndarray,
    name: str,
    channel: TTLChannel,
    snapshot: TTLSnapshot,
    rect: tuple[int, int, int, int],
    now: float,
) -> None:
    x1, y1, x2, y2 = rect
    _rounded_rect(canvas, x1, y1, x2, y2, PANEL)
    status, color = _channel_status(name, channel, snapshot, now)
    cv2.circle(canvas, (x1 + 34, y1 + 38), 11, color, -1, cv2.LINE_AA)
    _text(canvas, LABELS[name], (x1 + 56, y1 + 46), 0.65, TEXT, 2)
    _text(canvas, status, (x1 + 26, y1 + 102), 0.72, color, 2)
    _text(canvas, channel.detail, (x1 + 26, y1 + 140), 0.46, MUTED)
    lag = (
        "--"
        if channel.last_lag_ms is None
        else f"{channel.last_lag_ms:+.1f} ms"
    )
    _text(
        canvas,
        f"Pulses  {channel.pulse_count}     lag  {lag}",
        (x1 + 26, y2 - 25),
        0.50,
        MUTED,
    )


def _draw_trace(
    canvas: np.ndarray,
    channel: TTLChannel,
    color: tuple[int, int, int],
    plot_left: int,
    plot_right: int,
    y: int,
    now: float,
) -> None:
    low_y, high_y = y + 12, y - 22
    cv2.line(canvas, (plot_left, low_y), (plot_right, low_y), GRID, 1)
    if not channel.initialized:
        return
    start = now - TRACE_SECONDS
    state = False
    for when, value in channel.edges:
        if when <= start:
            state = value
        else:
            break
    last_x = plot_left
    current_y = high_y if state else low_y
    for when, value in channel.edges:
        if when < start or when > now:
            continue
        x = plot_left + int(
            (when - start) / TRACE_SECONDS * (plot_right - plot_left)
        )
        cv2.line(canvas, (last_x, current_y), (x, current_y), color, 2)
        next_y = high_y if value else low_y
        cv2.line(canvas, (x, current_y), (x, next_y), color, 2)
        current_y = next_y
        last_x = x
    cv2.line(canvas, (last_x, current_y), (plot_right, current_y), color, 2)


def render_dashboard(
    snapshot: TTLSnapshot, now: float | None = None
) -> np.ndarray:
    """Render one deterministic dashboard frame."""
    now = monotonic() if now is None else now
    canvas = np.full((HEIGHT, WIDTH, 3), BG, dtype=np.uint8)
    _text(canvas, "TableTop TTL Monitor", (44, 56), 1.12, TEXT, 2)
    _text(
        canvas,
        "Device-observed once-per-second sync - not the 120 Hz FLIR exposure trigger",
        (44, 90),
        0.55,
        MUTED,
    )

    reference = snapshot.channels["teensy"]
    ref_status, ref_color = _channel_status("teensy", reference, snapshot, now)
    _rounded_rect(canvas, 1250, 27, 1856, 105, PANEL, 15)
    period = (
        "--"
        if snapshot.reference_period_ms is None
        else f"{snapshot.reference_period_ms:.1f} ms"
    )
    _text(
        canvas,
        f"SOURCE {ref_status}   pin 0   period {period}",
        (1274, 74),
        0.52,
        ref_color,
        1,
    )

    cards = [
        (44, 140, 475, 390),
        (503, 140, 934, 390),
        (962, 140, 1393, 390),
        (1421, 140, 1856, 390),
    ]
    for name, rect in zip(("eyelink", "robot_1", "robot_2", "flir"), cards):
        _draw_card(canvas, name, snapshot.channels[name], snapshot, rect, now)

    _text(canvas, "ALIGNED LIVE TRACE", (44, 445), 0.56, MUTED, 2)
    _rounded_rect(canvas, 44, 470, 1856, 947, PANEL)
    plot_left, plot_right = 270, 1640
    for second in range(9):
        x = plot_left + int(second / 8 * (plot_right - plot_left))
        cv2.line(canvas, (x, 505), (x, 905), GRID, 1)
        _text(
            canvas,
            f"-{8 - second}s" if second < 8 else "now",
            (x - 18, 928),
            0.40,
            MUTED,
        )

    for index, name in enumerate(CHANNELS):
        y = 545 + index * 76
        color = COLORS[name]
        _text(canvas, LABELS[name], (72, y + 5), 0.54, color, 2)
        _draw_trace(
            canvas,
            snapshot.channels[name],
            color,
            plot_left,
            plot_right,
            y,
            now,
        )
        status, status_color = _channel_status(
            name, snapshot.channels[name], snapshot, now
        )
        _text(canvas, status, (1665, y + 5), 0.43, status_color, 1)

    _text(
        canvas,
        "This is a software-observed timing monitor, not an electrical oscilloscope.",
        (44, 986),
        0.48,
        MUTED,
    )
    _text(canvas, "Esc or Q closes this dashboard", (1600, 986), 0.44, MUTED)
    return canvas


def _switch_dwm_workspace(number: int) -> bool:
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
    node = TTLMonitorNode()
    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), name="ttl-dashboard-ros", daemon=True
    )
    spin_thread.start()

    _switch_dwm_workspace(6)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, WIDTH, HEIGHT)
    return_to_rig_at = monotonic() + 0.75
    returned_to_rig = False
    try:
        while not stopped.is_set() and rclpy.ok():
            cv2.imshow(WINDOW_NAME, render_dashboard(node.state.snapshot()))
            key = cv2.waitKey(100) & 0xFF
            if key in (ord("q"), 27):
                break
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
            if not returned_to_rig and monotonic() >= return_to_rig_at:
                _switch_dwm_workspace(1)
                returned_to_rig = True
    finally:
        stopped.set()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
