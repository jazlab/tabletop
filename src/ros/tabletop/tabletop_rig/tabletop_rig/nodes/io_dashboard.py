"""Compact noVNC workspace-5 dashboard for TableTop rig I/O checks."""

from __future__ import annotations

import ctypes
import os
import queue
import signal
import threading
from dataclasses import dataclass, field, replace
from time import monotonic

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from mingus.containers import Note
from mingus.midi import fluidsynth
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Header
from tabletop_interfaces.msg import TeensySensor
from tabletop_interfaces.srv import SetBuzzer, SetReward, SetSmartglass

WINDOW_NAME = "TableTop I/O Check"
WIDTH = 1900
HEIGHT = 1015
SENSOR_STALE_SECONDS = 0.5
EVENT_FLASH_SECONDS = 0.8

BG = (33, 28, 26)
PANEL = (53, 47, 44)
PANEL_2 = (63, 56, 52)
TEXT = (242, 239, 235)
MUTED = (164, 153, 146)
GREEN = (116, 205, 133)
AMBER = (96, 191, 239)
RED = (105, 105, 237)
BLUE = (229, 157, 90)
GREY = (105, 98, 94)
BORDER = (84, 76, 71)


@dataclass(frozen=True)
class Rect:
    x1: int
    y1: int
    x2: int
    y2: int

    def contains(self, x: int, y: int) -> bool:
        return self.x1 <= x <= self.x2 and self.y1 <= y <= self.y2


BUTTON_RECTS = {
    "smartglass": Rect(44, 555, 384, 790),
    "reward": Rect(412, 555, 752, 790),
    "sound": Rect(780, 555, 1120, 790),
    "left_buzzer": Rect(1148, 555, 1488, 790),
    "right_buzzer": Rect(1516, 555, 1856, 790),
}


@dataclass
class DashboardSnapshot:
    sensor_received_at: float = 0.0
    left_pressed: bool = False
    right_pressed: bool = False
    laser_broken: bool = False
    smartglass_revealed: bool = False
    reward_active: bool = False
    left_count: int = 0
    right_count: int = 0
    laser_count: int = 0
    left_event_at: float = 0.0
    right_event_at: float = 0.0
    laser_event_at: float = 0.0
    flic_online: bool = False
    flic_count: int = 0
    flic_address: str = ""
    flic_event_at: float = 0.0
    task_running: bool = False
    services: dict[str, bool] = field(default_factory=dict)
    deadlines: dict[str, float] = field(default_factory=dict)
    toast: str = "Waiting for rig I/O..."
    toast_error: bool = False
    sound_available: bool = False
    sound_error: str = ""


class RewardSound:
    """Small, process-local version of the task's configured reward tone."""

    def __init__(self) -> None:
        self.available = False
        self.error = ""
        self._lock = threading.Lock()
        self._timer: threading.Timer | None = None
        self._note = Note("C", octave=6)
        self._note.velocity = 127
        self._note.channel = 0

        if os.environ.get("TABLETOP_DASHBOARD_DISABLE_AUDIO") == "1":
            self.error = "disabled for test"
            return
        try:
            soundfont = os.path.join(
                get_package_share_directory("tabletop_rig"),
                "soundfonts",
                "moog.sf2",
            )
            if not fluidsynth.init(soundfont, driver="pulseaudio"):
                raise RuntimeError("FluidSynth could not open PulseAudio")
            fluidsynth.set_instrument(channel=0, midi_instr=62)
            self.available = True
        except Exception as exc:  # audio absence must not take down diagnostics
            self.error = str(exc)

    def play(self, duration: float = 0.5) -> None:
        if not self.available:
            raise RuntimeError(self.error or "reward sound unavailable")
        with self._lock:
            if self._timer is not None:
                self._timer.cancel()
                fluidsynth.stop_Note(self._note)
            fluidsynth.play_Note(self._note)
            self._timer = threading.Timer(duration, self.stop)
            self._timer.daemon = True
            self._timer.start()

    def stop(self) -> None:
        with self._lock:
            if self.available:
                fluidsynth.stop_Note(self._note)
            self._timer = None

    def close(self) -> None:
        with self._lock:
            if self._timer is not None:
                self._timer.cancel()
            if self.available:
                fluidsynth.stop_everything()
            self._timer = None


class IODashboardNode(Node):
    """ROS subscriptions and bounded output commands behind the GUI."""

    def __init__(self, *, enable_audio: bool = True) -> None:
        super().__init__("io_dashboard")
        self._lock = threading.RLock()
        self._state = DashboardSnapshot()
        self._commands: queue.SimpleQueue[str] = queue.SimpleQueue()
        self._previous_left: bool | None = None
        self._previous_right: bool | None = None
        self._previous_laser: bool | None = None
        self._last_graph_check = 0.0
        self._sound = RewardSound() if enable_audio else RewardSound.__new__(RewardSound)
        if not enable_audio:
            self._sound.available = False
            self._sound.error = "disabled for test"
            self._sound._lock = threading.Lock()
            self._sound._timer = None
            self._sound._note = Note("C", octave=6)

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._sensor_sub = self.create_subscription(
            TeensySensor, "/teensy/sensor", self._sensor_callback, qos
        )
        self._flic_sub = self.create_subscription(
            Header, "/flic/button_pressed_time", self._flic_callback, 10
        )
        self._smartglass_client = self.create_client(
            SetSmartglass, "/teensy/set_smartglass"
        )
        self._reward_client = self.create_client(SetReward, "/teensy/set_reward")
        self._buzzer_client = self.create_client(SetBuzzer, "/teensy/set_buzzer")
        self._control_timer = self.create_timer(0.05, self._control_tick)

        with self._lock:
            self._state.sound_available = self._sound.available
            self._state.sound_error = self._sound.error

    def _sensor_callback(self, msg: TeensySensor) -> None:
        now = monotonic()
        with self._lock:
            left = bool(msg.is_left_arm_locked)
            right = bool(msg.is_right_arm_locked)
            laser = bool(msg.is_safety_laser_broken)
            if self._previous_left is False and left:
                self._state.left_count += 1
                self._state.left_event_at = now
            if self._previous_right is False and right:
                self._state.right_count += 1
                self._state.right_event_at = now
            if self._previous_laser is False and laser:
                self._state.laser_count += 1
                self._state.laser_event_at = now
            self._previous_left = left
            self._previous_right = right
            self._previous_laser = laser
            self._state.sensor_received_at = now
            self._state.left_pressed = left
            self._state.right_pressed = right
            self._state.laser_broken = laser
            self._state.smartglass_revealed = bool(
                msg.is_smartglass_revealed
            )
            self._state.reward_active = bool(msg.is_reward_active)

    def _flic_callback(self, msg: Header) -> None:
        with self._lock:
            self._state.flic_count += 1
            self._state.flic_address = msg.frame_id
            self._state.flic_event_at = monotonic()
            self._state.toast = f"Flic registered: {msg.frame_id}"
            self._state.toast_error = False

    def enqueue(self, command: str) -> None:
        self._commands.put(command)

    def snapshot(self) -> DashboardSnapshot:
        with self._lock:
            return replace(
                self._state,
                services=dict(self._state.services),
                deadlines=dict(self._state.deadlines),
            )

    def _set_toast(self, message: str, *, error: bool = False) -> None:
        with self._lock:
            self._state.toast = message
            self._state.toast_error = error

    def _control_tick(self) -> None:
        now = monotonic()
        if now - self._last_graph_check >= 1.0:
            names = set(self.get_node_names())
            with self._lock:
                self._state.task_running = "commander" in names
                self._state.flic_online = "flic" in names
                self._state.services = {
                    "smartglass": self._smartglass_client.service_is_ready(),
                    "reward": self._reward_client.service_is_ready(),
                    "buzzer": self._buzzer_client.service_is_ready(),
                }
            self._last_graph_check = now

        for _ in range(8):
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                break
            self._run_command(command)

    def _output_allowed(self, service: str | None = None) -> bool:
        # Recheck the graph at click time instead of relying only on the
        # one-second display refresh. This closes the startup race in which a
        # Commander node appears just before an output test is clicked.
        task_running = "commander" in set(self.get_node_names())
        with self._lock:
            self._state.task_running = task_running
            if task_running:
                self._state.toast = "Output tests are locked while Commander is running"
                self._state.toast_error = True
                return False
            if service and not self._state.services.get(service, False):
                self._state.toast = f"{service.title()} service is offline"
                self._state.toast_error = True
                return False
        return True

    def _call(
        self,
        name: str,
        client,
        request,
        duration: float,
        success_message: str,
    ) -> None:
        with self._lock:
            if self._state.deadlines.get(name, 0.0) > monotonic():
                return
            self._state.deadlines[name] = monotonic() + duration
            self._state.toast = f"Starting {name.replace('_', ' ')} test..."
            self._state.toast_error = False
        future = client.call_async(request)

        def complete(done) -> None:
            try:
                response = done.result()
                if response is None or not response.success:
                    message = getattr(response, "message", "no response")
                    raise RuntimeError(message)
                self._set_toast(success_message)
            except Exception as exc:
                with self._lock:
                    self._state.deadlines[name] = 0.0
                self._set_toast(f"{name.replace('_', ' ').title()} failed: {exc}", error=True)

        future.add_done_callback(complete)

    def _run_command(self, command: str) -> None:
        now = monotonic()
        if command == "sound":
            if not self._output_allowed():
                return
            if not self._sound.available:
                self._set_toast(
                    f"Reward sound unavailable: {self._sound.error}", error=True
                )
                return
            try:
                self._sound.play(0.5)
                with self._lock:
                    self._state.deadlines["sound"] = now + 0.5
                self._set_toast("Reward sound played")
            except Exception as exc:
                self._set_toast(f"Reward sound failed: {exc}", error=True)
            return

        if command == "smartglass":
            if not self._output_allowed("smartglass"):
                return
            with self._lock:
                if now - self._state.sensor_received_at > SENSOR_STALE_SECONDS:
                    self._state.toast = "Cannot test smartglass: Teensy sensor data is stale"
                    self._state.toast_error = True
                    return
                target = not self._state.smartglass_revealed
            request = SetSmartglass.Request()
            request.reveal = target
            request.duration = Duration(seconds=1.0).to_msg()
            self._call(
                "smartglass",
                self._smartglass_client,
                request,
                1.0,
                "Smartglass changed for one second and will restore automatically",
            )
            return

        if command == "reward":
            if not self._output_allowed("reward"):
                return
            request = SetReward.Request()
            request.activate = True
            request.duration = Duration(seconds=0.2).to_msg()
            self._call(
                "reward",
                self._reward_client,
                request,
                0.2,
                "Juice solenoid pulsed for 200 ms",
            )
            return

        if command in ("left_buzzer", "right_buzzer"):
            if not self._output_allowed("buzzer"):
                return
            request = SetBuzzer.Request()
            request.left_arm = command == "left_buzzer"
            request.right_arm = command == "right_buzzer"
            side = "Left" if request.left_arm else "Right"
            self._call(
                command,
                self._buzzer_client,
                request,
                1.0,
                f"{side} hand buzzer pulsed for one second",
            )

    def destroy_node(self) -> bool:
        self._sound.close()
        return super().destroy_node()


def _rounded_rect(
    canvas: np.ndarray,
    rect: Rect,
    color: tuple[int, int, int],
    radius: int = 18,
    thickness: int = -1,
) -> None:
    if thickness != -1:
        cv2.rectangle(
            canvas, (rect.x1, rect.y1), (rect.x2, rect.y2), color, thickness, cv2.LINE_AA
        )
        return
    cv2.rectangle(
        canvas,
        (rect.x1 + radius, rect.y1),
        (rect.x2 - radius, rect.y2),
        color,
        -1,
    )
    cv2.rectangle(
        canvas,
        (rect.x1, rect.y1 + radius),
        (rect.x2, rect.y2 - radius),
        color,
        -1,
    )
    for x, y in (
        (rect.x1 + radius, rect.y1 + radius),
        (rect.x2 - radius, rect.y1 + radius),
        (rect.x1 + radius, rect.y2 - radius),
        (rect.x2 - radius, rect.y2 - radius),
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


def _input_card(
    canvas: np.ndarray,
    rect: Rect,
    title: str,
    state: str,
    detail: str,
    color: tuple[int, int, int],
    count: int,
) -> None:
    _rounded_rect(canvas, rect, PANEL)
    cv2.circle(canvas, (rect.x1 + 42, rect.y1 + 45), 13, color, -1, cv2.LINE_AA)
    _text(canvas, title, (rect.x1 + 70, rect.y1 + 53), 0.66, TEXT, 2)
    _text(canvas, state, (rect.x1 + 28, rect.y1 + 112), 0.88, color, 2)
    _text(canvas, detail, (rect.x1 + 28, rect.y1 + 148), 0.50, MUTED)
    _text(canvas, f"Events  {count}", (rect.x1 + 28, rect.y2 - 24), 0.52, MUTED)


def _output_card(
    canvas: np.ndarray,
    rect: Rect,
    title: str,
    duration: str,
    detail: str,
    enabled: bool,
    busy: bool,
    hovered: bool,
) -> None:
    fill = PANEL_2 if enabled else (48, 44, 42)
    if hovered and enabled and not busy:
        fill = (76, 67, 61)
    _rounded_rect(canvas, rect, fill)
    color = AMBER if busy else (BLUE if enabled else GREY)
    cv2.circle(canvas, (rect.x1 + 34, rect.y1 + 38), 10, color, -1, cv2.LINE_AA)
    _text(canvas, title, (rect.x1 + 56, rect.y1 + 46), 0.62, TEXT if enabled else MUTED, 2)
    _text(canvas, duration, (rect.x1 + 25, rect.y1 + 101), 0.78, color, 2)
    _text(canvas, detail, (rect.x1 + 25, rect.y1 + 140), 0.47, MUTED)
    label = "ACTIVE" if busy else ("CLICK TO TEST" if enabled else "OFFLINE / LOCKED")
    _rounded_rect(
        canvas,
        Rect(rect.x1 + 25, rect.y2 - 62, rect.x2 - 25, rect.y2 - 22),
        color if enabled else GREY,
        radius=10,
    )
    size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)[0]
    x = (rect.x1 + rect.x2 - size[0]) // 2
    _text(canvas, label, (x, rect.y2 - 36), 0.48, BG if enabled else TEXT, 1)


def render_dashboard(
    snapshot: DashboardSnapshot,
    *,
    now: float | None = None,
    hovered: str | None = None,
) -> np.ndarray:
    """Render a deterministic dashboard frame (also used by headless tests)."""
    now = monotonic() if now is None else now
    canvas = np.full((HEIGHT, WIDTH, 3), BG, dtype=np.uint8)
    sensor_fresh = now - snapshot.sensor_received_at <= SENSOR_STALE_SECONDS
    outputs_locked = snapshot.task_running

    _text(canvas, "TableTop I/O Check", (44, 58), 1.15, TEXT, 2)
    _text(
        canvas,
        "Live inputs and short, controller-bounded output tests",
        (44, 91),
        0.56,
        MUTED,
    )
    banner = (
        "OUTPUT TESTS LOCKED - COMMANDER IS RUNNING"
        if outputs_locked
        else "BENCH DIAGNOSTICS - OUTPUTS ARE READY WHEN THEIR SERVICE IS ONLINE"
    )
    banner_color = RED if outputs_locked else GREEN
    _rounded_rect(canvas, Rect(1260, 29, 1856, 88), PANEL, radius=14)
    _text(canvas, banner, (1282, 66), 0.46, banner_color, 1)

    _text(canvas, "INPUTS", (44, 137), 0.56, MUTED, 2)
    cards = [
        Rect(44, 160, 475, 410),
        Rect(503, 160, 934, 410),
        Rect(962, 160, 1393, 410),
        Rect(1421, 160, 1856, 410),
    ]

    flic_recent = now - snapshot.flic_event_at < EVENT_FLASH_SECONDS
    flic_color = GREEN if flic_recent else (BLUE if snapshot.flic_online else GREY)
    flic_state = "REGISTERED" if flic_recent else ("READY" if snapshot.flic_online else "NO FLIC NODE")
    flic_detail = snapshot.flic_address or "Press any Flic button"
    _input_card(canvas, cards[0], "Any Flic button", flic_state, flic_detail, flic_color, snapshot.flic_count)

    def hand_values(pressed: bool, event_at: float) -> tuple[str, tuple[int, int, int]]:
        if not sensor_fresh:
            return "NO SENSOR DATA", GREY
        if pressed:
            return "PRESSED", GREEN
        return "RELEASED", BLUE if now - event_at < EVENT_FLASH_SECONDS else MUTED

    left_state, left_color = hand_values(snapshot.left_pressed, snapshot.left_event_at)
    right_state, right_color = hand_values(snapshot.right_pressed, snapshot.right_event_at)
    _input_card(canvas, cards[1], "Left hand button", left_state, "Teensy pin 36", left_color, snapshot.left_count)
    _input_card(canvas, cards[2], "Right hand button", right_state, "Teensy pin 39", right_color, snapshot.right_count)

    if not sensor_fresh:
        laser_state, laser_detail, laser_color = "NO SENSOR DATA", "Teensy messages are stale", GREY
    elif snapshot.laser_broken:
        laser_state, laser_detail, laser_color = "BEAM BROKEN", "Motion safety input active", RED
    else:
        laser_state, laser_detail, laser_color = "CLEAR", "Safety beam intact", GREEN
    _input_card(canvas, cards[3], "Safety laser", laser_state, laser_detail, laser_color, snapshot.laser_count)

    _text(canvas, "TIMED OUTPUTS", (44, 515), 0.56, MUTED, 2)
    definitions = {
        "smartglass": ("Smartglass", "1 second", "Invert, then restore", "smartglass"),
        "reward": ("Juice solenoid", "200 ms", "Firmware-timed pulse", "reward"),
        "sound": ("Reward sound", "brief tone", "Task reward tone", None),
        "left_buzzer": ("Left hand buzzer", "1 second", "No lock movement", "buzzer"),
        "right_buzzer": ("Right hand buzzer", "1 second", "No lock movement", "buzzer"),
    }
    for name, rect in BUTTON_RECTS.items():
        title, duration, detail, service = definitions[name]
        available = snapshot.sound_available if name == "sound" else snapshot.services.get(service or "", False)
        enabled = available and not outputs_locked
        busy = snapshot.deadlines.get(name, 0.0) > now
        _output_card(canvas, rect, title, duration, detail, enabled, busy, hovered == name)

    smartglass = (
        "no Teensy data"
        if not sensor_fresh
        else ("transparent" if snapshot.smartglass_revealed else "opaque")
    )
    reward = (
        "no Teensy data"
        if not sensor_fresh
        else ("ACTIVE" if snapshot.reward_active else "off")
    )
    _text(canvas, f"Smartglass now: {smartglass}", (44, 838), 0.50, MUTED)
    _text(canvas, f"Juice now: {reward}", (410, 838), 0.50, MUTED)
    _text(
        canvas,
        "Tests never command either robot. Buzzer tests do not change arm-lock outputs.",
        (780, 838),
        0.50,
        MUTED,
    )

    toast_color = RED if snapshot.toast_error else GREEN
    _rounded_rect(canvas, Rect(44, 875, 1856, 961), PANEL, radius=16)
    cv2.circle(canvas, (78, 918), 10, toast_color, -1, cv2.LINE_AA)
    _text(canvas, snapshot.toast, (104, 925), 0.60, TEXT, 1)
    _text(canvas, "Esc or Q closes this dashboard", (1585, 925), 0.45, MUTED)
    return canvas


def _switch_dwm_workspace(number: int) -> bool:
    """Send Alt+number to dwm using XTest, matching the existing GUI services."""
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
    node = IODashboardNode()
    stopped = threading.Event()
    hovered: str | None = None

    def request_stop(*_args) -> None:
        stopped.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    executor_thread = threading.Thread(
        target=rclpy.spin, args=(node,), name="io-dashboard-ros", daemon=True
    )
    executor_thread.start()

    _switch_dwm_workspace(5)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, WIDTH, HEIGHT)

    def mouse(event: int, x: int, y: int, _flags: int, _param) -> None:
        nonlocal hovered
        hovered = next(
            (name for name, rect in BUTTON_RECTS.items() if rect.contains(x, y)),
            None,
        )
        if event == cv2.EVENT_LBUTTONUP and hovered is not None:
            node.enqueue(hovered)

    cv2.setMouseCallback(WINDOW_NAME, mouse)
    return_to_rig_at = monotonic() + 0.75
    returned_to_rig = False

    try:
        while not stopped.is_set() and rclpy.ok():
            cv2.imshow(
                WINDOW_NAME,
                render_dashboard(node.snapshot(), hovered=hovered),
            )
            key = cv2.waitKey(50) & 0xFF
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
        executor_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
