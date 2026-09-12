"""Passive noVNC workspace-7 dashboard for live session data health."""

from __future__ import annotations

import ctypes
import json
import os
import re
import shutil
import signal
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

import cv2
import numpy as np
import rclpy
from flir_camera_msgs.msg import ImageMetaData
from geometry_msgs.msg import PoseStamped, WrenchStamped
from mocap4r2_msgs.msg import Markers
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSPresetProfiles,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import UInt16
from std_srvs.srv import Trigger
from tabletop_interfaces.msg import EyelinkBatch, TaskEvent, TeensySensor
from ur_msgs.msg import IOStates

from tabletop_rig.nodes.ttl_dashboard import (
    EYELINK_MASK,
    FLIR_CAMERA,
    FLIR_LINE,
    OptitrackTTLDetector,
)

WINDOW_NAME = "TableTop Session Health"
WIDTH = 1900
HEIGHT = 1015
STREAM_STALE_SECONDS = 1.5
RATE_WINDOW_SECONDS = 5.0
TTL_SETTLE_SECONDS = 0.35
TTL_HEALTH_WINDOW_SECONDS = 60.0
PERSIST_PATH = Path(
    os.environ.get(
        "TABLETOP_SESSION_DASHBOARD_STATE",
        "/bags/.tabletop_session_health.json",
    )
)
BAG_ROOT = Path(os.environ.get("ROS_BAG_DIR", "/bags"))

CAMERAS = (
    "left_front_top_cam",
    "left_back_top_cam",
    "left_bottom_cam",
    "right_front_top_cam",
    "right_back_top_cam",
    "right_bottom_cam",
)
ROWS = (
    "teensy",
    "eyelink",
    "robot_1",
    "robot_2",
    "optitrack",
    *CAMERAS,
)
LABELS = {
    "teensy": "Teensy",
    "eyelink": "EyeLink",
    "robot_1": "Robot 1 / left",
    "robot_2": "Robot 2 / right",
    "optitrack": "OptiTrack",
    "left_front_top_cam": "FLIR left front",
    "left_back_top_cam": "FLIR left back",
    "left_bottom_cam": "FLIR left bottom",
    "right_front_top_cam": "FLIR right front",
    "right_back_top_cam": "FLIR right back",
    "right_bottom_cam": "FLIR right bottom",
}
REQUIRED_COMPONENTS = {
    "teensy": ("sensor",),
    "eyelink": ("samples",),
    "robot_1": ("pose", "wrench", "io"),
    "robot_2": ("pose", "wrench"),
    "optitrack": ("markers",),
    **{camera: ("metadata",) for camera in CAMERAS},
}
MAIN_COMPONENT = {
    "teensy": "sensor",
    "eyelink": "samples",
    "robot_1": "pose",
    "robot_2": "pose",
    "optitrack": "markers",
    **{camera: "metadata" for camera in CAMERAS},
}
TTL_TOLERANCE_SECONDS = {
    "eyelink": 0.100,
    "robot_1": 0.100,
    "optitrack": 0.200,
    FLIR_CAMERA: 0.100,
}
TTL_NA_DETAIL = {
    "robot_2": "N/A - intentionally unwired",
    **{
        camera: "N/A - TTL wired to left back only"
        for camera in CAMERAS
        if camera != FLIR_CAMERA
    },
}

TRANSPORT_LOSS_RE = re.compile(
    r"Messages lost on transport layer for topic '([^']+)'.*Total lost: (\d+)"
)

BG = (33, 28, 26)
PANEL = (53, 47, 44)
PANEL_ALT = (59, 53, 49)
TEXT = (242, 239, 235)
MUTED = (164, 153, 146)
GREEN = (116, 205, 133)
AMBER = (96, 191, 239)
RED = (105, 105, 237)
BLUE = (229, 157, 90)
GREY = (105, 98, 94)
BORDER = (84, 76, 71)
STOP_BUTTON = (1268, 28, 1500, 92)


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _format_duration(seconds: float) -> str:
    seconds = max(0, int(round(seconds)))
    hours, remainder = divmod(seconds, 3600)
    minutes, secs = divmod(remainder, 60)
    return f"{hours:d}:{minutes:02d}:{secs:02d}"


def _format_bytes(value: int) -> str:
    amount = float(max(0, value))
    for suffix in ("B", "KiB", "MiB", "GiB", "TiB", "PiB"):
        if amount < 1024.0 or suffix == "PiB":
            return f"{amount:.1f} {suffix}"
        amount /= 1024.0
    return "--"


def _arm_name(group_name: str) -> str | None:
    lowered = group_name.lower()
    if lowered.startswith("left"):
        return "left"
    if lowered.startswith("right"):
        return "right"
    return None


def _row_for_topic(topic: str) -> tuple[str, ...]:
    for camera in CAMERAS:
        if topic.startswith(f"/cam_sync/{camera}/"):
            return (camera,)
    if topic.startswith("/eyelink/"):
        return ("eyelink",)
    if topic.startswith("/teensy/"):
        return ("teensy",)
    if topic in ("/markers", "/rigid_bodies"):
        return ("optitrack",)
    if topic.startswith("/left_"):
        return ("robot_1",)
    if topic.startswith("/right_"):
        return ("robot_2",)
    if topic in ("/joint_states", "/dynamic_joint_states"):
        return ("robot_1", "robot_2")
    return ()


@dataclass
class ComponentState:
    received_at: float = 0.0
    total_units: int = 0
    recent: deque[tuple[float, int]] = field(
        default_factory=lambda: deque(maxlen=4096)
    )

    def rate(self, now: float) -> float | None:
        cutoff = now - RATE_WINDOW_SECONDS
        while len(self.recent) > 2 and self.recent[1][0] < cutoff:
            self.recent.popleft()
        if len(self.recent) < 2:
            return None
        first_time, first_total = self.recent[0]
        last_time, last_total = self.recent[-1]
        elapsed = last_time - first_time
        if elapsed <= 0:
            return None
        return (last_total - first_total) / elapsed


@dataclass(frozen=True)
class RowSnapshot:
    streaming: str = "NO DATA"
    stream_detail: str = ""
    ttl_status: str = "WAITING"
    ttl_detail: str = ""
    transport_lost: int = 0
    rate_hz: float | None = None


@dataclass(frozen=True)
class DiskSnapshot:
    path: str = ""
    total: int = 0
    used: int = 0
    free: int = 0
    error: str = ""


@dataclass(frozen=True)
class SessionSnapshot:
    status: str = "WAITING"
    session_id: str = ""
    duration_seconds: float = 0.0
    trial_count: int = 0
    object_misses: int = 0
    average_iti_seconds: float | None = None
    iti_samples: int = 0
    left_lap_seconds: float | None = None
    left_lap_samples: int = 0
    right_lap_seconds: float | None = None
    right_lap_samples: int = 0
    rows: dict[str, RowSnapshot] = field(default_factory=dict)
    disk: DiskSnapshot = field(default_factory=DiskSnapshot)


@dataclass(frozen=True)
class StopControlSnapshot:
    online: bool = False
    pending: bool = False
    message: str = ""


class SessionHealthState:
    """Thread-safe live metrics and completed-session persistence."""

    def __init__(self, persistence_path: Path = PERSIST_PATH) -> None:
        self.lock = threading.RLock()
        self.persistence_path = persistence_path
        self.running = False
        self.session_seen = False
        self.session_id = ""
        self.started_epoch = 0.0
        self.ended_epoch = 0.0
        self.trial_count = 0
        self.object_misses = 0
        self.iti_total = 0.0
        self.iti_samples = 0
        self.lap_total = {"left": 0.0, "right": 0.0}
        self.lap_samples = {"left": 0, "right": 0}
        self.last_depresented: float | None = None
        self.last_depresented_by_arm: dict[str, float | None] = {
            "left": None,
            "right": None,
        }
        self.components: dict[str, dict[str, ComponentState]] = {
            row: {} for row in ROWS
        }
        self.reference_rises: list[float] = []
        self.observed_rises: dict[str, list[float]] = {row: [] for row in ROWS}
        self.ttl_last_state: dict[str, bool | None] = {
            row: None for row in ROWS
        }
        self.transport_totals: dict[str, int] = {}
        self.transport_session_lost: dict[str, int] = {}
        self.disk = DiskSnapshot(path=str(BAG_ROOT))
        self._seen_events: set[tuple] = set()
        self._seen_event_order: deque[tuple] = deque(maxlen=10000)
        self._completed: SessionSnapshot | None = self._load_completed()

    def _reset_for_session(self, session_id: str, stamp: float) -> None:
        self.running = True
        self.session_seen = True
        self.session_id = session_id
        self.started_epoch = stamp
        self.ended_epoch = 0.0
        self.trial_count = 0
        self.object_misses = 0
        self.iti_total = 0.0
        self.iti_samples = 0
        self.lap_total = {"left": 0.0, "right": 0.0}
        self.lap_samples = {"left": 0, "right": 0}
        self.last_depresented = None
        self.last_depresented_by_arm = {"left": None, "right": None}
        self.components = {row: {} for row in ROWS}
        self.reference_rises = []
        self.observed_rises = {row: [] for row in ROWS}
        self.ttl_last_state = {row: None for row in ROWS}
        self.transport_session_lost = {}
        self._completed = None
        try:
            self.persistence_path.unlink(missing_ok=True)
        except OSError:
            pass

    def record_event(
        self, message: TaskEvent, now: float | None = None
    ) -> None:
        now = time.monotonic() if now is None else now
        stamp = _stamp_seconds(message.header.stamp)
        key = (
            message.header.stamp.sec,
            message.header.stamp.nanosec,
            int(message.event_type),
            message.session_id,
            int(message.trial_number),
            message.group_name,
            message.object_id,
        )
        completed: SessionSnapshot | None = None
        with self.lock:
            if key in self._seen_events:
                return
            if len(self._seen_event_order) == self._seen_event_order.maxlen:
                expired = self._seen_event_order.popleft()
                self._seen_events.discard(expired)
            self._seen_event_order.append(key)
            self._seen_events.add(key)

            if message.event_type == TaskEvent.TASK_STARTED:
                self._reset_for_session(message.session_id, stamp)
                return
            if not self.running or message.session_id != self.session_id:
                return
            if message.event_type == TaskEvent.TRIAL_PRESENTED:
                self.trial_count += 1
                if self.last_depresented is not None:
                    delta = stamp - self.last_depresented
                    if delta >= 0:
                        self.iti_total += delta
                        self.iti_samples += 1
                arm = _arm_name(message.group_name)
                if arm is not None:
                    previous = self.last_depresented_by_arm[arm]
                    if previous is not None:
                        delta = stamp - previous
                        if delta >= 0:
                            self.lap_total[arm] += delta
                            self.lap_samples[arm] += 1
            elif message.event_type == TaskEvent.TRIAL_DEPRESENTING:
                self.last_depresented = stamp
                arm = _arm_name(message.group_name)
                if arm is not None:
                    self.last_depresented_by_arm[arm] = stamp
            elif message.event_type == TaskEvent.OBJECT_MISS:
                self.object_misses += 1
            elif message.event_type == TaskEvent.TASK_ENDED:
                self.ended_epoch = max(stamp, self.started_epoch)
                self.running = False
                completed = self._make_snapshot(now, status="COMPLETE")
                self._completed = completed
        if completed is not None:
            self._save_completed(completed)

    def record_component(
        self,
        row: str,
        component: str,
        *,
        now: float | None = None,
        units: int = 1,
    ) -> None:
        now = time.monotonic() if now is None else now
        with self.lock:
            if self.session_seen and not self.running:
                return
            state = self.components[row].setdefault(
                component, ComponentState()
            )
            state.received_at = now
            state.total_units += max(0, units)
            state.recent.append((now, state.total_units))

    def record_ttl_state(self, row: str, asserted: bool, stamp: float) -> None:
        with self.lock:
            previous = self.ttl_last_state[row]
            self.ttl_last_state[row] = asserted
            if self.running and previous is not True and asserted:
                self.observed_rises[row].append(stamp)

    def record_reference_rise(self, stamp: float) -> None:
        if stamp <= 0:
            return
        with self.lock:
            if not self.running:
                return
            if self.reference_rises and stamp == self.reference_rises[-1]:
                return
            if stamp not in self.reference_rises:
                self.reference_rises.append(stamp)

    def record_transport_log(self, text: str) -> None:
        match = TRANSPORT_LOSS_RE.search(text)
        if match is None:
            return
        topic, value = match.groups()
        with self.lock:
            current = int(value)
            previous = self.transport_totals.get(topic)
            self.transport_totals[topic] = current
            if not self.running:
                return
            if previous is None or current < previous:
                increment = current
            else:
                increment = current - previous
            self.transport_session_lost[topic] = (
                self.transport_session_lost.get(topic, 0) + increment
            )

    def update_disk(self, path: Path = BAG_ROOT) -> None:
        try:
            usage = shutil.disk_usage(path)
            snapshot = DiskSnapshot(
                path=str(path),
                total=int(usage.total),
                used=int(usage.used),
                free=int(usage.free),
            )
        except OSError as exc:
            snapshot = DiskSnapshot(path=str(path), error=str(exc))
        with self.lock:
            self.disk = snapshot

    def _transport_lost_for_row(self, row: str) -> int:
        total = 0
        for topic, lost in self.transport_session_lost.items():
            if row in _row_for_topic(topic):
                total += lost
        return total

    def _ttl_for_row(self, row: str, now_epoch: float) -> tuple[str, str]:
        if row in TTL_NA_DETAIL:
            return "N/A", TTL_NA_DETAIL[row]

        mature_before = now_epoch - TTL_SETTLE_SECONDS
        window_start = mature_before - TTL_HEALTH_WINDOW_SECONDS
        window_ready = (
            now_epoch - self.started_epoch >= TTL_HEALTH_WINDOW_SECONDS
        )
        mature_refs = [
            stamp
            for stamp in self.reference_rises
            if window_start <= stamp <= mature_before
        ]
        if row == "teensy":
            if mature_refs:
                return (
                    "INTACT",
                    f"{len(mature_refs)} source pulses in past 60 s",
                )
            if window_ready:
                return "MISS", "No source pulses in past 60 s"
            return "WAITING", "Checking source TTL for 60 s"

        tolerance = TTL_TOLERANCE_SECONDS.get(row)
        if tolerance is None:
            return "N/A", "No TTL input configured"

        observations = [
            stamp
            for stamp in self.observed_rises[row]
            if window_start - tolerance <= stamp <= mature_before + tolerance
        ]
        used: set[int] = set()
        matched = 0
        for reference in mature_refs:
            candidates = [
                (abs(observed - reference), index)
                for index, observed in enumerate(observations)
                if index not in used
            ]
            if not candidates:
                continue
            distance, index = min(candidates)
            if distance <= tolerance:
                matched += 1
                used.add(index)
        if matched:
            return (
                "INTACT",
                f"{matched}/{len(mature_refs)} pulses matched in past 60 s",
            )
        if window_ready:
            return "MISS", "No matched pulses in past 60 s"
        if mature_refs:
            return (
                "WAITING",
                f"Checking TTL: 0/{len(mature_refs)} recent matches",
            )
        return "WAITING", "Checking TTL for 60 s"

    def _row_snapshots(
        self, now: float, now_epoch: float
    ) -> dict[str, RowSnapshot]:
        snapshots: dict[str, RowSnapshot] = {}
        for row in ROWS:
            components = self.components[row]
            required = REQUIRED_COMPONENTS[row]
            fresh = [
                name
                for name in required
                if name in components
                and now - components[name].received_at <= STREAM_STALE_SECONDS
            ]
            if len(fresh) == len(required):
                streaming = "LIVE"
                stream_detail = ", ".join(required)
            elif fresh:
                streaming = "PARTIAL"
                missing = [name for name in required if name not in fresh]
                stream_detail = "missing " + ", ".join(missing)
            else:
                streaming = "NO DATA"
                stream_detail = "waiting for " + ", ".join(required)

            main = components.get(MAIN_COMPONENT[row])
            rate = None if main is None else main.rate(now)
            ttl_status, ttl_detail = self._ttl_for_row(row, now_epoch)
            snapshots[row] = RowSnapshot(
                streaming=streaming,
                stream_detail=stream_detail,
                ttl_status=ttl_status,
                ttl_detail=ttl_detail,
                transport_lost=self._transport_lost_for_row(row),
                rate_hz=rate,
            )
        return snapshots

    def _make_snapshot(
        self,
        now: float,
        *,
        status: str | None = None,
        now_epoch: float | None = None,
    ) -> SessionSnapshot:
        now_epoch = time.time() if now_epoch is None else now_epoch
        if self.running:
            duration = max(0.0, now_epoch - self.started_epoch)
            current_status = "RUNNING"
        elif self.session_seen:
            duration = max(0.0, self.ended_epoch - self.started_epoch)
            current_status = "COMPLETE"
        else:
            duration = 0.0
            current_status = "WAITING"
        if status is not None:
            current_status = status
        return SessionSnapshot(
            status=current_status,
            session_id=self.session_id,
            duration_seconds=duration,
            trial_count=self.trial_count,
            object_misses=self.object_misses,
            average_iti_seconds=(
                self.iti_total / self.iti_samples if self.iti_samples else None
            ),
            iti_samples=self.iti_samples,
            left_lap_seconds=(
                self.lap_total["left"] / self.lap_samples["left"]
                if self.lap_samples["left"]
                else None
            ),
            left_lap_samples=self.lap_samples["left"],
            right_lap_seconds=(
                self.lap_total["right"] / self.lap_samples["right"]
                if self.lap_samples["right"]
                else None
            ),
            right_lap_samples=self.lap_samples["right"],
            rows=self._row_snapshots(now, now_epoch),
            disk=self.disk,
        )

    def snapshot(
        self,
        *,
        now: float | None = None,
        now_epoch: float | None = None,
    ) -> SessionSnapshot:
        now = time.monotonic() if now is None else now
        with self.lock:
            if not self.running and self._completed is not None:
                return replace(self._completed, disk=self.disk)
            return self._make_snapshot(now, now_epoch=now_epoch)

    def _save_completed(self, snapshot: SessionSnapshot) -> None:
        try:
            self.persistence_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.persistence_path.with_suffix(".json.tmp")
            with temporary.open("w", encoding="utf-8") as file:
                json.dump(asdict(snapshot), file, indent=2, sort_keys=True)
                file.write("\n")
            os.replace(temporary, self.persistence_path)
        except OSError:
            pass

    def _load_completed(self) -> SessionSnapshot | None:
        try:
            with self.persistence_path.open(encoding="utf-8") as file:
                data = json.load(file)
            if data.get("status") != "COMPLETE":
                return None
            rows = {}
            row_fields = RowSnapshot.__dataclass_fields__
            for name, row in data.get("rows", {}).items():
                # Older persisted snapshots included the removed
                # ``estimated_lost`` field. Ignore unknown legacy fields.
                current = {
                    key: value
                    for key, value in row.items()
                    if key in row_fields
                }
                rows[name] = RowSnapshot(**current)
            disk = DiskSnapshot(**data.get("disk", {}))
            self.session_seen = True
            self.session_id = str(data.get("session_id", ""))
            return SessionSnapshot(
                status="COMPLETE",
                session_id=self.session_id,
                duration_seconds=float(data.get("duration_seconds", 0.0)),
                trial_count=int(data.get("trial_count", 0)),
                object_misses=int(data.get("object_misses", 0)),
                average_iti_seconds=data.get("average_iti_seconds"),
                iti_samples=int(data.get("iti_samples", 0)),
                left_lap_seconds=data.get("left_lap_seconds"),
                left_lap_samples=int(data.get("left_lap_samples", 0)),
                right_lap_seconds=data.get("right_lap_seconds"),
                right_lap_samples=int(data.get("right_lap_samples", 0)),
                rows=rows,
                disk=disk,
            )
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            return None


class SessionHealthNode(Node):
    """Passive ROS subscriptions behind the workspace-7 dashboard."""

    def __init__(self) -> None:
        super().__init__("session_health_dashboard")
        self.state = SessionHealthState()
        self._eyelink_sample_sub = None
        self._optitrack_detector = OptitrackTTLDetector()
        self._last_teensy_edge = 0.0
        self._stop_lock = threading.Lock()
        self._stop_pending = False
        self._stop_message = ""
        self._stop_message_until = 0.0
        self._stop_client = self.create_client(Trigger, "/tabletop/stop")
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5000,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        rosout_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._dashboard_subscriptions = [
            self.create_subscription(
                TaskEvent,
                "/tabletop/task_event",
                self._task_event_callback,
                event_qos,
            ),
            self.create_subscription(
                Log,
                "/rosout",
                lambda msg: self.state.record_transport_log(msg.msg),
                rosout_qos,
            ),
            self.create_subscription(
                TeensySensor,
                "/teensy/sensor",
                self._teensy_callback,
                sensor_qos,
            ),
            self.create_subscription(
                UInt16,
                "/eyelink/ttl_input",
                self._eyelink_ttl_callback,
                sensor_qos,
            ),
            self.create_subscription(
                IOStates,
                "/left_io_and_status_controller/io_states",
                self._robot_io_callback,
                sensor_qos,
            ),
            self.create_subscription(
                PoseStamped,
                "/left_tcp_pose_broadcaster/pose",
                lambda msg: self._stamped_component("robot_1", "pose", msg),
                sensor_qos,
            ),
            self.create_subscription(
                PoseStamped,
                "/right_tcp_pose_broadcaster/pose",
                lambda msg: self._stamped_component("robot_2", "pose", msg),
                sensor_qos,
            ),
            self.create_subscription(
                WrenchStamped,
                "/left_force_torque_sensor_broadcaster/wrench",
                lambda msg: self._stamped_component("robot_1", "wrench", msg),
                sensor_qos,
            ),
            self.create_subscription(
                WrenchStamped,
                "/right_force_torque_sensor_broadcaster/wrench",
                lambda msg: self._stamped_component("robot_2", "wrench", msg),
                sensor_qos,
            ),
            self.create_subscription(
                Markers,
                "/markers",
                self._markers_callback,
                sensor_qos,
            ),
        ]
        for camera in CAMERAS:
            self._dashboard_subscriptions.append(
                self.create_subscription(
                    ImageMetaData,
                    f"/cam_sync/{camera}/meta",
                    lambda msg, camera=camera: self._camera_callback(
                        camera, msg
                    ),
                    sensor_qos,
                )
            )
        self._disk_timer = self.create_timer(5.0, self._disk_tick)
        self._disk_tick()

    def _now_epoch(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _task_event_callback(self, message: TaskEvent) -> None:
        self.state.record_event(message)
        if message.event_type == TaskEvent.TASK_STARTED:
            self._enable_eyelink_samples()
        elif message.event_type == TaskEvent.TASK_ENDED:
            self._disable_eyelink_samples()

    def _enable_eyelink_samples(self) -> None:
        if self._eyelink_sample_sub is not None:
            return
        self._eyelink_sample_sub = self.create_subscription(
            EyelinkBatch,
            "/eyelink/sample_array",
            self._eyelink_batch_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    def _disable_eyelink_samples(self) -> None:
        if self._eyelink_sample_sub is None:
            return
        self.destroy_subscription(self._eyelink_sample_sub)
        self._eyelink_sample_sub = None

    def _stamped_component(self, row: str, component: str, _message) -> None:
        self.state.record_component(row, component)

    def _teensy_callback(self, message: TeensySensor) -> None:
        header_time = _stamp_seconds(message.header.stamp)
        self.state.record_component("teensy", "sensor")
        edge = _stamp_seconds(message.sync_pulse_last_time_on)
        if edge > 0 and edge != self._last_teensy_edge:
            self._last_teensy_edge = edge
            self.state.record_reference_rise(edge)
        self.state.record_ttl_state(
            "teensy", bool(message.sync_pulse_state), header_time
        )

    def _eyelink_ttl_callback(self, message: UInt16) -> None:
        self.state.record_ttl_state(
            "eyelink",
            not bool(int(message.data) & EYELINK_MASK),
            self._now_epoch(),
        )

    def _eyelink_batch_callback(self, message: EyelinkBatch) -> None:
        if not message.samples:
            return
        self.state.record_component(
            "eyelink", "samples", units=len(message.samples)
        )
        for sample in message.samples:
            self.state.record_ttl_state(
                "eyelink",
                not bool(int(sample.input) & EYELINK_MASK),
                _stamp_seconds(sample.header.stamp),
            )

    def _robot_io_callback(self, message: IOStates) -> None:
        self.state.record_component("robot_1", "io")
        values = {
            int(item.pin): bool(item.state)
            for item in message.digital_in_states
        }
        if 3 in values:
            self.state.record_ttl_state(
                "robot_1", not values[3], self._now_epoch()
            )

    def _camera_callback(self, camera: str, message: ImageMetaData) -> None:
        self.state.record_component(camera, "metadata")
        if camera == FLIR_CAMERA:
            self.state.record_ttl_state(
                camera,
                bool(int(message.line_status) & (1 << FLIR_LINE)),
                _stamp_seconds(message.header.stamp),
            )

    def _markers_callback(self, message: Markers) -> None:
        stamp = _stamp_seconds(message.header.stamp)
        self.state.record_component("optitrack", "markers")
        positions = [
            (
                float(marker.translation.x),
                float(marker.translation.y),
                float(marker.translation.z),
            )
            for marker in message.markers
        ]
        detected = self._optitrack_detector.observe(positions, stamp)
        if detected is not None:
            self.state.record_ttl_state("optitrack", bool(detected[0]), stamp)

    def _disk_tick(self) -> None:
        self.state.update_disk(BAG_ROOT)

    def stop_control_snapshot(self) -> StopControlSnapshot:
        """Return operator-stop availability and request state."""
        with self._stop_lock:
            message = (
                self._stop_message
                if time.monotonic() <= self._stop_message_until
                else ""
            )
            return StopControlSnapshot(
                online=self._stop_client.service_is_ready(),
                pending=self._stop_pending,
                message=message,
            )

    def request_operator_stop(self) -> None:
        """Request task cancellation without commanding any new motion."""
        snapshot = self.state.snapshot()
        with self._stop_lock:
            if self._stop_pending:
                return
            if snapshot.status != "RUNNING":
                self._stop_message = "No active task"
                self._stop_message_until = time.monotonic() + 3.0
                return
            if not self._stop_client.service_is_ready():
                self._stop_message = "Commander stop service is offline"
                self._stop_message_until = time.monotonic() + 5.0
                return
            self._stop_pending = True
            self._stop_message = "Stop request sent"
            self._stop_message_until = time.monotonic() + 5.0

        future = self._stop_client.call_async(Trigger.Request())
        future.add_done_callback(self._operator_stop_finished)

    def _operator_stop_finished(self, future) -> None:
        try:
            response = future.result()
            message = response.message or (
                "Stop accepted" if response.success else "Stop rejected"
            )
        except BaseException as exc:
            message = f"Stop request failed: {exc}"
        with self._stop_lock:
            self._stop_pending = False
            self._stop_message = message
            self._stop_message_until = time.monotonic() + 5.0

    def destroy_node(self):
        self._disable_eyelink_samples()
        return super().destroy_node()


def _rounded_rect(
    canvas: np.ndarray,
    x1: int,
    y1: int,
    x2: int,
    y2: int,
    color: tuple[int, int, int],
    radius: int = 16,
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
    scale: float = 0.52,
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


def _status_color(value: str) -> tuple[int, int, int]:
    if value in ("LIVE", "INTACT", "COMPLETE"):
        return GREEN
    if value in ("PARTIAL", "WAITING", "N/A", "RUNNING"):
        return AMBER if value != "RUNNING" else BLUE
    return RED if value in ("MISS", "NO DATA") else GREY


def _metric_card(
    canvas: np.ndarray,
    x1: int,
    x2: int,
    title: str,
    value: str,
    detail: str,
    color: tuple[int, int, int] = TEXT,
) -> None:
    _rounded_rect(canvas, x1, 116, x2, 250, PANEL)
    _text(canvas, title.upper(), (x1 + 20, 148), 0.43, MUTED, 1)
    _text(canvas, value, (x1 + 20, 198), 0.84, color, 2)
    _text(canvas, detail, (x1 + 20, 228), 0.39, MUTED, 1)


def render_dashboard(
    snapshot: SessionSnapshot,
    stop_control: StopControlSnapshot | None = None,
) -> np.ndarray:
    """Render a deterministic workspace-7 frame for GUI and tests."""
    canvas = np.full((HEIGHT, WIDTH, 3), BG, dtype=np.uint8)
    _text(canvas, "TableTop Session Health", (44, 55), 1.10, TEXT, 2)
    subtitle = (
        snapshot.session_id
        if snapshot.session_id
        else "Waiting for the next task session"
    )
    _text(canvas, subtitle, (44, 88), 0.52, MUTED, 1)

    stop_control = stop_control or StopControlSnapshot()
    x1, y1, x2, y2 = STOP_BUTTON
    if snapshot.status == "RUNNING" and stop_control.pending:
        stop_color = AMBER
        stop_label = "STOPPING..."
    elif snapshot.status == "RUNNING" and stop_control.online:
        stop_color = RED
        stop_label = "STOP ROBOTS"
    elif snapshot.status == "RUNNING":
        stop_color = GREY
        stop_label = "STOP OFFLINE"
    else:
        stop_color = GREY
        stop_label = "NO ACTIVE TASK"
    _rounded_rect(canvas, x1, y1, x2, y2, stop_color, 14)
    _text(canvas, stop_label, (x1 + 29, y1 + 31), 0.52, BG, 2)
    _text(canvas, "cancel both goals", (x1 + 42, y1 + 53), 0.34, BG, 1)
    if stop_control.message:
        _text(canvas, stop_control.message[:42], (972, 88), 0.38, AMBER, 1)

    banner_color = _status_color(snapshot.status)
    _rounded_rect(canvas, 1525, 28, 1856, 92, PANEL)
    _text(
        canvas,
        snapshot.status,
        (1608, 69),
        0.64,
        banner_color,
        2,
    )

    cards = (
        (
            44,
            332,
            "Duration",
            _format_duration(snapshot.duration_seconds),
            "task event clock",
            TEXT,
        ),
        (
            350,
            638,
            "Trials",
            str(snapshot.trial_count),
            "successful presentations",
            GREEN,
        ),
        (
            656,
            944,
            "Object misses",
            str(snapshot.object_misses),
            "failed after retry",
            GREEN if snapshot.object_misses == 0 else RED,
        ),
        (
            962,
            1250,
            "Average ITI",
            "--"
            if snapshot.average_iti_seconds is None
            else f"{snapshot.average_iti_seconds:.2f} s",
            f"{snapshot.iti_samples} intervals",
            TEXT,
        ),
        (
            1268,
            1556,
            "Left full lap",
            "--"
            if snapshot.left_lap_seconds is None
            else f"{snapshot.left_lap_seconds:.2f} s",
            f"{snapshot.left_lap_samples} laps",
            TEXT,
        ),
        (
            1574,
            1856,
            "Right full lap",
            "--"
            if snapshot.right_lap_seconds is None
            else f"{snapshot.right_lap_seconds:.2f} s",
            f"{snapshot.right_lap_samples} laps",
            TEXT,
        ),
    )
    for card in cards:
        _metric_card(canvas, *card)

    disk = snapshot.disk
    _rounded_rect(canvas, 44, 270, 1856, 342, PANEL_ALT, 14)
    if disk.error:
        disk_text = f"HDD  {disk.path}  unavailable: {disk.error}"
        disk_color = RED
    elif disk.total:
        percent = disk.used / disk.total * 100.0
        disk_text = (
            f"HDD  {disk.path}     used {_format_bytes(disk.used)} / "
            f"{_format_bytes(disk.total)} ({percent:.1f}%)     "
            f"free {_format_bytes(disk.free)}"
        )
        disk_color = (
            RED if percent >= 95 else AMBER if percent >= 85 else GREEN
        )
    else:
        disk_text = f"HDD  {disk.path}  waiting for capacity data"
        disk_color = GREY
    _text(canvas, disk_text, (67, 316), 0.55, disk_color, 2)

    headers = (
        ("DATASTREAM", 58),
        ("STREAM", 350),
        ("TTL", 600),
        ("LOST AFTER START", 900),
        ("RATE", 1245),
        ("DETAIL", 1410),
    )
    for label, x in headers:
        _text(canvas, label, (x, 382), 0.43, MUTED, 2)

    row_top = 398
    row_height = 50
    for index, row in enumerate(ROWS):
        y1 = row_top + index * row_height
        y2 = y1 + row_height - 4
        cv2.rectangle(
            canvas,
            (44, y1),
            (1856, y2),
            PANEL if index % 2 == 0 else PANEL_ALT,
            -1,
        )
        state = snapshot.rows.get(row, RowSnapshot())
        stream_color = _status_color(state.streaming)
        ttl_color = _status_color(state.ttl_status)
        loss_color = GREEN if state.transport_lost == 0 else RED
        _text(canvas, LABELS[row], (58, y1 + 31), 0.50, TEXT, 1)
        cv2.circle(canvas, (350, y1 + 24), 7, stream_color, -1, cv2.LINE_AA)
        _text(canvas, state.streaming, (368, y1 + 30), 0.46, stream_color, 1)
        _text(canvas, state.ttl_status, (600, y1 + 30), 0.46, ttl_color, 1)
        _text(
            canvas,
            f"exact {state.transport_lost}",
            (900, y1 + 30),
            0.44,
            loss_color,
            1,
        )
        rate = "--" if state.rate_hz is None else f"{state.rate_hz:.1f} Hz"
        _text(canvas, rate, (1245, y1 + 30), 0.44, TEXT, 1)
        detail = (
            state.ttl_detail
            if state.ttl_status != "N/A"
            else state.stream_detail
        )
        _text(canvas, detail[:53], (1410, y1 + 30), 0.36, MUTED, 1)

    _text(
        canvas,
        "Loss = exact rosbag DDS transport reports after task startup.",
        (44, 982),
        0.43,
        MUTED,
        1,
    )
    _text(
        canvas, "Esc or Q closes this dashboard", (1605, 982), 0.42, MUTED, 1
    )
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
    node = SessionHealthNode()
    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        name="session-health-dashboard-ros",
        daemon=True,
    )
    spin_thread.start()

    _switch_dwm_workspace(7)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, WIDTH, HEIGHT)

    def mouse_callback(event, x, y, _flags, _parameter) -> None:
        x1, y1, x2, y2 = STOP_BUTTON
        if event == cv2.EVENT_LBUTTONUP and x1 <= x <= x2 and y1 <= y <= y2:
            node.request_operator_stop()

    cv2.setMouseCallback(WINDOW_NAME, mouse_callback)
    return_to_rig_at = time.monotonic() + 0.75
    returned_to_rig = False
    try:
        while not stopped.is_set() and rclpy.ok():
            cv2.imshow(
                WINDOW_NAME,
                render_dashboard(
                    node.state.snapshot(), node.stop_control_snapshot()
                ),
            )
            key = cv2.waitKey(100) & 0xFF
            if key in (ord("q"), 27):
                break
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
            if not returned_to_rig and time.monotonic() >= return_to_rig_at:
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
