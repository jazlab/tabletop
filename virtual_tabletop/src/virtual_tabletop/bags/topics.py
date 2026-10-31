"""Typed extractors for the rig topics the virtual tabletop replays.

Message definitions (for reference; decoding uses the schemas embedded in
the bag):

- ``/tabletop/task_event``: ``tabletop_interfaces/msg/TaskEvent``
- ``/teensy/sensor``: ``tabletop_interfaces/msg/TeensySensor``
- ``/flic/button_pressed_time``: ``std_msgs/msg/Header`` (stamp = press time)
"""

from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path

import numpy as np

from virtual_tabletop.bags.reader import iter_messages, stamp_to_sec

TASK_EVENT_TOPIC = "/tabletop/task_event"
TEENSY_TOPIC = "/teensy/sensor"
FLIC_TOPIC = "/flic/button_pressed_time"


class TaskEventType(IntEnum):
    """Constants of ``tabletop_interfaces/msg/TaskEvent``."""

    TASK_STARTED = 1
    TASK_ENDED = 2
    TRIAL_PRESENTED = 3
    TRIAL_DEPRESENTING = 4
    OBJECT_MISS = 5
    STIMULUS_REVEALED = 6
    MOTION_STARTED = 7
    GO_CUE = 8


@dataclass(frozen=True)
class TaskEvent:
    """One ``TaskEvent`` message.

    Attributes:
        t: Header stamp (seconds since epoch).
        type: Event type.
        session_id: Task session id.
        trial_number: Trial number (-1 for task-level events).
        group_name: Trial group (e.g. which arm), possibly empty.
        object_id: Trial object, possibly empty.
    """

    t: float
    type: TaskEventType | int
    session_id: str
    trial_number: int
    group_name: str
    object_id: str


@dataclass(frozen=True)
class TeensyStates:
    """``TeensySensor`` boolean channels as arrays (one row per message).

    Attributes:
        t: Header stamps (seconds since epoch), shape ``(N,)``.
        smartglass_revealed: Glass transparent.
        reward_active: Reward solenoid open.
        left_arm_locked: Left hand on its sensor.
        right_arm_locked: Right hand on its sensor.
        safety_laser_broken: Safety laser beam interrupted.
    """

    t: np.ndarray
    smartglass_revealed: np.ndarray
    reward_active: np.ndarray
    left_arm_locked: np.ndarray
    right_arm_locked: np.ndarray
    safety_laser_broken: np.ndarray


@dataclass(frozen=True)
class ButtonPress:
    """One Flic button press.

    Attributes:
        t: Press time (seconds since epoch).
        button: Button identifier from the header ``frame_id``.
    """

    t: float
    button: str


def _event_type(value: int) -> TaskEventType | int:
    try:
        return TaskEventType(value)
    except ValueError:
        return value


def read_task_events(path: str | Path) -> list[TaskEvent]:
    """Return the task lifecycle events in time order."""
    events = [
        TaskEvent(
            t=stamp_to_sec(msg.header.stamp),
            type=_event_type(msg.event_type),
            session_id=msg.session_id,
            trial_number=msg.trial_number,
            group_name=msg.group_name,
            object_id=msg.object_id,
        )
        for _, _, msg in iter_messages(path, [TASK_EVENT_TOPIC])
    ]
    return sorted(events, key=lambda e: e.t)


def read_teensy(path: str | Path) -> TeensyStates:
    """Return the Teensy sensor channels the replay shows."""
    fields = {
        "smartglass_revealed": "is_smartglass_revealed",
        "reward_active": "is_reward_active",
        "left_arm_locked": "is_left_arm_locked",
        "right_arm_locked": "is_right_arm_locked",
        "safety_laser_broken": "is_safety_laser_broken",
    }
    t, rows = [], []
    for _, _, msg in iter_messages(path, [TEENSY_TOPIC]):
        t.append(stamp_to_sec(msg.header.stamp))
        rows.append([bool(getattr(msg, f)) for f in fields.values()])
    order = np.argsort(t)
    values = np.asarray(rows, dtype=bool).reshape(-1, len(fields))[order]
    return TeensyStates(
        np.asarray(t)[order],
        **{name: values[:, i] for i, name in enumerate(fields)},
    )


def read_button_presses(path: str | Path) -> list[ButtonPress]:
    """Return Flic button presses in time order."""
    presses = [
        ButtonPress(stamp_to_sec(msg.stamp), msg.frame_id)
        for _, _, msg in iter_messages(path, [FLIC_TOPIC])
    ]
    return sorted(presses, key=lambda p: p.t)
