import json
from unittest.mock import Mock

import pytest
from tabletop_rig.utils.novnc_visibility import (
    VisibilityPublisher,
    WindowVisibility,
    should_render,
)


@pytest.mark.parametrize(
    "document,now,expected",
    [
        (
            {"version": 1, "updated_epoch": 100, "windows": {"View": False}},
            101,
            False,
        ),
        (
            {"version": 1, "updated_epoch": 100, "windows": {"View": True}},
            101,
            True,
        ),
        (
            {"version": 1, "updated_epoch": 100, "windows": {"Other": False}},
            101,
            True,
        ),
        (
            {"version": 1, "updated_epoch": 100, "windows": {"View": False}},
            104,
            True,
        ),
        (
            {"version": 1, "updated_epoch": 100, "windows": {"View": False}},
            99,
            True,
        ),
        (
            {"version": 1, "updated_epoch": True, "windows": {"View": False}},
            1,
            True,
        ),
        (
            {"version": 2, "updated_epoch": 100, "windows": {"View": False}},
            101,
            True,
        ),
        (
            {"version": 1, "updated_epoch": 100, "windows": {"View": 0}},
            101,
            True,
        ),
        ({"version": 1, "updated_epoch": 100, "windows": []}, 101, True),
        (None, 101, True),
        ([], 101, True),
    ],
)
def test_fail_open_policy(document, now, expected):
    assert should_render(document, "View", now) is expected


def test_publisher_transitions_heartbeat_and_stale_reader(
    tmp_path, monkeypatch
):
    import tabletop_rig.utils.novnc_visibility as module

    clock = [100.0]
    monkeypatch.setattr(module.time, "time", lambda: clock[0])
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    path = tmp_path / "visibility.json"
    writer = VisibilityPublisher(path)
    reader = WindowVisibility("View", path)
    assert reader()  # Missing status.
    writer.publish({"View": False})
    assert not reader()
    mtime = path.stat().st_mtime_ns
    writer.publish({"View": False})
    assert path.stat().st_mtime_ns == mtime
    clock[0] += 1.1
    writer.publish({"View": False})
    assert json.loads(path.read_text())["updated_epoch"] == 101.1
    assert not reader()
    writer.publish({"View": True})
    assert reader()
    writer.publish({"View": False})
    assert not reader()
    clock[0] += 3.1  # A dead supervisor must not leave a frozen GUI.
    assert reader()
    path.write_text("bad json")
    assert reader()
    assert not list(tmp_path.glob("*.tmp"))


def test_unwritable_status_does_not_stop_supervisor(tmp_path):
    parent = tmp_path / "not-a-directory"
    parent.write_text("x")
    VisibilityPublisher(parent / "status.json").publish({"View": False})


def test_hidden_session_gui_keeps_event_pump_stop_and_ros_thread(monkeypatch):
    """A mocked Stop click is handled even while every render is suppressed.

    No real ROS node, service call, GUI click or robot command is made.
    """
    from tabletop_rig.nodes import session_dashboard as gui
    from tabletop_rig.utils import novnc_visibility

    node = Mock()
    thread = Mock()
    monkeypatch.setattr(gui, "SessionHealthNode", lambda: node)
    monkeypatch.setattr(gui.threading, "Thread", lambda **_: thread)
    monkeypatch.setattr(gui.signal, "signal", Mock())
    monkeypatch.setattr(gui.rclpy, "init", Mock())
    monkeypatch.setattr(gui.rclpy, "ok", lambda: True)
    monkeypatch.setattr(gui.rclpy, "try_shutdown", Mock())
    monkeypatch.setattr(gui, "_switch_dwm_workspace", Mock())
    monkeypatch.setattr(
        novnc_visibility, "WindowVisibility", lambda _: lambda: False
    )
    callbacks = {}
    monkeypatch.setattr(gui.cv2, "namedWindow", Mock())
    monkeypatch.setattr(gui.cv2, "resizeWindow", Mock())
    monkeypatch.setattr(gui.cv2, "destroyAllWindows", Mock())
    monkeypatch.setattr(gui.cv2, "getWindowProperty", lambda *_: 1)
    monkeypatch.setattr(
        gui.cv2,
        "setMouseCallback",
        lambda _, callback: callbacks.update(mouse=callback),
    )
    render = Mock(side_effect=AssertionError("Hidden GUI must not render"))
    monkeypatch.setattr(gui, "render_dashboard", render)
    calls = []

    def events(_delay):
        calls.append(1)
        if len(calls) == 2:
            x, y, _, _ = gui.STOP_BUTTON
            callbacks["mouse"](gui.cv2.EVENT_LBUTTONUP, x + 1, y + 1, 0, None)
        return ord("q") if len(calls) == 3 else -1

    monkeypatch.setattr(gui.cv2, "waitKey", events)
    gui.main()
    assert len(calls) == 3
    thread.start.assert_called_once()
    node.request_operator_stop.assert_called_once()
    render.assert_not_called()
    node.state.snapshot.assert_not_called()
    node.destroy_node.assert_called_once()
