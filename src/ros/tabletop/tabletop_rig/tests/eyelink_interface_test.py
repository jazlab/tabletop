"""Unit tests for EyeLink rosbag endpoint identification."""

from types import SimpleNamespace

from tabletop_rig.interfaces.eyelink import _is_rosbag_recorder_endpoint


def test_recognizes_actual_rosbag_recorder_endpoint() -> None:
    assert _is_rosbag_recorder_endpoint(
        SimpleNamespace(node_name="rosbag2_recorder")
    )
    assert _is_rosbag_recorder_endpoint(
        SimpleNamespace(node_name="rosbag2_recorder_2")
    )


def test_rejects_dashboard_and_anonymous_subscribers() -> None:
    assert not _is_rosbag_recorder_endpoint(
        SimpleNamespace(node_name="session_health_dashboard")
    )
    assert not _is_rosbag_recorder_endpoint(SimpleNamespace())
