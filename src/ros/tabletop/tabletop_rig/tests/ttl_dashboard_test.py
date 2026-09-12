"""Regression tests for the passive workspace-6 TTL dashboard."""

import threading
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
from tabletop_interfaces.msg import Eyelink
from tabletop_rig.nodes.eyelink import Eyelink as EyelinkNode
from tabletop_rig.nodes.ttl_dashboard import (
    HEIGHT,
    WIDTH,
    OptitrackTTLDetector,
    TTLMonitorState,
    render_dashboard,
)


def _pulse_reference(state: TTLMonitorState, when: float) -> None:
    state.record_teensy(False, when - 0.2)
    state.record_teensy(True, when)
    state.record_teensy(False, when + 0.1)


def test_flir_line3_and_eyelink_status_pin12_are_decoded_independently() -> (
    None
):
    state = TTLMonitorState()
    state.record_flir(4, 0.9)
    state.record_eyelink(127, 0.9)
    _pulse_reference(state, 1.0)
    state.record_flir(12, 1.008)
    state.record_flir(4, 1.108)
    state.record_eyelink(95, 1.003)
    state.record_eyelink(127, 1.103)

    snapshot = state.snapshot()
    assert snapshot.channels["flir"].pulse_count == 1
    assert snapshot.channels["flir"].last_lag_ms == pytest.approx(8.0)
    assert snapshot.channels["eyelink"].pulse_count == 1
    assert snapshot.channels["eyelink"].last_lag_ms == pytest.approx(3.0)
    assert "Status pin 12 / bit 5" in snapshot.channels["eyelink"].detail


def test_robot_input_is_selected_only_after_three_correlated_edges() -> None:
    state = TTLMonitorState()
    state.record_robot("robot_1", {pin: False for pin in range(18)}, 0.5)

    for index, when in enumerate((1.0, 2.0, 3.0), start=1):
        _pulse_reference(state, when)
        values = {pin: False for pin in range(18)}
        values[7] = True
        state.record_robot("robot_1", values, when + 0.01)
        values[7] = False
        state.record_robot("robot_1", values, when + 0.11)
        if index < 3:
            assert state.snapshot().robot_inputs["robot_1"] is None

    assert state.snapshot().robot_inputs["robot_1"] == (7, True)

    _pulse_reference(state, 4.0)
    values = {pin: False for pin in range(18)}
    values[7] = True
    state.record_robot("robot_1", values, 4.01)
    values[7] = False
    state.record_robot("robot_1", values, 4.11)
    snapshot = state.snapshot()
    assert snapshot.channels["robot_1"].pulse_count == 1
    assert "DI 7" in snapshot.channels["robot_1"].detail


def test_constant_robot_inputs_are_online_but_not_misidentified() -> None:
    state = TTLMonitorState()
    values = {pin: pin == 2 for pin in range(18)}
    for when in (1.0, 2.0, 3.0, 4.0):
        _pulse_reference(state, when)
        state.record_robot("robot_1", values, when + 0.01)
    snapshot = state.snapshot()
    assert snapshot.robot_inputs["robot_1"] is None
    assert snapshot.channels["robot_1"].received_at > 0
    assert "no correlated input" in snapshot.channels["robot_1"].detail


def test_optitrack_detector_rejects_jitter_and_an_outside_marker() -> None:
    detector = OptitrackTTLDetector(
        center=np.zeros(3),
        axis=np.asarray((1.0, 0.0, 0.0)),
        roi_radius_m=0.02,
        on_threshold_m=0.002,
        off_threshold_m=0.001,
        debounce_seconds=0.015,
        calibration_samples=4,
        filter_samples=1,
    )

    for index, displacement in enumerate((0.0002, -0.0001, 0.0001, 0.0)):
        detector.observe([(displacement, 0.0, 0.0)], index * 0.01)

    assert detector.calibrated
    assert detector.observe([(0.05, 0.0, 0.0)], 0.05) is None
    assert not detector.marker_seen
    assert detector.observe([(0.0004, 0.0, 0.0)], 0.06)[0] is False

    assert detector.observe([(0.0034, 0.0, 0.0)], 0.10)[0] is False
    asserted = detector.observe([(0.0035, 0.0, 0.0)], 0.12)
    assert asserted is not None and asserted[0] is True

    assert detector.observe([(0.0003, 0.0, 0.0)], 0.20)[0] is True
    cleared = detector.observe([(0.0002, 0.0, 0.0)], 0.22)
    assert cleared is not None and cleared[0] is False


def test_optitrack_pulse_is_counted_and_aligned_to_teensy() -> None:
    state = TTLMonitorState()
    state._optitrack_detector = OptitrackTTLDetector(
        center=np.zeros(3),
        axis=np.asarray((1.0, 0.0, 0.0)),
        roi_radius_m=0.02,
        on_threshold_m=0.002,
        off_threshold_m=0.001,
        debounce_seconds=0.015,
        calibration_samples=2,
        filter_samples=1,
    )
    state.record_optitrack([(0.0, 0.0, 0.0)], 0.80)
    state.record_optitrack([(0.0, 0.0, 0.0)], 0.82)
    _pulse_reference(state, 1.0)
    state.record_optitrack([(0.0034, 0.0, 0.0)], 1.005)
    state.record_optitrack([(0.0034, 0.0, 0.0)], 1.025)
    state.record_optitrack([(0.0, 0.0, 0.0)], 1.105)
    state.record_optitrack([(0.0, 0.0, 0.0)], 1.125)

    channel = state.snapshot().channels["optitrack"]
    assert channel.pulse_count == 1
    assert channel.last_lag_ms == pytest.approx(25.0)
    assert channel.state is False
    assert "travel" in channel.detail


def test_render_has_full_size_for_live_and_missing_channels() -> None:
    state = TTLMonitorState({"robot_1": (0, True)})
    state.record_eyelink(127, 9.9)
    state.record_flir(4, 9.9)
    state.record_robot("robot_1", {0: False}, 9.9)
    _pulse_reference(state, 10.0)
    state.record_eyelink(95, 10.005)
    state.record_flir(12, 10.008)
    state.record_robot("robot_1", {0: True}, 10.004)
    image = render_dashboard(state.snapshot(), now=10.02)
    assert image.shape == (HEIGHT, WIDTH, 3)
    assert image.dtype == np.uint8
    assert np.any(image != image[0, 0])


def test_eyelink_ttl_status_is_change_driven_with_bounded_heartbeat() -> None:
    node = object.__new__(EyelinkNode)
    node.ttl_input_publisher = MagicMock()
    node._last_ttl_input = None
    node._last_ttl_publish_at = 0.0
    node.param = MagicMock(return_value=10.0)
    message = Eyelink(input=127)

    with patch(
        "tabletop_rig.nodes.eyelink.time.monotonic",
        side_effect=(1.0, 1.01, 1.2, 1.21),
    ):
        node._publish_ttl_input(message)
        node._publish_ttl_input(message)
        node._publish_ttl_input(message)
        message.input = 95
        node._publish_ttl_input(message)

    assert node.ttl_input_publisher.publish.call_count == 3
    assert [
        call.args[0].data
        for call in node.ttl_input_publisher.publish.call_args_list
    ] == [127, 127, 95]


def test_eyelink_initializes_button_state_without_event_queue() -> None:
    node = object.__new__(EyelinkNode)
    node.param = MagicMock(
        side_effect={
            "simulate": False,
            "tracker_address": "192.0.2.1",
        }.__getitem__
    )
    node.log = MagicMock()
    node._close_data_file = MagicMock()
    tracker = MagicMock()

    with (
        patch("tabletop_rig.nodes.eyelink.PYLINK_AVAILABLE", True),
        patch(
            "tabletop_rig.nodes.eyelink.EyeLinkTracker",
            return_value=tracker,
        ),
    ):
        node.init_tracker()

    tracker.flushKeybuttons.assert_called_once_with(0)


def test_eyelink_idle_poll_maps_button_one_without_starting_recording() -> (
    None
):
    node = object.__new__(EyelinkNode)
    node.simulate = False
    node._recording_active = threading.Event()
    node._retrieval_lock = threading.Lock()
    node._retrieving = False
    node._last_ttl_idle_poll_error_at = 0.0
    node._ttl_idle_poll_started = True
    node.tracker = MagicMock()
    node.tracker.getButtonStates.side_effect = (0, 1)
    params = {
        "ttl_idle_button": 1,
        "ttl_idle_value": 127,
        "ttl_asserted_value": 95,
    }
    node.param = MagicMock(side_effect=params.__getitem__)
    node._publish_ttl_value = MagicMock()

    node._poll_idle_ttl_input()
    node._poll_idle_ttl_input()

    assert [
        call.args[0] for call in node._publish_ttl_value.call_args_list
    ] == [
        127,
        95,
    ]
    assert node.tracker.getButtonStates.call_count == 2


def test_eyelink_idle_poll_is_disabled_during_recording() -> None:
    node = object.__new__(EyelinkNode)
    node.simulate = False
    node._recording_active = threading.Event()
    node._recording_active.set()
    node._retrieval_lock = threading.Lock()
    node._retrieving = True
    node.tracker = MagicMock()
    node._publish_ttl_value = MagicMock()

    node._poll_idle_ttl_input()

    node.tracker.getButtonStates.assert_not_called()
    node._publish_ttl_value.assert_not_called()
