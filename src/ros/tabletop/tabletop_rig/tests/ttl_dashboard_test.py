"""Regression tests for the passive workspace-6 TTL dashboard."""

from unittest.mock import MagicMock, patch

import numpy as np
import pytest
from tabletop_interfaces.msg import Eyelink
from tabletop_rig.nodes.eyelink import Eyelink as EyelinkNode
from tabletop_rig.nodes.ttl_dashboard import (
    HEIGHT,
    WIDTH,
    TTLMonitorState,
    render_dashboard,
)


def _pulse_reference(state: TTLMonitorState, when: float) -> None:
    state.record_teensy(False, when - 0.2)
    state.record_teensy(True, when)
    state.record_teensy(False, when + 0.1)


def test_flir_line3_and_eyelink_bit3_are_decoded_independently() -> None:
    state = TTLMonitorState()
    state.record_flir(4, 0.9)
    state.record_eyelink(255, 0.9)
    _pulse_reference(state, 1.0)
    state.record_flir(12, 1.008)
    state.record_flir(4, 1.108)
    state.record_eyelink(247, 1.003)
    state.record_eyelink(255, 1.103)

    snapshot = state.snapshot()
    assert snapshot.channels["flir"].pulse_count == 1
    assert snapshot.channels["flir"].last_lag_ms == pytest.approx(8.0)
    assert snapshot.channels["eyelink"].pulse_count == 1
    assert snapshot.channels["eyelink"].last_lag_ms == pytest.approx(3.0)


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
        state.record_robot("robot_2", values, when + 0.01)
    snapshot = state.snapshot()
    assert snapshot.robot_inputs["robot_2"] is None
    assert snapshot.channels["robot_2"].received_at > 0
    assert "no correlated input" in snapshot.channels["robot_2"].detail


def test_render_has_full_size_for_live_and_missing_channels() -> None:
    state = TTLMonitorState({"robot_1": (0, True), "robot_2": (1, False)})
    state.record_eyelink(255, 9.9)
    state.record_flir(4, 9.9)
    state.record_robot("robot_1", {0: False}, 9.9)
    state.record_robot("robot_2", {1: True}, 9.9)
    _pulse_reference(state, 10.0)
    state.record_eyelink(247, 10.005)
    state.record_flir(12, 10.008)
    state.record_robot("robot_1", {0: True}, 10.004)
    state.record_robot("robot_2", {1: False}, 10.006)
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
    message = Eyelink(input=255)

    with patch(
        "tabletop_rig.nodes.eyelink.time.monotonic",
        side_effect=(1.0, 1.01, 1.2, 1.21),
    ):
        node._publish_ttl_input(message)
        node._publish_ttl_input(message)
        node._publish_ttl_input(message)
        message.input = 247
        node._publish_ttl_input(message)

    assert node.ttl_input_publisher.publish.call_count == 3
    assert [
        call.args[0].data
        for call in node.ttl_input_publisher.publish.call_args_list
    ] == [255, 255, 247]
