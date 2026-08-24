"""Regression tests for the noVNC I/O dashboard and bounded outputs."""

import threading
from unittest.mock import MagicMock

import numpy as np
from rclpy.duration import Duration
from std_msgs.msg import Header
from tabletop_interfaces.msg import TeensySensor
from tabletop_interfaces.srv import SetBuzzer, SetSmartglass
from tabletop_rig.nodes import mock_teensy as mock_module
from tabletop_rig.nodes.io_dashboard import (
    HEIGHT,
    WIDTH,
    DashboardSnapshot,
    IODashboardNode,
    render_dashboard,
)
from tabletop_rig.nodes.mock_teensy import MockTeensy


def _bare_dashboard() -> IODashboardNode:
    node = object.__new__(IODashboardNode)
    node._lock = threading.RLock()
    node._state = DashboardSnapshot()
    node._previous_left = None
    node._previous_right = None
    node._previous_laser = None
    return node


def test_input_callbacks_count_only_rising_events() -> None:
    node = _bare_dashboard()
    released = TeensySensor()
    node._sensor_callback(released)
    assert node.snapshot().left_count == 0

    pressed = TeensySensor()
    pressed.is_left_arm_locked = True
    pressed.is_right_arm_locked = True
    pressed.is_safety_laser_broken = True
    node._sensor_callback(pressed)
    state = node.snapshot()
    assert (state.left_count, state.right_count, state.laser_count) == (1, 1, 1)

    node._sensor_callback(pressed)
    state = node.snapshot()
    assert (state.left_count, state.right_count, state.laser_count) == (1, 1, 1)


def test_flic_callback_records_any_button_address() -> None:
    node = _bare_dashboard()
    node._flic_callback(Header(frame_id="80:e4:da:71:12:34"))
    state = node.snapshot()
    assert state.flic_count == 1
    assert state.flic_address == "80:e4:da:71:12:34"
    assert "Flic registered" in state.toast


def test_render_is_full_size_for_healthy_stale_and_task_locked_states() -> None:
    healthy = DashboardSnapshot(
        sensor_received_at=10.0,
        flic_online=True,
        services={"smartglass": True, "reward": True, "buzzer": True},
        sound_available=True,
    )
    stale = DashboardSnapshot(sensor_received_at=10.0)
    locked = DashboardSnapshot(sensor_received_at=10.0, task_running=True)

    for state, now in ((healthy, 10.1), (stale, 11.0), (locked, 10.1)):
        image = render_dashboard(state, now=now)
        assert image.shape == (HEIGHT, WIDTH, 3)
        assert image.dtype == np.uint8
        assert np.any(image != image[0, 0])


def test_smartglass_command_is_one_second_and_inverts_current_state() -> None:
    node = _bare_dashboard()
    node._state.sensor_received_at = 1e12  # fresh relative to monotonic()
    node._state.smartglass_revealed = True
    node._output_allowed = MagicMock(return_value=True)
    node._smartglass_client = object()
    node._call = MagicMock()

    node._run_command("smartglass")

    request = node._call.call_args.args[2]
    assert request.reveal is False
    assert Duration.from_msg(request.duration).nanoseconds == 1_000_000_000
    assert node._call.call_args.args[3] == 1.0


def test_buzzer_commands_never_use_arm_lock_service() -> None:
    node = _bare_dashboard()
    node._output_allowed = MagicMock(return_value=True)
    node._buzzer_client = object()
    node._call = MagicMock()

    node._run_command("left_buzzer")
    request = node._call.call_args.args[2]
    assert isinstance(request, SetBuzzer.Request)
    assert request.left_arm is True
    assert request.right_arm is False
    assert node._call.call_args.args[3] == 1.0


def _bare_mock_teensy() -> MockTeensy:
    node = object.__new__(MockTeensy)
    node.log = MagicMock()
    node.smartglass_revealed = True
    node.smartglass_restore_state = True
    node.smartglass_timer = MagicMock()
    node.buzzer_timer = MagicMock()
    return node


def test_mock_buzzer_pulse_does_not_change_arm_lock_outputs() -> None:
    node = _bare_mock_teensy()
    mock_module.digital_output_pin_states[
        mock_module.LEFT_ARM_LOCK_CONTROL_PIN
    ] = True
    mock_module.digital_output_pin_states[
        mock_module.RIGHT_ARM_LOCK_CONTROL_PIN
    ] = False

    response = node.set_buzzer_callback(
        SetBuzzer.Request(left_arm=True, right_arm=False),
        SetBuzzer.Response(),
    )
    assert response.success
    assert mock_module.digital_output_pin_states[
        mock_module.LEFT_ARM_LOCK_CONTROL_PIN
    ] is True
    assert mock_module.digital_output_pin_states[
        mock_module.RIGHT_ARM_LOCK_CONTROL_PIN
    ] is False
    assert mock_module.digital_output_pin_states[
        mock_module.LEFT_ARM_BUZZER_CONTROL_PIN
    ] is True
    node.buzzer_timer.reset.assert_called_once_with()

    node.buzzer_timer_callback()
    assert mock_module.digital_output_pin_states[
        mock_module.LEFT_ARM_BUZZER_CONTROL_PIN
    ] is False
    assert mock_module.digital_output_pin_states[
        mock_module.RIGHT_ARM_BUZZER_CONTROL_PIN
    ] is False


def test_mock_temporary_smartglass_restores_prior_state() -> None:
    node = _bare_mock_teensy()
    request = SetSmartglass.Request()
    request.reveal = False
    request.duration = Duration(seconds=1.0).to_msg()

    response = node.set_smartglass_callback(request, SetSmartglass.Response())
    assert response.success
    assert node.smartglass_revealed is False
    assert node.smartglass_restore_state is True
    assert node.smartglass_timer.timer_period_ns == 1_000_000_000
    node.smartglass_timer.reset.assert_called_once_with()

    node.smartglass_timer_callback()
    assert node.smartglass_revealed is True
    assert mock_module.digital_output_pin_states[
        mock_module.SMARTGLASS_CONTROL_PIN
    ] is True
    node.smartglass_timer.cancel.assert_called()
