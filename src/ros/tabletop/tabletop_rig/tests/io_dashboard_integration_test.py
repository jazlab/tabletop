"""Isolated ROS graph integration test for the TableTop I/O dashboard."""

import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Header
from tabletop_rig.nodes import mock_teensy as mock_module
from tabletop_rig.nodes.io_dashboard import IODashboardNode
from tabletop_rig.nodes.mock_teensy import MockTeensy


def _wait_for(condition, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(0.02)
    raise AssertionError("condition did not become true before timeout")


def test_dashboard_topics_and_bounded_outputs_end_to_end() -> None:
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    mock = MockTeensy()
    dashboard = IODashboardNode(enable_audio=False)
    probe = Node("io_dashboard_integration_probe")
    flic_pub = probe.create_publisher(Header, "/flic/button_pressed_time", 10)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (mock, dashboard, probe):
        executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        _wait_for(
            lambda: all(
                dashboard.snapshot().services.get(name, False)
                for name in ("smartglass", "reward", "buzzer")
            )
        )
        _wait_for(lambda: dashboard.snapshot().sensor_received_at > 0.0)

        flic_pub.publish(Header(frame_id="80:e4:da:71:12:34"))
        _wait_for(lambda: dashboard.snapshot().flic_count == 1)

        for pin in (
            mock_module.LEFT_ARM_LOCKED_STATE_PIN,
            mock_module.RIGHT_ARM_LOCKED_STATE_PIN,
            mock_module.SAFETY_LASER_BROKEN_STATE_PIN,
        ):
            mock_module._change_input_pin_state(pin, False)
        _wait_for(
            lambda: not dashboard.snapshot().left_pressed
            and not dashboard.snapshot().right_pressed
            and not dashboard.snapshot().laser_broken
        )
        for pin in (
            mock_module.LEFT_ARM_LOCKED_STATE_PIN,
            mock_module.RIGHT_ARM_LOCKED_STATE_PIN,
            mock_module.SAFETY_LASER_BROKEN_STATE_PIN,
        ):
            mock_module._change_input_pin_state(pin, True)
        _wait_for(
            lambda: dashboard.snapshot().left_count >= 1
            and dashboard.snapshot().right_count >= 1
            and dashboard.snapshot().laser_count >= 1
        )

        initial_smartglass = mock.smartglass_revealed
        dashboard.enqueue("smartglass")
        _wait_for(lambda: mock.smartglass_revealed is not initial_smartglass)
        _wait_for(
            lambda: mock.smartglass_revealed is initial_smartglass,
            timeout=1.5,
        )

        dashboard.enqueue("reward")
        _wait_for(lambda: mock.reward_active)
        _wait_for(lambda: not mock.reward_active, timeout=0.6)

        left_lock_before = mock_module.digital_output_pin_states[
            mock_module.LEFT_ARM_LOCK_CONTROL_PIN
        ]
        right_lock_before = mock_module.digital_output_pin_states[
            mock_module.RIGHT_ARM_LOCK_CONTROL_PIN
        ]
        dashboard.enqueue("left_buzzer")
        _wait_for(
            lambda: mock_module.digital_output_pin_states[
                mock_module.LEFT_ARM_BUZZER_CONTROL_PIN
            ]
        )
        assert (
            mock_module.digital_output_pin_states[
                mock_module.LEFT_ARM_LOCK_CONTROL_PIN
            ]
            is left_lock_before
        )
        assert (
            mock_module.digital_output_pin_states[
                mock_module.RIGHT_ARM_LOCK_CONTROL_PIN
            ]
            is right_lock_before
        )
        _wait_for(
            lambda: not mock_module.digital_output_pin_states[
                mock_module.LEFT_ARM_BUZZER_CONTROL_PIN
            ],
            timeout=1.5,
        )

        dashboard.enqueue("right_buzzer")
        _wait_for(
            lambda: mock_module.digital_output_pin_states[
                mock_module.RIGHT_ARM_BUZZER_CONTROL_PIN
            ]
        )
        _wait_for(
            lambda: not mock_module.digital_output_pin_states[
                mock_module.RIGHT_ARM_BUZZER_CONTROL_PIN
            ],
            timeout=1.5,
        )
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        for node in (probe, dashboard, mock):
            node.destroy_node()
        rclpy.try_shutdown()
