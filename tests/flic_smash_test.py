"""Tests for hardware smash-event timestamp matching."""

from tabletop_py.flic.smash import SmashLatencyMatcher


def test_first_teensy_snapshot_is_baseline_and_repeats_are_ignored():
    matcher = SmashLatencyMatcher("90:88:a9:50:66:0d")

    assert matcher.observe_teensy(1_000_000_000) == []
    assert matcher.baseline_ready
    assert matcher.observe_teensy(1_000_000_000) == []
    assert matcher.pending_teensy_ns == []


def test_events_pair_in_either_arrival_order_using_hardware_timestamps():
    addr = "90:88:a9:50:66:0d"
    matcher = SmashLatencyMatcher(addr)
    matcher.observe_teensy(1_000_000_000)

    assert matcher.observe_flic(addr, 2_012_000_000) == []
    samples = matcher.observe_teensy(2_000_000_000)
    assert len(samples) == 1
    assert samples[0].delta_ms == 12.0

    assert matcher.observe_teensy(3_000_000_000) == []
    samples = matcher.observe_flic(addr, 2_994_000_000)
    assert len(samples) == 1
    assert samples[0].delta_ms == -6.0


def test_wrong_button_and_stale_events_do_not_pair():
    addr = "90:88:a9:50:66:0d"
    matcher = SmashLatencyMatcher(addr, pairing_window_ms=100.0)
    matcher.observe_teensy(1_000_000_000)

    assert matcher.observe_flic("90:88:a9:50:5f:db", 2_000_000_000) == []
    assert matcher.pending_flic_ns == []

    assert matcher.observe_teensy(2_000_000_000) == []
    assert matcher.observe_flic(addr, 2_500_000_000) == []
    assert matcher.expired_teensy_events == 1


def test_pairing_window_must_be_positive():
    try:
        SmashLatencyMatcher("90:88:a9:50:66:0d", pairing_window_ms=0)
    except ValueError as error:
        assert "must be positive" in str(error)
    else:
        raise AssertionError("Expected invalid pairing window to fail")
