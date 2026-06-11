"""Unit tests for the camera synchronization analysis.

This module contains pytest test cases for the pure analysis logic in
tabletop_rig.nodes.system_check used by the ``flir_sync`` system
check. Synthetic frame data simulates the failure modes the live
check must detect:

- Hardware desynchronization (a camera free-running off the trigger)
- Header stamp skew (driver failing to group simultaneous frames)
- Dropped frames (incomplete frame groups)

Test Classes:
    TestAnalyzeCameraSync: Tests for analyze_camera_sync.

Example:
    pytest tests/camera_sync_test.py -v
"""

import pytest
from tabletop_rig.nodes.system_check import (
    SyncAnalysis,
    analyze_camera_sync,
)

# ~120 Hz trigger period in nanoseconds.
PERIOD_NS = 8_333_333

# Arbitrary per-camera hardware clock epoch offset. The camera
# hardware clocks are free-running, so each camera has a different
# epoch even when perfectly synchronized.
EPOCH_NS = 10**12


def make_synced_frames(
    cameras: tuple[str, ...] = ("cam_a", "cam_b", "cam_c"),
    n_frames: int = 100,
) -> dict[str, list[tuple[int, int]]]:
    """Create perfectly synchronized per-camera frame data.

    Args:
        cameras: Camera names.
        n_frames: Number of frames per camera.

    Returns:
        Mapping from camera name to (stamp ns, camera time ns) tuples.
    """
    return {
        cam: [
            (i * PERIOD_NS, k * EPOCH_NS + i * PERIOD_NS)
            for i in range(n_frames)
        ]
        for k, cam in enumerate(cameras)
    }


class TestAnalyzeCameraSync:
    """Tests for the analyze_camera_sync function."""

    def test_synchronized(self):
        """Perfectly synchronized cameras produce perfect metrics."""
        analysis = analyze_camera_sync(make_synced_frames())

        assert isinstance(analysis, SyncAnalysis)
        assert analysis.cameras == ["cam_a", "cam_b", "cam_c"]
        assert analysis.frame_counts == {
            "cam_a": 100,
            "cam_b": 100,
            "cam_c": 100,
        }
        assert analysis.estimated_rate == pytest.approx(120.0, abs=0.01)
        assert analysis.complete_fraction == 1.0
        assert analysis.max_stamp_spread == 0.0
        assert analysis.max_hw_interval_spread == 0.0
        assert analysis.n_duplicate_frames == 0

    def test_hardware_desync(self):
        """A camera free-running 2% off the trigger is detected.

        The header stamps still group perfectly (the driver assigns
        them), but the hardware clock intervals differ across cameras,
        which is the signature of a camera not locked to the trigger.
        """
        frames = make_synced_frames()
        frames["cam_c"] = [
            (i * PERIOD_NS, 2 * EPOCH_NS + int(i * PERIOD_NS * 0.98))
            for i in range(100)
        ]

        analysis = analyze_camera_sync(frames)

        assert analysis.max_stamp_spread == 0.0
        # 2% of an 8.3 ms period is ~167 us per frame interval.
        assert analysis.max_hw_interval_spread > 100e-6

    def test_stamp_skew(self):
        """Header stamps skewed by 1 ms on one camera are detected."""
        frames = make_synced_frames()
        frames["cam_c"] = [
            (i * PERIOD_NS + 1_000_000, 2 * EPOCH_NS + i * PERIOD_NS)
            for i in range(100)
        ]

        analysis = analyze_camera_sync(frames)

        assert analysis.max_stamp_spread >= 1e-3
        assert analysis.max_hw_interval_spread == 0.0

    def test_dropped_frames(self):
        """A camera dropping half its frames lowers completeness."""
        frames = make_synced_frames()
        frames["cam_c"] = [
            (i * PERIOD_NS, 2 * EPOCH_NS + i * PERIOD_NS)
            for i in range(0, 100, 2)
        ]

        analysis = analyze_camera_sync(frames)

        assert analysis.complete_fraction == pytest.approx(0.5, abs=0.02)
        # Dropped frames must not corrupt the hardware interval check:
        # intervals spanning a dropped frame cover the same trigger
        # pulses for every camera.
        assert analysis.max_hw_interval_spread == 0.0

    def test_double_rate_camera(self):
        """A camera free-running at double rate lowers completeness.

        The doubled frames dominate the period estimate, so they land
        in their own groups rather than duplicating existing ones,
        and the failure manifests as incomplete groups.
        """
        frames = make_synced_frames()
        frames["cam_c"] = [
            (
                i * PERIOD_NS // 2,
                2 * EPOCH_NS + i * PERIOD_NS // 2,
            )
            for i in range(200)
        ]

        analysis = analyze_camera_sync(frames)

        assert analysis.complete_fraction == pytest.approx(0.5, abs=0.02)

    def test_duplicate_frames(self):
        """Spurious extra frames within a period are detected."""
        frames = make_synced_frames()
        # Inject an extra cam_c frame shortly after a few real ones,
        # e.g. a bouncing trigger line causing double exposures.
        frames["cam_c"] = sorted(
            frames["cam_c"]
            + [
                (
                    i * PERIOD_NS + PERIOD_NS // 10,
                    2 * EPOCH_NS + i * PERIOD_NS + PERIOD_NS // 10,
                )
                for i in (10, 20, 30)
            ]
        )

        analysis = analyze_camera_sync(frames)

        assert analysis.n_duplicate_frames == 3

    def test_too_few_frames_raises(self):
        """Fewer than two frames per camera cannot be analyzed."""
        frames = {"cam_a": [(0, 0)], "cam_b": []}

        with pytest.raises(ValueError, match="frame period"):
            analyze_camera_sync(frames)
