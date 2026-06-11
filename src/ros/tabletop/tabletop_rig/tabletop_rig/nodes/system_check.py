"""ROS2 node for verifying rig components are working properly.

This module provides a node that runs a configurable set of system
checks against the live rig and reports a pass/fail summary. It is
intended for verifying hardware that runs in separate containers
(cameras, teensy, etc.) where conventional launch-based integration
tests are impractical.

Checks are implemented as ``check_<name>`` coroutine methods on the
SystemCheck node and selected via the ``checks`` parameter. Analysis
logic is kept in pure module-level functions so it can also be unit
tested offline with pytest.

Checks provided:
    flir_sync: Verify the synchronized FLIR cameras are exposing and
        publishing simultaneously. Frames are clustered by header
        stamp (the synchronized driver assigns identical stamps to
        frames from the same trigger pulse) and the per-frame
        intervals of each camera's hardware clock are compared across
        cameras to confirm the exposures are locked to the same
        external trigger.

Parameters:
    checks: List of check names to run.
    flir_sync.namespace: Namespace of the synchronized camera driver.
    flir_sync.duration: How long to collect frames in seconds.
    flir_sync.discovery_timeout: How long to wait for camera topics to
        appear in the DDS graph in seconds.
    flir_sync.min_frames_per_camera: Minimum frames per camera.
    flir_sync.max_stamp_spread: Max allowed header stamp spread within
        a frame group in seconds (0.0: stamps must be identical).
    flir_sync.max_hw_interval_spread: Max allowed spread of the
        per-frame hardware clock intervals across cameras in seconds.
    flir_sync.min_complete_fraction: Minimum fraction of frame groups
        that must contain a frame from every camera.
    flir_sync.expected_cameras: Cameras that must be present, or
        "null" to discover from active topics.

Example:
    ros2 run tabletop_rig system_check
    ros2 run tabletop_rig system_check --ros-args \\
        -p checks:="[flir_sync]" -p flir_sync.duration:=10.0
"""

import asyncio
import re
import statistics
import sys
import traceback
from dataclasses import asdict, dataclass, field
from typing import Any

import rclpy
from flir_camera_msgs.msg import ImageMetaData
from rclpy.executors import ConditionReachedException

from tabletop_py.utils.common import yaml_dump_string
from tabletop_rig.executors import AIOExecutor
from tabletop_rig.nodes.base import BaseNode


@dataclass
class CheckResult:
    """Outcome of a single system check.

    Attributes:
        name: Name of the check.
        passed: Whether the check passed.
        failures: Human-readable description of each failed criterion.
        details: Check-specific metrics for logging/inspection.
    """

    name: str
    passed: bool
    failures: list[str] = field(default_factory=list)
    details: dict[str, Any] = field(default_factory=dict)


@dataclass
class SyncAnalysis:
    """Synchronization metrics computed from per-camera frame data.

    Attributes:
        cameras: Sorted camera names.
        frame_counts: Number of frames received per camera.
        estimated_rate: Estimated frame rate in Hz.
        n_groups: Number of frame groups (clusters of frames whose
            header stamps lie within half a frame period).
        n_interior_groups: Groups excluding the first/last, which may
            be truncated by the start/end of the collection window.
        n_complete_groups: Interior groups containing exactly one
            frame from every camera.
        complete_fraction: n_complete_groups / n_interior_groups.
        max_stamp_spread: Largest header stamp spread within any
            interior group in seconds.
        max_hw_interval_spread: Largest across-camera spread of the
            hardware clock interval between consecutive complete
            groups in seconds.
        n_duplicate_frames: Frames sharing a group with another frame
            from the same camera (indicates mismatched frame rates).
    """

    cameras: list[str]
    frame_counts: dict[str, int]
    estimated_rate: float
    n_groups: int
    n_interior_groups: int
    n_complete_groups: int
    complete_fraction: float
    max_stamp_spread: float
    max_hw_interval_spread: float
    n_duplicate_frames: int


def analyze_camera_sync(
    frames: dict[str, list[tuple[int, int]]],
) -> SyncAnalysis:
    """Analyze synchronization of per-camera frame timestamps.

    Frames from all cameras are clustered into groups by header stamp
    using half the estimated frame period as the cluster tolerance,
    so both perfectly synchronized stamps (identical) and slightly
    skewed stamps are grouped correctly and the actual spread can be
    measured.

    Two complementary metrics are computed:

    1. Header stamp spread within each group. The synchronized driver
       assigns identical stamps to frames matched to the same trigger
       pulse, so any nonzero spread means the driver failed to group
       the frames.
    2. Hardware clock interval spread. Each camera latches its own
       free-running hardware clock at exposure time (``camera_time``).
       The clock epochs differ between cameras, but the *interval*
       between two trigger pulses must be the same for every camera,
       so the across-camera spread of per-frame intervals directly
       measures exposure synchronization, independent of any
       timestamping done by the driver on the host.

    Args:
        frames: Mapping from camera name to a list of
            (header stamp in ns, camera hardware time in ns) tuples.

    Returns:
        SyncAnalysis with the computed metrics.

    Raises:
        ValueError: If no camera produced at least two frames (the
            frame period cannot be estimated).
    """
    cameras = sorted(frames)
    frame_counts = {cam: len(recs) for cam, recs in frames.items()}

    # Estimate the trigger period from per-camera header stamp diffs.
    diffs: list[int] = []
    for recs in frames.values():
        stamps = sorted(stamp for stamp, _ in recs)
        diffs.extend(b - a for a, b in zip(stamps, stamps[1:]))
    if not diffs:
        raise ValueError(
            "Cannot estimate frame period: "
            "no camera produced at least two frames"
        )
    period_ns = statistics.median(diffs)

    # Cluster frames from all cameras into groups by header stamp.
    records = sorted(
        (stamp, camera_time, cam)
        for cam, recs in frames.items()
        for stamp, camera_time in recs
    )
    groups: list[list[tuple[int, int, str]]] = []
    for record in records:
        if groups and record[0] - groups[-1][0][0] <= period_ns / 2:
            groups[-1].append(record)
        else:
            groups.append([record])

    # The first and last groups may be truncated by the start/end of
    # the collection window, so exclude them from completeness stats.
    interior = groups[1:-1]
    n_duplicates = sum(
        len(group) - len({record[2] for record in group}) for group in interior
    )

    def is_complete(group: list[tuple[int, int, str]]) -> bool:
        return len(group) == len(cameras) and {
            record[2] for record in group
        } == set(cameras)

    n_complete = sum(1 for group in interior if is_complete(group))
    max_stamp_spread_ns = max(
        (group[-1][0] - group[0][0] for group in interior),
        default=0,
    )

    # Compare the hardware clock interval between consecutive complete
    # groups across cameras. Skipping incomplete groups is fine: the
    # interval still covers the same trigger pulses for every camera.
    complete_by_cam = [
        {record[2]: record for record in group}
        for group in interior
        if is_complete(group)
    ]
    hw_spreads_ns: list[int] = []
    for prev, curr in zip(complete_by_cam, complete_by_cam[1:]):
        intervals = [curr[cam][1] - prev[cam][1] for cam in cameras]
        hw_spreads_ns.append(max(intervals) - min(intervals))

    return SyncAnalysis(
        cameras=cameras,
        frame_counts=frame_counts,
        estimated_rate=1e9 / period_ns,
        n_groups=len(groups),
        n_interior_groups=len(interior),
        n_complete_groups=n_complete,
        complete_fraction=(n_complete / len(interior) if interior else 0.0),
        max_stamp_spread=max_stamp_spread_ns / 1e9,
        max_hw_interval_spread=max(hw_spreads_ns, default=0) / 1e9,
        n_duplicate_frames=n_duplicates,
    )


class SystemCheck(BaseNode):
    """Node that runs system checks against the live rig.

    Each check is a ``check_<name>`` coroutine method returning a
    CheckResult. The ``checks`` parameter selects which checks to run;
    results are logged and returned by run_checks().
    """

    default_params = BaseNode.default_params | {
        "checks": ["flir_sync"],
        "flir_sync.namespace": "/cam_sync",
        "flir_sync.duration": 5.0,
        "flir_sync.discovery_timeout": 5.0,
        "flir_sync.min_frames_per_camera": 20,
        "flir_sync.max_stamp_spread": 0.0,
        "flir_sync.max_hw_interval_spread": 100.0e-6,
        "flir_sync.min_complete_fraction": 0.95,
        "flir_sync.expected_cameras": "null",
    }

    def __init__(self):
        """Initialize the system check node."""
        super().__init__("system_check")
        self.log(f"SystemCheck initialized, checks: {self.param('checks')}")

    async def run_checks(self) -> list[CheckResult]:
        """Run all configured checks and log a summary.

        Returns:
            List of CheckResult, one per configured check.
        """
        results: list[CheckResult] = []
        for name in self.param("checks"):
            method = getattr(self, f"check_{name}", None)
            if method is None:
                results.append(
                    CheckResult(name, False, [f"Unknown check '{name}'"])
                )
                continue

            self.log(f"Running check '{name}'")
            try:
                result: CheckResult = await method()
            except Exception as e:
                self.log(traceback.format_exc(), severity="ERROR")
                result = CheckResult(name, False, [f"Exception: {e!r}"])
            results.append(result)

            status = "PASSED" if result.passed else "FAILED"
            severity = "INFO" if result.passed else "ERROR"
            details = yaml_dump_string(result.details)
            self.log(f"Check '{name}' {status}\n{details}", severity=severity)
            for failure in result.failures:
                self.log(f"  - {failure}", severity="ERROR")

        n_passed = sum(result.passed for result in results)
        self.log(f"System check complete: {n_passed}/{len(results)} passed")
        return results

    async def check_flir_sync(self) -> CheckResult:
        """Check that the synchronized FLIR cameras are in sync.

        Subscribes to every ``<namespace>/<camera>/meta`` topic, collects
        frames for the configured duration and verifies that frames
        from all cameras form complete groups with identical header
        stamps and matching hardware clock intervals.

        Returns:
            CheckResult with synchronization metrics in details.
        """
        namespace: str = self.param("flir_sync.namespace").rstrip("/")
        duration: float = self.param("flir_sync.duration")
        discovery_timeout: float = self.param("flir_sync.discovery_timeout")

        # Discover camera meta topics in the configured namespace,
        # retrying until DDS graph discovery has caught up.
        pattern = re.compile(rf"^{re.escape(namespace)}/([^/]+)/meta$")
        deadline = self.ros_time() + discovery_timeout
        while True:
            cameras = sorted(
                match.group(1)
                for topic, _ in self.get_topic_names_and_types()
                if (match := pattern.match(topic))
            )
            if cameras or self.ros_time() >= deadline:
                break
            await asyncio.sleep(0.2)

        failures: list[str] = []
        expected = self.param("flir_sync.expected_cameras")
        if expected is not None:
            missing = sorted(set(expected) - set(cameras))
            if missing:
                failures.append(f"Expected cameras not found: {missing}")
        if not cameras:
            failures.append(f"No camera meta topics found under '{namespace}'")
            return CheckResult("flir_sync", False, failures)

        # Collect (header stamp, hardware time) pairs per camera.
        frames: dict[str, list[tuple[int, int]]] = {cam: [] for cam in cameras}

        def make_callback(cam: str):
            def callback(msg: ImageMetaData):
                stamp = msg.header.stamp
                frames[cam].append(
                    (stamp.sec * 10**9 + stamp.nanosec, msg.camera_time)
                )

            return callback

        subscriptions = [
            self.create_subscription(
                ImageMetaData,
                f"{namespace}/{cam}/meta",
                make_callback(cam),
                50,
            )
            for cam in cameras
        ]
        self.log(
            f"Collecting frames for {duration:.1f} s from "
            f"{len(cameras)} cameras: {cameras}"
        )
        try:
            await asyncio.sleep(duration)
        finally:
            for subscription in subscriptions:
                self.destroy_subscription(subscription)

        min_frames: int = self.param("flir_sync.min_frames_per_camera")
        for cam, recs in sorted(frames.items()):
            if len(recs) < min_frames:
                failures.append(
                    f"Camera '{cam}' produced {len(recs)} frames, "
                    f"expected at least {min_frames}"
                )

        try:
            analysis = analyze_camera_sync(frames)
        except ValueError as e:
            failures.append(str(e))
            return CheckResult(
                "flir_sync",
                False,
                failures,
                {cam: len(recs) for cam, recs in frames.items()},
            )

        max_stamp_spread: float = self.param("flir_sync.max_stamp_spread")
        if analysis.max_stamp_spread > max_stamp_spread:
            failures.append(
                f"Header stamp spread {analysis.max_stamp_spread:.9f} s "
                f"exceeds {max_stamp_spread:.9f} s: the driver did not "
                f"assign identical stamps to simultaneous frames"
            )

        max_hw_spread: float = self.param("flir_sync.max_hw_interval_spread")
        if analysis.max_hw_interval_spread > max_hw_spread:
            failures.append(
                f"Hardware clock interval spread "
                f"{analysis.max_hw_interval_spread:.9f} s exceeds "
                f"{max_hw_spread:.9f} s: camera exposures are not locked "
                f"to the same trigger pulse"
            )

        min_complete: float = self.param("flir_sync.min_complete_fraction")
        if analysis.complete_fraction < min_complete:
            failures.append(
                f"Only {analysis.complete_fraction:.1%} of frame groups "
                f"contain all cameras, expected at least "
                f"{min_complete:.1%}"
            )

        if analysis.n_duplicate_frames > 0:
            failures.append(
                f"{analysis.n_duplicate_frames} frames grouped with "
                f"another frame from the same camera: cameras may be "
                f"free-running at different rates"
            )

        return CheckResult(
            "flir_sync", not failures, failures, asdict(analysis)
        )


async def main_async(args=None) -> int:
    """Async entry point for the system check node.

    Initializes ROS2, runs the configured checks and returns a
    process exit code.

    Args:
        args: Command line arguments (passed to rclpy.init).

    Returns:
        0 if all checks passed, 1 otherwise.
    """
    rclpy.init(args=args)

    results: list[CheckResult] = []
    try:
        executor = AIOExecutor()
        node = SystemCheck()
        executor.add_node(node)

        try:
            task = executor.create_task(node.run_checks())
            try:
                await executor.spin_until_future_complete(task)
            except* ConditionReachedException:
                # Raised by the executor when the future completes.
                pass
            results = task.result() or []
        finally:
            node.destroy_node()
            executor.shutdown()
    finally:
        rclpy.try_shutdown()  # type: ignore

    return 0 if results and all(r.passed for r in results) else 1


def main(args=None):
    """Entry point for the system_check node."""
    try:
        sys.exit(asyncio.run(main_async(args)))
    except KeyboardInterrupt:
        print("Keyboard interrupt")


if __name__ == "__main__":
    main()
