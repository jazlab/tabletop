"""Display eye-only 3D gaze in RViz for a short, idle-rig bench test.

Sample subscription *starts hardware acquisition*. This tool never changes
the latest-session symlink, calls the force-stop service, or commands a robot.
Keep the task launcher idle until the preview exits.
"""

import argparse
import json
import os
import signal
import tempfile
import time
from contextlib import contextmanager
from pathlib import Path

import numpy as np

SOURCE = "/eyelink/sample_array"
OUTPUT = "/gaze_preview/point"
NODE = "tabletop_gaze_preview"
PARAM = "session_bag_dir"


def _features(sample):
    values = np.asarray(
        [
            sample.left_x,
            sample.left_y,
            sample.right_x,
            sample.right_y,
            sample.left_pupil,
            sample.right_pupil,
        ],
        dtype=float,
    )
    if (
        np.isfinite(values).all()
        and (abs(values[:4]) <= 20000).all()
        and (values[4:] > 0).all()
    ):
        return values
    return None


@contextmanager
def _scratch_route(get_path, set_path, scratch, normal):
    """Restore only a path we own, including on an uncertain initial write."""
    if get_path() != normal:
        raise RuntimeError(
            "EyeLink capture directory changed; leaving it alone"
        )
    restored = False

    def restore():
        nonlocal restored
        if restored:
            return
        current = get_path()
        if current == scratch:
            set_path(normal)
        elif current != normal:
            raise RuntimeError(
                "EyeLink capture path changed elsewhere; not overwriting it"
            )
        restored = True

    try:
        set_path(scratch)
        if get_path() != scratch:
            raise RuntimeError(
                "EyeLink did not confirm the scratch capture path"
            )
        yield restore
    finally:
        restore()


def preview(decoder, *, seconds, check):
    import rclpy
    from rclpy.duration import Duration
    from rclpy.parameter import Parameter
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from tabletop_interfaces.msg import EyelinkBatch
    from visualization_msgs.msg import Marker

    from tabletop_rig.utils.recording_storage import (
        validate_recording_environment,
    )

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(NODE, enable_rosout=False)
    stop = False
    latest = None

    def interrupt(*_):
        nonlocal stop
        stop = True

    def recv(batch):
        nonlocal latest
        if batch.samples:
            latest = batch.samples[-1]

    def wait(future):
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done():
            raise RuntimeError("EyeLink parameter service timed out")
        return future.result()

    params = AsyncParameterClient(node, "/eyelink")

    def get_path():
        return wait(params.get_parameters([PARAM])).values[0].string_value

    def set_path(path):
        result = wait(
            params.set_parameters_atomically([Parameter(PARAM, value=path)])
        ).result
        if not result.successful:
            raise RuntimeError(
                f"EyeLink refused the capture directory: {result.reason}"
            )

    def require_idle():
        names = [n for n, _ in node.get_node_names_and_namespaces()]
        subscribers = node.get_subscriptions_info_by_topic(SOURCE)
        if (
            names.count(NODE) != 1
            or any(n == "commander" or "rosbag" in n for n in names)
            or any(ep.node_name != NODE for ep in subscribers)
        ):
            raise RuntimeError(
                "A task, recorder, or other EyeLink subscriber is present; preview requires an idle rig."
            )
        sources = node.get_publishers_info_by_topic(SOURCE)
        if len(sources) != 1 or (
            sources[0].node_namespace,
            sources[0].node_name,
        ) != ("/", "eyelink"):
            raise RuntimeError(
                "Expected only the real /eyelink batch publisher"
            )

    def marker(action, sample=None):
        msg = Marker()
        msg.header.frame_id = "world"
        if sample is not None:
            msg.header.stamp = sample.header.stamp
        else:
            msg.header.stamp = node.get_clock().now().to_msg()
        msg.ns, msg.id = NODE, 0
        msg.type, msg.action = Marker.SPHERE, action
        msg.pose.orientation.w = 1.0
        msg.scale.x = msg.scale.y = msg.scale.z = 0.045
        msg.color.r, msg.color.g, msg.color.b, msg.color.a = (
            1.0,
            0.49,
            0.13,
            1.0,
        )
        msg.lifetime = Duration(seconds=0.25).to_msg()
        return msg

    handlers = {
        s: signal.signal(s, interrupt) for s in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        if not params.wait_for_services(timeout_sec=5):
            raise RuntimeError("EyeLink parameter services unavailable")
        until = time.monotonic() + 2
        while not stop and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.1)
        require_idle()
        root = Path(os.environ["ROS_BAG_DIR"])
        normal = str(root / "latest")
        if (
            not root.is_absolute()
            or root == Path("/")
            or not root.is_dir()
            or get_path() != normal
        ):
            raise RuntimeError(
                "EyeLink is not on its normal capture mount/path; aborting"
            )
        validate_recording_environment(str(root))
        if check or stop:
            print("Idle EyeLink found. No capture started.", flush=True)
            return

        parent = root / ".gaze-preview"
        parent.mkdir(exist_ok=True)
        if parent.is_symlink():
            raise RuntimeError("Refusing redirected preview storage")
        scratch = tempfile.mkdtemp(prefix="gaze-", dir=parent)
        validate_recording_environment(scratch)
        print(f"EyeLink-only scratch capture: {scratch}", flush=True)
        pub = node.create_publisher(Marker, OUTPUT, 1)
        require_idle()
        with _scratch_route(get_path, set_path, scratch, normal) as release:
            if stop:
                return
            sub = node.create_subscription(
                EyelinkBatch,
                SOURCE,
                recv,
                QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            try:
                start = time.monotonic()
                render, guard, last_stamp = start, start, -1
                shown = False
                while not stop and time.monotonic() < start + seconds:
                    rclpy.spin_once(node, timeout_sec=0.025)
                    now = time.monotonic()
                    if now >= guard:
                        require_idle()
                        guard = now + 1
                    if latest is None:
                        if now - start > 12:
                            raise RuntimeError("No EyeLink samples received")
                        continue
                    # EyeLink pins _active_session_dir before sending the first
                    # sample. Its EDF transfer/integrity/CSV use that pinned path.
                    release()
                    if now < render:
                        continue
                    render = now + 0.05
                    stamp = (
                        latest.header.stamp.sec * 10**9
                        + latest.header.stamp.nanosec
                    )
                    age_ns = node.get_clock().now().nanoseconds - stamp
                    if stamp <= last_stamp or not 0 <= age_ns <= 200_000_000:
                        continue
                    last_stamp = stamp
                    x = _features(latest)
                    xyz = decoder.predict_m(x) if x is not None else None
                    if xyz is None or not np.isfinite(xyz).all():
                        pub.publish(marker(Marker.DELETE))
                        continue
                    dot = marker(Marker.ADD, latest)
                    (
                        dot.pose.position.x,
                        dot.pose.position.y,
                        dot.pose.position.z,
                    ) = map(float, xyz)
                    pub.publish(dot)
                    if not shown:
                        print(
                            "Live gaze visible on noVNC tab 2. Ctrl-C stops the preview.",
                            flush=True,
                        )
                        shown = True
            finally:
                # Dropping our one subscription triggers EyeLink's normal
                # last-subscriber cleanup; never force-stop another consumer.
                node.destroy_subscription(sub)
                pub.publish(marker(Marker.DELETE))
        if latest is not None:
            report = Path(scratch) / "eyelink_integrity.json"
            until = time.monotonic() + 60
            while time.monotonic() < until and not report.is_file():
                time.sleep(0.1)
            if not report.is_file() or not json.loads(report.read_text()).get(
                "edf_saved"
            ):
                raise RuntimeError(
                    f"Scratch EDF finalization not confirmed: inspect {scratch} before starting another session"
                )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        for s, handler in handlers.items():
            signal.signal(s, handler)


def main(argv=None):
    from tabletop_py.gaze.pupil_corrected import PupilCorrectedDecoder

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        type=Path,
        required=True,
        help="Pupil-corrected ensemble NPZ",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Check readiness without starting EyeLink capture",
    )
    parser.add_argument(
        "--seconds",
        type=int,
        choices=range(1, 601),
        default=120,
        metavar="1-600",
    )
    args = parser.parse_args(argv)
    preview(
        PupilCorrectedDecoder.load(args.model),
        seconds=args.seconds,
        check=args.check,
    )


if __name__ == "__main__":
    main()
