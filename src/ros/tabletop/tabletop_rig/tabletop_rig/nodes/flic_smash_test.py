"""No-robot hardware latency test for simultaneous Teensy/Flic presses."""

import argparse
import csv
import statistics
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from tabletop_interfaces.msg import TeensySensor

from tabletop_py.flic.smash import SmashLatencyMatcher, SmashLatencySample


def time_msg_to_ns(msg) -> int:
    """Convert a ROS builtin time message to integer nanoseconds."""
    return int(msg.sec) * 1_000_000_000 + int(msg.nanosec)


class FlicSmashTest(Node):
    """Pair Teensy and Flic timestamps without initializing Commander."""

    def __init__(
        self,
        *,
        target_addr: str,
        sample_count: int,
        pairing_window_ms: float,
        output_path: Path,
    ):
        super().__init__("flic_smash_test")
        self.target_addr = target_addr.lower()
        self.sample_count = sample_count
        self.matcher = SmashLatencyMatcher(
            self.target_addr, pairing_window_ms=pairing_window_ms
        )
        self.latencies_ms: list[float] = []
        self.complete = False

        output_path.parent.mkdir(parents=True, exist_ok=True)
        self._output_file = output_path.open("x", newline="")
        self._csv_writer = csv.writer(self._output_file)
        self._csv_writer.writerow(
            ["sample", "teensy_ns", "flic_ns", "flic_minus_teensy_ms"]
        )
        self._output_file.flush()

        self.create_subscription(
            TeensySensor,
            "/teensy/sensor",
            self._teensy_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Header,
            "/flic/button_pressed_time",
            self._flic_callback,
            10,
        )

        self.get_logger().info(
            f"Smash test listening for Flic {self.target_addr}; "
            f"target samples={self.sample_count}; output={output_path}"
        )

    def _teensy_callback(self, msg: TeensySensor):
        was_ready = self.matcher.baseline_ready
        samples = self.matcher.observe_teensy(
            time_msg_to_ns(msg.button_last_time_pressed)
        )
        if not was_ready and self.matcher.baseline_ready:
            self.get_logger().info(
                "Teensy baseline acquired. Smash the two buttons together, "
                "then fully release them; wait at least 2 seconds between trials."
            )
        self._record_samples(samples)

    def _flic_callback(self, msg: Header):
        samples = self.matcher.observe_flic(
            msg.frame_id, time_msg_to_ns(msg.stamp)
        )
        self._record_samples(samples)

    def _record_samples(self, samples: list[SmashLatencySample]):
        for sample in samples:
            self.latencies_ms.append(sample.delta_ms)
            sample_number = len(self.latencies_ms)
            mean_ms = statistics.fmean(self.latencies_ms)
            std_ms = (
                statistics.pstdev(self.latencies_ms)
                if sample_number > 1
                else 0.0
            )
            self._csv_writer.writerow(
                [
                    sample_number,
                    sample.teensy_ns,
                    sample.flic_ns,
                    f"{sample.delta_ms:.6f}",
                ]
            )
            self._output_file.flush()
            self.get_logger().info(
                f"Sample {sample_number}/{self.sample_count}: "
                f"Flic - Teensy = {sample.delta_ms:+.3f} ms | "
                f"mean={mean_ms:+.3f} ms, std={std_ms:.3f} ms, "
                f"min={min(self.latencies_ms):+.3f} ms, "
                f"max={max(self.latencies_ms):+.3f} ms"
            )

            if sample_number >= self.sample_count:
                self.complete = True
                self.get_logger().info(
                    "Requested smash-test sample count reached"
                )
                break

    def report_summary(self):
        """Log final statistics and any unmatched expired events."""
        if self.latencies_ms:
            mean_ms = statistics.fmean(self.latencies_ms)
            std_ms = (
                statistics.pstdev(self.latencies_ms)
                if len(self.latencies_ms) > 1
                else 0.0
            )
            message = (
                f"Final N={len(self.latencies_ms)}: mean={mean_ms:+.3f} ms, "
                f"std={std_ms:.3f} ms, min={min(self.latencies_ms):+.3f} ms, "
                f"max={max(self.latencies_ms):+.3f} ms; "
                f"expired Teensy={self.matcher.expired_teensy_events}, "
                f"expired Flic={self.matcher.expired_flic_events}"
            )
        else:
            message = "No paired smash-test samples recorded"

        # SIGINT may invalidate rosout before this finally block runs. Preserve
        # the summary without emitting a misleading ROS publisher error.
        if rclpy.ok():
            if self.latencies_ms:
                self.get_logger().info(message)
            else:
                self.get_logger().warning(message)
        else:
            print(message, flush=True)

    def destroy_node(self):
        if hasattr(self, "_output_file") and not self._output_file.closed:
            self._output_file.close()
        super().destroy_node()


def default_output_path() -> Path:
    """Return a timestamped output path in the mounted TableTop log tree."""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(f"/tabletop/log/flic_smash/flic_smash_{stamp}.csv")


def main(args=None):
    """Run the standalone Flic/Teensy hardware smash test."""
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-address", default="90:88:a9:50:66:0d")
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--pairing-window-ms", type=float, default=250.0)
    parser.add_argument("--output", type=Path, default=None)
    non_ros_args = rclpy.utilities.remove_ros_args(args)
    options = parser.parse_args(non_ros_args[1:])

    if options.samples <= 0:
        parser.error("--samples must be positive")

    node = FlicSmashTest(
        target_addr=options.target_address,
        sample_count=options.samples,
        pairing_window_ms=options.pairing_window_ms,
        output_path=options.output or default_output_path(),
    )
    try:
        while rclpy.ok() and not node.complete:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.report_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
