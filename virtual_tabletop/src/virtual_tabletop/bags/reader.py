"""Read rig MCAP bags without ROS.

rosbag2 writes a session as a directory of (possibly split) ``.mcap`` files
plus ``metadata.yaml``. Each MCAP embeds the ROS 2 message definitions it
uses, so ``mcap-ros2-support`` can decode every message, including the rig's
custom ``tabletop_interfaces`` types, with no ROS installation.
"""

import argparse
from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages


@dataclass(frozen=True)
class TopicSummary:
    """Per-topic statistics from the MCAP summary sections.

    Attributes:
        topic: Topic name.
        schema: ROS message type, e.g. ``sensor_msgs/msg/JointState``.
        count: Number of messages.
    """

    topic: str
    schema: str
    count: int


@dataclass(frozen=True)
class BagSummary:
    """Overview of a bag.

    Attributes:
        files: The MCAP files, in recording order.
        start_ns: First message log time (ns since epoch).
        end_ns: Last message log time (ns since epoch).
        topics: Per-topic summaries, sorted by topic name.
    """

    files: list[Path]
    start_ns: int
    end_ns: int
    topics: list[TopicSummary]

    @property
    def duration(self) -> float:
        """Bag duration in seconds."""
        return (self.end_ns - self.start_ns) * 1e-9


@dataclass(frozen=True)
class JointStates:
    """``sensor_msgs/JointState`` samples as arrays.

    Attributes:
        names: Joint names, in column order.
        t: Header stamps in seconds since epoch, shape ``(N,)``.
        position: Positions in radians, shape ``(N, len(names))``.
        velocity: Velocities, same shape, NaN where not published.
    """

    names: list[str]
    t: np.ndarray
    position: np.ndarray
    velocity: np.ndarray


def bag_files(path: str | Path) -> list[Path]:
    """Return the MCAP files of a bag (a directory or a single file)."""
    path = Path(path)
    if path.is_file():
        return [path]
    files = sorted(path.glob("*.mcap"))
    if not files:
        raise FileNotFoundError(f"No .mcap files in {path}")
    return files


def summarize(path: str | Path) -> BagSummary:
    """Summarize a bag from its MCAP summary sections (no decoding)."""
    files = bag_files(path)
    counts: dict[str, int] = {}
    schemas: dict[str, str] = {}
    start, end = None, None
    for file in files:
        with file.open("rb") as stream:
            summary = make_reader(stream).get_summary()
        if summary is None or summary.statistics is None:
            raise ValueError(f"{file} has no summary section")
        stats = summary.statistics
        if stats.message_count:
            start = min(
                start or stats.message_start_time, stats.message_start_time
            )
            end = max(end or stats.message_end_time, stats.message_end_time)
        for channel_id, count in stats.channel_message_counts.items():
            channel = summary.channels[channel_id]
            counts[channel.topic] = counts.get(channel.topic, 0) + count
            schemas[channel.topic] = summary.schemas[channel.schema_id].name
    topics = [
        TopicSummary(topic, schemas[topic], counts[topic])
        for topic in sorted(counts)
    ]
    return BagSummary(files, start or 0, end or 0, topics)


def iter_messages(
    path: str | Path, topics: Iterable[str] | None = None
) -> Iterator[tuple[str, int, Any]]:
    """Yield ``(topic, log_time_ns, message)`` in log-time order per file.

    Args:
        path: Bag directory or MCAP file.
        topics: Restrict to these topics (all if ``None``).
    """
    topics = list(topics) if topics is not None else None
    for file in bag_files(path):
        for item in read_ros2_messages(str(file), topics=topics):
            yield item.channel.topic, item.log_time_ns, item.ros_msg


def stamp_to_sec(stamp: Any) -> float:
    """Convert a ``builtin_interfaces/Time`` to float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def read_joint_states(
    path: str | Path, topic: str = "/joint_states"
) -> JointStates:
    """Load a ``JointState`` topic into arrays.

    Messages may list joints in any order or subset; columns follow the
    sorted union of all joint names, with NaN where a joint is missing.
    """
    rows = []
    for _, _, msg in iter_messages(path, [topic]):
        rows.append(
            (
                stamp_to_sec(msg.header.stamp),
                list(msg.name),
                list(msg.position),
                list(msg.velocity),
            )
        )
    names = sorted({name for _, row_names, _, _ in rows for name in row_names})
    column = {name: i for i, name in enumerate(names)}
    t = np.empty(len(rows))
    position = np.full((len(rows), len(names)), np.nan)
    velocity = np.full_like(position, np.nan)
    for i, (stamp, row_names, pos, vel) in enumerate(rows):
        t[i] = stamp
        cols = [column[name] for name in row_names]
        position[i, cols] = pos
        if vel:
            velocity[i, cols] = vel
    return JointStates(names, t, position, velocity)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Summarize a rig MCAP bag (no ROS required)."
    )
    parser.add_argument("bag", type=Path, help="bag directory or .mcap file")
    parser.add_argument(
        "--show", metavar="TOPIC", help="print the first messages of a topic"
    )
    parser.add_argument("-n", type=int, default=3, help="messages to show")
    args = parser.parse_args()

    summary = summarize(args.bag)
    print(f"{len(summary.files)} file(s), {summary.duration:.1f} s")
    width = max((len(t.topic) for t in summary.topics), default=0)
    for t in summary.topics:
        print(f"  {t.topic:<{width}}  {t.count:>9}  {t.schema}")
    if args.show:
        for i, (_, log_ns, msg) in enumerate(
            iter_messages(args.bag, [args.show])
        ):
            if i >= args.n:
                break
            print(f"--- {args.show} @ {log_ns * 1e-9:.6f}\n{msg}")


if __name__ == "__main__":
    main()
