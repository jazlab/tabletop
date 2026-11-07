"""ROS-free access to rosbag2 MCAP files.

Wraps ``mcap`` + ``mcap-ros2-support`` and fixes one incompatibility: rosbag2
records service introspection topics (``/<service>/_service_event``) with a
schema named ``pkg/srv/Name_Event`` whose text is the ``.srv`` definition
(``SRV: ...``). ``mcap_ros2`` rejects that name and cannot parse the text, so
:func:`normalize_schema` rewrites it into the equivalent message definition
(``service_msgs/ServiceEventInfo info``, ``Request[<=1] request``,
``Response[<=1] response``), which has the same CDR wire layout.
"""

from __future__ import annotations

import re
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from mcap.reader import make_reader
from mcap.records import Schema
from mcap_ros2._dynamic import generate_dynamic, serialize_dynamic

SEPARATOR = "=" * 80

# Jazzy service_msgs/msg/ServiceEventInfo (char[16] has the same wire format as uint8[16]).
SERVICE_EVENT_INFO = """uint8 REQUEST_SENT=0
uint8 REQUEST_RECEIVED=1
uint8 RESPONSE_SENT=2
uint8 RESPONSE_RECEIVED=3
uint8 event_type
builtin_interfaces/Time stamp
uint8[16] client_gid
int64 sequence_number"""

TIME_MSG = "int32 sec\nuint32 nanosec"


def _split_sections(text: str) -> list[str]:
    parts = re.split(r"^=+\s*$", text, flags=re.MULTILINE)
    return [p.strip("\n") for p in parts if p.strip()]


def normalize_schema(name: str, text: str) -> tuple[str, str]:
    """Return a (schema_name, schema_text) pair that ``mcap_ros2`` can parse.

    Non-service schemas are returned unchanged.
    """
    if not name.endswith("_Event") or "SRV:" not in text:
        return name, text
    pkg = name.split("/")[0]
    srv = name.split("/")[-1][: -len("_Event")]
    sections = _split_sections(text)
    srv_section = next(s for s in sections if s.lstrip().startswith("SRV:"))
    body = srv_section.split("\n", 1)[1] if "\n" in srv_section else ""
    req, _, resp = body.partition("\n---")
    resp = resp.lstrip("-").lstrip("\n") if resp else ""
    req_name = f"{pkg}/{srv}Request"
    resp_name = f"{pkg}/{srv}Response"
    out = [
        "service_msgs/ServiceEventInfo info",
        f"{req_name}[<=1] request",
        f"{resp_name}[<=1] response",
        SEPARATOR,
        "MSG: service_msgs/ServiceEventInfo",
        SERVICE_EVENT_INFO,
        SEPARATOR,
        f"MSG: {req_name}",
        req.strip("\n"),
        SEPARATOR,
        f"MSG: {resp_name}",
        resp.strip("\n"),
    ]
    deps = [s for s in sections if s.lstrip().startswith("MSG:")]
    if not any(
        s.lstrip().startswith("MSG: builtin_interfaces/Time\n")
        or s.strip() == "MSG: builtin_interfaces/Time"
        for s in deps
    ):
        deps.append("MSG: builtin_interfaces/Time\n" + TIME_MSG)
    for d in deps:
        out += [SEPARATOR, d]
    return f"{pkg}/{srv}Event", "\n".join(out) + "\n"


_decoder_cache: dict[tuple[str, bytes], Callable[[bytes], Any]] = {}


def decoder_for(schema: Schema) -> Callable[[bytes], Any]:
    # Keyed by content, not id(schema): schema objects from earlier bags are freed and
    # their ids reused, which silently handed one type's decoder to another type.
    key = (schema.name, bytes(schema.data))
    if key not in _decoder_cache:
        name, text = normalize_schema(schema.name, schema.data.decode())
        _decoder_cache[key] = generate_dynamic(name, text)[name]
    return _decoder_cache[key]


def encoder_for(schema_name: str, schema_text: str) -> Callable[[Any], bytes]:
    """CDR encoder for a (possibly service-event) schema; used by test fixtures."""
    name, text = normalize_schema(schema_name, schema_text)
    return serialize_dynamic(name, text)[name]


def bag_files(bag_dir: Path) -> list[Path]:
    files = sorted(Path(bag_dir).glob("*.mcap"))
    if not files:
        raise FileNotFoundError(f"no .mcap files in {bag_dir}")
    return files


@dataclass
class TopicInfo:
    topic: str
    schema: str
    count: int = 0


@dataclass
class BagSummary:
    files: list[Path]
    start_ns: int | None
    end_ns: int | None
    topics: dict[str, TopicInfo] = field(default_factory=dict)
    schema_texts: dict[str, str] = field(default_factory=dict)

    def count(self, topic: str) -> int:
        t = self.topics.get(topic)
        return t.count if t else 0


def summarize(bag_dir: Path) -> BagSummary:
    files = bag_files(bag_dir)
    out = BagSummary(files=files, start_ns=None, end_ns=None)
    for f in files:
        with open(f, "rb") as fh:
            s = make_reader(fh).get_summary()
        if s is None:
            raise RuntimeError(
                f"{f} has no summary section (unfinished recording?)"
            )
        st = s.statistics
        if st.message_count:
            out.start_ns = (
                st.message_start_time
                if out.start_ns is None
                else min(out.start_ns, st.message_start_time)
            )
            out.end_ns = (
                st.message_end_time
                if out.end_ns is None
                else max(out.end_ns, st.message_end_time)
            )
        for cid, ch in s.channels.items():
            sc = s.schemas[ch.schema_id]
            ti = out.topics.setdefault(ch.topic, TopicInfo(ch.topic, sc.name))
            ti.count += st.channel_message_counts.get(cid, 0)
            out.schema_texts.setdefault(sc.name, sc.data.decode())
    return out


@dataclass
class Message:
    topic: str
    log_time_ns: int
    msg: Any


def iter_messages(bag_dir: Path, topics: Iterable[str]) -> Iterator[Message]:
    """Decoded messages for the given topics, in log-time order within each file."""
    topics = list(topics)
    for f in bag_files(bag_dir):
        with open(f, "rb") as fh:
            reader = make_reader(fh)
            for schema, channel, message in reader.iter_messages(
                topics=topics, log_time_order=True
            ):
                yield Message(
                    channel.topic,
                    message.log_time,
                    decoder_for(schema)(message.data),
                )


def stamp_ns(t: Any) -> int:
    """builtin_interfaces/Time → integer nanoseconds."""
    return int(t.sec) * 1_000_000_000 + int(t.nanosec)
