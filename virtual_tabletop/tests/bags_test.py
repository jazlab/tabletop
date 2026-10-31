"""Bag reader tests against a synthetic rosbag2-style MCAP."""

import numpy as np
import pytest
from mcap_ros2.writer import Writer
from virtual_tabletop.bags import reader

TIME_MSG = "int32 sec\nuint32 nanosec"
HEADER_MSG = "builtin_interfaces/Time stamp\nstring frame_id"
JOINT_STATE_MSGDEF = "\n".join(
    [
        "std_msgs/Header header",
        "string[] name",
        "float64[] position",
        "float64[] velocity",
        "float64[] effort",
        "=" * 80,
        "MSG: std_msgs/Header",
        HEADER_MSG,
        "=" * 80,
        "MSG: builtin_interfaces/Time",
        TIME_MSG,
    ]
)
NAMES = ["left_elbow_joint", "right_elbow_joint"]


@pytest.fixture
def bag_dir(tmp_path):
    """Two split MCAP files, joint order swapped in the second."""
    for part, names in enumerate([NAMES, NAMES[::-1]]):
        with open(tmp_path / f"bag_{part}.mcap", "wb") as f:
            writer = Writer(f)
            schema = writer.register_msgdef(
                "sensor_msgs/msg/JointState", JOINT_STATE_MSGDEF
            )
            for i in range(5):
                k = part * 5 + i
                t_ns = 1_000_000_000 + k * 10_000_000
                writer.write_message(
                    "/joint_states",
                    schema,
                    {
                        "header": {
                            "stamp": {"sec": 1, "nanosec": k * 10_000_000},
                            "frame_id": "",
                        },
                        "name": names,
                        "position": [
                            0.1 * k if n == NAMES[0] else -0.1 * k
                            for n in names
                        ],
                        "velocity": [],
                        "effort": [],
                    },
                    log_time=t_ns,
                    publish_time=t_ns,
                )
            writer.finish()
    return tmp_path


def test_summarize(bag_dir):
    summary = reader.summarize(bag_dir)
    assert len(summary.files) == 2
    assert [(t.topic, t.count) for t in summary.topics] == [
        ("/joint_states", 10)
    ]
    assert summary.topics[0].schema == "sensor_msgs/msg/JointState"
    assert summary.duration == pytest.approx(0.09)


def test_read_joint_states_reorders_columns(bag_dir):
    js = reader.read_joint_states(bag_dir)
    assert js.names == sorted(NAMES)
    np.testing.assert_allclose(js.t - js.t[0], np.arange(10) * 0.01)
    np.testing.assert_allclose(js.position[:, 0], 0.1 * np.arange(10))
    np.testing.assert_allclose(js.position[:, 1], -0.1 * np.arange(10))
    assert np.isnan(js.velocity).all()
