"""Replay tests: object tracking and timelines from synthetic bags."""

import mujoco
import numpy as np
import pytest
from bags_test import JOINT_STATE_MSGDEF
from mcap_ros2.writer import Writer
from virtual_tabletop.assets import build, named_poses
from virtual_tabletop.replay.objects import ObjectTracker
from virtual_tabletop.replay.timeline import build_timeline

from virtual_tabletop import ik

IDENTITY = np.array([1.0, 0.0, 0.0, 0.0])


def test_tracker_attach_carry_release():
    mounts = np.array([[0.0, 0.0, 1.0], [0.3, 0.0, 1.0]])
    tracker = ObjectTracker(mounts, np.tile(IDENTITY, (2, 1)))

    pos, _ = tracker.update({"left": (np.array([0.0, 0.0, 1.5]), IDENTITY)})
    np.testing.assert_allclose(pos, mounts)
    assert tracker.holding("left") is None

    # Reach object 1's mount with a 1 cm offset: it attaches.
    eef = np.array([0.3, 0.01, 1.0])
    tracker.update({"left": (eef, IDENTITY)})
    assert tracker.holding("left") == 1

    # Carry it: it keeps the grasp offset; object 0 stays put.
    pos, _ = tracker.update({"left": (eef + [0.0, 0.5, 0.2], IDENTITY)})
    np.testing.assert_allclose(pos[1], mounts[1] + [0.0, 0.5, 0.2])
    np.testing.assert_allclose(pos[0], mounts[0])

    # Return: it detaches and snaps back to its mount, and lingering at the
    # mount does not grasp it again.
    tracker.update({"left": (eef, IDENTITY)})
    tracker.update({"left": (eef, IDENTITY)})
    assert tracker.holding("left") is None
    pos, _ = tracker.update({"left": (eef + [0.0, 0.0, 0.5], IDENTITY)})
    assert tracker.holding("left") is None
    np.testing.assert_allclose(pos, mounts)


def test_tracker_rotation_follows_eef():
    tracker = ObjectTracker(np.array([[0.0, 0.0, 0.0]]), np.array([IDENTITY]))
    offset = np.array([0.0, 0.0, 0.01])
    tracker.update({"left": (-offset, IDENTITY)})
    # Turn the eef 90 degrees about x: the 1 cm offset rotates with it.
    quarter_x = np.array([np.cos(np.pi / 4), np.sin(np.pi / 4), 0, 0])
    pos, quat = tracker.update({"left": (np.zeros(3), quarter_x)})
    np.testing.assert_allclose(pos[0], [0.0, -0.01, 0.0], atol=1e-12)
    np.testing.assert_allclose(np.abs(quat[0] @ quarter_x), 1.0)


@pytest.fixture(scope="module")
def model():
    return build.load_model(build.build())


def _left_qpos_at_object(model, object_id: str) -> dict[str, float]:
    """Left-arm joints putting the eef at ``object_id``'s seated pose."""
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    left = list(named_poses.load_named_poses()["left"]["idle"])
    body = model.body(object_id)
    result = ik.qpos_from_site_pose(
        model,
        data,
        "left_eef",
        target_pos=model.body_pos[body.id],
        target_quat=model.body_quat[body.id],
        joint_names=left,
        max_steps=500,
        tol=1e-10,
    )
    assert result.success
    return {
        name: float(data.qpos[model.jnt_qposadr[model.joint(name).id]])
        for name in left
    }


def test_timeline_tracks_object_through_fetch_and_return(model, tmp_path):
    poses = named_poses.load_named_poses()
    idle = {**poses["left"]["idle"], **poses["right"]["idle"]}
    grasp = {**idle, **_left_qpos_at_object(model, "small_object_0")}
    names = list(idle)
    # idle -> grasp -> idle -> grasp -> idle, 1 s per leg, 100 Hz.
    waypoints = [idle, grasp, idle, grasp, idle]
    samples = []
    for a, b in zip(waypoints, waypoints[1:]):
        for s in np.linspace(0, 1, 100, endpoint=False):
            samples.append([(1 - s) * a[n] + s * b[n] for n in names])
    samples.append([idle[n] for n in names])

    bag = tmp_path / "bag.mcap"
    with open(bag, "wb") as f:
        writer = Writer(f)
        schema = writer.register_msgdef(
            "sensor_msgs/msg/JointState", JOINT_STATE_MSGDEF
        )
        for k, position in enumerate(samples):
            t_ns = 5_000_000_000 + k * 10_000_000
            writer.write_message(
                "/joint_states",
                schema,
                {
                    "header": {
                        "stamp": {
                            "sec": t_ns // 10**9,
                            "nanosec": t_ns % 10**9,
                        },
                        "frame_id": "",
                    },
                    "name": names,
                    "position": position,
                    "velocity": [],
                    "effort": [],
                },
                log_time=t_ns,
                publish_time=t_ns,
            )
        writer.finish()

    timeline = build_timeline(bag, model, rate=50)
    obj = model.body("small_object_0").mocapid[0]
    held = timeline.held["left"]
    assert held[timeline.frame_at(0.5)] == -1  # approaching
    assert held[timeline.frame_at(1.5)] == obj  # carried away
    assert held[timeline.frame_at(3.9)] == -1  # put back
    assert np.all(timeline.held["right"] == -1)

    seated = model.body_pos[model.body("small_object_0").id]
    carried = timeline.mocap_pos[timeline.frame_at(2.0), obj]
    assert np.linalg.norm(carried - seated) > 0.1
    np.testing.assert_allclose(timeline.mocap_pos[-1, obj], seated)
    assert timeline.glass_revealed is None
    assert timeline.trials == []
