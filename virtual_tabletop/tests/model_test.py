"""Checks that the generated MuJoCo model matches the rig's sources."""

import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from virtual_tabletop.assets import (
    named_poses,
    rig_config,
    rig_model,
    rl_model,
    robots,
    subject,
)
from virtual_tabletop.assets.sim_config import load_subject_config


@pytest.fixture(scope="module")
def model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(rig_model.build_rig_model()))


@pytest.fixture(scope="module")
def urdf_joints() -> dict[str, ET.Element]:
    root = ET.parse(robots.expand_urdf()).getroot()
    return {j.find("child").get("link"): j for j in root.findall("joint")}


def _origin(element: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = [float(v) for v in element.get("xyz").split()]
        rpy = [float(v) for v in element.get("rpy").split()]
        transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    return transform


def _urdf_fk(joints, link: str, q: dict[str, float]) -> np.ndarray:
    """Forward kinematics by chaining URDF joint transforms directly."""
    transform = np.eye(4)
    while link in joints:
        joint = joints[link]
        local = _origin(joint.find("origin"))
        if joint.get("type") == "revolute":
            axis = [float(v) for v in joint.find("axis").get("xyz").split()]
            rot = np.eye(4)
            rot[:3, :3] = Rotation.from_rotvec(
                np.asarray(axis) * q[joint.get("name")]
            ).as_matrix()
            local = local @ rot
        transform = local @ transform
        link = joint.find("parent").get("link")
    return transform


def test_fk_matches_urdf(model, urdf_joints):
    data = mujoco.MjData(model)
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = {}
        for i in range(model.njnt):
            q[model.joint(i).name] = rng.uniform(-np.pi, np.pi)
            data.qpos[model.jnt_qposadr[i]] = q[model.joint(i).name]
        mujoco.mj_kinematics(model, data)
        for side in robots.ARM_SIDES:
            for frame in ("tool0", "eef"):
                site = model.site(f"{side}_{frame}")
                expected = _urdf_fk(urdf_joints, f"{side}_{frame}", q)
                np.testing.assert_allclose(
                    data.site_xpos[site.id], expected[:3, 3], atol=1e-5
                )
                np.testing.assert_allclose(
                    data.site_xmat[site.id].reshape(3, 3),
                    expected[:3, :3],
                    atol=1e-5,
                )


def test_calibration_applied(urdf_joints):
    """Joint origins come from the per-arm calibration, not UR defaults."""
    left = urdf_joints["left_upper_arm_link"].find("origin")
    right = urdf_joints["right_upper_arm_link"].find("origin")
    assert left.get("xyz") != right.get("xyz")


def test_robot_bases_match_launch_file(model):
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    for mount in robots.read_arm_mounts():
        site = model.site(f"{mount.side}_base")
        expected = [float(v) for v in mount.xyz.split()]
        # UR's base frame sits 180 degrees about z from base_link, at the
        # same origin.
        np.testing.assert_allclose(data.site_xpos[site.id], expected)


def test_grid_objects_placed_from_config(model):
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    scene = rig_config.load_rig_scene()
    assert model.nmocap == len(scene.grid_objects)
    for obj in scene.grid_objects:
        body = model.body(obj.object_id)
        np.testing.assert_allclose(data.xpos[body.id], obj.pose.position)
        np.testing.assert_allclose(
            np.abs(data.xquat[body.id] @ np.asarray(obj.pose.quat)), 1.0
        )


def test_rpy_convention_is_static_xyz():
    pose = rig_config.Pose.from_config({"rpy": [0.1, 0.2, 0.3]})
    w, x, y, z = pose.quat
    expected = (
        Rotation.from_rotvec([0, 0, 0.3])
        * Rotation.from_rotvec([0, 0.2, 0])
        * Rotation.from_rotvec([0.1, 0, 0])
    )
    np.testing.assert_allclose(
        Rotation.from_quat([x, y, z, w]).as_matrix(), expected.as_matrix()
    )


def test_home_key_holds_under_physics(model):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    robot = [
        model.jnt_qposadr[i]
        for i in range(model.njnt)
        if model.joint(i).name.startswith(robots.ARM_SIDES)
    ]
    start = data.qpos[robot].copy()
    for _ in range(500):
        mujoco.mj_step(model, data)
    np.testing.assert_allclose(data.qpos[robot], start, atol=1e-3)
    assert data.ncon == 0


def test_home_key_is_srdf_idle(model):
    poses = named_poses.load_named_poses()
    key = model.key("home")
    for side in robots.ARM_SIDES:
        for joint, value in poses[side]["idle"].items():
            adr = model.jnt_qposadr[model.joint(joint).id]
            # MJCF stores 6 significant figures.
            assert key.qpos[adr] == pytest.approx(value, abs=1e-5)


def test_present_pose_matches_commander_comment(model):
    """SRDF 'present' puts both eefs at commander.yaml's presentation point."""
    data = mujoco.MjData(model)
    poses = named_poses.load_named_poses()
    for side in robots.ARM_SIDES:
        for joint, value in poses[side]["present"].items():
            data.qpos[model.jnt_qposadr[model.joint(joint).id]] = value
    mujoco.mj_kinematics(model, data)
    for side in robots.ARM_SIDES:
        np.testing.assert_allclose(
            data.site_xpos[model.site(f"{side}_eef").id],
            [0.9525, 0.4, 0.6],
            atol=0.005,
        )


def test_subject_and_smartglass_present(model):
    assert model.geom("smartglass").id >= 0
    for side in ("right", "left"):
        assert model.body(f"{subject.prefix(side)}humerus").id >= 0
        joints = subject.joint_names(model, side)
        assert len(joints) == 7 + 16
        assert sum(subject.is_finger_joint(n) for n in joints) == 16
        assert model.site(subject.hand_site(side)).id >= 0


def test_each_model_uses_its_own_shoulder_placement(model):
    placements = load_subject_config()["placements"]
    models = {"rig": model, "rl": rl_model.load_rl_model()}
    for name, m in models.items():
        data = mujoco.MjData(m)
        mujoco.mj_kinematics(m, data)
        for side, pose in placements[name].items():
            frame = data.xpos[m.body(subject.prefix(side) + "sternum").id]
            np.testing.assert_allclose(
                frame, pose["shoulder_position"], atol=1e-6
            )
    rig_y = placements["rig"]["right"]["shoulder_position"][1]
    rl_y = placements["rl"]["right"]["shoulder_position"][1]
    assert rig_y < rl_y  # the RL arms sit further forward


def test_rl_model_has_monkey_servos_and_no_robots_grid_or_glass():
    m = rl_model.load_rl_model()
    names = {m.body(b).name for b in range(m.nbody)}
    assert not any(n.startswith(("left_", "right_")) for n in names)
    assert m.nmocap == len(rl_model.RL_STIMULI)
    geoms = {m.geom(g).name for g in range(m.ngeom)}
    assert "smartglass" not in geoms
    joints = [n for s in ("right", "left") for n in subject.joint_names(m, s)]
    servoed = {m.joint(m.actuator_trnid[a, 0]).name for a in range(m.nu)}
    assert servoed == set(joints)  # one position servo per monkey joint


def test_left_arm_mirrors_right_arm(model):
    """Same joint angles on both arms give mirror-image postures."""
    data = mujoco.MjData(model)
    sides = {
        s: [
            model.jnt_qposadr[model.joint(n).id]
            for n in subject.joint_names(model, s)
        ]
        for s in ("right", "left")
    }
    midline = 0.9525
    rng = np.random.default_rng(0)
    right_range = model.jnt_range[
        [model.joint(n).id for n in subject.joint_names(model, "right")]
    ]
    bodies = [
        model.body(i).name.split("/")[-1]
        for i in range(model.nbody)
        if model.body(i).name.startswith(subject.prefix("right"))
    ]
    for _ in range(20):
        q = rng.uniform(right_range[:, 0], right_range[:, 1])
        data.qpos[sides["right"]] = q
        data.qpos[sides["left"]] = q
        mujoco.mj_kinematics(model, data)
        for name in bodies:
            r = data.xpos[model.body(subject.prefix("right") + name).id]
            left = data.xpos[model.body(subject.prefix("left") + name).id]
            np.testing.assert_allclose(
                left, [2 * midline - r[0], r[1], r[2]], atol=1e-5
            )


def test_monkey_bones_collide_with_each_other(model):
    """Bones touch each other (e.g. thumb into palm), but not at rest."""
    data = mujoco.MjData(model)

    def bone_pairs():
        mujoco.mj_forward(model, data)
        pairs = set()
        for contact in data.contact[: data.ncon]:
            names = tuple(
                sorted(
                    model.body(model.geom_bodyid[g]).name
                    for g in (contact.geom1, contact.geom2)
                )
            )
            if all(n.startswith("monkey/") for n in names):
                pairs.add(names)
        return pairs

    assert bone_pairs() == set()  # rest: arms hanging, fingers straight
    for joint, degrees in (
        ("thumb_d0_abduction", 85),
        ("thumb_d1_flexion", 50),
    ):
        data.qpos[model.joint(f"monkey/right/{joint}").qposadr[0]] = (
            np.radians(degrees)
        )
    assert ("monkey/right/palm", "monkey/right/thumb_d1") in bone_pairs()
