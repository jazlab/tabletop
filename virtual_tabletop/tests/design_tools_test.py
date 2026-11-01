"""Tests for the design tools (arm explorer, arm report)."""

import pytest


def test_explorer_sliders_set_mirrored_angles():
    from virtual_tabletop.design.arm_explorer import Explorer, explorer_model

    model = explorer_model()
    assert model.nu == 23  # one slider per right-arm joint
    ex = Explorer(model, independent=False, task="foraging_ordered")
    elbow = model.actuator("right/elbow_flexion").id
    assert tuple(model.actuator_ctrlrange[elbow]) == pytest.approx(
        (0, 120), abs=0.01
    )
    ex.data.ctrl[elbow] = 90
    ex.apply()
    for side in ("right", "left"):
        joint = model.joint(f"monkey/{side}/elbow_flexion")
        assert ex.data.qpos[joint.qposadr[0]] == pytest.approx(1.5707963)


@pytest.mark.slow
def test_arm_report_best_reach_touches_with_arm_joints():
    from virtual_tabletop.design.arm_report import Arm, best_reach

    arm = Arm.load()
    reach = best_reach(arm, arm.joints, starts=4)
    assert reach.gap < 0  # the hull overlaps the object: a touch
