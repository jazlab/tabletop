from tabletop_rig.nodes.novnc_layout_supervisor import workspace_for_title


def test_standard_window_workspace_mapping():
    assert workspace_for_title("/some/path/rig.rviz* - RViz") is None
    assert workspace_for_title("/some/path/camera_robot.rviz* - RViz") == 1
    assert workspace_for_title("TableTop Six Camera Views") == 1
    assert workspace_for_title("/some/path/left_back_rig.rviz* - RViz") == 2
    assert (
        workspace_for_title("TableTop Camera Trigger / Exposure Monitor") == 3
    )
    assert workspace_for_title("TableTop I/O Check") == 4
    assert workspace_for_title("TableTop TTL Monitor") == 5
    assert workspace_for_title("TableTop Session Health") == 6


def test_standard_layout_uses_exactly_six_contiguous_workspaces():
    from tabletop_rig.nodes.novnc_layout_supervisor import WORKSPACE_TITLES
    from tabletop_rig.utils.novnc_workspace import WORKSPACE_COUNT

    assert WORKSPACE_COUNT == 6
    assert {workspace for _, workspace in WORKSPACE_TITLES} == set(range(1, 7))


def test_unrelated_window_is_not_managed():
    assert workspace_for_title("URSim Teach Pendant") is None
