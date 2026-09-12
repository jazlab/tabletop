from tabletop_rig.nodes.novnc_layout_supervisor import workspace_for_title


def test_standard_window_workspace_mapping():
    assert workspace_for_title("/some/path/rig.rviz* - RViz") == 1
    assert workspace_for_title("/some/path/camera_robot.rviz* - RViz") == 2
    assert workspace_for_title("/some/path/left_back_rig.rviz* - RViz") == 3
    assert (
        workspace_for_title("TableTop Camera Trigger / Exposure Monitor") == 4
    )
    assert workspace_for_title("TableTop I/O Check") == 5
    assert workspace_for_title("TableTop TTL Monitor") == 6
    assert workspace_for_title("TableTop Session Health") == 7


def test_unrelated_window_is_not_managed():
    assert workspace_for_title("URSim Teach Pendant") is None
