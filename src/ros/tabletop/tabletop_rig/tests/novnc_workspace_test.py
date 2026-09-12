from pathlib import Path

import pytest
from tabletop_rig.utils import novnc_workspace


def test_reservation_switches_and_returns(monkeypatch, tmp_path: Path):
    calls = []
    monkeypatch.setattr(
        novnc_workspace,
        "switch_dwm_workspace",
        lambda number: calls.append(number) or True,
    )

    with novnc_workspace.reserve_dwm_workspace(6, lock_path=tmp_path / "lock"):
        calls.append("mapped")

    assert calls == [6, "mapped", 1]


def test_reservation_returns_after_exception(monkeypatch, tmp_path: Path):
    calls = []
    monkeypatch.setattr(
        novnc_workspace,
        "switch_dwm_workspace",
        lambda number: calls.append(number) or True,
    )

    with pytest.raises(RuntimeError):
        with novnc_workspace.reserve_dwm_workspace(
            4, lock_path=tmp_path / "lock"
        ):
            raise RuntimeError("window creation failed")

    assert calls == [4, 1]


def test_failed_switch_does_not_change_return_workspace(
    monkeypatch, tmp_path: Path
):
    calls = []
    monkeypatch.setattr(
        novnc_workspace,
        "switch_dwm_workspace",
        lambda number: calls.append(number) or False,
    )

    with novnc_workspace.reserve_dwm_workspace(5, lock_path=tmp_path / "lock"):
        pass

    assert calls == [5]


@pytest.mark.parametrize("workspace", [0, 10])
def test_invalid_workspace_is_rejected(workspace: int, tmp_path: Path):
    with pytest.raises(ValueError):
        with novnc_workspace.reserve_dwm_workspace(
            workspace, lock_path=tmp_path / "lock"
        ):
            pass
