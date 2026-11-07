import shutil
from pathlib import Path

import pytest

from tabletop_nwb.convert import convert
from tabletop_nwb.testing.synth import make_session

REPO_BAGS = Path(__file__).resolve().parents[2] / "bags"


def _quiet(*a, **k):
    pass


@pytest.fixture(scope="session")
def rig_session(tmp_path_factory):
    """A synthetic rig session, converted once (do not mutate: copy with `fresh_copy`)."""
    d, truth = make_session(
        tmp_path_factory.mktemp("rig"), "rig", duration_s=40.0, seed=1
    )
    convert(d, log=_quiet)
    return d, truth


@pytest.fixture(scope="session")
def sim_session(tmp_path_factory):
    d, truth = make_session(
        tmp_path_factory.mktemp("sim"), "sim", duration_s=30.0, seed=2
    )
    convert(d, log=_quiet)
    return d, truth


@pytest.fixture
def fresh_copy(tmp_path):
    def _copy(session_dir: Path) -> Path:
        dst = tmp_path / session_dir.name
        shutil.copytree(session_dir, dst)
        # inventory.json records absolute paths; point it at the copy
        inv = dst / "nwb" / "inventory.json"
        inv.write_text(inv.read_text().replace(str(session_dir), str(dst)))
        return dst

    return _copy
