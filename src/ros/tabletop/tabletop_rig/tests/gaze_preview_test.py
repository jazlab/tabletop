"""The idle preview must never replace someone else's capture directory."""

from types import SimpleNamespace

import numpy as np
import pytest
from tabletop_rig.nodes.gaze_preview import _features, _scratch_route


def test_pinned_capture_restores_normal_route_early_and_once():
    paths = ["normal"]
    with _scratch_route(
        lambda: paths[-1], paths.append, "scratch", "normal"
    ) as release:
        assert paths[-1] == "scratch"
        release()
        assert paths[-1] == "normal"
        release()
    assert paths == ["normal", "scratch", "normal"]


def test_restore_on_error_or_interrupt():
    paths = ["normal"]
    with pytest.raises(KeyboardInterrupt):
        with _scratch_route(
            lambda: paths[-1], paths.append, "scratch", "normal"
        ):
            raise KeyboardInterrupt
    assert paths[-1] == "normal"


def test_do_not_overwrite_foreign_capture_change():
    paths = ["normal"]
    with pytest.raises(RuntimeError, match="elsewhere"):
        with _scratch_route(
            lambda: paths[-1], paths.append, "scratch", "normal"
        ):
            paths.append("another-session")
    assert paths[-1] == "another-session"


def test_recover_after_ambiguous_server_reply():
    paths = ["normal"]

    def write(path):
        paths.append(path)
        if path == "scratch":
            raise TimeoutError

    with pytest.raises(TimeoutError):
        with _scratch_route(lambda: paths[-1], write, "scratch", "normal"):
            pytest.fail("Must not subscribe before set succeeds")
    assert paths[-1] == "normal"


def test_initial_foreign_capture_dir_is_not_changed():
    paths = ["another-session"]
    with pytest.raises(RuntimeError, match="leaving it alone"):
        with _scratch_route(
            lambda: paths[-1], paths.append, "scratch", "normal"
        ):
            pytest.fail("Must not subscribe")
    assert paths == ["another-session"]


def test_features_are_binocular_and_match_model_column_order():
    sample = SimpleNamespace(
        left_x=1, left_y=2, right_x=3, right_y=4, left_pupil=5, right_pupil=6
    )
    np.testing.assert_array_equal(_features(sample), [1, 2, 3, 4, 5, 6])
    sample.right_pupil = 0
    assert _features(sample) is None
    sample.right_pupil = 6
    sample.left_x = -32768
    assert _features(sample) is None
    sample.left_x = np.nan
    assert _features(sample) is None
