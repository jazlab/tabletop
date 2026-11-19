"""No participant samples or hardware needed."""

import numpy as np
import pytest

from tabletop_py.gaze.pupil_corrected import (
    PupilCorrectedDecoder,
    fit_aligned,
    moving_indices,
)


def synthetic_data():
    rng = np.random.default_rng(8)
    g = np.arange(2200) / 1000.0
    y = rng.normal([780.0, 220.0, 430.0], [45.0, 53.0, 80.0], (len(g), 3))
    eyes = y @ rng.normal(size=(3, 4)) + rng.normal(size=(len(g), 4))
    pupils = rng.normal(50, 5, (len(g), 2))
    x = np.column_stack([eyes + pupils @ rng.normal(size=(2, 4)), pupils])
    ids = np.where(g > 1.65, 1, 0)
    valid = np.ones(len(g), dtype=bool)
    valid[350:380] = False
    bad = np.zeros(len(g), dtype=bool)
    bad[420:472] = True
    return dict(g=g, y=y, x=x, ids=ids, valid=valid, bad=bad, duration=1.5)


def test_fit_export_and_no_leakage(tmp_path):
    import torch

    d = synthetic_data()
    count = torch.get_num_threads()
    torch.set_num_threads(1)
    try:
        model = fit_aligned(d, "synthetic")
        moving = moving_indices(d)
        np.testing.assert_array_equal(
            moving,
            np.flatnonzero(
                d["valid"] & ~d["bad"] & (d["g"] >= 0.25) & (d["g"] < 1.475)
            )[::2],
        )
        # Even absurd changes to ALL excluded labels must leave the model unchanged.
        excluded = np.ones(len(d["x"]), dtype=bool)
        excluded[moving] = False
        d["y"][excluded] = np.nan
        second = fit_aligned(d, "synthetic")
    finally:
        torch.set_num_threads(count)
    path = tmp_path / "model.npz"
    model.save(path)
    with pytest.raises(FileExistsError):
        model.save(path)
    loaded = PupilCorrectedDecoder.load(path)
    assert loaded.session == "synthetic"
    eye = d["x"][-6:]
    result = loaded.predict_m(eye)
    assert result.shape == (6, 3)
    np.testing.assert_array_equal(result, model.predict_m(eye))
    np.testing.assert_array_equal(result, second.predict_m(eye))
    np.testing.assert_allclose(result[0], loaded.predict_m(eye[0]), atol=1e-6)
    assert result.mean(0)[0] == pytest.approx(0.78, abs=0.15)
    with pytest.raises(ValueError, match="Invalid binocular"):
        loaded.predict_m([1, 2, 3, 4, 5, 0])
    with pytest.raises(ValueError, match="six-channel"):
        loaded.predict_m([1, 2, 3, 4])


def test_corrupt_mask_or_leaky_time_axis_is_rejected():
    d = synthetic_data()
    d["ids"][350] = 1
    with pytest.raises(ValueError, match="overlap"):
        moving_indices(d)
    d["g"][35] = d["g"][34]
    with pytest.raises(ValueError, match="time axis"):
        moving_indices(d)


def test_incompatible_model_format_rejected(tmp_path):
    path = tmp_path / "wrong.npz"
    np.savez(path, session="test", format_version=88)
    with pytest.raises(ValueError, match="format"):
        PupilCorrectedDecoder.load(path)
