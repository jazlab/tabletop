"""Mutation tests (PLAN §10.5): corrupt a good conversion; the suite must FAIL the named check."""

import json
import shutil

import h5py
import numpy as np
import pytest

from tabletop_nwb import convert as convert_mod
from tabletop_nwb.convert import convert
from tabletop_nwb.testing.synth import make_session
from tabletop_nwb.validation import validate


def _failed_ids(nwb_dir):
    results, _ = validate(nwb_dir)
    return {r.id for r in results if r.status == "FAIL"}, results


def _assert_fails(nwb_dir, cid):
    ids, results = _failed_ids(nwb_dir)
    assert cid in ids, f"expected {cid} to FAIL; failures were {sorted(ids)}"


def _h5(path):
    return h5py.File(path, "r+")


def test_baseline_has_no_failures(rig_session, fresh_copy):
    ids, _ = _failed_ids(fresh_copy(rig_session[0]) / "nwb")
    assert not ids


def test_missing_video_file(rig_session, fresh_copy):
    d = fresh_copy(rig_session[0])
    (d / "nwb" / "video.nwb").unlink()
    _assert_fails(d / "nwb", "A1")


def test_missing_video_segment(rig_session, fresh_copy):
    d = fresh_copy(rig_session[0])
    next((d / "unbag").glob("*/segment_000001.mkv")).unlink()
    _assert_fails(d / "nwb", "A4")


def test_dropped_camera(rig_session, fresh_copy):
    d = fresh_copy(rig_session[0])
    with _h5(d / "nwb" / "video.nwb") as f:
        del f["acquisition/left_front_top_cam"]
    _assert_fails(d / "nwb", "C3")


def test_eyelink_shifted_20ms(rig_session, fresh_copy):
    d = fresh_copy(rig_session[0])
    with _h5(d / "nwb" / "task.nwb") as f:
        f["acquisition/EyeTracking/left_eye/timestamps"][:] += 0.020
    _assert_fails(d / "nwb", "E4")


def test_eyelink_drift_not_refitted(rig_session, fresh_copy):
    d = fresh_copy(rig_session[0])
    with _h5(d / "nwb" / "task.nwb") as f:
        ts = f["acquisition/EyeTracking/left_eye/timestamps"]
        ts[:] = ts[:] * (1 + 300e-6)
    # 300 ppm over 40 s is up to 12 ms: closure must catch it
    _assert_fails(d / "nwb", "E4")


def test_video_shifted_one_frame(rig_session, fresh_copy):
    d = fresh_copy(rig_session[0])
    with _h5(d / "nwb" / "video.nwb") as f:
        f[
            "processing/camera_metadata/left_back_top_cam_exposure_time/timestamps"
        ][:] += 1 / 120 * 2
    ids, _ = _failed_ids(d / "nwb")
    assert {"E4", "E5"} & ids


def test_trials_swapped(rig_session, fresh_copy):
    d = fresh_copy(rig_session[0])
    with _h5(d / "nwb" / "task.nwb") as f:
        s = f["intervals/trials/start_time"]
        v = s[:]
        v[[0, 1]] = v[[1, 0]]
        s[:] = v
    _assert_fails(d / "nwb", "F2")


def test_session_start_mismatch(rig_session, fresh_copy):
    d = fresh_copy(rig_session[0])
    with _h5(d / "nwb" / "video.nwb") as f:
        del f["session_start_time"]
        f.create_dataset(
            "session_start_time", data="2020-01-01T00:00:00+00:00"
        )
    _assert_fails(d / "nwb", "B1")


def test_reward_rows_lost_in_conversion(tmp_path, monkeypatch):
    d, _ = make_session(tmp_path, "rig", duration_s=20.0)
    real = convert_mod.write_task.__globals__["service_calls"]
    monkeypatch.setitem(
        convert_mod.write_task.__globals__,
        "service_calls",
        lambda df: real(df).iloc[::2],
    )
    convert(d, log=lambda *a: None)
    _assert_fails(d / "nwb", "C1")


def test_task_event_dropped_in_conversion(tmp_path, monkeypatch):
    d, _ = make_session(tmp_path, "rig", duration_s=20.0)
    real = convert_mod.extract

    def lossy(*a, **k):
        ex = real(*a, **k)
        ex.tables["task_event"] = ex.tables["task_event"].iloc[1:]
        return ex

    monkeypatch.setattr(convert_mod, "extract", lossy)
    convert(d, log=lambda *a: None)
    ids, _ = _failed_ids(d / "nwb")
    assert {"C1", "F1"} <= ids


def test_sync_pulses_missing(tmp_path, monkeypatch):
    d, _ = make_session(tmp_path, "rig", duration_s=20.0)
    import tabletop_nwb.sync as sync_mod

    monkeypatch.setattr(
        sync_mod,
        "teensy_reference",
        lambda *a, **k: sync_mod.Pulses(np.array([]), np.array([])),
    )
    convert(d, log=lambda *a: None)
    _assert_fails(d / "nwb", "E1")


def test_rig_eyelink_without_pulses(tmp_path):
    """Simulator-style random EyeLink input on a rig session: must not fit, and must FAIL."""
    d, _ = make_session(
        tmp_path, "rig", duration_s=20.0, eyelink_random_input=True
    )
    convert(d, profile="rig", log=lambda *a: None)
    rep = json.loads((d / "nwb" / "sync_report.json").read_text())
    assert rep["fits"]["eyelink"]["status"] != "ok"
    _assert_fails(d / "nwb", "E1")


def test_spikeglx_present_but_not_converted(tmp_path):
    """Absent ≠ lost: SpikeGLX data in the session must not be silently skipped."""
    d, _ = make_session(tmp_path, "rig", duration_s=10.0)
    (d / "ephys").mkdir()
    (d / "ephys" / "run_g0_t0.imec0.ap.meta").write_text("imSampRate=30000\n")
    convert(d, log=lambda *a: None)
    assert (
        json.loads((d / "nwb" / "manifest.json").read_text())["profile"]
        == "rig+ephys"
    )
    _assert_fails(d / "nwb", "A1")


def test_sim_labelled(sim_session, fresh_copy):
    d = fresh_copy(sim_session[0])
    with _h5(d / "nwb" / "task.nwb") as f:
        del f["general/keywords"]
    _assert_fails(d / "nwb", "A5")


@pytest.fixture(autouse=True)
def _no_leftover_validation_dirs():
    yield
    shutil.rmtree("validation", ignore_errors=True)
