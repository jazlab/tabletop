"""End-to-end: synthetic sessions with ground truth → convert → validate."""

import json

import numpy as np
import pytest
from pynwb import NWBHDF5IO

from tabletop_nwb.convert import convert
from tabletop_nwb.session import ProfileError
from tabletop_nwb.testing.synth import T0_NS, make_session
from tabletop_nwb.validation import validate


def _fails(results):
    return [f"{r.id} {r.message}" for r in results if r.status == "FAIL"]


def test_rig_session_validates(rig_session):
    d, _ = rig_session
    results, summary = validate(d / "nwb")
    assert summary["ok"], _fails(results)
    assert summary["profile"] == "rig"
    assert (d / "nwb" / "validation" / "report.md").is_file()


def test_sim_session_validates(sim_session):
    d, _ = sim_session
    results, summary = validate(d / "nwb")
    assert summary["ok"], _fails(results)
    assert summary["profile"] == "sim"
    assert not (d / "nwb" / "video.nwb").exists()
    m = json.loads((d / "nwb" / "manifest.json").read_text())
    assert (
        m["files"]["video.nwb"]["status"] == "absent"
        and m["files"]["ephys.nwb"]["status"] == "absent"
    )


def test_sim_eyelink_not_pulse_fitted(sim_session):
    d, _ = sim_session
    rep = json.loads((d / "nwb" / "sync_report.json").read_text())
    assert rep["fits"]["eyelink"]["status"] != "ok"
    assert rep["fits"]["teensy"]["n_stream_pulses"] > 0


def test_ground_truth_alignment(rig_session):
    """Written timestamps must equal true host time (EyeLink: within the 1 ms tracker quantization)."""
    d, truth = rig_session
    with NWBHDF5IO(str(d / "nwb" / "task.nwb"), "r") as io:
        n = io.read()
        start_ns = int(
            json.loads((d / "nwb" / "manifest.json").read_text())[
                "session_start_ns"
            ]
        )
        t = n.acquisition["eyelink_input"].timestamps[:]
        true = (
            T0_NS + np.arange(len(t)) * 1_000_000 - start_ns
        ) / 1e9  # synth samples every 1 ms of host time
        err_ms = np.abs(t - true) * 1e3
        assert np.median(err_ms) < 1.0 and err_ms.max() < 2.5, (
            np.median(err_ms),
            err_ms.max(),
        )
        assert len(n.trials) == truth.n_trials
        assert (
            int((~n.trials.completed[:].astype(bool)).sum()) == truth.n_aborted
        )
        pulses = n.processing["sync"]["teensy_sync_pulses"]["timestamp"][:]
        true_p = (np.array(truth.pulse_onsets_ns) - start_ns) / 1e9
        assert len(pulses) == truth.n_pulses
        assert (
            np.abs(pulses - true_p).max() < 0.002
        )  # teensy jitter 0.2 ms rms
    with NWBHDF5IO(str(d / "nwb" / "video.nwb"), "r") as io:
        v = io.read()
        cam = v.acquisition["left_back_top_cam"]
        frames = cam.timestamps[:]
        host_f = (
            T0_NS
            + int(0.5e9)
            + np.arange(len(frames)) * int(1e9 / 120)
            - start_ns
        ) / 1e9
        assert (
            np.median(np.abs(frames - host_f)) * 1e3 < 4.2
        )  # within half a frame after removing the injected offset
        assert (
            len(v.acquisition["right_bottom_cam"].timestamps)
            == len(frames) - 3
        )  # 3 dropped frames
        assert len(v.intervals["dropped_frames"]) == 1


def test_session_start_is_shared(rig_session):
    d, _ = rig_session
    starts = set()
    for f in ("task.nwb", "video.nwb"):
        with NWBHDF5IO(str(d / "nwb" / f), "r") as io:
            starts.add(io.read().session_start_time)
    assert len(starts) == 1


def test_profile_mismatch_rig_as_sim(tmp_path):
    d, _ = make_session(tmp_path, "rig", duration_s=8.0)
    with pytest.raises(ProfileError, match="rig-only sources"):
        convert(d, profile="sim", log=lambda *a: None)


def test_profile_mismatch_sim_as_rig(tmp_path):
    d, _ = make_session(tmp_path, "sim", duration_s=8.0)
    with pytest.raises(ProfileError, match="looks simulated"):
        convert(d, profile="rig", log=lambda *a: None)


def test_profile_rig_ephys_without_spikeglx(tmp_path):
    d, _ = make_session(tmp_path, "rig", duration_s=8.0)
    with pytest.raises(ProfileError, match="no SpikeGLX"):
        convert(d, profile="rig+ephys", log=lambda *a: None)
