"""Unit tests: bag decoding fix, pulse detection, matching/fitting, trials."""

import json
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from tabletop_nwb.bag import decoder_for, encoder_for, normalize_schema
from tabletop_nwb.extract import service_calls
from tabletop_nwb.sync import Pulses, match_and_fit, pulses_from_levels
from tabletop_nwb.trials import assign_by_window, build_trials

SCHEMAS = json.loads(
    (
        Path(__file__).parents[1] / "src/tabletop_nwb/testing/schemas.json"
    ).read_text()
)


class _Schema:
    def __init__(self, name, text):
        self.name, self.data = name, text.encode()


def test_service_event_schema_roundtrip():
    name, text = SCHEMAS["/teensy/set_reward/_service_event"]
    assert name.endswith("_Event") and "SRV:" in text
    new_name, new_text = normalize_schema(name, text)
    assert new_name == "tabletop_interfaces/SetRewardEvent"
    assert "SetRewardRequest[<=1] request" in new_text
    msg = {
        "info": {
            "event_type": 1,
            "stamp": {"sec": 5, "nanosec": 7},
            "client_gid": [3] * 16,
            "sequence_number": 9,
        },
        "request": [
            {"activate": True, "duration": {"sec": 0, "nanosec": 250_000_000}}
        ],
        "response": [],
    }
    data = encoder_for(name, text)(msg)
    out = decoder_for(_Schema(name, text))(data)
    assert (
        out.info.sequence_number == 9
        and out.request[0].activate
        and out.request[0].duration.nanosec == 250_000_000
    )
    assert out.response == []


def test_normalize_leaves_messages_alone():
    name, text = SCHEMAS["/tabletop/task_event"]
    assert normalize_schema(name, text) == (name, text)


def test_service_calls_dedup_client_and_server():
    rows = []
    for seq in (1, 2):
        for et, gid in [(0, "c"), (1, "s"), (2, "s"), (3, "c")]:
            rows.append(
                {
                    "log_ns": seq * 100 + et,
                    "info_event_type": et,
                    "info_ns": seq * 1000 + et,
                    "sequence_number": seq,
                    "client_gid": gid,
                    "req_activate": True if et < 2 else None,
                }
            )
    calls = service_calls(pd.DataFrame(rows))
    assert len(calls) == 2
    assert list(calls.time_ns) == [1001, 2001]  # server REQUEST_RECEIVED
    assert (calls.time_source == "request_received").all()
    client_only = service_calls(
        pd.DataFrame([r for r in rows if r["client_gid"] == "c"])
    )
    assert list(client_only.time_ns) == [1000, 2000]


def test_pulses_from_levels_width_filter():
    t = np.arange(0, 5, 0.001)
    lvl = ((t % 1.0) < 0.1) & (t > 0.5)  # 100 ms pulses
    lvl[2500] = True  # 1-sample glitch
    p = pulses_from_levels(t, lvl)
    assert len(p.onsets) == 4
    np.testing.assert_allclose(p.widths, 0.1, atol=0.002)


def _train(rng, n=200):
    return np.cumsum(1.0 + rng.uniform(0.05, 0.2, n) - 0.125) + 10


def test_match_recovers_offset_and_drift():
    rng = np.random.default_rng(0)
    ref = _train(rng)
    drift, offset = 80e-6, 54321.0
    stream = ref * (1 + drift) + offset + rng.normal(0, 0.0003, len(ref))
    keep = np.ones(len(ref), bool)
    keep[[5, 6, 50, 120]] = False  # dropouts
    s = np.sort(
        np.concatenate([stream[keep][10:180], [stream[60] + 0.4]])
    )  # partial overlap + spurious pulse
    f = match_and_fit(
        "x",
        "test",
        Pulses(ref, np.full(len(ref), 0.1)),
        Pulses(s, np.full(len(s), 0.1)),
    )
    assert f.status == "ok"
    assert f.n_matched >= 160
    assert abs(f.drift_ppm - (1 / (1 + drift) - 1) * 1e6) < 5
    pred = f.to_rel_s(stream[keep][10:180], 0)
    np.testing.assert_allclose(pred, ref[keep][10:180], atol=0.0015)


def test_match_rejects_random_input():
    rng = np.random.default_rng(1)
    ref = _train(rng)
    t = np.arange(0, 200, 0.001)
    p = pulses_from_levels(t, rng.random(len(t)) < 0.5)
    f = match_and_fit(
        "eyelink", "test", Pulses(ref, np.full(len(ref), 0.1)), p
    )
    assert f.status != "ok"


def test_match_rejects_unrelated_train():
    rng = np.random.default_rng(2)
    ref = _train(rng)
    other = _train(np.random.default_rng(99))
    f = match_and_fit(
        "x",
        "test",
        Pulses(ref, np.ones(len(ref))),
        Pulses(other, np.ones(len(other))),
    )
    assert f.status != "ok"


def _te(rows):
    return pd.DataFrame(
        rows,
        columns=[
            "header_ns",
            "log_ns",
            "event_type",
            "event_name",
            "session_id",
            "trial_number",
            "group_name",
            "object_id",
        ],
    )


def test_trials_aborted_and_out_of_order():
    ev = _te(
        [
            (0, 0, 1, "TASK_STARTED", "s", -1, "", ""),
            (10, 0, 3, "P", "s", 1, "R", "a"),
            (15, 0, 8, "GO", "s", 1, "R", "a"),
            (20, 0, 4, "D", "s", 1, "R", "a"),
            (30, 0, 3, "P", "s", 0, "L", "b"),  # aborted: no DEPRESENTING
            (40, 0, 3, "P", "s", 2, "R", "c"),
            (45, 0, 4, "D", "s", 2, "R", "c"),
            (50, 0, 2, "TASK_ENDED", "s", -1, "", ""),
        ]
    )
    tr = build_trials(ev, end_ns=60)
    assert list(tr.trial_number) == [1, 0, 2]
    assert list(tr.completed) == [True, False, True]
    assert list(tr.stop_ns) == [20, 40, 45]
    assert tr.stop_reason.iloc[1] == "next_trial"
    assert tr.go_cue_time_ns.iloc[0] == 15 and np.isnan(
        tr.go_cue_time_ns.iloc[1]
    )


def test_assign_by_window_edges():
    tr = pd.DataFrame({"start_ns": [10, 30], "stop_ns": [20, 40]})
    idx = assign_by_window(np.array([9, 10, 20, 21, 30, 40, 41]), tr)
    assert list(idx) == [-1, 0, 0, -1, 1, 1, -1]


@pytest.mark.parametrize("profile", ["rig", "sim"])
def test_synth_writes_readable_bag(tmp_path, profile):
    from tabletop_nwb.bag import iter_messages, summarize
    from tabletop_nwb.testing.synth import make_session

    d, truth = make_session(tmp_path, profile, duration_s=8.0)
    s = summarize(d / "bag")
    assert s.count("/teensy/sensor") == truth.counts["/teensy/sensor"]
    n = sum(
        1
        for _ in iter_messages(
            d / "bag", ["/teensy/set_reward/_service_event"]
        )
    )
    assert n == truth.counts.get("/teensy/set_reward/_service_event", 0)
