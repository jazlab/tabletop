from collections import deque

import pandas as pd
import pytest
from tabletop_rig.nodes import eyelink

from tabletop_py.gaze.preprocess import eyelink_array_to_samples


class FakeSample:
    def __init__(self, timestamp_ms: int):
        self.timestamp_ms = timestamp_ms

    def getTime(self) -> int:
        return self.timestamp_ms


class FakeTracker:
    def __init__(self, items):
        self.items = deque(items)
        self.current = None
        self.reset_calls = 0

    def getNextData(self) -> int:
        if not self.items:
            self.current = None
            return 0
        data_type, self.current = self.items.popleft()
        return data_type

    def getFloatData(self):
        return self.current

    def resetData(self) -> None:
        self.reset_calls += 1


def test_drain_eyelink_buffer_preserves_large_backlog(monkeypatch):
    monkeypatch.setattr(eyelink, "Sample", FakeSample)
    samples = [FakeSample(i) for i in range(100_000)]
    tracker = FakeTracker(
        [(eyelink.SAMPLE_TYPE, sample) for sample in samples]
    )

    drained, non_samples, invalid_samples = eyelink.drain_eyelink_buffer(
        tracker
    )

    assert drained == samples
    assert non_samples == 0
    assert invalid_samples == 0
    assert tracker.reset_calls == 0


def test_drain_eyelink_buffer_skips_events_and_malformed_samples(monkeypatch):
    monkeypatch.setattr(eyelink, "Sample", FakeSample)
    samples = [FakeSample(1), FakeSample(2)]
    tracker = FakeTracker(
        [
            (eyelink.SAMPLE_TYPE, samples[0]),
            (24, object()),
            (eyelink.SAMPLE_TYPE, None),
            (eyelink.SAMPLE_TYPE, samples[1]),
        ]
    )

    drained, non_samples, invalid_samples = eyelink.drain_eyelink_buffer(
        tracker
    )

    assert drained == samples
    assert non_samples == 1
    assert invalid_samples == 1


def test_retrieval_stats_report_tracker_timestamp_gaps():
    stats = eyelink.EyelinkRetrievalStats()

    missing = [
        stats.observe_sample(timestamp_ms, expected_period_ms=1)
        for timestamp_ms in [100, 101, 104, 105, 107]
    ]

    assert missing == [0, 0, 2, 0, 1]
    assert stats.samples == 5
    assert stats.tracker_gap_events == 2
    assert stats.estimated_missing_samples == 3
    assert stats.max_missing_samples == 2


def test_batcher_flushes_real_only_partial_batch():
    batcher = eyelink.EyelinkSampleBatcher(batch_size=3)
    samples = [object() for _ in range(5)]

    assert batcher.add(samples[0]) is None
    assert batcher.add(samples[1]) is None
    assert batcher.add(samples[2]) == samples[:3]
    assert batcher.add(samples[3]) is None
    assert batcher.add(samples[4]) is None
    assert batcher.flush() == samples[3:]
    assert batcher.flush() == []


def test_array_preprocessing_handles_variable_and_old_fixed_batches():
    frame = pd.DataFrame(
        {
            "samples[0].header.stamp.sec": [1, 3],
            "samples[0].header.stamp.nanosec": [0, 0],
            "samples[0].eyelink_time_ms": [1000, 3000],
            "samples[1].header.stamp.sec": [2, float("nan")],
            "samples[1].header.stamp.nanosec": [0, float("nan")],
            "samples[1].eyelink_time_ms": [2000, float("nan")],
        }
    )

    samples = eyelink_array_to_samples(frame)

    assert samples["header.stamp.sec"].tolist() == [1, 2, 3]
    assert samples["eyelink_time_ms"].tolist() == [1000, 2000, 3000]


def test_array_preprocessing_rejects_empty_input():
    with pytest.raises(ValueError, match="No Eyelink samples"):
        eyelink_array_to_samples(pd.DataFrame())
