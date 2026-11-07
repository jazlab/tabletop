"""Regression tests for bugs found running on real bags."""

import gc

from tabletop_nwb.bag import iter_messages, summarize
from tabletop_nwb.testing.synth import make_session


def test_decoder_cache_survives_many_bags(tmp_path):
    """Decoding several bags in one process must not reuse a freed schema's decoder (id() reuse)."""
    dirs = [
        make_session(tmp_path / str(i), p, duration_s=6.0, seed=i)[0]
        for i, p in enumerate(["rig", "sim", "rig"])
    ]
    for _ in range(3):
        for d in dirs:
            topics = list(summarize(d / "bag").topics)
            n = sum(1 for _ in iter_messages(d / "bag", topics))
            assert n > 0
            gc.collect()
