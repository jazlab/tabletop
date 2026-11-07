"""Helpers shared by the NWB writers."""

from __future__ import annotations

import json
import uuid
from pathlib import Path

import numpy as np
from hdmf.backends.hdf5 import H5DataIO
from hdmf.common import DynamicTable, VectorData
from pynwb import NWBHDF5IO, NWBFile
from pynwb.event import DurationVectorData, EventsTable, TimestampVectorData
from pynwb.file import Subject

from . import __version__
from .session import Inventory
from .timeline import Timeline

LARGE = 10_000


def compressed(a):
    a = np.asarray(a)
    return (
        H5DataIO(a, compression="gzip", compression_opts=4)
        if a.size > LARGE
        else a
    )


def _column(name: str, values, description: str) -> VectorData:
    v = (
        list(values)
        if (len(values) and isinstance(values[0], str))
        else np.asarray(values)
    )
    return VectorData(name=name, description=description, data=v)


def events_table(
    name: str,
    description: str,
    timestamps,
    *,
    durations=None,
    source: str | None = None,
    columns: dict[str, tuple] | None = None,
) -> EventsTable:
    """EventsTable from column arrays. ``columns`` maps name → (values, description)."""
    cols = [
        TimestampVectorData(
            name="timestamp",
            description="seconds since session start",
            data=np.asarray(timestamps, dtype=np.float64),
        )
    ]
    if durations is not None:
        cols.append(
            DurationVectorData(
                name="duration",
                description="seconds (NaN when unknown)",
                data=np.asarray(durations, dtype=np.float64),
            )
        )
    for cname, (vals, desc) in (columns or {}).items():
        cols.append(_column(cname, vals, desc))
    return EventsTable(
        name=name,
        description=description,
        source_description=source,
        columns=cols,
    )


def dynamic_table(
    name: str, description: str, columns: dict[str, tuple]
) -> DynamicTable:
    return DynamicTable(
        name=name,
        description=description,
        columns=[_column(k, v, d) for k, (v, d) in columns.items()],
    )


SIM_SUBJECT = {
    "subject_id": "simulated",
    "species": "Macaca mulatta",
    "sex": "U",
    "description": "simulated session (mock hardware); no real subject",
}


def new_nwbfile(
    inv: Inventory, tl: Timeline, kind: str, description: str
) -> NWBFile:
    md = inv.session_metadata or {}
    keywords = ["tabletop", f"profile:{inv.profile}", kind]
    nwb = NWBFile(
        session_description=f"{description} Recording profile: {inv.profile}"
        + (
            " (SIMULATED: mock hardware, not real data)."
            if inv.profile == "sim"
            else "."
        ),
        identifier=str(uuid.uuid4()),
        session_start_time=tl.start_datetime,
        timestamps_reference_time=tl.start_datetime,
        session_id=inv.session_id,
        experimenter=md.get("experimenter"),
        lab=md.get("lab"),
        institution=md.get("institution"),
        experiment_description=md.get("experiment_description"),
        keywords=keywords,
        source_script=f"tabletop_nwb {__version__}",
        source_script_file_name="tabletop_nwb",
    )
    subj = md.get("subject") or (SIM_SUBJECT if inv.profile == "sim" else None)
    if subj:
        nwb.subject = Subject(
            **{
                k: v
                for k, v in subj.items()
                if k
                in (
                    "subject_id",
                    "species",
                    "sex",
                    "age",
                    "date_of_birth",
                    "description",
                    "weight",
                    "genotype",
                    "strain",
                )
            }
        )
    return nwb


def provenance_table(
    inv: Inventory, tl: Timeline, extra: dict | None = None
) -> DynamicTable:
    items = {
        "converter_version": __version__,
        "profile": inv.profile,
        "profile_source": inv.profile_source,
        "profile_reasons": json.dumps(inv.profile_reasons),
        "session_start_ns": str(tl.start_ns),
        "sync_origin_ns": str(tl.origin_ns),
        "inventory": json.dumps(inv.to_json(), default=str),
        "stream_time_methods": json.dumps(
            {
                k: {"method": v.method, "source": v.source}
                for k, v in tl.streams.items()
            }
        ),
        "reordered_samples": json.dumps(tl.reordered),
    }
    items.update(extra or {})
    return dynamic_table(
        "provenance",
        "Conversion provenance (key/value; values are JSON or text).",
        {
            "key": (list(items), "item name"),
            "value": ([str(v) for v in items.values()], "value"),
        },
    )


def write(nwb: NWBFile, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.stem + ".partial.nwb")
    try:
        with NWBHDF5IO(str(tmp), "w") as io:
            io.write(nwb)
    except BaseException:
        tmp.unlink(missing_ok=True)
        raise
    tmp.replace(path)
