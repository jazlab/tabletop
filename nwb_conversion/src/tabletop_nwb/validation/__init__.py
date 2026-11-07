"""Post-conversion test suite: ``tt-nwb validate <session>/nwb``."""

from __future__ import annotations

import json
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

from .checks import (
    CATEGORIES,
    FAIL,
    INFO,
    PASS,
    SKIP,
    WARN,
    Result,
    run_checks,
)

ORDER = {FAIL: 0, WARN: 1, PASS: 2, INFO: 3, SKIP: 4}


def _jsonable(v):
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, dict):
        return {str(k): _jsonable(x) for k, x in v.items()}
    if hasattr(v, "item"):
        return v.item()
    return v


def validate(
    nwb_dir: Path, out_dir: Path | None = None
) -> tuple[list[Result], dict]:
    nwb_dir = Path(nwb_dir)
    results = run_checks(nwb_dir)
    counts = Counter(r.status for r in results)
    manifest = json.loads((nwb_dir / "manifest.json").read_text())
    summary = {
        "session_id": manifest.get("session_id"),
        "profile": manifest.get("profile"),
        "generated": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "counts": {
            k: counts.get(k, 0) for k in (PASS, WARN, FAIL, INFO, SKIP)
        },
        "ok": counts.get(FAIL, 0) == 0,
    }
    out_dir = Path(out_dir) if out_dir else nwb_dir / "validation"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "report.json").write_text(
        json.dumps(
            {
                **summary,
                "results": [
                    {
                        "id": r.id,
                        "category": CATEGORIES[r.category],
                        "status": r.status,
                        "message": r.message,
                        "measured": _jsonable(r.measured),
                        "threshold": _jsonable(r.threshold),
                    }
                    for r in results
                ],
            },
            indent=1,
            default=str,
        )
    )
    (out_dir / "report.md").write_text(markdown(results, summary))
    return results, summary


def markdown(results: list[Result], summary: dict) -> str:
    c = summary["counts"]
    lines = [
        f"# NWB validation: {summary['session_id']}",
        "",
        f"Profile **{summary['profile']}** · {'✅ no failures' if summary['ok'] else '❌ FAILURES'} · "
        f"PASS {c['PASS']} · WARN {c['WARN']} · FAIL {c['FAIL']} · INFO {c['INFO']} · SKIP {c['SKIP']}",
        "",
    ]
    for status in (FAIL, WARN):
        rs = [r for r in results if r.status == status]
        if rs:
            lines += (
                [f"## {status} ({len(rs)})", ""]
                + [f"- **{r.id}** {r.message}" for r in rs]
                + [""]
            )
    lines += ["## All checks", ""]
    for cat, title in CATEGORIES.items():
        rs = sorted(
            (r for r in results if r.category == cat),
            key=lambda r: (r.id, ORDER[r.status]),
        )
        if not rs:
            continue
        lines += [
            f"### {cat}. {title}",
            "",
            "| id | status | check |",
            "| --- | --- | --- |",
        ]
        lines += [
            f"| {r.id} | {r.status} | {r.message.replace('|', '/')} |"
            for r in rs
        ] + [""]
    return "\n".join(lines)
