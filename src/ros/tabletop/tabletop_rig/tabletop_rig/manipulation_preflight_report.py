"""Formatting helpers for saved manipulation preflight reports."""

from typing import Any


def format_report_summary(report: dict[str, Any]) -> str:
    """Format a compact physical-grid summary for terminal output."""
    objects = report.get("objects", {})
    cells: dict[tuple[int, int], tuple[str, str]] = {}
    for object_id, result in objects.items():
        grid_idx = result.get("grid_idx")
        if not isinstance(grid_idx, (list, tuple)) or len(grid_idx) != 2:
            continue
        cells[(int(grid_idx[0]), int(grid_idx[1]))] = (
            str(result.get("status", "UNKNOWN")),
            object_id,
        )

    lines = ["MANIPULATION PREFLIGHT RESULTS"]
    if not cells:
        lines.append("No object results were found in the report.")
        return "\n".join(lines)

    rows = sorted({idx[0] for idx in cells})
    columns = sorted({idx[1] for idx in cells})
    cell_width = max(
        12,
        max(
            len(f"{status} {object_id}")
            for status, object_id in cells.values()
        ),
    )
    header = "row | " + " | ".join(
        f"column {column}".ljust(cell_width) for column in columns
    )
    lines.extend((header, "-" * len(header)))
    for row in rows:
        row_cells = []
        for column in columns:
            value = cells.get((row, column))
            label = "--" if value is None else f"{value[0]} {value[1]}"
            row_cells.append(label.ljust(cell_width))
        lines.append(f"{row:>3} | " + " | ".join(row_cells))

    pass_count = sum(
        result.get("status") == "PASS" for result in objects.values()
    )
    unavailable = [
        (object_id, result)
        for object_id, result in objects.items()
        if result.get("status") != "PASS"
    ]
    lines.append("")
    lines.append(f"TOTAL: {pass_count} PASS, {len(unavailable)} UNAVAILABLE")
    if unavailable:
        lines.append("UNAVAILABLE DETAILS:")
        for object_id, result in sorted(
            unavailable, key=lambda item: tuple(item[1].get("grid_idx", []))
        ):
            lines.append(
                f"  grid_idx={result.get('grid_idx')} {object_id}: "
                f"{result.get('reason', 'no reason recorded')}"
            )
    return "\n".join(lines)
