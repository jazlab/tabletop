"""Can the monkey press each button without going through the object?

Informs **shoulder placement** for the RL model (``placements.rl`` in
``config/subject.yaml``) and checks **button placement**.

::

    uv run vtt-button-reach           # each button, right arm
    uv run vtt-button-reach --scan    # vs shoulder offsets

Uses :class:`~virtual_tabletop.rl.press_paths.PressPlanner`: a bounded
optimisation over one arm's 7 joints (fingers straight) bringing the
fingertip to the button while penalising overlap with the rest of the
object (e.g. the cup's walls). A button is **reachable** when the fingertip
gets within 1 cm with at most 2 mm of overlap and the hand touches the
button. ``--scan`` repeats this with both arms shifted forward (+y) and
up/down (z) in the compiled model (no rebuild).
"""

import argparse

import numpy as np

from virtual_tabletop.rl.press_paths import Posture, PressPlanner

#: Shoulder offsets scanned (m): forward, and up/down.
SCAN_FORWARD = (0.0, 0.02, 0.04, 0.06, 0.08, 0.10)
SCAN_UP = (-0.04, -0.02, 0.0, 0.02, 0.04, 0.06, 0.08, 0.10)


def _row(r: Posture) -> str:
    return (
        f"{r.geometry:13s} {r.side:5s} tip {100 * r.tip_error:5.1f} cm  "
        f"overlap {100 * r.overlap:4.1f} cm  presses "
        f"{'yes' if r.presses else 'no ':3s}"
        f"  -> {'REACHABLE' if r.reachable() else 'no'}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--scan", action="store_true", help="scan shoulder offsets"
    )
    args = parser.parse_args()
    planner = PressPlanner()
    geometries = list(planner.scene.stimuli)
    if not args.scan:
        for g in geometries:
            r = planner.solve(g)
            print(_row(r))
            print(
                "  posture (deg):",
                {
                    k.split("/")[-1]: round(float(np.degrees(v)))
                    for k, v in r.angles.items()
                },
            )
        return
    print("shoulder offset (forward, up) cm from config/subject.yaml:")
    for dy in SCAN_FORWARD:
        for dz in SCAN_UP:
            planner.shift_shoulders(dy, dz)
            rows = [planner.solve(g) for g in geometries]
            ok = all(r.reachable() for r in rows)
            details = "; ".join(
                f"{r.geometry} tip {100 * r.tip_error:.1f} ovl "
                f"{100 * r.overlap:.1f}"
                for r in rows
            )
            print(
                f"  ({100 * dy:+3.0f}, {100 * dz:+3.0f})  "
                f"{'BOTH OK' if ok else '       '}  {details}",
                flush=True,
            )


if __name__ == "__main__":
    main()
