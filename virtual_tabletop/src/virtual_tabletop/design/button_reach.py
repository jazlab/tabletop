"""Can the monkey press each button without going through the object?

Informs **shoulder placement** for the RL model (``placements.rl`` in
``config/subject.yaml``) and checks **button placement**.

::

    uv run vtt-button-reach                      # current placement
    uv run vtt-button-reach --offset 0.04 0.02   # shoulders moved (m)
    uv run vtt-button-reach --scan               # a grid of offsets

Uses :class:`~virtual_tabletop.rl.press_paths.PressPlanner`: a bounded
optimisation over one arm's 7 joints (fingers straight) bringing the
fingertip to the button while penalising overlap with the rest of the
object (e.g. the cup's walls). A button is **reachable** when the fingertip
gets within 1 cm with at most 2 mm of overlap and the hand touches the
button. Every presentation pose of the task and both arms are checked.
``--offset`` and ``--scan`` shift both arms forward (+y) and up/down (z)
in the compiled model (no rebuild).

The optimisation is multi-start (``--starts``); borderline cases can fail
by chance at low effort, so the scan (24 starts) is a guide and candidates
should be confirmed with ``--offset`` (default 96 starts, two seeds).
"""

import argparse

from virtual_tabletop.rl.press_paths import Posture, PressPlanner
from virtual_tabletop.rl.scene import presentation_poses

#: Shoulder offsets scanned (m): forward, and up/down.
SCAN_FORWARD = (0.0, 0.02, 0.04, 0.06, 0.08, 0.10)
SCAN_UP = (-0.04, -0.02, 0.0, 0.02, 0.04, 0.06, 0.08, 0.10)


def _row(r: Posture, pose: int) -> str:
    return (
        f"pose {pose}  {r.geometry:13s} {r.side:5s} tip {100 * r.tip_error:5.1f} cm  "
        f"overlap {100 * r.overlap:4.1f} cm  presses "
        f"{'yes' if r.presses else 'no ':3s}"
        f"  -> {'REACHABLE' if r.reachable() else 'no'}"
    )


def _best(planner, geometry, side, pose, starts, seeds):
    """The best of several seeded multi-start solves."""
    results = [
        planner.solve(geometry, side, starts=starts, seed=s, pose=pose)
        for s in seeds
    ]
    return min(
        results, key=lambda r: (not r.reachable(), r.tip_error + r.overlap)
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--offset",
        nargs=2,
        type=float,
        metavar=("FORWARD", "UP"),
        help="check with the shoulders moved by these offsets (m)",
    )
    parser.add_argument(
        "--scan", action="store_true", help="scan a grid of offsets"
    )
    parser.add_argument("--starts", type=int, help="optimiser restarts")
    args = parser.parse_args()
    planner = PressPlanner()
    poses = presentation_poses()
    cases = [
        (i, pose, geometry, side)
        for i, pose in enumerate(poses)
        for geometry in planner.scene.stimuli
        for side in planner.scene.sides
    ]
    if not args.scan:
        if args.offset:
            planner.shift_shoulders(*args.offset)
        starts = args.starts or 96
        for i, pose, geometry, side in cases:
            r = _best(planner, geometry, side, pose, starts, (0, 1))
            print(_row(r, i), flush=True)
        return
    print("shoulder offset (forward, up) cm from config/subject.yaml:")
    for dy in SCAN_FORWARD:
        for dz in SCAN_UP:
            planner.shift_shoulders(dy, dz)
            failed = []
            for i, pose, geometry, side in cases:
                r = _best(
                    planner, geometry, side, pose, args.starts or 24, (0,)
                )
                if not r.reachable():
                    failed.append(f"p{i} {geometry[:5]} {side[0]}")
            status = "ALL OK " if not failed else f"{len(failed)} fail"
            print(
                f"  ({100 * dy:+3.0f}, {100 * dz:+3.0f})  {status}  "
                + ", ".join(failed),
                flush=True,
            )


if __name__ == "__main__":
    main()
