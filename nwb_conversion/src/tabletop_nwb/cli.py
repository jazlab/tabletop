"""``tt-nwb``: convert and validate TableTop sessions.

tt-nwb convert  <session_dir> [--profile sim|rig|rig+ephys] [--out DIR]
tt-nwb validate <nwb_dir>
tt-nwb run      <session_dir>... [--profile ...]   # convert + validate
tt-nwb synth    <root> [--profile rig|sim]         # write a synthetic session
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def _validate(nwb_dir: Path) -> bool:
    from .validation import validate

    results, summary = validate(nwb_dir)
    c = summary["counts"]
    print(
        f"    validation: PASS {c['PASS']} · WARN {c['WARN']} · FAIL {c['FAIL']} · INFO {c['INFO']} · "
        f"SKIP {c['SKIP']} → {nwb_dir / 'validation' / 'report.md'}"
    )
    for r in results:
        if r.status in ("FAIL", "WARN"):
            print(f"      {r.status} {r.id} {r.message}")
    return summary["ok"]


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        prog="tt-nwb",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = p.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("convert")
    c.add_argument("session_dir", type=Path)
    c.add_argument("--profile", choices=["sim", "rig", "rig+ephys"])
    c.add_argument("--out", type=Path)
    v = sub.add_parser("validate")
    v.add_argument("nwb_dir", type=Path)
    r = sub.add_parser("run")
    r.add_argument("session_dirs", type=Path, nargs="+")
    r.add_argument("--profile", choices=["sim", "rig", "rig+ephys"])
    s = sub.add_parser("synth")
    s.add_argument("root", type=Path)
    s.add_argument("--profile", choices=["sim", "rig"], default="rig")
    s.add_argument("--duration", type=float, default=30.0)
    s.add_argument("--seed", type=int, default=0)
    a = p.parse_args(argv)

    from .session import ProfileError

    if a.cmd == "convert":
        from .convert import convert

        try:
            convert(a.session_dir, a.out, a.profile)
        except ProfileError as e:
            print(f"profile error: {e}", file=sys.stderr)
            return 2
        return 0
    if a.cmd == "validate":
        return 0 if _validate(a.nwb_dir) else 1
    if a.cmd == "run":
        from .convert import convert

        ok = True
        for d in a.session_dirs:
            try:
                convert(d, None, a.profile)
            except ProfileError as e:
                print(f"profile error ({d.name}): {e}", file=sys.stderr)
                ok = False
                continue
            ok &= _validate(Path(d).resolve() / "nwb")
        return 0 if ok else 1
    if a.cmd == "synth":
        from .testing.synth import make_session

        d, t = make_session(
            a.root, a.profile, duration_s=a.duration, seed=a.seed
        )
        print(f"{d}  ({t.n_pulses} pulses, {t.n_trials} trials)")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
