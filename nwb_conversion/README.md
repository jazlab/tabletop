# tabletop_nwb

Converts a recorded TableTop session into NWB files that share one timeline:
`task.nwb` (behavior, I/O, eye tracking, robot, sync) and `video.nwb`
(cameras). It then checks the result with a post-conversion test suite.
`ephys.nwb` and `spike_sorting.nwb` are planned but not implemented yet.

[PLAN.md](PLAN.md) covers the design, the open decisions, and the status and
to-do list (§11).

## Usage

This is a standalone uv project and doesn't need ROS.

```bash
cd nwb_conversion
uv sync
uv run tt-nwb run ../bags/session_2026-10-31_20-16-48     # convert + validate
uv run tt-nwb convert <session_dir> [--profile sim|rig|rig+ephys] [--out DIR]
uv run tt-nwb validate <session_dir>/nwb
uv run tt-nwb synth /tmp/synth --profile rig               # synthetic session + ground_truth.json
uv run pytest
```

Outputs go to `<session_dir>/nwb/`:
- `task.nwb`, `video.nwb`
- `manifest.json`: what was written, what is absent and why
- `inventory.json`
- `sync_report.json`
- `validation/report.md` and `validation/report.json`

`tt-nwb run` and `tt-nwb validate` exit non-zero if any check FAILs.

## Profiles

| Profile | Session | Files |
| --- | --- | --- |
| `sim` | laptop, `tt-compose --profile=sim` | `task.nwb` |
| `rig` | rig without ephys | `task.nwb`, `video.nwb` |
| `rig+ephys` | full rig | all four (ephys not implemented yet, so validation FAILs) |

The profile is inferred from the session directory unless `--profile` is
given. A requested profile that contradicts the data stops the conversion. See
PLAN §3.1.
