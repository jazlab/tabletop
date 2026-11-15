# Offline session export

`tt-export` exports a finalized MCAP session to per-topic CSVs and timestamped
H.264 videos, using the existing C++ `tabletop_unbag` exporter. It does not
launch a task or command either robot. Original bags are mounted read-only.

## Recommended daily workflow

Keep recordings and final exports on the NAS; use local SSD only for temporary
CSV spools. This needs space for the spools, not for another full raw recording.
Run exports between sessions: offline conversion competes with acquisition for
CPU, GPU, network, and storage bandwidth.

From the host, after the task and recorder have finished:

```bash
cd /path/to/tabletop
source setup.bash
tt-export /mnt/tabletop-nas/sessions/SESSION_NAME \
  --skip-unmatched-images --pause-rig --check
tt-export /mnt/tabletop-nas/sessions/SESSION_NAME \
  --skip-unmatched-images --pause-rig
```

Replace the workspace and session paths. `--check` validates readiness without
creating directories or stopping services. `--pause-rig` explicitly permits
temporarily stopping the running UR, FLIR, Teensy, OptiTrack, and EyeLink
services. Their original container IDs are saved in the output directory and
only those previously running are restarted when the command exits, including
after an ordinary error or Ctrl-C. It refuses a detected active TableTop task,
recorder, or export. Do not start a new task while exporting. A host crash or
forced kill can require manually restoring the saved services.

Prerequisites are the normal host Python environment, Docker, a built
`tabletop_unbag`, and the local `jazlabtabletop/ros-base` image. The default GPU
path also needs a working NVIDIA driver/container runtime with video support.
Use `--video-codec libx264` for CPU encoding; it can be substantially slower.
No container image is implicitly downloaded.

Default destinations and limits:

| Item | Default |
| --- | --- |
| Final export | `SESSION_NAME/export/` |
| Temporary CSV spools | `WORKSPACE/.cache/export-scratch/SESSION_NAME-PATH_HASH/` |
| Initial scratch/output free-space reserve | 150 GB / 250 GB |
| Video | H.264 NVENC, constant-quality setting 28, nominal 120 Hz |
| Segments | 300 seconds, exact per-frame timestamp sidecars |
| Worker jobs | 6 |

Use `--output`, `--scratch-root`, and the `--minimum-*-gb` options for other
destinations or session sizes. These reserves are checks, not quotas or size
predictions. The worker also checks remaining output space while exporting.
For a configured NAS backend, both input and output must resolve to the
expected mounted CIFS share; a missing mount cannot silently fall back to local
disk. `--nas-source //server/share` sets an explicit expected share.

Completed CSV spools are deleted by the exporter. Failed or interrupted work
can leave temporary files; keep them until resuming or inspecting the failure.
The command never deletes the original MCAP/EDF. Original recordings remain
the archival source; an export is not an independent backup.

## Unmatched images and video timing

The launcher requires `--skip-unmatched-images` when nonempty
`image_raw_unmatched` topics exist. This is an explicit omission of those image
messages only: their metadata is still exported and their original frames remain
in the MCAP. They are **not** automatically matched, inserted into videos, or
filled with fabricated frames. If unmatched images are scientifically important,
export them separately with the lower-level exporter and retain their raw clock
and frame-ID information.

The standard six camera streams can use six concurrent NVENC sessions. A tested
driver opened only eight encoder contexts; exporting six normal plus six rare
unmatched streams together exceeded that limit. The launcher therefore refuses
more than six nonempty video topics in GPU mode. Changing `--jobs` alone does
not limit the number of persistent per-topic encoders.

Videos are lossy viewing/pose-tracking proxies, not bit-exact camera archives.
The nominal video frame rate does not replace actual exposure timing. Use each
segment's nanosecond header/bag timestamp sidecar for alignment, including
duplicate timestamps or acquisition gaps. Conversion preserves recorded timing;
it cannot repair source synchronization or missing acquisition data. See the
[exporter README](https://github.com/jazlab/tabletop/blob/rig-reliability/src/ros/tabletop/tabletop_unbag/README.md)
for low-level options and resume semantics.

## Verification, logs, and resume

The output contains `export_provenance.json`, timestamped logs and saved service
state, and the worker's `export_manifest.json`. Before reporting success,
`tt-export` compares complete/succeeded/failed totals and every selected topic
count against the original bag metadata. Deliberately omitted image counts are
recorded separately. This is count validation, not a complete independent decode
or CSV-content audit; retain the source and investigate any reported discrepancy.

Keep the terminal open; progress is printed approximately once a minute.
Ctrl-C stops the export container before restoring acquisition services. To
resume, rerun the same command with the same output directory. The provenance
must match, completed outputs use the worker's resume behavior, and no blanket
overwrite is requested. A completed verified export is refused rather than
silently redone. The workspace lock prevents two instances of this launcher
from exporting concurrently.

## Measured performance (November 2026)

A native 406-second, 81.9 GB recording exported in 544 seconds to about 6.0 GB:
25 CSVs and 292,546 camera frames across 12 video segments. All 2,282,691 source
messages were accounted for, with zero conversion failures. Independent checks
covered CSV row counts, video packet counts, timestamp sidecars, and short
beginning/end decode samples. Recorded duplicate source timestamps remained
visible rather than being silently removed.

For a 60-second all-stream excerpt, NAS output with SSD scratch took 84 seconds,
versus 91.5 seconds with NAS scratch. All-SSD export plus verified copy to NAS
took about 85 seconds overall, so staging the final export on SSD did not help
that fixture. These are measured examples, not throughput guarantees.

A 2.77-hour, 2.03 TB session is projected to take roughly 4–6 hours, yielding
about 147 GB of output and about 67 GB peak scratch usage. Its full export has
not yet been completed; these figures extrapolate from the short-session tests.
The tested policy omits ten archived unmatched images while retaining their
metadata and all original recordings.
