# November reliability and export checkpoint

This records retained changes since the September 11 integration checkpoint
(parent `ab1ae33`, FLIR driver `719c138`). It complements the historical
[rig reliability checkpoint](rig-reliability-checkpoint.md); that earlier page
describes the earlier snapshot, not every current setting.

## Execution and task startup

- The right shoulder-pan/base joint has a model-level 2.4 rad/s velocity ceiling.
  Retimed cached and newly planned trajectories, and direct execution, are
  checked against configured per-joint bounds before submission. The global
  acceleration ceiling remains 14 rad/s²; this is not the abandoned global
  acceleration/jerk experiment.
- Removed the extra `big_object_6` presentation/return velocity scale to avoid
  compounding the new base-joint ceiling. Narrow object-specific safety and
  clearance handling otherwise remains in place.
- Dashboard stop/operator cancellation is treated as orderly cancellation,
  rather than misreported as a fatal execution fault. Both arms still receive
  prompt cancellation; no cleanup trajectory is launched on an operator stop.
  Actual execution errors retain the failure path.
- Recorded tasks wait for this launch's combined recording preflight and
  EyeLink recorder discovery before task sync pulses, `TASK_STARTED`, or trials.
  Delayed Commander launch explicitly inherits those gate flags and the launch
  timestamp, preventing early startup after the temporary launch scope ends.

See [manipulation reliability](manipulation-reliability.md) and
[task readiness](tasks.md#recording-readiness-at-task-startup).

## Acquisition and storage

- Added opt-in NAS recording helpers (`tt-nas-mount`, `tt-recording-storage`,
  `tt-storage-preflight`), Compose mounts, and expected-share/free-space checks.
  NAS failure is not permission to record silently to the underlying local
  mountpoint. Existing HDD recording remains available.
- Hardened rosbag discovery for late robot topics and service-event topics;
  preflight verifies actual robot recording rather than discovery alone.
  Recording settings and launch-scoped readiness remain explicit.
- EyeLink finalization waits for capture flush, EDF transfer, and capture
  reporting. Derived CSV conversion runs afterward in a separate low-priority
  worker, with its own status/log, rather than holding the bag open for the
  conversion. The original EDF is retained on conversion failure. Capture
  failures remain visible and are not masked by later conversion success.
- Headerless, sample-only EyeLink ASCII files retain their first sample during
  conversion instead of consuming it as a nonexistent header.
- The FLIR submodule adds separate SDK, callback, driver queue, main/unmatched
  publication, and publish-failure counters; queue high-water marks; and wall
  versus thread-CPU publication timing. Interval peaks make transient stalls
  visible. Counter reports continue during synchronization warmup. This
  instrumentation does not increase acquisition queues or change robot DDS
  reliability.
- Clarified that per-camera raw FrameIDs are not a global inter-camera sync key.

See [configuration](configuration.md), [capture](peripheral-capture.md), and
[troubleshooting](troubleshooting.md).

## noVNC and monitoring

- Removed the redundant automatically launched standalone rig view and
  renumbered the remaining layout to six workspaces. The first is a lightweight
  six-camera grid; the second retains the color camera plus full rig scene.
- Hidden camera/grid and RViz views suppress unnecessary preview/render work.
  A visualization plugin handles RViz render visibility. Acquisition and bag
  recording continue independently of the selected GUI workspace.
- Updated layout supervision, I/O, exposure/TTL, and session dashboards for the
  workspace mapping, including operator-stop handling and current health labels.

See [noVNC layout](novnc-layout.md) for services, mapping, and recovery.

## Offline export

Added the portable `tt-export` launcher for NAS source/output with bounded local
SSD scratch, read-only originals, explicit idle-service pausing/restoration,
resume provenance, progress logs, and exact selected-topic count validation.
Unmatched-image omission requires explicit consent and is documented; no
automatic post-hoc frame repair is claimed.

The exporter itself already supports one-pass CSV spooling and direct segmented
H.264 conversion. The new launcher packages the tested workflow rather than
introducing a different CSV/video format. The [offline export guide](offline-export.md)
records measured short-session results, projected long-session costs, and
validation limitations. The full 2.77-hour export remains a separate operation,
not part of publishing this checkpoint.

## Scope and host maintenance

This checkpoint does not merge into `main`, start a real task, delete recordings,
or change the production invariant-affordance motion preset. Local diagnostic
motion presets are not part of this checkpoint.

Host NVIDIA package rollback/update policy is operating-system state, not
reproduced by checking out this branch. On the export test host, userspace/kernel
driver mismatch was resolved by restoring matching 580.173.02 components.
Version checks (`nvidia-smi` and `/proc/driver/nvidia/version`) should agree after
maintenance. Driver updates or a manual-update policy require deliberate host
administration; holding updates also means security maintenance must be planned.
No host credentials, certificates/private keys, recordings, or downloaded driver
packages are included here.

## Checkpoint checks

The publication checks passed 298 focused ROS/task tests (excluding the local
diagnostic motion-preset test), two isolated real-DDS recorder-discovery tests,
22 host export/EDF tests, three freshly compiled FLIR counter tests, and seven
existing timing/render-policy tests. ROS checks used temporary directories and
a network-isolated container; the DDS fixture publishes synthetic messages,
not robot commands. The documentation site builds successfully, with existing
API docstring warnings. No real robot motion or full long-session export was
performed for these publication checks.
