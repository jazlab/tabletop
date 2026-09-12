# Rig reliability integration checkpoint

This checkpoint records the retained changes made after parent commit
`b7eb9c1` (`fix(rig): harden synchronized hardware monitoring`). It is a
release-oriented overview of the current behavior; experiments that were later
reverted are intentionally omitted. The linked guide pages remain the detailed
operating reference.

## Robot execution and manipulation

- All joints use a 14 rad/s² acceleration ceiling. Normal velocity scaling is
  unchanged, so the limit targets acceleration and deceleration transients
  rather than globally slowing each trajectory.
- The first `IDLE -> PRE_FETCH` motion after startup uses a 0.3 acceleration
  scale. Full normal scaling resumes after the first successful pre-fetch.
- `small_object_14` receives a short outward/downward attached-object
  clearance move and narrow departure scaling before the common fetched pose.
- Right-arm `small_object_26` has a narrow launch pre-fetch acceleration
  override. Right-arm `big_object_6` has a 0.75 velocity scale only for its
  presentation and unpresentation transitions. Cached and newly planned paths
  pass through the same retiming and validation path.
- Execution accepts a missing action-result response only after a two-second
  delivery grace, and only if the controller reports success, the arm is nearly
  stationary, and it is at the expected endpoint. Active or stuck goals still
  fail normally.
- Startup checks the real current state for collisions, including an arm that
  begins at a mount or in the presentation region. This prevents one arm from
  remaining stranded while the other starts the task.
- Presentation-region motion is guarded by the safety curtain. A curtain
  interruption cancels both controller goals promptly; recovery waits for a
  stable clear period and is bounded. Ctrl-C and the dashboard stop request use
  the same prompt cancellation path and do not start cleanup motion.
- Manipulation preflight fingerprints geometry and motion settings. Real tasks
  admit only currently validated objects, while bounded planning retries,
  recovery waypoints, attached-object clearance, and one immediate trial retry
  improve retrieval without permanently removing a failed object.
- Normal finite completion restores objects while the glass is opaque and skips
  an unnecessary final idle move.

See [Manipulation Reliability](manipulation-reliability.md) for the operational
details and recovery semantics.

## Invariant-affordance task

The new `InvariantAffordanceTask` alternates `small_object_0` on the left and
`small_object_28` on the right. Each object is presented at the center, revealed
for one second, and then occluded. After a 0.5-second glass-settle interval, its
arm follows a collision-checked path to a uniformly sampled position and a
random orientation.

The orientation sampler is uniform on 3D rotations. Shuffled equal-area tilt
strata rotate among upside-down, sideways, and upright categories, preventing
feasibility filtering from repeatedly favoring upright poses. Planning can use
the arm's full side of the rig and complex paths; a centerline allowance lets
both arms reach the common presentation region. Candidates are planned and
validated before reveal, and accepted trajectories are stretched to the fixed
three-second occluded interval. The go tone and both one-second hand buzzers
occur on schedule, followed by a two-second final-pose hold. Button responses
are deliberately not scored.

The task has a long-run configuration and a mock test configuration. See
[Tasks & Experiments](tasks.md) for the launch command and timing details.

## Synchronized acquisition and data integrity

### FLIR cameras

- The synchronized driver consumes its frame queue FIFO and tolerates bounded
  out-of-order timestamp observations.
- Invalid, rewound, nonadvancing, or implausibly jumping camera clocks reset the
  estimator safely. Estimator catch-up is bounded so malformed timestamps
  cannot cause runaway allocation or starve robot control.
- Matched metadata now records the camera frame ID and whether its synchronized
  timestamp is valid.
- Frames that cannot be matched are preserved on per-camera
  `image_raw_unmatched` and `meta_unmatched` archival topics, with raw camera
  time and frame ID retained. These records are useful for post-hoc recovery
  and no longer masquerade as matched frames.
- Camera acquisition remains active independent of transient subscriber
  discovery. Metadata history/QoS depths and the recorder's camera overrides
  absorb short scheduling stalls without switching the robot-critical streams
  to reliable delivery.
- FLIR preflight retries up to three times and saves one calibration snapshot
  per camera instead of recording identical `CameraInfo` messages at frame
  rate.
- Optional CPU-affinity variables and loopback Fast DDS transport are available
  for the high-bandwidth camera path. They default to unset so unfamiliar hosts
  are not pinned accidentally. A 16 GB container ceiling is a final containment
  guard, not an expected operating footprint.

The driver is pinned to JazLab's `codex/rig-reliability-v2` integration branch,
commit `719c138`.

### EyeLink, OptiTrack, robots, and Teensy

- EyeLink batches carry sequence accounting. Shutdown stops EyeLink, closes and
  transfers the EDF, writes a retrieval report, and only then permits the main
  bag to close. Service discovery waits longer so a delayed EyeLink service
  cannot create a false source-loss report.
- OptiTrack republishes continuously while active, including after a Motive
  window restart. Real task startup restarts the OptiTrack service and requires
  a live `/markers` sample before Commander is created. Loopback UDP avoids a
  stale shared-memory endpoint after reconnects.
- A bounded, hysteretic detector extracts the approximately 1 Hz reference pulse
  from the moving passive OptiTrack solenoid marker inside its calibrated spatial
  region.
- Robot joint state, dynamic joint state, direct TCP pose, direct six-axis
  force/torque, and the left robot's DI3 TTL observation are recorded. Increased
  recorder history absorbs short stalls without enabling the high-overhead
  reliable mode that previously disrupted control.
- Sync pulses are enabled for task sessions and disabled outside them. The I/O
  dashboard can request a firmware-bounded ten-second diagnostic pulse train.

The OptiTrack driver is pinned to JazLab's `codex/rig-reliability` integration
branch, commit `be173b2`.

## Recording and session closeout

- Recorded sessions use MCAP with a 100 GB split size and a 10 GB writer cache.
  Topic-specific QoS depths cover all six matched and unmatched camera streams,
  robot streams, EyeLink, OptiTrack, Teensy, Flic, and task lifecycle events.
- `TaskEvent` messages mark task/trial lifecycle, presentation, depresentation,
  and genuine object misses for both live monitoring and offline analysis.
- Recording can target the configured HDD mount. Startup refuses a recorded real
  task when that destination is unset or has competing reads, avoiding camera
  starvation from simultaneous export or analysis.
- Post-session integrity reporting compares exact transport loss, matched versus
  archived-unmatched camera counts, EyeLink source sequences and EDF state, and
  other configured streams. It avoids estimating missing frames from imperfect
  timestamps.
- The bag stays open during orderly device shutdown so final diagnostics and
  EyeLink transfer state are retained.

## noVNC workspaces and security

- The workspace layout is supervised and repaired on startup: camera/RViz views,
  the exposure/TTL view, the I/O check dashboard, the TTL dashboard, and the
  session health dashboard each occupy one predictable workspace.
- Workspace 5 provides live Flic, hand-button, and safety-curtain state plus
  bounded controls for smartglass, reward, buzzers, sound, and the ten-second
  sync test. Test actions do not command either robot.
- Workspace 6 shows the source pulse and device-observed EyeLink, robot, FLIR,
  and OptiTrack-marker TTLs. It displays the approximately 1 Hz synchronization
  pulse, not the FLIR cameras' 120 Hz exposure trigger.
- Workspace 7 reports task duration, trials, real object misses, average ITI,
  per-arm full-lap time, stream freshness, exact post-startup transport loss,
  robust 60-second TTL health, and HDD capacity. Its stop button calls
  Commander's normal cancellation service. The completed session remains visible
  until the next task starts.
- noVNC is password protected and served over TLS. Credentials, the private key,
  and Xauthority data are generated locally under git-ignored `.novnc/`; the CA
  certificate can be imported on trusted client computers.

See [I/O Check Dashboard](io-dashboard.md), [TTL Sync Dashboard](ttl-dashboard.md),
[Session Health Dashboard](session-dashboard.md), and
[noVNC Workspace Layout](novnc-layout.md).

## Teensy firmware and hardware mapping

The retained standard firmware publishes sensors at approximately 100 Hz and
uses these rig mappings:

| Function | Teensy pin / behavior |
| --- | --- |
| Smartglass | 3; high means transparent |
| Left/right arm locks | outputs 4 / 5 |
| Left/right buzzers | outputs 41 / 40 |
| Juice reward | output 26 |
| Approximately 1 Hz reference sync | output 31; 100 ms pulse |
| OptiTrack marker solenoid | output 12; mirrors enabled sync |
| FLIR exposure trigger | output 33; independent 120 Hz square wave |
| Safety curtain | input 25; high means broken |
| Left/right hand sensors | inputs 36 / 39; active low |

The normal build leaves the wired response input disabled so pin 36 belongs
exclusively to the left-hand sensor. A temporary smash-test build can explicitly
assign a response pin without deleting the standard behavior. Firmware services
bound smartglass, solenoid, reward, buzzer, and sync-test durations, so a lost
client cannot leave a diagnostic output asserted indefinitely. Agent liveness
and clock synchronization are handled independently to tolerate ordinary USB
scheduling jitter and reconnect cleanly after replacing a Teensy.

## Offline export

`tabletop_unbag` now performs one sequential MCAP read while dispatching bounded
per-topic CSV queues and a shared image/video worker pool. CSV schemas are
learned into compact spools and materialized without rereading the bag. Direct
video mode demosaics frames into resumable H.264 Matroska segments and writes an
exact nanosecond timestamp sidecar for every frame; the source MCAP remains the
lossless scientific record.

Exports support destination free-space checks, storage-plugin inference and
reindexing, topic inclusion/exclusion, atomic outputs, restartable video
segments, manifests with expected/success/failure counts, NVIDIA encoding in an
offline-only Compose profile, and CPU `libx264` fallback. The exporter never
launches or commands the robots. See the `tabletop_unbag` README for the full
CLI and format tradeoffs.

## Validation performed

- Parent ROS packages have been rebuilt in the project build container without
  commanding hardware.
- Robot safety, execution-grace, preflight, dashboard, recording-integrity,
  noVNC-layout, EyeLink-interface, and task-runner behavior have focused tests.
- The invariant-affordance planner and task logic passed its focused mock suite,
  including repeated randomized planning cycles.
- The FLIR estimator test executable passed all six functional tests. Upstream
  repository-wide formatting checks are separate from these functional tests.
- Real-rig observations informed the narrow object/transition overrides, but no
  real robot is ever started by a build, unit test, dashboard, or exporter.

## Reproducing this checkpoint

Clone with submodules, or synchronize an existing clone after pulling:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

The parent commit pins exact submodule commits, while `.gitmodules` also names
the corresponding JazLab integration branches for intentional future updates.
