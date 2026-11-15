# Hardware & Safety

Each device runs as its own ROS 2 node in its own Docker container. Because all
containers share the host network and IPC, they form a single ROS graph. See
[Architecture §3](../architecture.md) for the runtime topic/service graph.

## Devices

| Device | Node / driver | Role |
| --- | --- | --- |
| UR5e arm(s) | `ur_robot_driver` + `controller_manager` | motion execution (left/right) |
| Teensy 4.1 | micro-ROS agent ⇄ firmware | arm locks, safety laser, reward, smartglass pane, sync pulse, gloves |
| Flic buttons | `flic` node (scapy BLE sniffer) | subject response device |
| EyeLink | `eyelink` node | eye-gaze samples + smooth-pursuit action |
| FLIR cameras | `cam_sync` (SynchronizedCameraDriver) | hardware-synchronized video |
| OptiTrack | `optitrack_driver` (mocap4r2) | rigid-body poses |

For the one-time setup of the EyeLink and OptiTrack host computers (and the
network that connects them to the host), see
[Real Hardware Setup](../getting-started/real-hardware.md#eyelink-and-optitrack-computers).

The `flic` node opens the **host's** Bluetooth controller (`hci0` by default,
selected by its `device_id` parameter) rather than a device mounted into the
container, so that controller must exist and be free before the `real` profile
starts — see
[Real Hardware → Bluetooth adapter](../getting-started/real-hardware.md#bluetooth-adapter-flic-buttons).

MoveIt runs **in-process** inside the Commander (`moveit_py`), not as a separate
`move_group` node — planning is in-process; execution is dispatched to the UR
`controller_manager` action servers.

## Hardware synchronization

The Teensy emits a hardware trigger pulse wired to the FLIR cameras' input line,
so all cameras expose simultaneously. The synchronized driver estimates nominal
capture epochs from callback host times, while each camera's FrameID chunk is
used to detect skipped frames. Verify sync with:

```bash
ros2 run tabletop_rig system_check
```

FrameID is a per-camera counter, not a globally shared synchronization key.
Cameras can start with different, stable counter offsets, so corresponding
frames do not necessarily have equal numeric FrameIDs. Establish and validate
any per-camera offset from a known-good synchronized interval for each session
rather than assuming equality.

ROS header timestamps are non-decreasing but are not guaranteed to be unique or
identical across every camera frame. Under host scheduling jitter, adjacent
physical frames can receive the same estimated header timestamp and a later
frame can compensate with a larger timestamp interval, even while `camera_time`
and FrameID remain continuous. For offline analysis:

- use each camera's `meta` stream (`camera_frame_id`, `camera_time`, and
  synchronization-validity fields) to detect skips and validate the per-camera
  frame relationship;
- join repeated header timestamps by occurrence order, never by timestamp alone;
- retain the exact video timestamp sidecar (`header_stamp_ns`, `bag_time_ns`, and
  `pts_adjusted`) and the original MCAP data; video PTS may be adjusted to remain
  strictly increasing when source header stamps repeat; and
- validate camera alignment against the timing tolerance required by the
  experiment. Treat frames marked synchronization-invalid as unmatched archival
  data rather than synchronized observations.

### Localizing FLIR losses

The synchronized driver emits one structured `[FLIR_PIPELINE]` line per camera
every five seconds using its existing status timer. Normal sessions retain it
in the driver log and recorded `/rosout`; it adds no per-frame log messages or
new ROS topics. Camera input queues, DDS reliability, image quality, trigger
rate and planning parameters are not changed by these diagnostics.

| Fields | Meaning |
| --- | --- |
| `sdk_received`, `sdk_skipped`, `sdk_incomplete`, `sdk_timeouts` | Cumulative SDK acquisition statistics. `sdk_skipped` is a gap between complete-frame IDs delivered by the SDK, not a claim about the physical cause. Incomplete frames can also contribute to those gaps; do not add the two counters as independent losses. |
| `callbacks`, `enqueued`, `queue_dropped` | Complete frames reaching the driver and accepted/rejected by its bounded input queue. A rising `queue_dropped` with flat SDK gap counters localizes losses to the driver queue. |
| `queue_depth`, `queue_limit`, `queue_peak`, `queue_peak_interval` | Current occupancy, configured limit, lifetime peak and recent five-second peak. |
| `published_main`, `published_unmatched`, `publish_failed` | Publication outcomes. Unmatched frames are preserved archival data, not queue drops. |
| `publish_wall_us`, `publish_cpu_us`, `publish_calls` | Cumulative time/count around timestamp matching, image construction, transport compression/publication and metadata publication. Thread CPU time excludes time descheduled or blocked; wall time includes both. These do not identify a particular codec function. |
| `*_max_interval`, `callback_age_us_max` | Recent/lifetime timing peaks. Callback age is SDK callback host time to start of driver processing, including callback preparation and queue delay; it is not an exposure timestamp or a pure scheduler-latency measurement. |

Lifetime counters reset when the camera object is recreated. Recent interval
peaks reset when logged. SDK and driver snapshots are collected sequentially,
so counts can differ transiently by in-flight frames. No SDK feature queries,
payload copies or logging are added to the acquisition callback; bookkeeping
uses its existing queue mutex. Publishing timing adds two thread CPU-clock
reads and a short counter update per frame. Unit tests cover independent loss
counters, unmatched archival outcomes and interval-versus-lifetime peaks.

For load diagnosis, compare camera counters with `/proc/<pid>/task/*/schedstat`,
CPU utilization, recorder losses and NAS network throughput. Keep simulations
off the real ROS domain. The commissioning test used a private-network
planning-only container, a read-only workspace, no hardware devices, and
`ROS_DOMAIN_ID=197`; the separate recorder launched only `rosbag.launch.py`,
never Commander or a task. A dark scene compresses more cheaply and writes
less data, so repeat qualification with normal experimental lighting before
extrapolating its results to a real session.

The 2026-11-14 diagnostic test reproduced **1,806** missing frame IDs in the
saved MCAP, exactly matching **1,806 driver input-queue drops**, with zero SDK
skips/incompletes. NAS-only and intermittent-planning phases had zero drops;
continuous single-arm planning had 11, and simultaneous two-arm planning had
1,795. No camera queue or planner-count setting changed. The test identifies a
reproducible CPU/scheduling bottleneck, but does not prove the same allocation
of losses retrospectively for sessions that lacked these counters. The dark
scene also meant only about 26–28 MB/s of NAS writes, versus about 203 MB/s in
the preceding real task, so it was not a new full-bandwidth NAS qualification.
Keep normal-lighting, full-rate acquisition in the next acceptance test.

## Safety interlock

The Teensy firmware publishes `teensy/sensor` (a `TeensySensor` message) at a
configurable rate. The `TeensyInterface` inside the Commander gates every robot
motion through `safe_to_execute`, which requires the safety conditions to hold
continuously for a configured time and the sensor message to be fresh.

!!! warning "Arm-lock check is off by default"
    By default `safe_to_execute` enforces **only** the safety laser
    (`is_safety_laser_broken`). The arm-lock check (both arms seated in the
    restraints) is gated by the `safe_to_execute.require_arm_locks` parameter in
    `commander.yaml`, which defaults to `false` — so the arm-lock state the
    firmware publishes is **not** enforced unless you set
    `require_arm_locks: true`. Enabling it is a safety-relevant decision; the
    laser-only default preserves the rig's historical behaviour. The mechanism
    that holds / detects the hands (currently a button per hand, not a physical
    lock) is being reworked — see [Known Issues](../known-issues.md).

Standard firmware reads the active-low left-hand sensor on Teensy pin 36 and the
right-hand sensor on pin 39. Its wired response-button input is disabled. A
`tt-build microros --button-pin <pin>` bench build temporarily enables that
response input and forces any hand-lock feedback sharing the selected pin to the
fail-safe `unlocked` state.

## Firmware

The Teensy micro-controller firmware lives under
`src/microros/` and is built with PlatformIO (not colcon):

```bash
tt-build microros
```

The Teensy firmware implements the `tabletop_interfaces` services (`SetArmLock`,
`SetReward`, `SetSolenoid`, `SetSmartglass`, `Ping`) and publishes
`TeensySensor`. When changing it, keep the micro-ROS executor handle counts (and
`colcon.meta`) in sync with the number of entities. When in doubt, unplug and
replug, then re-run `tt-env-gen` so Docker re-mounts the device.
