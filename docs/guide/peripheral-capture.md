# Peripheral capture and diagnostics

This page documents the camera, EyeLink, robot-telemetry, and Flic changes on
the `rig-reliability` branch. It also gives the operator procedures for the new
diagnostic tools. None of the diagnostic commands on this page initializes the
robot commander or requests robot motion.

## What changed

| Area | Behavior |
| --- | --- |
| FLIR access | Cameras use stable `/dev/flir/<serial>` links and the container's `video` group instead of world-writable USB permissions. |
| Camera preview | Six isolated DDS readers publish bounded 640-pixel-wide, 10 Hz preview streams without reducing the full-rate acquisition topics used for recording. |
| Camera layouts | One RViz layout shows all six previews with the planning scene; another shows the color left-back preview with the full planning scene. |
| Trigger monitor | Camera exposure metadata is displayed as a low-bandwidth live timing view for all six cameras. |
| EyeLink retrieval | Every buffered link sample is drained in order, published in variable-length batches, and checked for tracker-time gaps. |
| Robot telemetry | Bags include direct left/right TCP poses and left/right six-axis force/torque wrench topics. |
| Flic detection | The first advertisement is timestamped immediately, then a serialized connect/disconnect sequence silences the remaining advertisement burst. |
| Smash test | A standalone, no-robot node pairs the wired Teensy onset timestamp with the first Flic advertisement and writes per-trial latency to CSV. |

## FLIR camera monitoring

The synchronized FLIR driver continues to acquire and publish the original
camera streams at the configured hardware rate. The `camera-preview` service
subscribes to the compressed streams and republishes reduced previews under:

```text
/cam_preview/<camera_name>/image_raw
```

All six grayscale previews are limited to 10 Hz and a maximum width of 640
pixels. The left-back camera additionally publishes a demosaiced color view:

```text
/cam_preview/left_back_top_cam/image_color
```

This separation keeps the always-on GUI inexpensive; it does **not** lower the
rate or resolution of the acquisition and recording topics.

The committed layouts are:

- `tabletop_rig/config/camera_robot.rviz`: all six previews on the left and the
  MoveIt planning scene/full rig on the right.
- `tabletop_rig/config/left_back_rig.rviz`: color left-back preview on the left
  and the MoveIt planning scene/full rig on the right.
- `tabletop_rig/config/rig.rviz`: the main rig view, reduced to a 10 Hz redraw
  rate to lower idle CPU use.

The Compose stack uses UDPv4 for these GUI readers. This avoids Fast DDS shared
memory stalls seen when one RViz participant consumes all six large streams.

### Exposure-trigger monitor

The `camera-trigger-monitor` service displays camera-reported exposure metadata
for all six cameras. A healthy view shows each camera as `LIVE`, approximately
120 events per second when the rig is configured for 120 Hz, and zero incomplete
groups.

The monitor verifies that each camera reports an exposure following the shared
Teensy Line0 trigger and that the six metadata timestamps group together. It is
not an oscilloscope and does not directly measure the electrical trigger voltage.

If previews stop, inspect the services without restarting acquisition first:

```bash
docker compose ps flir camera-preview camera-trigger-monitor
docker compose logs --tail 100 flir camera-preview camera-trigger-monitor
```

See [Real Hardware Setup](../getting-started/real-hardware.md) for the udev rule
and USB-buffer setup.

## I/O check dashboard

See the dedicated [I/O Check Dashboard](io-dashboard.md) guide for the complete
pin map, firmware/interface changes, safety bounds, installation procedure, and
troubleshooting notes.

noVNC workspace 5 contains the compact TableTop I/O Check dashboard. Its input
cards show any Flic press received on `/flic/button_pressed_time`, the live
left- and right-hand sensor states, and the safety-laser state. Missing or stale
Teensy data is grey instead of being presented as healthy.

The lower row provides short bench tests:

- smartglass changes state for one second and then returns to its previous state;
- the juice solenoid opens for 200 ms;
- the configured task reward tone plays briefly;
- either hand buzzer runs for one second without changing an arm-lock output.

Smartglass restoration and the juice/buzzer cutoffs are enforced by Teensy
timers, not by the GUI. The output row automatically locks whenever a
`commander` node is present, while live input monitoring remains available.
This panel is for bench diagnostics and never commands either robot.

The dashboard starts with both the real and simulation Compose profiles. To
restart it without restarting the rest of the stack:

```bash
cd /path/to/tabletop
docker compose restart io-dashboard
```

## EyeLink online sample retrieval

The online EyeLink path now uses `getNextData()`/`getFloatData()` to drain the
tracker's buffered samples in order. This replaces a newest-sample-only path
that could discard samples accumulated while Python was briefly descheduled.

`/eyelink/sample_array` now carries `tabletop_interfaces/EyelinkBatch`, whose
sample sequence is variable length. The normal batch target is 50 samples, with
a maximum publication latency of 10 ms; a final partial batch is flushed when
recording stops. The fixed-length `EyelinkArray` definition remains available
for compatibility with older bags.

At the end of retrieval, inspect the summary log:

```text
EyeLink retrieval summary: samples=... published_samples=... batches=...
tracker_gap_events=... estimated_missing=... invalid_samples=...
timestamp_discontinuities=...
```

A healthy session has `estimated_missing=0`, `invalid_samples=0`, and
`timestamp_discontinuities=0`. The EDF file remains the authoritative tracker
recording. Live gaze prediction still receives individual samples through the
node's bounded in-process queue; batching reduces ROS publication overhead
without intentionally adding inference latency.

## Robot pose and force/torque recording

When rosbag recording is enabled, `tabletop_rig/config/rosbag.yaml` now records:

```text
/left_tcp_pose_broadcaster/pose
/right_tcp_pose_broadcaster/pose
/left_force_torque_sensor_broadcaster/wrench
/right_force_torque_sensor_broadcaster/wrench
```

The pose and wrench values also appear in `/dynamic_joint_states`, but the
direct topics are easier to inspect and export. The configured
`wrench_filtered` topics were removed from the recording list because the
controllers advertise them without publishing payloads in the current rig.

## Flic advertisement handling

The Flic node timestamps and publishes the first matching BLE advertisement
before starting any reset work. In the background it then:

1. Stops scanning.
2. Waits 1.0 second.
3. Connects to the button.
4. Holds the connection for 0.2 seconds.
5. Disconnects and resumes scanning.
6. Suppresses repeats from that button for a further 0.5 seconds.

Reset sequences are serialized because the Bluetooth controller can initiate
only one connection at a time. A failure to resume scanning closes the client
instead of leaving a silently dead listener. These timings match the validated
standalone detector behavior.

## Flic/Teensy smash test

The smash test measures:

```text
first Flic advertisement timestamp - Teensy interrupt-latched button timestamp
```

It includes BLE advertising/scan latency and any mechanical difference between
the two switch closures. It therefore measures end-to-end response detection,
not Bluetooth processing time alone.

The node subscribes only to `/teensy/sensor` and
`/flic/button_pressed_time`; it does not create a commander or move a robot.
Results are written under `log/flic_smash/`, which is intentionally Git-ignored.

### Standard wiring

Standard firmware reserves Teensy pin 36 for the active-low left-hand sensor and
pin 39 for the active-low right-hand sensor. Each switch must close its input to
GND; the firmware enables `INPUT_PULLUP`. The wired response-button input is
disabled, so `is_button_pressed` is false and `button_last_time_pressed` is zero.
Pin 38 is not assigned by the standard firmware.

### Temporary response-button input

`tt-build microros --button-pin <pin>` enables the wired response input on that
pin for one firmware build/upload. If the selected pin is 36 or 39, the response
button temporarily owns that interrupt and the corresponding hand-lock feedback
is forced to the fail-safe `unlocked` value. **Never run a robot task with this
temporary firmware.**

For a response/Flic smash test on pin 36, stop robot tasks, unplug the left-hand
sensor, connect the response switch between pin 36 and GND, then run:

```bash
cd /path/to/tabletop
docker compose stop teensy
docker compose run --rm builder tt-build microros --button-pin 36
sleep 3
docker compose start teensy
docker compose run --rm commander ros2 run tabletop_rig flic_smash_test \
  --target-address AA:BB:CC:DD:EE:FF --samples 20
```

Replace `AA:BB:CC:DD:EE:FF` with the test button's Bluetooth address. Wait for
`Teensy baseline acquired`, fully release both switches after every trial, and
wait at least two seconds before the next trial. The node prints each latency
plus running mean, standard deviation, minimum, and maximum.

The three-second pause lets `/dev/ttyACM0` disappear and re-enumerate after the
upload before Docker reattaches the serial device. Restore standard firmware and
reconnect the left-hand sensor immediately after the bench test:

```bash
docker compose stop teensy
docker compose run --rm builder tt-build microros
sleep 3
docker compose start teensy
```

The restored firmware disables the wired response input and returns pin 36 to
left-hand feedback. Pin 39 remains right-hand feedback.
