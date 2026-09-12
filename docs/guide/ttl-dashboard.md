# TTL Sync Dashboard

The TableTop TTL Monitor is a passive timing dashboard on noVNC workspace 6.
It compares the once-per-second synchronization pulse as observed independently
by EyeLink, Robot 1, the designated FLIR sync camera, and OptiTrack. It never
commands a robot or changes an output.

## Physical source pins

Standard Teensy firmware generates two unrelated signals:

| Teensy pin | Signal | Timing |
| ---: | --- | --- |
| 31 | experiment synchronization pulse | 100 ms high, ~1 Hz during task |
| 33 | FLIR exposure trigger | 120 Hz square wave |

Pin 31 is the primary synchronization output. Pin 12 can mirror pin 31 only while
the optional sync-pulse solenoid mode is armed; it is not the primary TTL
output. The workspace-6 monitor follows pin 31 and intentionally excludes the
120 Hz pin-33 signal.

The reference pulse is intentionally idle outside task execution. It starts
after task preflight succeeds and is stopped on completion, error, or Ctrl-C.
For bench checks, workspace 5 has a firmware-bounded ten-second test button.

## Observed device inputs

The live hardware trace established these mappings:

| Device | Observed ROS source | TTL interpretation |
| --- | --- | --- |
| EyeLink | `/eyelink/ttl_input` | DB25 status pin 12 / status bit 5, active-low (`127` idle, `95` asserted) |
| Robot 1 / left | `/left_io_and_status_controller/io_states` | digital input 3, active-low |
| FLIR | `/cam_sync/left_back_top_cam/meta` | Line3, bit 3 of `line_status` |
| OptiTrack | `/markers` | displacement of the sync-solenoid marker |
| Reference | `/teensy/sensor` | `sync_pulse_state` from Teensy pin 31 |

Only `left_back_top_cam` showed the once-per-second Line3 transition. Its
metadata changed between `4` and `12`, meaning bit 3 changed, and the edge
followed the Teensy reference by roughly 8-25 ms in the software-observed
probes. The other five cameras showed
no Line3 transition, matching the rig's single-camera sync-input wiring.

The EyeLink signal is connected to DB25 status pin 12 rather than a data pin.
The status register presents that line as mask `0x20`; the installed interface
pulls it low during each pulse. A recording-only hardware check observed all
9 of 9 Teensy pulses at EyeLink, with each EyeLink interval measuring
100-101 ms. The dashboard mask can be overridden for a different installation
with `TABLETOP_TTL_EYELINK_MASK`.

Robot 1 publishes about 425 I/O messages per second across digital inputs 0-17.
The installed isolated interface feeds Robot 1 digital input 3. Its robot-side
output is powered from the UR controller's 24 V and 0 V rails and is inverted:
DI 3 is high while the synchronization pulse is idle and low while the pulse is
asserted. Loaded measurements were 16 V idle and 3.9 V asserted, within the UR
digital-input thresholds. The dashboard therefore interprets DI 3 as active-low.

Recorded sessions also include the raw `/left_io_and_status_controller/io_states`
stream. This preserves DI3 edges for offline alignment with the EyeLink, FLIR,
Teensy, and OptiTrack synchronization streams.

If Teensy pin 31 is wired directly to a UR digital input, the voltage levels
are incompatible:
the [Teensy 4.1 output HIGH is 3.3 V](https://www.pjrc.com/store/teensy41.html),
while the [UR5e controller defines -3 to 5 V as OFF and requires 11 to 30 V
for ON](https://www.universal-robots.com/manuals/EN/HTML/SW5_23/Content/prod-usr-man/complianceUR5e/H_g5_sections/installation/controller_i_o.htm).
Use an appropriately designed 3.3 V-to-24 V interface, such as an isolated or
transistor interface suitable for the UR PNP input. Never connect UR 24 V back
into a Teensy signal pin.

For installations with a fixed known mapping, auto-detection can be overridden:

```bash
TABLETOP_TTL_ROBOT1_PIN=3
TABLETOP_TTL_ROBOT1_ACTIVE_LOW=true
```

### OptiTrack solenoid marker

The sync signal also drives a push solenoid carrying a passive OptiTrack marker.
On the installed rig, the asserted pulse moves the marker 3.44 mm along the
measured travel axis while the resting projected RMS jitter is about 0.31 mm.
The dashboard selects the marker nearest its configured rest location within a
3 cm spatial region, learns the precise resting position when it starts, and
projects motion along that axis. A three-sample median filter, separate 1.8 mm
assert and 0.9 mm clear thresholds, and a 15 ms debounce reject camera vibration
without materially changing the displayed timing.

Marker identity is not required. Other loose or rigid-body markers can be
present as long as they do not enter the solenoid marker's spatial region. If
the solenoid or OptiTrack coordinate frame moves, measure the new rest point and
travel axis and update:

```bash
TABLETOP_TTL_OPTITRACK_CENTER=-0.48133,-0.08284,0.53280
TABLETOP_TTL_OPTITRACK_AXIS=-0.97052,0.23989,0.02343
TABLETOP_TTL_OPTITRACK_ROI_RADIUS_M=0.03
```

The detector is observational only: it subscribes to `/markers` and never
commands the solenoid, OptiTrack, or either robot.

## EyeLink remains passive

The existing EyeLink gaze publisher starts tracker recording when a gaze
subscriber appears. A permanently running dashboard must not subscribe to that
stream directly. The EyeLink node instead publishes a small
`/eyelink/ttl_input` status topic. While idle, it polls EyeLink button 1 at 100 Hz
after calling `flushKeybuttons(0)`. The zero argument enables current button
state updates without filling pylink's eight-event queue. This does not start
retrieval or create an EDF recording. During recording, idle polling stops and
the raw parallel-port value in each tracker sample drives the same topic.

EyeLink Host configuration maps button 1 to DB25 status pin 12. The node maps
the normalized idle button state back to raw-equivalent values (`127` idle,
`95` asserted), so workspace 6 uses identical decoding in both modes. The
workspace-5 ten-second output test is therefore visible at EyeLink even while
the tracker is idle.

## Display

The upper cards show current state, pulse count, selected input, and observed lag
from the Teensy reference edge. The lower eight-second trace aligns:

1. Teensy source;
2. EyeLink;
3. Robot 1 / left;
4. the FLIR sync camera;
5. the OptiTrack solenoid marker.

Missing messages, missing recent pulses, and an undetected robot input have
separate states. This is a software-observed timing monitor rather than an
electrical oscilloscope; it confirms what each driver reports to ROS.

The service uses UDPv4 DDS because it consumes high-rate data from multiple
hardware containers. To create or restart it:

```bash
cd /path/to/tabletop
docker compose up -d --force-recreate ttl-dashboard
```

Restart the EyeLink service once after installing the code so it advertises the
new passive status topic:

```bash
docker compose restart eyelink
```

Do not restart EyeLink during an active task or recording.

## Verification

The tests use synthetic aligned and missing TTL streams. They cover Line3 and
EyeLink status-pin decoding, Robot 1 input detection, rejection of constant robot
inputs, OptiTrack jitter and out-of-region marker rejection, OptiTrack pulse
alignment, rendering, and bounded EyeLink status publication.

```bash
docker compose run --rm builder python3 -m pytest -q \
  src/ros/tabletop/tabletop_rig/tests/ttl_dashboard_test.py
```
