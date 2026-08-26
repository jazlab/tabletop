# TTL Sync Dashboard

The TableTop TTL Monitor is a passive timing dashboard on noVNC workspace 6.
It compares the once-per-second synchronization pulse as observed independently
by EyeLink, Robot 1, and the designated FLIR sync camera. It never
commands a robot or changes an output.

## Physical source pins

Standard Teensy firmware generates two unrelated signals:

| Teensy pin | Signal | Timing |
| ---: | --- | --- |
| 0 | experiment synchronization pulse | 100 ms high, approximately once per second |
| 33 | FLIR exposure trigger | 120 Hz square wave |

Pin 0 is the primary synchronization output. Pin 12 can mirror pin 0 only while
the optional sync-pulse solenoid mode is armed; it is not the primary TTL
output. The workspace-6 monitor follows pin 0 and intentionally excludes the
120 Hz pin-33 signal.

## Observed device inputs

The live hardware trace established these mappings:

| Device | Observed ROS source | TTL interpretation |
| --- | --- | --- |
| EyeLink | `/eyelink/ttl_input` | input bit 3, active-low (`255` idle, `247` asserted) |
| Robot 1 / left | `/left_io_and_status_controller/io_states` | digital input 3, active-low |
| FLIR | `/cam_sync/left_back_top_cam/meta` | Line3, bit 3 of `line_status` |
| Reference | `/teensy/sensor` | `sync_pulse_state` from Teensy pin 0 |

Only `left_back_top_cam` showed the once-per-second Line3 transition. Its
metadata changed between `4` and `12`, meaning bit 3 changed, and the edge
followed the Teensy reference by roughly 8-25 ms in the software-observed
probes. The other five cameras showed
no Line3 transition, matching the rig's single-camera sync-input wiring.

Robot 1 publishes about 425 I/O messages per second across digital inputs 0-17.
The installed isolated interface feeds Robot 1 digital input 3. Its robot-side
output is powered from the UR controller's 24 V and 0 V rails and is inverted:
DI 3 is high while the synchronization pulse is idle and low while the pulse is
asserted. Loaded measurements were 16 V idle and 3.9 V asserted, within the UR
digital-input thresholds. The dashboard therefore interprets DI 3 as active-low.

If Teensy pin 0 is wired directly to UR DI 0, the voltage levels are incompatible:
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

## EyeLink remains passive

The existing EyeLink gaze publisher starts tracker recording when a gaze
subscriber appears. A permanently running dashboard must not subscribe to that
stream directly. The EyeLink node now publishes a small `/eyelink/ttl_input`
status topic from its already-running retrieval loop. Subscribing to this status
does not start retrieval or create an EDF recording.

Consequently the EyeLink card says `WAITING FOR RECORDING` while the tracker is
idle. It becomes live automatically when a task or another intended gaze
consumer starts EyeLink recording.

## Display

The upper cards show current state, pulse count, selected input, and observed lag
from the Teensy reference edge. The lower eight-second trace aligns:

1. Teensy source;
2. EyeLink;
3. Robot 1 / left;
4. the FLIR sync camera.

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
EyeLink bit decoding, Robot 1 input detection, rejection of constant robot
inputs, rendering, and bounded EyeLink status publication.

```bash
docker compose run --rm builder python3 -m pytest -q \
  src/ros/tabletop/tabletop_rig/tests/ttl_dashboard_test.py
```
