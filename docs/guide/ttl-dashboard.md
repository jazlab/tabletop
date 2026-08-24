# TTL Sync Dashboard

The TableTop TTL Monitor is a passive timing dashboard on noVNC workspace 6.
It compares the once-per-second synchronization pulse as observed independently
by EyeLink, both UR robots, and the designated FLIR sync camera. It never
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
| Robot 1 / left | `/left_io_and_status_controller/io_states` | correlated digital input, auto-detected |
| Robot 2 / right | `/right_io_and_status_controller/io_states` | correlated digital input, auto-detected |
| FLIR | `/cam_sync/left_back_top_cam/meta` | Line3, bit 3 of `line_status` |
| Reference | `/teensy/sensor` | `sync_pulse_state` from Teensy pin 0 |

Only `left_back_top_cam` showed the once-per-second Line3 transition. Its
metadata changed between `4` and `12`, meaning bit 3 changed, and the edge
followed the Teensy reference by roughly 8-25 ms in the software-observed
probes. The other five cameras showed
no Line3 transition, matching the rig's single-camera sync-input wiring.

The two UR controllers were each sampled at about 425 I/O messages per second
across digital inputs 0-17. No input changed during that observation. The
dashboard therefore does not guess a pin. It selects an input only after the
same edge has correlated with three Teensy pulses. Until then it reports
`NO CORRELATED TTL`. This makes missing wiring or a wrong UR input configuration
visible instead of reporting a false healthy state.

For installations with a fixed known mapping, auto-detection can be overridden:

```bash
TABLETOP_TTL_ROBOT1_PIN=0
TABLETOP_TTL_ROBOT1_ACTIVE_LOW=false
TABLETOP_TTL_ROBOT2_PIN=0
TABLETOP_TTL_ROBOT2_ACTIVE_LOW=false
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
4. Robot 2 / right; and
5. the FLIR sync camera.

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
EyeLink bit decoding, three-edge robot input detection, rejection of constant
robot inputs, rendering, and bounded EyeLink status publication.

```bash
docker compose run --rm builder python3 -m pytest -q \
  src/ros/tabletop/tabletop_rig/tests/ttl_dashboard_test.py
```
