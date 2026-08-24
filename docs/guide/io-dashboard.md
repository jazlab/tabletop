# I/O Check Dashboard

The TableTop I/O Check dashboard is an always-available bench diagnostic on
noVNC workspace 5. It provides a compact view of the rig's discrete inputs and
short, bounded tests for non-robot outputs. The dashboard never commands either
robot.

## What changed

The I/O work adds the following behavior:

- any Flic press is displayed from `/flic/button_pressed_time`, including the
  button address and a cumulative event count;
- the left-hand sensor on Teensy pin 36, right-hand sensor on pin 39, and safety
  laser on pin 25 are displayed from `/teensy/sensor`;
- stale or missing Teensy messages are shown as unavailable rather than as a
  healthy inactive state;
- smartglass can be inverted for one second and automatically restored;
- the juice/reward solenoid can be pulsed for 200 ms;
- the same brief reward tone used by the task can be played through PulseAudio;
- either hand buzzer can be pulsed for one second without changing either
  arm-restraint output; and
- every output button is disabled while a `commander` node is present.

The dashboard is started by the `io-dashboard` Compose service for both real
and simulated profiles. It moves its window to noVNC workspace 5 at startup and
then returns the visible desktop to workspace 1.

## Teensy firmware and interfaces

The timed hardware outputs are bounded by Teensy timers. They therefore return
to a safe state even if the dashboard closes or its ROS process is delayed.

| Function | Teensy pin | ROS interface | Bound |
| --- | ---: | --- | ---: |
| Left-hand sensor | 36 | `/teensy/sensor` | input |
| Right-hand sensor | 39 | `/teensy/sensor` | input |
| Safety laser | 25 | `/teensy/sensor` | input |
| Smartglass | 3 | `/teensy/set_smartglass` | 1 s in dashboard |
| Juice/reward solenoid | 26 | `/teensy/set_reward` | 200 ms in dashboard |
| Left-hand buzzer | 41 | `/teensy/set_buzzer` | 1 s in firmware |
| Right-hand buzzer | 40 | `/teensy/set_buzzer` | 1 s in firmware |

`SetBuzzer.srv` was added so a buzzer can be tested without using
`SetArmLock.srv` and without moving an arm-restraint output.
`SetSmartglass.srv` now accepts a duration. A positive duration temporarily
changes the glass and restores its previous state; a zero duration preserves
the normal persistent task behavior.

The firmware now creates six ROS services. Its statically allocated micro-ROS
pool was increased from five to eight service slots, leaving two spare slots.
A compile-time assertion prevents a future service addition from silently
exceeding the configured pool. An undersized pool causes the firmware to create
a session, register only five services, tear the session down, and repeat; on
the dashboard this looks like Teensy-dependent cards briefly flickering online.

## Response-button pin behavior

Standard firmware reserves pin 36 for the left-hand sensor and disables the
wired response-button input. Pin 39 remains the right-hand sensor. A temporary
bench build can enable a wired response input with:

```bash
tt-build microros --button-pin <pin>
```

If the selected pin is 36 or 39, the response input owns that interrupt and the
corresponding hand feedback is forced to the fail-safe `unlocked` state. Never
run a robot task with such a temporary firmware build. Reinstall standard
firmware immediately after a response/Flic smash test.

## Installation after firmware changes

Stop the micro-ROS agent before uploading so it releases the serial port. The
Teensy disappears from `/dev` briefly while rebooting, so wait before recreating
the agent container:

```bash
cd /path/to/tabletop
docker compose stop teensy
docker compose run --rm builder tt-build microros
sleep 3
docker compose up -d --force-recreate teensy io-dashboard
```

Using `up --force-recreate` after the pause ensures Docker binds the
re-enumerated serial device. Starting the old container before the device has
returned can fail with `/dev/ttyACM0: no such file or directory` and leave all
Teensy-backed dashboard cards offline.

The dashboard updates live; refreshing the noVNC browser page is unnecessary.

## Simulation and verification

`MockTeensy` implements the buzzer and temporary-smartglass behavior so the GUI
can be exercised without physical hardware. Regression tests cover event
counting, stale-state rendering, Commander lockout, smartglass restoration,
bounded reward and buzzer timing, and the guarantee that buzzer tests do not
change arm-restraint outputs.

Useful checks are:

```bash
docker compose run --rm builder tt-build colcon --packages-up-to tabletop_rig
docker compose run --rm builder pytest -q \
  src/ros/tabletop/tabletop_rig/tests/io_dashboard_test.py \
  src/ros/tabletop/tabletop_rig/tests/io_dashboard_integration_test.py
docker compose run --rm builder tt-build microros --clean --no-upload
```

The final command is build-only and does not flash or operate the real rig.

## Troubleshooting

If only Flic and reward sound are available, the host-side dashboard is running
but the Teensy is not visible to ROS. Check:

```bash
docker compose ps -a teensy io-dashboard
docker compose logs --tail 100 teensy io-dashboard
ls -l /dev/ttyACM* /dev/serial/by-id/
```

If the agent repeatedly registers five services and disconnects, rebuild and
flash the standard firmware containing the eight-slot micro-ROS service pool.
If the agent container exited while the Teensy rebooted, wait for the serial
device and recreate the container using the installation sequence above.
