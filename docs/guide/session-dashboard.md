# Session Health Dashboard

The session-health dashboard occupies noVNC workspace 7. Its monitoring path
is passive. The sole control is the red **STOP ROBOTS** button, available only
while a Commander task is active. It cancels both active trajectory goals and
the task through the same shutdown path as Ctrl-C; it never starts cleanup or
other robot motion. This is a software stop, not a substitute for the physical
emergency stop.

The dashboard starts with the normal Compose stack. Click **7** in noVNC to
open it. While a task is active it reports:

- elapsed task time and successful presentation count;
- real object misses (an object that still could not be presented after its one
  immediate retry);
- average ITI, measured from one arm beginning to leave the presentation pose
  to the next arm reaching its presentation pose;
- average full-lap time for each arm, measured from that arm beginning to
  depresent an object until the same arm next reaches presentation;
- stream state and recent rate for Teensy, EyeLink, both robots, OptiTrack, and
  all six FLIR cameras;
- synchronization-TTL integrity for every wired destination;
- exact post-startup DDS transport loss; and
- total, used, free, and percentage-used capacity of the configured recording
  drive.

The completed session remains visible, including its final counters, until the
next task starts. A small summary is stored as
`.tabletop_session_health.json` on the configured recording drive so it also
survives a dashboard restart.

## Operator stop

Click **STOP ROBOTS** once to cancel the active task and both robot trajectory
goals. The button reports `STOPPING...` while the request is pending, becomes
unavailable after the task ends, and does nothing when there is no active task.
If Commander is unavailable it reads `STOP OFFLINE` rather than pretending a
stop can be delivered. Keep the physical emergency stop accessible: browser,
network, ROS, and host failures can all prevent a software button from working.
Remote dashboard access uses noVNC's authenticated TLS endpoint; see the
[usage guide](../getting-started/usage.md) for login and CA setup.

## Reading stream and loss status

`LIVE` means every required component for that row arrived in the preceding
1.5 seconds. Robot rows require pose and wrench data; Robot 1 also requires its
digital-I/O stream. FLIR status uses metadata rather than image subscriptions,
which keeps the monitor lightweight while still proving that the camera driver
is producing frames. EyeLink sample monitoring is enabled only between task
start and task end, so this always-running dashboard does not itself start an
EyeLink recording.

Loss is the exact increase since task start in rosbag's DDS transport-loss
counter for that row. The dashboard no longer estimates lost samples from its
own subscription timestamps: this passive GUI intentionally uses lightweight
best-effort subscriptions and may not process every high-rate message. Use the
post-session integrity report for source-level frame and sample checks.

## TTL rows

Teensy is the reference pulse source. The wired destinations are EyeLink,
Robot 1 digital input 3, the left-back-top FLIR camera, and the bounded
OptiTrack solenoid-marker detector. Robot 2 is intentionally unwired, and the
other five FLIR rows show `N/A` because the sync input goes to only one camera.
TTL health uses a rolling 60-second window. A wired destination remains
`INTACT` as long as at least one Teensy reference pulse in that window has a
matching destination edge within the stream's timing tolerance. It becomes
`MISS` only after a full minute with no matched pulse; during the first minute
without a match it shows `WAITING`. This prevents a momentary dashboard callback
drop from falsely marking a physically present TTL as absent.

## Recording-drive capacity

Capacity is read from `ROS_BAG_DIR` inside the dashboard container. Compose
mounts `ROS_BAG_DIR_MNT` there, so the displayed filesystem is the actual HDD
used for the session rather than the container's root disk. The banner turns
amber at 85% used and red at 95% used.
