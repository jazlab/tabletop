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

MoveIt runs **in-process** inside the Commander (`moveit_py`), not as a separate
`move_group` node — planning is in-process; execution is dispatched to the UR
`controller_manager` action servers.

## Hardware synchronization

The Teensy emits a hardware trigger pulse wired to the FLIR cameras' input line,
so all cameras expose simultaneously; the synchronized driver stamps grouped
frames identically and uses the FrameID chunk as the sync key. Verify sync with:

```bash
ros2 run tabletop_rig system_check
```

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
