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

The Teensy firmware publishes `teensy/sensor` (a `TeensySensor` message) at
100 Hz. The `TeensyInterface` inside the Commander gates every robot motion
through `safe_to_execute`, which requires the safety conditions to hold
continuously for a configured time and the sensor message to be fresh.

!!! warning "Arm-lock check currently disabled"
    `safe_to_execute` presently enforces **only** the safety laser
    (`is_safety_laser_broken`). The arm-lock check (both arms seated in the
    restraints) is commented out in `interfaces/teensy.py:_msg_safe_to_execute`
    and is **not** enforced, even though the firmware still publishes the
    arm-lock state. See [Known Issues](../known-issues.md) (#17); re-enabling it
    is a safety-relevant decision.

## Firmware

The Teensy and Flic micro-controller firmware live under
`tabletop_micro/` and are built with PlatformIO (not colcon):

```bash
tt-build microros
```

The Teensy firmware implements the `tabletop_interfaces` services (`SetArmLock`,
`SetReward`, `SetSolenoid`, `SetSmartglass`, `Ping`) and publishes
`TeensySensor`. When changing it, keep the micro-ROS executor handle counts (and
`colcon.meta`) in sync with the number of entities. When in doubt, unplug and
replug, then re-run `tt-env-gen` so Docker re-mounts the device.
