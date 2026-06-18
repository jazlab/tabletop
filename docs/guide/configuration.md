# Configuration

Configuration enters the system at three levels: the Docker **environment**
(`.env`), ROS **launch arguments**, and ROS **parameter files** (YAML). This
page maps each config file to what consumes it; see
[Architecture §4](../architecture.md) for the full parameter-flow diagram.

## Environment (`.env`)

`tt-env-gen` generates `.env` from `.env.example`. It **validates** required
variables and **auto-detects** a few:

- **Auto-detected:** NVIDIA GPU (`COMMANDER_RUNTIME`, CUDA vars), FLIR cameras
  (`FLIR_DEV_0..N` from `/dev/flir/*` udev symlinks), and the PulseAudio socket
  (`PULSE_*`).
- **Set by you:** serial device paths (currently only `TEENSY_DEV`) are
  **not** auto-detected — edit them to match your hardware. Regenerate with
  `tt-env-gen` (or `tt-env-gen --clean` to start fresh) after any hardware
  change.

`compose.yaml` reads these for device mounts, runtimes, and volumes.

### Required variables (set in `.env.example`)

| Variable | Description | Default |
| --- | --- | --- |
| `NOVNC_DISPLAY` | X11 display number for the noVNC server | `:20.0` |
| `NOVNC_WIDTH` / `NOVNC_HEIGHT` | X11 display width / height (pixels) | `1920` / `1080` |
| `NOVNC_PORT` | Localhost port serving the noVNC interface | `8080` |
| `CUDA_VERSION` | CUDA version suffix for PyTorch (must match your GPU driver) | `130` |
| `BIND_CONSISTENCY` | Docker bind-mount consistency mode | `cached` |
| `TEENSY_DEV` | Serial device path for the Teensy micro-controller | `/dev/ttyACM0` |
| `FLIR_MAX_DEVS` | Maximum number of FLIR cameras mapped into containers | `6` |

`tt-env-gen` validates that these are present (it does **not** auto-detect
serial device paths — set `TEENSY_DEV` to match your hardware).

### Auto-generated variables (by `tt-env-gen`)

`tt-env-gen` automatically detects and configures:

- **NVIDIA GPU** — detects `nvidia-smi`; sets `COMMANDER_RUNTIME=nvidia` and the
  `NVIDIA_*` / CUDA variables (clears them if no GPU is present).
- **FLIR cameras** — detects `/dev/flir/*` udev symlinks and maps them to
  `FLIR_DEV_0..N` (up to `FLIR_MAX_DEVS`).
- **PulseAudio** — detects the PulseAudio socket and configures the `PULSE_*`
  mount variables for audio passthrough (falls back to `/dev/null` if not
  found).

## Parameter files (config → consumer)

| Config | Consumed by | Drives |
| --- | --- | --- |
| `tabletop_rig/config/commander.yaml` | `commander.launch.py` → Commander | all interface parameters |
| `tabletop_rig/config/flir_synchronized.yaml` | `flir_synchronized.launch.py` | camera serials, trigger/chunk settings, poses |
| `tabletop_rig/config/dual_controllers.yaml` | `dual_ur.launch.py` → controller_manager | left/right controller definitions |
| `tabletop_rig/config/update_rate.yaml` | ur/dual_ur/multi_ur launch | ros2_control update rate (Hz) |
| `tabletop_rig/config/optitrack.yaml` | `optitrack.launch.py` | server address, ports, QoS |
| `tabletop_rig/config/rosbag.yaml` | `rosbag.launch.py` | recorded topics/services, bag size |
| `tabletop_rig/config/object_reset/*.yaml` | Commander `reset_object` | reset-motion strategies (drawer/spin) |
| `tabletop_tasks/config/<task>.yaml` | `tasks.launch.py` → `run_tasks` | task class + kwargs + trial generator |
| `tabletop_description/config/*_calibration.yaml` | `(dual_)rsp.launch.py` | per-arm UR kinematics |
| `tabletop_moveit_config/config/*.yaml` | `commander.launch.py`, `moveit.launch.py` | planners, limits, controllers |

Every config file is now commented inline; open the file to see per-parameter
documentation.

## The common / override pattern

`commander.yaml` is the master parameter file for the `Commander` node. Its
interface sections resolve via `BaseInterface.param(name)`, which looks up
`<iface_prefix>.<name>` and falls back to `common_<kind>_interface.<name>`. For
example, `left_ur_interface.namespace` overrides the shared
`common_ur_interface.*`. The same common/override idiom appears in
`flir_synchronized.yaml` (`camera_params_common` vs. `camera_params`) and task
configs.

At launch, `commander.launch.py` merges `commander.yaml` with a per-session
`/tmp/commander_overrides.yaml` (e.g. `robot_mode`, `initial_object`).

!!! tip "MoveIt configs"
    Files under `tabletop_moveit_config/config/` (OMPL, Pilz, CHOMP, STOMP,
    kinematics, joint limits, controllers) follow standard MoveIt conventions
    and are auto-discovered by `MoveItConfigsBuilder`. Each carries a header
    comment describing its role; refer to the
    [MoveIt docs](https://moveit.picknik.ai/) for the individual parameters.
