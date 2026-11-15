# Configuration

Configuration enters the system at three levels: the Docker **environment**
(`.env`), ROS **launch arguments**, and ROS **parameter files** (YAML). This
page maps each config file to what consumes it; see
[Architecture §4](../architecture.md) for the full parameter-flow diagram.

## Environment (`.env`)

`tt-env-gen` generates `.env` from `.env.example`. It **validates** required
variables and **auto-detects** a few:

- **Auto-detected:** FLIR cameras (`FLIR_DEV_0..N` from `/dev/flir/*` udev
  symlinks) and the PulseAudio socket (`PULSE_*`).
- **Set by you:** serial device paths (currently only `TEENSY_DEV`) and the
  container runtime (`COMMANDER_RUNTIME`, plus `CUDA_VERSION`) are **not**
  auto-detected — edit them to match your hardware. Regenerate with
  `tt-env-gen` (or `tt-env-gen --clean` to start fresh) after any hardware
  change.
- **Derived:** `tt-env-gen` computes `UV_EXTRA` and the `NVIDIA_*` variables
  from `COMMANDER_RUNTIME`, so re-run it after editing that value.

`compose.yaml` reads these for device mounts, runtimes, and volumes.

For sustained six-camera recording, the FLIR publisher and rosbag recorder
load `tabletop_rig/config/fastdds_flir.xml`. It retains UDP compatibility for
the preview processes while increasing the local Fast DDS shared-memory
segment from 512 KiB to 64 MiB and its descriptor queue from 512 to 8192.
Both the current and legacy Fast DDS profile environment-variable names are
set because Jazzy's packaged 2.14 library still recognizes the legacy name.
The profile is scoped to the high-bandwidth FLIR driver and recorder processes;
the six low-bandwidth static-transform publishers retain their default profile
so they do not each reserve an unnecessary 64 MiB segment.

### Required variables (set in `.env.example`)

| Variable | Description | Default |
| --- | --- | --- |
| `NOVNC_DISPLAY` | X11 display number for the noVNC server | `:20.0` |
| `NOVNC_WIDTH` / `NOVNC_HEIGHT` | X11 display width / height (pixels) | `1920` / `1080` |
| `NOVNC_PORT` | Host port serving the authenticated HTTPS/WSS noVNC interface | `8080` |
| `COMMANDER_RUNTIME` | Container runtime for the `commander` and `dev` services — `nvidia` for GPU access, `runc` for CPU only | `runc` |
| `CUDA_VERSION` | CUDA version suffix for PyTorch (must match your GPU driver); required only when `COMMANDER_RUNTIME=nvidia` | `130` |
| `BIND_CONSISTENCY` | Docker bind-mount consistency mode (macOS/Windows only, ignored on Linux) | `cached` |
| `TEENSY_DEV` | Serial device path for the Teensy micro-controller | `/dev/ttyACM0` |
| `FLIR_MAX_DEVS` | Maximum number of FLIR cameras mapped into containers | `6` |

### NAS session recording (opt-in)

Mount the NAS **outside the repository and `.cache`** before starting ROS
drivers. The tested SMB configuration is `vers=3.1.1,cache=none,mfsymlinks`.
`cache=none` avoids multi-second dirty-cache flushes at MCAP file changes;
`mfsymlinks` is required for the recorder/EyeLink `latest` session pointer.
Passwords are prompted locally and are not stored in `.env` or a credentials
file. The mount must be re-established after reboot:

```bash
source setup.bash
tt-nas-mount --source //NAS_ADDRESS/tabletop
tt-recording-storage nas /mnt/tabletop-nas //NAS_ADDRESS/tabletop
docker compose up -d --no-deps --force-recreate eyelink session-dashboard
```

NAS selection saves a private `.env` backup and the previous local bag root.
Recorder MCAP, EyeLink EDF/CSV/integrity reports, dashboard storage statistics,
and exporter inputs share `/mnt/tabletop-nas/sessions` (container `/bags`).
NAS image subscriptions use a **bounded 512-frame best-effort reader queue**
to bridge MCAP rollovers; publisher reliability, raw camera buffers, robot
speed/acceleration, and CPU affinity are unchanged. NAS shutdown gives data
finalization up to 180 seconds of container grace; robot cancellation remains
immediate and does not wait for data transfer.

Connect the NAS network adapter **before** bringing up acquisition drivers.
Fast DDS host identifiers depend on network interfaces; hot-plugging an
adapter can disable shared-memory delivery between old and new processes.
NAS startup checks reject this mismatch rather than automatically restarting
robot controllers. Refresh the acquisition stack only while tasks are stopped.

The host preflight checks the expected SMB share, mount options, and competing
reads using `/proc/fs/cifs/Stats`. Recorder and EyeLink also reject a missing
or incorrect container mount. There is no silent local-directory/HDD fallback.
Do not copy/export/analyze the same share during high-bandwidth recording.
Healthy-link validation does not certify data preservation through NAS power
failure or a disconnected network; use a UPS and qualify multi-hour sessions.

To return to the previous HDD setup while stopped:

```bash
tt-recording-storage hdd
docker compose up -d --no-deps --force-recreate eyelink session-dashboard
```

Existing NAS sessions are not moved or deleted by rollback. The NAS remains
mounted until explicitly unmounted; no `/etc/fstab` entry is installed.
Compose will not auto-create missing recording directories: after reboot,
mount the NAS first, then bring up/recreate its data-consumer containers. This
prevents empty local directories from shadowing a later NAS mount. Service
debug logs stay local to keep network I/O out of controller logging paths;
session data, EDF/CSV, integrity reports, and `/rosout` in MCAP are on the NAS.
`latest` is a Linux `mfsymlinks` pointer; other NAS clients should open the
timestamped session directory directly if they display it as a small file.

Validation on the reference 2.5 Gb/s Synology connection used a passive,
all-stream recording with six 120 Hz cameras, robot telemetry, EyeLink,
OptiTrack, and Teensy data, including a 100 GB MCAP rollover. No task or robot
motion was commanded. Retain a longer operator-run qualification before relying
on a new NAS/network configuration for multi-hour experiments; a successful
short test does not cover network outages or recording while another client
loads the NAS. An unrelated first-sample omission in sample-only EyeLink
EDF-to-CSV conversion was also corrected; original EDF and MCAP samples were
unaffected.

### Optional camera CPU affinity

The camera publisher, recorder, and preview conversion expose optional
taskset-compatible CPU lists in `.env`:

```bash
TABLETOP_FLIR_CPU_SET=0-11
TABLETOP_RECORDER_CPU_SET=12-15,24-31
TABLETOP_PREVIEW_CPU_SET=16-23
```

These example values match the P-core/E-core layout of the reference
32-logical-CPU rig, but they are intentionally **not enabled by default**. A/B
testing on that host increased FLIR loss when affinity was enabled because the
camera driver already consumes roughly five CPU cores and benefited from the
Linux scheduler's full CPU set. Leave the values blank unless measurements on
the target host demonstrate an improvement. The settings do not change the UR
controller's CPU affinity or scheduling policy.

`tt-env-gen` validates that these are present (it does **not** auto-detect
serial device paths or the container runtime — set `TEENSY_DEV` and
`COMMANDER_RUNTIME` to match your hardware).

### Auto-generated variables (by `tt-env-gen`)

`tt-env-gen` automatically detects and configures:

- **FLIR cameras** — detects `/dev/flir/*` udev symlinks and maps them to
  `FLIR_DEV_0..N` (up to `FLIR_MAX_DEVS`).
- **PulseAudio** — detects the PulseAudio socket and configures the `PULSE_*`
  mount variables for audio passthrough (falls back to `/dev/null` if not
  found).
- **GPU variables** — derived from the `COMMANDER_RUNTIME` you set (see below),
  not from probing the host: `UV_EXTRA` and the `NVIDIA_*` variables.

### GPU access (NVIDIA container runtime)

GPU access is **opt-in**: the runtime is not auto-detected, so a fresh `.env`
runs the `commander` and `dev` containers on plain `runc` with CPU-only PyTorch
wheels until you say otherwise. To enable it:

1. Install the [NVIDIA driver](https://www.nvidia.com/en-us/drivers/) and the
   [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
   on the host, so Docker has an `nvidia` runtime to use.
2. Edit `.env` (**not** `.env.example`):

    ```bash
    COMMANDER_RUNTIME=nvidia
    CUDA_VERSION=130          # match your driver; see pyproject.toml for the
                              # available cu* extras (126, 128, 130)
    ```

3. Re-run `tt-env-gen` so the derived variables are refreshed, then recreate the
   containers:

    ```bash
    tt-env-gen
    tt-compose --profile=<sim|real> up --force-recreate
    ```

`tt-env-gen` accepts only `nvidia` or `runc` and errors out on anything else
(or on `nvidia` with an empty `CUDA_VERSION`). Setting `nvidia` **without** the
NVIDIA Container Toolkit installed leaves `.env` valid but makes Docker fail to
start the `commander`/`dev` containers with an unknown-runtime error — see
[Troubleshooting](troubleshooting.md#environment-configuration).

| `COMMANDER_RUNTIME` | `UV_EXTRA` | `NVIDIA_VISIBLE_DEVICES` / `NVIDIA_DRIVER_CAPABILITIES` |
| --- | --- | --- |
| `nvidia` | `--extra cu$CUDA_VERSION` | `all` / `all` |
| `runc` | `--extra cpu` | empty / empty |

!!! warning "Upgrading from an autodetecting `.env`"
    Earlier versions of `tt-env-gen` probed `nvidia-smi` and wrote
    `COMMANDER_RUNTIME` into the *Dynamic Configuration* block at the bottom of
    `.env`. If your `.env` predates this change, the variable is still down
    there holding whatever autodetection last decided. Set the value you want in
    that file (or run `tt-env-gen --clean` to regenerate from the current
    `.env.example` — this discards your other `.env` edits) and re-run
    `tt-env-gen`.

## Parameter files (config → consumer)

| Config | Consumed by | Drives |
| --- | --- | --- |
| `tabletop_rig/config/commander.yaml` | `commander.launch.py` → Commander | all interface parameters |
| `tabletop_rig/config/flir_synchronized.yaml` | `flir_synchronized.launch.py` | camera serials, trigger/chunk settings, poses |
| `tabletop_rig/config/dual_controllers.yaml` | `dual_ur.launch.py` → controller_manager | left/right controller definitions |
| `tabletop_rig/config/optitrack.yaml` | `optitrack.launch.py` | server address, ports, QoS |
| `tabletop_rig/config/rosbag.yaml` | `rosbag.launch.py` | recorded topics/services, bag size |
| `tabletop_rig/config/object_reset/*.yaml` | Commander `reset_object` | reset-motion strategies (drawer/spin) |
| `tabletop_tasks/config/<task>.yaml` | `tasks.launch.py` → `run_tasks` | task class + kwargs + trial generator |
| `tabletop_description/config/*_calibration.yaml` | `dual_rsp.launch.py` | per-arm UR kinematics |
| `tabletop_moveit_config/config/*.yaml` | `commander.launch.py`, `moveit.launch.py` | planners, limits, controllers |

Every config file is now commented inline; open the file to see per-parameter
documentation.

!!! tip "FLIR GenICam node reference"
    The `blackfly_s.yaml` and `flir_synchronized.yaml` config files map ROS
    parameter names to GenICam "node" paths on the camera (e.g.
    `AcquisitionControl/TriggerMode`). The authoritative list of nodes for the
    **BFS-U3-23S3** model used in this rig is the
    [FLIR BFS-U3-23S3 GenICam node reference](https://softwareservices.flir.com/BFS-U3-23S3/latest/Model/public/index.html).
    Node names, allowed values, and availability may differ for other Blackfly S
    variants — consult the corresponding FLIR model page for other cameras.

For a detailed breakdown of every parameter accepted by the `Commander` node and
its interfaces, see [Node & Interface Parameters](parameters.md).

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
