# Troubleshooting

Battle-tested notes for the current command set. For a symptom →
starting-point table, see
[Architecture §6](../architecture.md). Known code/config bugs are tracked in
[Known Issues](../known-issues.md).

## Quick reference

```bash
# Host
source setup.bash                          # set up environment (add to ~/.bashrc)
tt-env-gen                                  # (re)generate .env after hardware changes
tt-compose --profile='*' pull              # pull the prebuilt Docker images (all services)
tt-build colcon --all                      # build the full workspace (modules incl.) — first build
tt-compose --profile=sim up                # start a simulation session

# Inside a container (or Dev Container)
tt-build colcon                            # build tabletop packages
tt-build colcon --clean-tabletop           # clean rebuild
tt-launch commander robot_mode:=mock       # commander only (rig comes from the compose profile)
tt-launch tasks task:=foraging_ordered     # run a task (starts a commander too)
```

## Docker

- **Containers rebuilding independently / stale layers.** Stop everything
  (`tt-compose --profile=<p> down`, or `docker container stop $(docker ps -q)`),
  then prune project artifacts with `docker system prune -af` (optionally
  `--volumes`), re-pull the images with `tt-compose --profile='*' pull`, and
  rebuild the workspace with `tt-build colcon --all`. Be aware the prune clears
  build cache for *all* Docker projects on the machine.

## Build

- **Computer freezes while building.** You're out of memory — too many packages
  × workers building at once. Limit parallelism:

    ```bash
    tt-build colcon --workers 1   # single-threaded (slow but safe)
    tt-build colcon --workers 2   # compromise
    ```

## Editor (Dev Container)

- **`Unable to connect to VS Code server: connect ENOENT`** when running
  `code <path>` / `cursor <path>` inside the Dev Container: open a new terminal
  and run it again.
- **Debugging:** use the VS Code debugger (launch configs are already set up) —
  you don't need `gdb -ex` or a debug build. If something is placed (or absent)
  for a non-obvious reason, it was probably deliberate; check git history before
  "fixing" it.

## ROS

- **Visualize the URDF:**

    ```bash
    ros2 launch tabletop_description dual_view_robot.launch.py
    ```

- **Mock-hardware demo:** there is no single `rig` launch anymore — bring up the
  `sim` profile (mock UR, teensy-sim, flic-sim, eyelink-sim, rviz) and run a task
  on top:

    ```bash
    tt-compose --profile=sim up
    tt-compose run --rm commander tt-launch tasks task:=foraging_ordered
    ```

    To launch one mock subsystem on its own instead, use its target directly,
    e.g. `tt-launch teensy simulate:=true` or `tt-launch flic simulate:=true`.

- **Commander executor.** Don't run the Commander on the `AIOExecutor` — its
  services intermittently stop responding. The Commander must use the
  thread-based executor, and the rclpy executor must run in a *separate thread*
  from the asyncio loop. See [Architecture §5.2](../architecture.md) and the
  signal-handling notes below.

## Commander / tasks

- **Robot is holding a grid object at trial start.** Either tell the commander
  which grid index it holds — `tt-launch tasks initial_object:=5,0 …` — or place
  the object back manually before starting.

## Flic buttons

The current `flic` node sniffs BLE directly with scapy; the old `flicd` daemon
container has been **removed** (its definition is preserved in
`deprecated/compose-services.yaml`). Most remaining pain is host Bluetooth.

- **`No such device` on startup / no `/flic` node in the ROS graph.** The host
  has no `hci0`: the Bluetooth hardware is present but the kernel drivers never
  initialized it, so there is no controller for the node to open. Check the host
  first — this is not a container problem:

    ```bash
    ls /sys/class/bluetooth/     # empty means no adapter; expect hci0
    ```

    If it is empty, load the drivers (`btusb` plus your vendor module —
    `btintel` for Intel AX2xx, `btrtl`/`btbcm`/`btmtk` for others) and restart
    the container:

    ```bash
    sudo modprobe btusb
    sudo modprobe btintel
    ls /sys/class/bluetooth/                 # hci0 should now appear
    tt-compose --profile=real restart flic
    ```

    Full walkthrough, including how to make it stick across reboots:
    [Real Hardware → Bluetooth adapter](../getting-started/real-hardware.md#bluetooth-adapter-flic-buttons).

- **Free the Bluetooth adapter** (the host BlueZ stack can hold it). This is the
  *other* failure mode: `hci0` exists, but something else owns it — the node
  takes exclusive HCI user-channel access and cannot share:

    ```bash
    sudo systemctl stop bluetooth
    sudo systemctl disable bluetooth     # and set AutoEnable=false in
                                         # /etc/bluetooth/main.conf to keep it off
    ```

- **Bluetooth device not found / won't connect.** Restart the machine (sometimes
  twice). The `flic` container needs the `NET_ADMIN` capability for raw packet
  capture (already set in `compose.yaml`).

- **Last-resort Bluetooth incantations:**

    ```bash
    sudo modprobe -r btusb && sudo modprobe btusb   # reload the driver
    rfkill unblock bluetooth                          # unblock
    hciconfig hci0 reset                              # reset adapter
    btmgmt info                                       # adapter info
    ```

- **FlicPiano.** For stress relief, the retired `deprecated/flic-button/piano.py`
  plays notes from button presses (needs the `mingus` package from the `dev`
  dependency group). It targets the old `flicd`-based client; see
  `deprecated/README.md`.

## Teensy

- **After changing firmware,** make sure the micro-ROS `executor` is initialized
  with the correct number of handles (and `colcon.meta` matches), then rebuild:

    ```bash
    tt-build microros
    ```

- **Serial debugging:** attach to the device with PlatformIO from a container
  that has it mounted, e.g. `pio device monitor -p "$TEENSY_DEV"`.
- **Device not mounted in Docker:** regenerate `.env` after plugging it in so
  the device path is captured: `tt-env-gen`. When in doubt, unplug and replug.

## Robot (UR5e)

- **Calibration is saved on the robot.** Re-running the extraction does not
  change a stored calibration. The per-arm `*_calibration.yaml` files in
  `tabletop_description/config/` are machine-generated — don't hand-edit them.
- **"Joint … from the starting state is outside bounds".** The arm was moved
  into a configuration outside the controller's valid range. Manually move it
  back into bounds (you may need to spin the joint fully around).
- **Robot pose looks off relative to the rig.** The ground is a reliable
  reference point.
- **Protective stops during object resets.** Tune the reset request's
  `velocity_scaling_factor` / `acceleration_scaling_factor` in the relevant
  `object_reset/*.yaml` to soften jerky motion.

## Environment & configuration

- **`tt-*` command not found** → you didn't `source setup.bash` (add it to
  `~/.bashrc`).
- **Docker can't see a device** (FLIR/Teensy) → `tt-env-gen --clean`, then check
  `.env` and `tt-compose ps`.
- **`.env` missing/corrupt** → `tt-env-gen --clean` regenerates it from
  `.env.example`.
- **No GPU inside the commander / Dev Container** (`nvidia-smi` missing, torch
  reports no CUDA) → GPU access is opt-in and **not** autodetected. Set
  `COMMANDER_RUNTIME=nvidia` (and a `CUDA_VERSION` your driver supports) in
  `.env`, re-run `tt-env-gen` so `UV_EXTRA`/`NVIDIA_*` are regenerated, then
  recreate the container (`tt-compose --profile=<sim|real> up
  --force-recreate`) — a running container keeps the runtime it started with.
  See [Configuration](configuration.md#gpu-access-nvidia-container-runtime).
- **`unknown or invalid runtime name: nvidia`** → `COMMANDER_RUNTIME=nvidia`
  but the [NVIDIA Container
  Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
  isn't installed/registered with Docker. Install it (`docker info | grep -i
  runtimes` should list `nvidia`) or set `COMMANDER_RUNTIME=runc` and re-run
  `tt-env-gen`.
- **`COMMANDER_RUNTIME must be set to 'runc' or 'nvidia'`** (or the matching
  `CUDA_VERSION` error) → `tt-env-gen` validating `.env`; fix the value in
  `.env`, not in `.env.example`.
- Prefer the `tt-compose` wrapper over raw `docker compose` — it sets up the
  environment for you.

## Gaze estimation

Run the pipeline steps individually for debugging:

```bash
tt-gaze-preprocess -d /path/to/session --visualize
tt-gaze-train -d /path/to/session
tt-gaze-visualize -d /path/to/session
```

If training stalls or loss won't drop, check that preprocessing produced no
NaNs, that EyeLink and OptiTrack data are synchronized, and that the config
hyperparameters are sane. To convert an EyeLink EDF to CSV (needs `edf2asc` from
the EyeLink Developers Kit):

```bash
python -m tabletop_py.gaze.edf recording.edf -o recording.csv
```

## FLIR cameras

Make sure the host is configured for the cameras (USB buffer size and
`/dev/flir/*` udev symlinks) per
[Real Hardware Setup → Host configuration](../getting-started/real-hardware.md#host-configuration-ubuntu-2404),
then capture the device paths:

```bash
tt-env-gen                               # capture FLIR_DEV_* into .env
```

USB device numbers change whenever a camera or its hub is unplugged, power
cycled, or reset. Regenerating `.env` does not update the device mounts of an
existing container, and `docker compose restart flir` reuses those stale
mounts. If the driver reports that a camera was removed, previews stop, or
Docker reports `error gathering device information ... no such file or
directory`, refresh the paths and **recreate** the FLIR container:

```bash
cd /path/to/tabletop
tt-env-gen
docker compose up -d --force-recreate --no-deps flir
```

Use `up --force-recreate`, not `restart`, after any FLIR USB topology change.
The persistent `/dev/flir/<serial>` links let `tt-env-gen` find each camera's
new raw `/dev/bus/usb/...` path; recreating the container applies those paths.

Only if a clean recreation still fails should you use `tt-flir-reset`, which
reloads udev, factory-resets the cameras, and regenerates the environment.
Check synchronization afterward with
`ros2 run tabletop_rig system_check`.

## Performance

For real-time control, pin the CPU to the `performance` governor (see
[Real Hardware Setup → CPU governor](../getting-started/real-hardware.md#cpu-governor-real-time-control)).
Build/clean helpers:

```bash
tt-build colcon --workers 1|2                      # avoid build OOM
tt-clean --tabletop-colcon                         # clean tabletop build
tt-clean --all-colcon                              # clean all colcon builds
tt-clean --logs                                    # clean logs
tt-clean --tabletop-cache                          # clean planning/trajectory caches
```

## Architecture deep-dives

### Commander signal handling

Clean Commander shutdown was hard-won:

- **MoveItPy** installs its own signal handler — construct it asking it *not* to.
- **rclpy** installs its own handler too — call `rclpy.init` after `asyncio.run`,
  or with `SignalHandlerOptions.NO`.
- `moveit_py` `execute_and_wait` / `stop_execution` **hang** if the UR
  controllers stop at the same time as the Commander. This happens when you
  launch the driver in the same process (`rig`/`tasks` with `ur_launch:=true`).
  **Run the UR driver in its own container/process.**

### Planning (smooth pursuit / ConcatPlanRequest)

A long freeze during TOTG post-processing usually means too many waypoints in a
`ConcatPlanRequest`. If you need many waypoints, disable
`post_process_after_concat`.

### Sound

No audio in the Dev Container usually means the PulseAudio port changed. Rebuild
the Dev Container, or run from the host via `tt-launch` (which re-detects the
socket through `tt-env-gen`).
