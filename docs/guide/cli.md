# CLI & Tooling

Sourcing `setup.bash` puts the `tt-*` commands on your `PATH`. They are split
by where they run: `bin/common` (both), and `bin/host` or `bin/container`
depending on context. Under the hood, the host wrappers mostly shell out to
`tt-compose run --rm <service> …`. See
[Architecture §2.2](../architecture.md) for exactly what each one runs.

## Host commands (`bin/host`)

| Command | Description |
|---|---|
| `tt-compose` | Wrapper for `docker compose` with TableTop defaults (generates `.env` if missing) |
| `tt-build` | Build the ROS 2 workspace via the `ros-base` container |
| `tt-launch` | Launch ROS 2 nodes via a temporary `commander` container |
| `tt-env-gen` | Generate `.env` from `.env.example` with hardware detection |
| `tt-dev-attach` | Open a shell in a running container (starting it if needed) |
| `tt-flir-reset` | Reset FLIR cameras (reload udev, factory reset, regenerate env) |
| `tt-microros-build` | Build/upload Teensy & Flic firmware via the `microros-builder` container |

!!! note "Host setup scripts"
    udev rules, USB buffer size, CPU scaling, and the robot network are now
    plain scripts under `scripts/configure/`, run by path (not `tt-*`
    commands), because they make persistent privileged changes to the host.

## Container commands (`bin/container`)

| Command | Description |
|---|---|
| `tt-build` | Build ROS 2 packages with colcon |
| `tt-launch` | Launch ROS 2 nodes (commander, rig, tasks, …) |
| `tt-create-graph` | Generate the ROS 2 node/topic graph (`docs/graph.md`) |
| `tt-kill-ros` | Kill all running ROS 2 processes |
| `tt-microros-build` | Build/upload firmware via PlatformIO |

## Common commands (`bin/common`)

| Command | Description |
|---|---|
| `tt-clean` | Clean build artifacts, logs, caches, etc. (by flag) |
| `tt-robot-scp` | Copy URCaps (`ur_robot/programs/*.urcap`) to the physical robot |

## `tt-build` options

```
-c, --clean-tabletop   Clean tabletop packages before building
--clean-all            Clean the entire workspace before building
--clean-cmake          Clear CMake caches before building
-a, --all              Build all packages (tabletop + modules, e.g. moveit2)
-p, --packages-up-to   Build the given packages and their dependencies
-m, --only-modules     Build only external modules
-w, --workers N        Limit parallel workers (low-memory systems)
--build-debug          Build with debug symbols (default: release)
--clang                Use clang (default: gcc)
--linker NAME          Linker to use (default: mold)
--foxglove             Also build the Foxglove MoveIt message converter
-v, --verbose          Verbose build output
```

## `tt-launch` targets

`tt-launch <target> [ros2 launch args…]`. Targets: `commander`, `rig`, `tasks`,
`ur`, `dual_ur`, `teensy`, `flic`, `eyelink`, `flir`, `flir_synchronized`,
`flir_calibrate`, `optitrack`, `rosbag`, `rosbag_convert`, `rviz`, `foxglove`,
`moveit`, `discovery`.

```bash
tt-launch rig robot_mode:=mock teensy_simulate:=true
tt-launch tasks task:=foraging_ordered robot_mode:=ursim
tt-launch commander robot_mode:=real
```

## Python CLI tools

Installed as entry points (available after sourcing `setup.bash`):

| Command | Description |
|---|---|
| `tt-gaze-calibrate` | Full gaze calibration pipeline |
| `tt-gaze-preprocess` | Preprocess eye-tracking + marker data |
| `tt-gaze-train` | Train the gaze estimation model |
| `tt-gaze-predict` | Predict gaze on a new session |
| `tt-gaze-visualize` | Visualize calibration data and predictions |
| `tt-flic-client` | Flic button client (Flic SDK protocol) |
| `tt-flic-scapy` | Flic button client using a raw scapy BLE sniffer |
