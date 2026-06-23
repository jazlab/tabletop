# CLI & Tooling

Sourcing `setup.bash` puts the `tt-*` commands on your `PATH`. They are split
by where they run: `bin/common` (both), and `bin/host` or `bin/container`
depending on context. Under the hood, the host wrappers mostly shell out to
`docker compose` (regenerating `.env` first). See
[Architecture §2.2](../architecture.md) for exactly what each one runs.

## Host commands (`bin/host`)

| Command | Description |
| --- | --- |
| `tt-compose` | Wrapper for `docker compose` with TableTop defaults (generates `.env` if missing) |
| `tt-build` | Build a component (`colcon`/`microros`/`foxglove`) via the privileged `builder` container |
| `tt-env-gen` | Generate `.env` from `.env.example` with hardware detection |
| `tt-attach` | Open a shell in a compose service (a fresh container by default; `-e` to reuse a running one) |
| `tt-flir-reset` | Reset FLIR cameras (reload udev, factory reset, regenerate env) |

!!! note "Host setup"
    Real-hardware host configuration (udev rules, USB buffer size, CPU
    governor, the TableTop network, URCaps) is documented as Ubuntu 24.04
    procedures in [Real Hardware Setup](../getting-started/real-hardware.md),
    not shipped as `tt-*` commands or scripts.

## Container commands (`bin/container`)

| Command | Description |
| --- | --- |
| `tt-build` | Build a component: `colcon` (workspace), `microros` (firmware), or `foxglove` (plugin) |
| `tt-launch` | Launch ROS 2 nodes (commander, rig, tasks, …) |
| `tt-kill-ros` | Kill all running ROS 2 processes |

## Common commands (`bin/common`)

| Command | Description |
| --- | --- |
| `tt-clean` | Clean build artifacts, logs, caches, etc. (by flag) |

## `tt-build` components

`tt-build <component> [options…]`, where `<component>` is one of:

| Component | Builds |
| --- | --- |
| `colcon` | the ROS 2 workspace (colcon) |
| `microros` | the Teensy & Flic micro-controller firmware (PlatformIO) |
| `foxglove` | the Foxglove MoveIt converter plugin (`.foxe` written to `$TABLETOP_DIR`) |

!!! note "First build"
    Run `tt-build colcon --all` once to build the external modules (moveit2,
    etc.) as well; afterwards `tt-build colcon` rebuilds just the tabletop
    packages. There is no `all` component — build `microros` and `foxglove`
    separately when you need them.

### `colcon` options

```text
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
-v, --verbose          Verbose build output
```

### `microros` options

```text
--clean                Clean the micro-ROS build directory first
--no-upload            Build only (no upload)
--compiledb            Generate compile_commands.json for IDE integration
```

### `foxglove` options

```text
-o, --output <path>    Where to write the packaged .foxe (default: $TABLETOP_DIR).
                       A directory keeps the packaged name; a path ending in
                       .foxe renames the plugin to it.
```

## `tt-launch` targets

`tt-launch` runs inside a container — open one with `tt-attach <service>` (or use
the Dev Container), or run it as a one-shot from the host by prefixing it with
`tt-compose run --rm commander`. `tt-launch <target> [ros2 launch args…]`.
Targets: `commander`, `rig`, `tasks`, `dual_ur`, `teensy`, `flic`,
`eyelink`, `flir_no_sync`, `flir_synchronized`, `flir_calibrate`, `optitrack`,
`rosbag`, `rosbag_convert`, `rviz`, `foxglove`, `moveit`, `discovery`.

```bash
tt-launch rig robot_mode:=mock teensy_simulate:=true
tt-launch tasks task:=foraging_ordered robot_mode:=mock
tt-launch commander robot_mode:=real
```

## Python CLI tools

Installed as entry points (available after sourcing `setup.bash`):

| Command | Description |
| --- | --- |
| `tt-gaze-calibrate` | Full gaze calibration pipeline |
| `tt-gaze-preprocess` | Preprocess eye-tracking + marker data |
| `tt-gaze-train` | Train the gaze estimation model |
| `tt-gaze-predict` | Predict gaze on a new session |
| `tt-gaze-visualize` | Visualize calibration data and predictions |
| `tt-flic-scapy` | Flic button client using a raw scapy BLE sniffer |
