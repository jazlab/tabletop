# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

TableTop is a ROS 2-based robotics platform for controlling Universal Robots UR5e arms in a monkey electrophysiology research rig. The project runs entirely in Docker containers and uses MoveIt 2 for motion planning.

## Common Commands

The `tt-*` commands are added to `PATH` by `setup.bash`. They are split into
`bin/host` (run on the host, mostly thin wrappers around
`tt-compose run --rm <service> …`), `bin/container` (run inside a container),
and `bin/common` (both). `tt-build` exists on both sides (the host wrapper runs
the real build in the privileged `builder` container). `tt-launch` is
container-only: run it from inside a container (the Dev Container, or a shell
opened with `tt-attach`) or as a one-shot from the host via
`tt-compose run --rm commander tt-launch …`.

### Building

`tt-build` takes a required component (`colcon`, `microros`, or `foxglove`):

```bash
# Build the full workspace incl. external modules (moveit2) — run this first
tt-build colcon --all

# Build tabletop packages only (most common, after the first build)
tt-build colcon

# Clean tabletop packages first, then build
tt-build colcon -c     # --clean-tabletop  (--clean-all wipes the whole ws)

# Build the given packages and their dependencies
tt-build colcon -p <package_name>

# Build all colcon packages including external modules (moveit2, etc.)
tt-build colcon -a     # --all

# Build only external modules
tt-build colcon -m     # --only-modules

# Build/upload the Teensy firmware (PlatformIO)
tt-build microros

# Package the Foxglove MoveIt plugin (.foxe written to $TABLETOP_DIR by
# default, or to the -o/--output path)
tt-build foxglove
```

Other useful colcon flags: `-w/--workers N` (low-memory systems),
`--build-debug`, `--clang`, `-v/--verbose`. Run `tt-build colcon --help` for the
full set.

### Running

`tt-launch <target> [ros2 launch args…]`, run inside a container or via
`tt-compose run --rm commander tt-launch …` from the host. Each target maps to a
single launch file (`tt-launch` is now purely a `ros2 launch` wrapper); there is
no longer an aggregate `rig` target (the old `rig.launch.py` was retired to
`deprecated/` — bring the rig up with compose profiles instead, see Docker
below). Targets:
`commander`, `tasks`, `dual_ur`, `teensy`, `flic`, `eyelink`,
`flir_no_sync`, `flir_synchronized`, `flir_calibrate`, `optitrack`, `rosbag`,
`rviz`, `foxglove`, `moveit`.

```bash
# Launch the main commander node
tt-launch commander robot_mode:=mock   # mock | real

# Launch tasks (spins a commander, optionally with rosbag recording, and
# runs the task on top of an already-running rig started via a compose profile)
tt-launch tasks task:=foraging_ordered robot_mode:=mock

# Launch the dual UR driver stack (both arms)
tt-launch dual_ur robot_mode:=real     # or robot_mode:=mock

# Launch a single hardware subsystem (each runs in its own compose service)
tt-launch teensy simulate:=true

# Launch visualization (renders to the noVNC display)
tt-launch rviz
```

> The full rig is no longer launched from one target. Each subsystem
> (`dual_ur`, `teensy`, `flic`, `eyelink`, `optitrack`, `flir*`, `rviz`) runs in
> its own compose service; `tt-compose --profile=sim up` (or `=real`) starts the
> whole set, then `tt-launch tasks …` runs a commander on top.

### Docker

All Docker interactions use the `tt-compose` wrapper (on the host), which
handles environment generation and project defaults.

```bash
# Pull the prebuilt Docker images, then build the workspace (from host).
# `--profile='*'` selects every service so all images are pulled.
tt-compose --profile='*' pull
tt-build colcon --all

# Start containers using profiles (from host)
tt-compose --profile=sim up        # Simulation with mock hardware
tt-compose --profile=real up       # Real hardware

# Show container status
tt-compose ps

# Stop and remove containers
tt-compose --profile=sim down

# Launch tasks from the host (spins up a temporary commander container)
tt-compose run --rm commander tt-launch tasks task:=foraging_ordered robot_mode:=mock
```

The user-facing profiles are `sim` and `real` (the `real` profile
includes the FLIR cameras). Other profiles exist for narrower jobs: `builder`
(the privileged build container used by `tt-build`), `commander` (temporary
container spun up to run `tt-launch`), `dev` (the Dev Container), and `template`
(the `ros-base` extends-only base image, never run directly). `tt-env-gen`
regenerates `.env` (from `.env.example`) whenever hardware changes — device
paths are baked into it.

### Testing

```bash
# Run Python tests
pytest tests/

# Run a single test file
pytest tests/gaze_estimation_test.py

# Run ROS package tests
colcon test --packages-select <package_name>
```

### Linting

Pre-commit hooks run automatically on commit. Manual execution:

```bash
# Run all pre-commit hooks
pre-commit run --all-files

# Run ruff only
ruff check --fix .
ruff format .
```

## Architecture

### Package Structure

```text
src/
├── tabletop_py/              # ROS-independent Python utilities (COLCON_IGNOREd,
│   └── tabletop_py/          #   pure Python, built by uv — never imports ROS)
│       ├── gaze/             # Eye-gaze estimation/tracking ML models + CLI tools
│       ├── flic/             # Flic Bluetooth button client
│       └── utils/            # Common utilities (yaml/dict/mesh helpers)
├── microros/                 # Teensy firmware (COLCON_IGNOREd; built with
│   └── tabletop_teensy/      #   PlatformIO via tt-build microros, NOT colcon —
│                             #   implements tabletop_interfaces in C)
├── foxglove_moveit_msg_converter/   # Foxglove MoveIt plugin source (git submodule,
│                             #   COLCON_IGNOREd; packaged via tt-build foxglove)
└── ros/
    ├── tabletop/             # Main ROS 2 packages
    │   ├── tabletop_rig/         # Core rig control (nodes, interfaces)
    │   ├── tabletop_tasks/       # Experiment task definitions
    │   ├── tabletop_interfaces/  # ROS message/service/action definitions
    │   ├── tabletop_description/ # URDF robot descriptions + UR calibration
    │   ├── tabletop_moveit_config/ # MoveIt planning configurations
    │   └── tabletop_unbag/       # C++ ament_cmake pkg: `unbag` bag→CSV/image
    │                             #   exporter (offline analysis tool; see below)
    └── modules/              # External dependencies (git submodules)
        ├── moveit2/          # Custom MoveIt fork
        ├── flir_camera_driver/   # Spinnaker-based FLIR driver
        ├── mocap4r2*/        # OptiTrack motion-capture drivers
        ├── micro-ROS-Agent/  # micro-ROS bridge for the Teensy
        ├── trac_ik/          # IK solver
        └── ...               # image_transport, …
```

`tabletop_py`, `src/microros`, and `src/foxglove_moveit_msg_converter` all carry a
`COLCON_IGNORE` marker, so `colcon`/`tt-build` skip them: `tabletop_py` is
installed by `uv` and imported directly, `src/microros/tabletop_teensy` is
firmware flashed by PlatformIO (`tt-build microros`), and
`src/foxglove_moveit_msg_converter` is the Foxglove plugin source packaged by
`tt-build foxglove`. The firmware and `tabletop_py` still matter to the ROS
side — `tabletop_rig` wraps `tabletop_py`, and the firmware implements
`tabletop_interfaces` services (`SetArmLock`, `SetReward`, `SetSolenoid`,
`SetSmartglass`, `Ping`) in C.

### Bag export: `tabletop_unbag` (and the legacy Python converter)

- `tabletop_unbag` — the **current** bag exporter: a standalone C++ `ament_cmake`
  package (built by the normal `tt-build colcon`, **not** COLCON_IGNOREd)
  providing the dependency-light `unbag` executable:
  `ros2 run tabletop_unbag unbag BAG_DIR`. Generic message flattening to
  per-topic CSV plus image-topic decode, but multithreaded, streaming (bounded
  memory), and resumable, with no Python/pandas/rclpy at runtime. CSV output is
  byte-for-byte identical to the old Python version (one intentional difference:
  fixed-size primitive arrays like `CameraInfo.k` are expanded to indexed
  columns). It is an **offline analysis tool**, not a runtime node. Full
  details: `tabletop_unbag/README.md`.
- `tabletop_rig/tabletop_rig/utils/rosbag.py` (`rosbag_to_csv`) — the **legacy**
  Python converter it was ported from. Depends on rclpy/pandas. Its `tt-launch`
  target was removed; it survives only as the `ros2 run tabletop_rig
  rosbag_to_csv` entry point and the import used by the gaze-calibration scripts.
  Slated for retirement in Wave 2 (move to `deprecated/`; gaze calibration to
  call `tabletop_unbag` instead) — see `docs/fix-plan.md`. Prefer `unbag`.

### tabletop_rig Architecture

The main control package follows a layered interface pattern:

```text
nodes/                       # ROS 2 node classes (exported from nodes/__init__.py)
├── base.py          # BaseNode: parameter handling, service calls, logging
├── commander.py     # Commander: main orchestrator, coordinates all interfaces
├── eyelink.py       # Eyelink eye tracker node
├── flic.py          # Flic button response time node
├── system_check.py  # SystemCheck: live diagnostics (e.g. FLIR sync check)
└── mock_teensy.py, mock_dashboard_client.py, mock_robot_state_helper.py
                     # Mock hardware nodes/helpers for sim & mock modes

interfaces/
├── base.py          # BaseInterface: logging mixin, node reference, param() lookup
├── teensy.py        # Arm locks, safety laser, reward, smartglass, sync pulse
├── ur.py            # URInterface: UR dashboard, mode changes, recovery state machine
├── flic.py          # Flic button response time measurement
├── eyelink.py       # Eye tracking integration
├── sound.py         # Audio feedback (fluidsynth)
└── moveit/
    ├── moveit.py              # MoveItInterface: unified top-level interface
    ├── object_manipulation.py # Pick/present/return state machine (ManipulationState)
    ├── plan_and_execute.py    # Motion planning, consults the trajectory cache
    ├── trajectory_cache.py    # Cache facade (selects backend)
    ├── trajectory_cache_kdtree.py # KD-tree fuzzy nearest-neighbour lookup
    ├── trajectory_cache_lmdb.py   # LMDB-backed persistent store
    └── requests.py            # Pydantic request models
```

The planning-scene/ACM duties that once lived in a standalone
`planning_scene.py` now live inside `MoveItInterface` (see relationships below).

**Interface relationships** (composition, not a single linear chain — every
interface extends `BaseInterface`):

```text
BaseInterface
├── MoveItInterface            # MoveItPy + planning scene, collision objects, ACM
├── PlanAndExecuteInterface    # plan/execute + trajectory cache
│   └── ObjectManipulationInterface   # pick/present/return state machine;
│         holds a reference to the shared MoveItInterface (composition)
├── URInterface               # UR dashboard / recovery, one per arm
├── TeensyInterface, FlicInterface, EyelinkInterface, SoundInterface
└── ManipulationContextManager  # per-arm bundle: a URInterface + an
                                 #   ObjectManipulationInterface, sharing MoveIt
```

> Note: an older `PlanningSceneInterface` class no longer exists — its
> planning-scene/ACM duties were absorbed into `MoveItInterface`. If you find a
> stray reference to the old `PlanningSceneInterface → … → MoveItInterface`
> chain, trust the class declarations above.

### Commander Node

The `Commander` class in `nodes/commander.py` is the main entry point that:

- Aggregates the shared interface objects (`MoveItInterface`, `TeensyInterface`,
  `FlicInterface`, `EyelinkInterface`, `SoundInterface`) plus one
  `ManipulationContextManager` per arm (each owning a `URInterface` and an
  `ObjectManipulationInterface`). Arms are defined by the `robot_interface_names`
  parameter, not hardcoded.
- Provides high-level experiment control methods (fetch_object, present_object, etc.)
- Handles safety interlocks via the Teensy sensor callback (`_teensy_sensor_callback`),
  which stops execution when `safe_to_execute` goes false during a presented motion
- Supports the async context manager pattern for setup/cleanup
- Runs the task coroutine injected at launch (`run_tasks` from `tabletop_tasks`)

### Custom Executors

`executors.py` provides asyncio-compatible ROS 2 executors that bridge ROS 2's
callback model with Python's async/await pattern. The public `AIOExecutor`
aliases the optimized variant (`_AIOExecutorOptimized`); nodes that need robust
error reporting use `ErrorHandlingMultiThreadedExecutor`. Per the hard-won notes
in `docs/guide/troubleshooting.md` (Commander signal handling), the Commander
must use the thread-based executor and the UR driver must run in a separate
process from the Commander.

## Code Style

- **Python**: Google-style docstrings, ruff formatting (79 char line limit)
- **Target**: Python 3.12
- **Ruff rules**: E4, E7, E9, F, I (errors and imports)
- **Units**: REP 103 (meters, seconds, radians)

## Key Configuration Files

- `compose.yaml` - Docker service definitions (profiles, devices, mounts)
- `.env.example` → `.env` - environment variables; `.env` is **generated** by
  `tt-env-gen` (never edit `.env` by hand for auto-detected values)
- `setup.bash` - single source of environment truth, sourced by every script and
  container entrypoint; detects host-vs-container via `TABLETOP_CONTAINER` and
  selects the uv venv (`.venv` host / `.venv.container` container)
- `ruff.toml` - Python linting configuration
- `.pre-commit-config.yaml` - Pre-commit hooks
- `bin/` - the `tt-*` commands (`common/`, `host/`, `container/`)
- Real-hardware host configuration (udev/USB/CPU/network/URCaps) is documented
  as Ubuntu 24.04 procedures in `docs/getting-started/real-hardware.md` (the old
  `scripts/configure/` shell scripts were removed)
- ROS parameter files live in each package's `config/`; see
  `docs/guide/configuration.md` for the config → consumer map

## Environment

- **ROS Distro**: Jazzy
- **Python**: 3.12 (managed via uv)
- **Container user**: `mules` (matches host UID/GID)
- **VNC access**: <http://localhost>:<NOVNC_PORT>/vnc.html (when using novnc containers)
- All containers share `network_mode: host` and `ipc: host`, so every ROS node
  sees every other node regardless of which container it runs in

## Further Reading

The `docs/` tree (published at <https://jazlab.github.io/tabletop/>) is the
canonical source for setup, usage, and troubleshooting; `README.md` is now just
a high-level overview that points here. Setup/usage live under
`docs/getting-started/` (`setup.md`, `real-hardware.md`, `usage.md`), with the
design rationale in `docs/design-choices.md`.

For a deeper conceptual map (runtime topic/service graph, launch hierarchy,
parameter flow, and "where to look when X breaks"), see `docs/architecture.md`.
Other useful docs: `docs/known-issues.md` (review findings),
`docs/guide/troubleshooting.md` (battle-tested troubleshooting), and the other
guides under `docs/guide/`.

Superseded subsystems (the old `flicd`-based Flic stack, the UR simulator, the
Flic ESP32 firmware, unused MoveIt configs, the ROS graph tooling) are archived
under `deprecated/` — see `deprecated/README.md`. Nothing there is built or on a
code path; it is reference-only.
