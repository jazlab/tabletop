# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

TableTop is a ROS 2-based robotics platform for controlling Universal Robots UR5e arms in a monkey electrophysiology research rig. The project runs entirely in Docker containers and uses MoveIt 2 for motion planning.

## Common Commands

### Building

All commands are run inside the Docker container (via Dev Container or `docker exec`).

```bash
# Build tabletop packages only (most common)
tt-build

# Build with clean workspace
tt-build --clean

# Build specific packages
tt-build -p <package_name>

# Build all packages including external modules (moveit2, etc.)
tt-build --all

# Build only external modules
tt-build --only-modules
```

### Running

```bash
# Launch the main commander node
tt-launch commander

# Launch the full rig (all hardware interfaces)
tt-launch rig

# Launch tasks
tt-launch tasks

# Launch UR driver
tt-launch ur robot_mode:=real  # or robot_mode:=mock

# Launch visualization
tt-launch rviz
```

### Docker

All Docker interactions use the `tt-compose` wrapper (on the host), which
handles environment generation and project defaults.

```bash
# Build Docker images and full ROS 2 workspace (from host)
tt-compose build

# Start containers using profiles (from host)
tt-compose --profile=sim up        # Simulation with mock hardware
tt-compose --profile=ursim up      # UR Simulator
tt-compose --profile=real up       # Real hardware

# Show container status
tt-compose ps

# Stop and remove containers
tt-compose --profile=sim down

# Launch tasks from the host (spins up a commander container)
tt-launch tasks task:=foraging_ordered robot_mode:=mock
```

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

```
src/
├── tabletop_py/              # ROS-independent Python utilities
│   ├── gaze/                 # Eye-gaze estimation/tracking ML models
│   ├── flic/                 # Flic Bluetooth button client
│   └── utils/                # Common utilities
└── ros/
    ├── tabletop/             # Main ROS 2 packages
    │   ├── tabletop_rig/     # Core rig control (nodes, interfaces)
    │   ├── tabletop_tasks/   # Experiment task definitions
    │   ├── tabletop_interfaces/  # ROS message/service definitions
    │   ├── tabletop_description/ # URDF robot descriptions
    │   ├── tabletop_moveit_config/ # MoveIt planning configurations
    │   └── tabletop_teensy/  # Teensy micro-controller interface
    └── modules/              # External dependencies (git submodules)
        ├── moveit2/          # Custom MoveIt fork
        └── ...
```

### tabletop_rig Architecture

The main control package follows a layered interface pattern:

```
nodes/
├── base.py          # BaseNode: parameter handling, service calls, logging
├── commander.py     # Commander: main orchestrator, coordinates all interfaces
├── eyelink.py       # Eyelink eye tracker node
├── flic.py          # Flic button response time node
├── mock_*.py        # Mock hardware nodes for testing

interfaces/
├── base.py          # BaseInterface: logging mixin, node reference
├── teensy.py        # Arm locks, safety laser, reward, smartglass
├── dashboard.py     # UR robot dashboard (mode changes, recovery)
├── flic.py          # Flic button response time measurement
├── eyelink.py       # Eye tracking integration
├── sound.py         # Audio feedback
└── moveit/
    ├── planning_scene.py      # Collision objects, ACM management
    ├── plan_and_execute.py    # Motion planning with caching
    ├── object_manipulation.py # Pick-and-place state machine
    ├── trajectory_cache.py    # Fuzzy trajectory caching (SQLite)
    ├── requests.py            # Pydantic request models
    └── moveit.py              # Unified MoveIt interface (top-level)
```

**Inheritance hierarchy:**
```
BaseInterface
└── PlanningSceneInterface
    └── PlanAndExecuteInterface
        └── ObjectManipulationInterface
            └── MoveItInterface
```

### Commander Node

The `Commander` class in `nodes/commander.py` is the main entry point that:
- Aggregates all interface objects (MoveIt, Teensy, Flic, Eyelink, Dashboard, Sound)
- Provides high-level experiment control methods (fetch_object, present_object, etc.)
- Handles safety interlocks via Teensy sensor callbacks
- Supports async context manager pattern for setup/cleanup

### Custom Executors

`executors.py` provides asyncio-compatible ROS 2 executors (`AIOExecutor`, `SimpleAIOExecutor`) that bridge ROS 2's callback model with Python's async/await pattern.

## Code Style

- **Python**: Google-style docstrings, ruff formatting (79 char line limit)
- **Target**: Python 3.12
- **Ruff rules**: E4, E7, E9, F, I (errors and imports)
- **Units**: REP 103 (meters, seconds, radians)

## Key Configuration Files

- `compose.yaml` - Docker service definitions
- `setup.bash` - Environment setup (sourced in containers)
- `ruff.toml` - Python linting configuration
- `.pre-commit-config.yaml` - Pre-commit hooks
- `env_files/` - Environment variable files for different configurations

## Environment

- **ROS Distro**: Jazzy
- **Python**: 3.12 (managed via UV)
- **Container user**: `mules` (matches host UID/GID)
- **VNC access**: http://localhost:8080/vnc.html (when using novnc containers)
