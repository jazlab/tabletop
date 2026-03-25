# TableTop

## Description

TableTop is a meta-package for the TableTop project, containing multiple ROS 2
packages and utilities. This repository provides ROS 2 control over one or more
Universal Robots UR5e robots in a monkey electrophysiology rig. It also includes
a virtual tabletop environment for simulation and system testing purposes.
Additionally, the package supports recording sensor and robot state space data
using ROS 2 Bag files. We additionally provide post-processing tools for motion
correction, spike sorting, NWB file conversion, etc.

## Table of Contents

* [Design Choices](#design-choices)
* [Requirements](#requirements)
* [Setup](#setup)
* [Usage](#usage)
  * [Building and Starting Docker Containers](#building-and-starting-docker-containers)
  * [Dev Container (VSCode)](#dev-container-vscode)
  * [Running Tasks](#running-tasks)
* [Project Structure](#project-structure)
* [CLI Commands](#cli-commands)
  * [Host Commands](#host-commands-binhost)
  * [Container Commands](#container-commands-bincontainer)
  * [Common Commands](#common-commands-bincommon)
  * [Build Command Options](#build-command-options)
  * [Launch Command Options](#launch-command-options)
* [Python CLI Tools](#python-cli-tools)
* [Configuration](#configuration)
  * [Environment Setup](#environment-setup-setupbash)
  * [Environment Variables](#environment-variables-env)
* [Contributing](#contributing)
* [License](#license)
* [FAQ](#faq)
* [Troubleshooting](#troubleshooting)

## Design Choices

The following decisions were made when designing the TableTop project:

### ROS 2

The project uses [ROS 2](https://docs.ros.org/en/jazzy/index.html) as the main
framework for controlling the UR5e robot as well as recording and compiling
sensor and state space data. There were several reasons for choosing to use
ROS 2 over a bespoke solution, some of which are delineated below:

* ROS 2 provides a powerful and flexible framework for building complex
    distributed software systems with many interdependent components
* ROS 2's message-based architecture allows for easy communication between
    different nodes in the system
* ROS 2 has built-in recording and playback capabilities via ROS 2 Bag files
* ROS 2 provides many built-in packages for customizing each stage of the robot
    control pipeline
* Universal Robots has a robust ROS 2 driver for the UR5e robot, making it
    easy to integrate with existing ROS 2 pipelines
* ROS 2 has support for real-time kernels, which can be used to ensure that
    critical tasks are executed in time (e.g., closed-loop motion control)

A bespoke solution would require a significant amount of development time
and would limit the ability to incorporate new features and functionality,
such as incorporating feedback data into robot control.

### Moveit

The project uses [MoveIt 2](https://moveit.picknik.ai/main/index.html) for
planning and control of the UR5e robot. Reasons for using MoveIt 2 include:

* MoveIt 2 provides utilities for planning and controlling the robot, as well
    as for visualizing the robot and environment state spaces in real-time.
* MoveIt 2 supports a variety of motion controllers and planning algorithms,
    each with its own customization options.
* The Universal Robots ROS 2 driver comes with Moveit 2 functionality
    pre-configured.

The alternative (sending URScript commands directly to the robot) limits the
control capabilities to those provided by the robot's software, which do not
support complex scenario planning and feedback control.

### Docker

The project runs entirely in [Docker](https://www.docker.com/) containers,
which provide:

* OS-agnostic development and deployment environments, making it accessible
    to users regardless of hardware constraints
* An isolated environment for each component of the software stack, each with
    its own dependencies and configurations already set up
* A consistent and reproducible environment for development and deployment
* A quick and easy way to run software without having to install and configure
    dependencies manually

Developing and deploying on bare metal would require manual dependency
management and configuration, which can be time-consuming, error-prone, and
often system-breaking. It also limits the platform compatibility, as ROS 2 is
intended and optimized for Ubuntu. The Universal Robot Simulator also cannot be
installed on Apple Silicon and must be run in a Docker container (with
modifications made in the `ursim/` directory and `compose.yaml` file).

## Requirements

This package requires the following software to be installed on your system before
building and running the project:

* [Docker](https://docs.docker.com/get-docker/)
* [[Optional] Visual Studio Code](https://code.visualstudio.com/) (for Dev
    Container usage)
* [[Optional] PlatformIO](https://platformio.org/install/) (for Teensy
    Micro-Controller usage)
* [[Optional] Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
    (for using Nvidia GPUs in Docker containers)

Follow the installation instructions in the links above for each requirement
(or use the helper scripts `scripts/docker_install.sh`,
`scripts/platformio_install.sh`, and `scripts/nvidia_ctk_install.sh` in the
`scripts/` directory).

**Note**: If you are running MacOS on Apple Silicon (all M-series chips),
you should enable the **Use Rosetta for x86/amd64 for emulation on Mac Silicon**
option in the Docker settings:

* Open Docker Desktop
* In the menu bar, click the gear (⚙) icon
* Go to General
* Make sure **Use Rosetta x86/amd64 for emulation on Mac Silicon** is enabled

You may experience issues with the `ursim` container otherwise.

**Note**: If you are running a system without an Nvidia GPU, or if you do not
want to permit the container GPU access, the `tt-env-gen` script will
automatically detect this and configure the containers accordingly.

## Setup

### Minimal Installation

1. Clone the TableTop repository:

    ```bash
    git clone https://github.com/jazlab/tabletop.git
    ```

2. Navigate to `tabletop` directory and download the submodules:

    ```bash
    cd tabletop
    git submodule sync
    git submodule update --init --recursive --remote
    ```

### Teensy Micro-Controller Setup

This is only required if you want to use the real Teensy micro-controller.
If you intend only to simulate the Teensy, you can skip this section.

1. Update udev rules:

    ```bash
    tt-udev-configure
    ```

2. Install PlatformIO Core:

    ```bash
    ./scripts/install/platformio.sh
    ```

    **Note**: This script will add PlatformIO to your `PATH`. You may need to
    restart your shell or open a new terminal session to use it.

3. Build and upload the Teensy firmware:

    ```bash
    tt-teensy-build
    ```

    **Note (once again)**: You can do this in either the container or on your host machine.
    If you do it in the container, you will have to modify `compose.yaml` for
    the desired container as follows (note that this modification is already
    made for the Dev Container in `.devcontainer/compose.devcontainer.yaml`):
    This will mount the `/dev` directory from the host machine to the container,
    allowing you to upload the Teensy firmware. Note that this will also require
    you to run the container in `privileged` mode, which may pose security risks.

    **Note (last I promise)**: Make sure you press the reset button on the Teensy before running
    this script. Regardless, this command **will fail** in the uploading stage.
    This is expected behavior, as is most failure. Just run the script again
    and again (for a max of three total attempts). If it still fails, give up
    and go home. Alternatively, figure out what else is wrong (possible reasons
    for failure: you forgot to plug in your teensy, you forgot to follow the
    instructions above, *I* forgot to update the instructions above, etc.).

### Setting up the physical UR5e Robot

This section is only relevant if you want to control the real robot. If you
intend only to simulate the robot, you can skip this section.

#### Creating the robot subnetwork

To create a local network over which to communicate with the robot, run the
following:

```bash
tt-robot-network
```

This will create a new network interface with the first 3 octets of the
`ROBOT_IP` (found in `env_files/robot.env`) and set the host machines
IP address to the `REVERSE_IP` (also found in `env_files/robot.env`).
These two IP addresses will be used for the remainder of the instructions.

#### Setting the robot IP address

With the network created, you can must now

1. On the Teach Pendant, click the "hamburger" (menu) icon in the top right
    corner of the window.
2. Click **Settings**.
3. Go to **System->Network**
4. Change the network method to **Static Address**
5. Fill out the fields with the following values (the rest can remain default):
    * **IP Address**: `ROBOT_IP`
    * **Subnet Mask**: `255.255.255.0`
6. Click **Apply**.

#### Installing and configuring the `external_control` URCap

The `external_control` URCap is required to command the robot from the host
machine (or in our case, the docker container).
To copy it to the robot, call the following command:

```bash
tt-robot-scp
```

This will copy any `*.urcap` files in the `ursim/programs/` directory to the robot.

You must then install them on the robot using the Teach Pendant:

1. In the **Settings** menu, go to **System->URCaps**
2. Click the **+** icon and select the urcap file you wish to install (e.g.
    `external_control.urcap`)
3. Click **Restart**. This will restart the robot and load the new URCap.

*You must do this for each URCap you wish to use.*

You must now configure the URCap with the appropriate IP settings:

1. In the **Installation** tab, go to **URCaps->External Control**
2. Fill out the fields with the following values:
    * **Host IP**: `REVERSE_IP`
    * **Custom Port**: `50002`
    * **Host Name**: `REVERSE_IP`

Okay last step. You have to create a program to use the URCap.

1. Click **New->Program** at the top of the window. This should pull up the
**Program** tab.
2. Click **URCaps->External Control** in the left sidebar. This will add the
    `external_control` URCap to the program.

After you have done all of the above, save the program and installation by
clicking **Save->Save All** at the top of the window. Make sure to save the
program with the name `external_control.urp` and the installation with the name
`default.installation` so that the commander loads the correct program and
installation when the rig program is started.

#### Enabling Remote Control Mode

You must enable **Remote Control Mode** on the robot's Teach Pendant in order
to control the robot using the `external_control` URCap.

1. In the **Settings** menu, go to **System->Remote Control**
2. Click **Enable**
3. Click **Exit** to exit the settings menu.
4. Click the **Local** button in the top right corner of the window.
5. Select **Remote Control** from the dropdown.

*You should not need to do this for the simulator.*

## Usage

### Building and Starting Docker Containers

All Docker interactions use the `tt-compose` wrapper command, which handles
environment generation, project defaults, and passes arguments through to
`docker compose`.

1. Make sure Docker is installed and running. Source the setup script
    (or add it to your `.bashrc`):

    ```bash
    source <path_to_repo>/setup.bash
    ```

2. Build the Docker images (optionally without cache) and ROS 2 workspace:

    ```bash
    tt-compose build [--no-cache]
    ```

    This builds all Docker images, then automatically builds the full ROS 2
    workspace (including all submodules and all TableTop packages) inside the
    container. Use this when setting up for the first time or after pulling
    major changes. To just build the ROS and Python packages without building
    the Docker containers, use the following:

    ```bash
    # On the host machine or inside the Dev Container
    tt-build [--all]
    ```

3. Start the containers using Docker Compose profiles. The available profiles
    group services by use case:

    | Profile | Services Started | Use Case |
    | ------- | ---------------- | -------- |
    | `sim` | novnc, ur-mock, teensy-sim, flic-sim, eyelink-sim, foxglove, rviz | Simulation with mock hardware |
    | `ursim` | novnc, ursim | UR Simulator (virtual teach pendant) |
    | `real` | novnc, autoheal, ur, teensy, flic, flicd, eyelink, foxglove, rviz, optitrack | Real hardware |

    For simulation (most common for development):

    ```bash
    # Use `--detach` to run in background, or press `d` once containers have started to detach
    tt-compose --profile=sim up [--detach]
    ```

    For real hardware:

    ```bash
    tt-compose --profile=real up
    ```

    To bring up individual containers:

    ```bash
    tt-compose up <container-name>
    ```

    Other useful `tt-compose` commands:

    ```bash
    # Show status of all containers
    tt-compose ps

    # Follow container logs
    tt-compose [--profile=<profile>] logs -f [<contaienr-name>]

    # Restart containers
    tt-compose [--profile=<profile>] restart [<container-name>]

    # Stop and remove containers
    tt-compose [--profile=<profile>] down [<container-name>]

    ```

4. Access the noVNC web interface at `http://localhost:8080/vnc.html` to
    interact with GUIs (RViz, UR Simulator, etc.). To scale the display
    correctly, click the drawer icon on the left, then the gear icon, and
    set **Scaling Mode** to **Local Scaling**.

5. Access the Foxglove web interface by nav `https://app.foxglove.dev`, logging in,
    clicking "Open connection > Foxglove WebSocket", then entering `ws://localhost:8765`
    in "WebSocket URL" (should be default unless the FOXGLOVE_PORT variable has been
    changed in the `setup.bash` file, see [Configuration](#configuration) below).

### Dev Container (VSCode)

For development with syntax highlighting, intellisense, and debugging, use the
VSCode Dev Container:

1. Install the **Dev Containers** extension in VSCode.

2. Call `tt-env-gen` to update the `.env` file.

3. Open the project folder in VSCode. When prompted, click **Reopen
    in Container**, or use the command palette (`Ctrl+Shift+P`) and select
    **Dev Containers: Rebuild and Reopen in Container**.

Once inside the Dev Container, you can build and launch ROS 2 packages
directly:

```bash
# Build tabletop packages
tt-build

# Launch tasks (see Running Tasks below)
tt-launch tasks
```

**Note:** If you make changes to the source code for any of the nodes
running in docker containers, you will need to restart or rebuild them
for changes to take effect

### Running Tasks

Tasks are behavioral experiments run on the rig. Launch them using `tt-launch`
either from inside a container (Dev Container or `docker exec`) or from the
host (which will automatically spin up a `commander` container):

```bash
# From the host machine or inside a container
tt-launch tasks
```

To stop running an ongoing task, you can press `Ctrl-C` in the terminal.

#### Launch Arguments

The task launcher (`tabletop_tasks/launch/tasks.launch.py`) accepts the
following arguments:

| Argument | Default | Options | Description |
| -------- | ------- | ------- | ----------- |
| `task` | `foraging_ordered` | Any config filename (without `.yaml`) | Task configuration to run |
| `robot_mode` | `mock` | `mock`, `ursim`, `real` | Robot connection mode |

Examples:

```bash
# Run with default foraging task and mock robot
tt-launch tasks

# Run smooth pursuit task with random PTP motion
tt-launch tasks task:=smooth_pursuit_random

# Run with the real robot
tt-launch tasks task:=foraging_ordered robot_mode:=real
```

**Note:** `robot_mode` should correspond to the Docker compose profile you used to start the other containers.

#### Task Configuration

Task configurations are YAML files located in `src/ros/tabletop/tabletop_tasks/config/`.

Currently available configurations:

| Config File | Task Type | Description |
| ----------- | --------- | ----------- |
| `foraging_ordered.yaml` | ForagingTask | Ordered object foraging trials |
| `foraging_random.yaml` | ForagingTask | Randomized object foraging trials |
| `foraging.yaml` | ForagingTask | Base foraging configuration |
| `present_ordered.yaml` | PresentTask | Ordered object presentation |
| `present_random.yaml` | PresentTask | Randomized object presentation |
| `smooth_pursuit_random.yaml` | SmoothPursuitTask | Random waypoint smooth pursuit |
| `smooth_pursuit_spiral.yaml` | SmoothPursuitTask | Spiral trajectory smooth pursuit |
| `smooth_pursuit_spiral_test.yaml` | SmoothPursuitTask | Spiral smooth pursuit (test) |
| `smooth_pursuit_sin.yaml` | SmoothPursuitTask | Sinusoidal smooth pursuit |

Each configuration defines a list of tasks with their parameters. For example,
a foraging task config specifies object IDs, presentation poses, trial
ordering, timing durations, and reward settings. A smooth pursuit config
specifies the motion type (random, spiral, sinusoidal), trajectory parameters,
and velocity scaling.

To see available parameters for each task type, you can see the inspect the class
definitions for each task in `/src/ros/tabletop/tabletop_tasks/tabletop_tasks/tasks/`,
as well as the trial generator definitions in
`/src/ros/tabletop/tabletop_tasks/tabletop_tasks/trial_generators/`

To create a new task configuration, copy an existing YAML file and modify the
parameters as needed.

## Project Structure

```text
tabletop/
├── bin/                          # CLI commands (see CLI Commands)
│   ├── host/                     # Host-only commands
│   ├── container/                # Container-only commands
│   └── common/                   # Commands for both host and container
├── docker/                       # Dockerfiles and container configs
├── src/
│   ├── tabletop_py/              # ROS-independent Python utilities
│   │   ├── gaze/                 # Eye-gaze estimation/tracking ML models
│   │   ├── flic/                 # Flic Bluetooth button client
│   │   └── utils/                # Common utilities
│   └── ros/
│       ├── tabletop/             # Main ROS 2 packages
│       │   ├── tabletop_rig/     # Core rig control (nodes, interfaces)
│       │   ├── tabletop_tasks/   # Experiment task definitions
│       │   ├── tabletop_interfaces/  # ROS message/service definitions
│       │   ├── tabletop_description/ # URDF robot descriptions
│       │   ├── tabletop_moveit_config/ # MoveIt planning configurations
│       │   └── tabletop_teensy/  # Teensy micro-controller interface
│       └── modules/              # External dependencies (git submodules)
│           ├── moveit2/          # Custom MoveIt fork
│           └── ...
├── ur_robot/                     # URCaps and programs for the UR5e
├── config/                       # Configuration files
├── env_files/                    # Environment variable files
├── compose.yaml                  # Docker Compose service definitions
└── setup.bash                    # Environment setup script
```

## CLI Commands

The TableTop project provides a set of `tt-*` commands that are automatically added
to your PATH when you source `setup.bash`. Commands are organized by where they
should be executed:

### Host Commands (`bin/host/`)

These commands are available on the host machine (outside Docker containers):

| Command | Description |
| ------- | ----------- |
| `tt-compose` | Wrapper for `docker compose` with TableTop-specific defaults |
| `tt-docker` | Wrapper for `docker` with TableTop environment variables |
| `tt-launch` | Launch ROS 2 nodes via a temporary `commander` container |
| `tt-env-gen` | Generate `.env` file from `.env.example` with dynamic hardware detection |
| `tt-dev-attach` | Attach to a running Dev Container |
| `tt-buildkit` | Configure Docker BuildKit for optimized builds |
| `tt-robot-network` | Create network interface for physical UR5e communication |
| `tt-flicd` | Start the Flic daemon container with auto-restart on disconnect |
| `tt-flir-reset` | Reset FLIR cameras (reload udev, factory reset, regenerate env) |
| `tt-udev-configure` | Configure udev rules for hardware devices |
| `tt-usbfs-configure` | Configure USB filesystem for FLIR cameras |
| `tt-cpu-speed-scaling-disable` | Disable CPU frequency scaling (for real-time performance) |
| `tt-port-forward` | Forward ports for remote access |

### Container Commands (`bin/container/`)

These commands are available inside Docker containers (rig, devcontainer):

| Command | Description |
| ------- | ----------- |
| `tt-build` | Build ROS 2 packages with colcon |
| `tt-launch` | Launch ROS 2 nodes (commander, rig, tasks, etc.) |
| `tt-clean-ws` | Clean the colcon workspace |
| `tt-clean-logs` | Clean ROS and colcon log files |
| `tt-dep-install` | Install ROS 2 dependencies via rosdep |
| `tt-display-set` | Configure DISPLAY variable for GUI (novnc or x11) |
| `tt-calibrate` | Run UR5e calibration and save to config file |
| `tt-create-graph` | Generate ROS 2 node/topic graph visualization |
| `tt-kill-ros` | Kill all running ROS 2 processes |

### Common Commands (`bin/common/`)

These commands work on both host and container:

| Command | Description |
| ------- | ----------- |
| `tt-teensy-build` | Build and upload Teensy firmware via PlatformIO |
| `tt-robot-scp` | Copy URCap files to the physical robot |

### Build Command Options

The `tt-build` command supports several options:

```bash
tt-build [options]

Options:
  --clean           Clean tabletop packages before building
  --clean-all       Clean entire workspace before building
  -a, --all         Build all packages (including moveit2)
  -p, --packages    Build specific packages and their dependencies
  -m, --only-modules Build only external modules (moveit2, etc.)
  -w, --workers N   Limit parallel workers (useful for low-memory systems)
  --build-debug     Build with debug symbols
  --clang           Use clang compiler
  --linker NAME     Use specified linker (default: mold)
  -v, --verbose     Verbose output
```

Examples:

```bash
# Build only tabletop packages (most common)
tt-build

# Build with clean workspace
tt-build --clean

# Build specific package
tt-build -p tabletop_rig

# Build all packages including moveit2
tt-build --all

# Build with limited parallelism (for low-memory systems)
tt-build --workers 2
```

### Launch Command Options

The `tt-launch` command provides shortcuts for common launch configurations:

```bash
tt-launch <type> [ros2_launch_args...]

Types:
  commander       Launch the Commander node only
  rig             Launch the full rig (all hardware interfaces)
  tasks           Launch the task runner
  ur              Launch UR driver only
  teensy          Launch Teensy interface only
  flic            Launch Flic button interface only
  eyelink         Launch Eyelink eye tracker
  flir            Launch FLIR camera driver
  flir_calibrate  Launch FLIR camera calibration
  optitrack       Launch OptiTrack motion capture
  rosbag          Launch ROS bag recording
  rosbag_convert  Convert ROS bag to CSV
  rviz            Launch RViz visualization
  foxglove        Launch Foxglove bridge
```

Examples:

```bash
# Launch full rig with mock hardware
tt-launch rig robot_mode:=mock use_mock_teensy:=true

# Launch tasks with specific configuration
tt-launch tasks task:=foraging_ordered robot_mode:=ursim

# Launch commander for the real robot
tt-launch commander robot_mode:=real
```

## Python CLI Tools

The `tabletop_py` package provides command-line tools for gaze estimation and
data processing. These are available after sourcing `setup.bash`:

| Command | Description |
| ------- | ----------- |
| `tt-gaze-calibrate` | Run the full gaze calibration pipeline |
| `tt-gaze-preprocess` | Preprocess eye tracking and marker data |
| `tt-gaze-train` | Train gaze estimation neural network |
| `tt-gaze-predict` | Run gaze prediction on new data |
| `tt-gaze-visualize` | Visualize calibration data and predictions |
| `tt-flic-client` | Flic Bluetooth button client |

### Gaze Calibration Pipeline

```bash
# Run full calibration pipeline
tt-gaze-calibrate -d /path/to/session [--visualize]

# Preprocess data only
tt-gaze-preprocess -d /path/to/session [--visualize]

# Train model only (assumes preprocessed data exists)
tt-gaze-train -d /path/to/session [--visualize]

# Visualize results (assumed preprocessing and training has already been done)
tt-gaze-visualize -d /path/to/session

# Use the trained model to predict on a new session
tt-gaze-predict -d /path/to/different/session [--visualize]

```

## Configuration

### Environment Setup (`setup.bash`)

The `setup.bash` file configures the shell environment for TableTop development.
Source it in your `.bashrc` or run it manually:

```bash
source /path/to/tabletop/setup.bash
```

This script:

* Sets `TABLETOP_DIR` to the repository root
* Configures ROS 2 environment variables (`RMW_IMPLEMENTATION`, `ROS_LOG_DIR`, etc.)
* Sets robot IP addresses (`ROBOT_IP`, `REVERSE_IP`, `SIM_ROBOT_IP`, `SIM_REVERSE_IP`)
* Activates the Python virtual environment (`.venv/`)
* Sources the colcon workspace (`install/setup.bash`)
* Adds `bin/common/` and context-specific `bin/` directories to PATH

### Environment Variables (`.env`)

The `.env` file contains Docker Compose configuration. Generate it using:

```bash
tt-env-gen --clean  # Generate from scratch using defaults from .env.example
tt-env-gen          # Regenerate only "auto-generated" variables
```

#### Required Variables (set in `.env.example`)

| Variable | Description | Default |
| -------- | ----------- | ------- |
| `USER_NAME` | Container username (useful to match host) | `mules` |
| `USER_UID` | Container user ID (useful to match host) | `1000` |
| `USER_GID` | Container group ID (useful to match host) | `1000` |
| `NOVNC_DISPLAY` | X11 display number for noVNC server | `:20.0` |
| `CUDA_VERSION` | CUDA version suffix for PyTorch (must be compatible with your GPU driver version) | `130` |
| `BIND_CONSISTENCY` | Docker bind mount consistency mode | `cached` |

#### Auto-Generated Variables (by `tt-env-gen`)

The `tt-env-gen` script automatically detects and configures:

* **NVIDIA GPU**: Sets `COMMANDER_RUNTIME=nvidia` and configures CUDA
* **FLIR Cameras**: Detects `/dev/flir/*` devices and maps them to `FLIR_DEV_*`
* **Teensy**: Detects `/dev/ttyACM0` and sets `TEENSY_DEV`
* **PulseAudio**: Configures audio socket mounting for sound playback
* **Kitty Terminal**: Configures Kitty remote control socket

## Contributing

Contributions are welcome! To contribute, follow these steps:

1. Fork the repository to your GitHub account by clicking the "Fork"
    button.
2. Clone the forked repository to your local machine using the command
    `git clone <url>`.
3. Install the dependencies using the command `pip install -r requirements-dev.txt`.
4. Install pre-commit hooks using the command `pre-commit install`.
5. Create a new branch for your changes using the command
    `git checkout -b <branch-name>`.
6. Make your changes, commit them using the command `git commit -am "<commit-message>"`,
    and push them to your forked repository using the command
    `git push origin <branch-name>`.
7. Create a pull request to the original repository by clicking the
    "New pull request" button.

Please follow the coding standards and best practices described in the
[ROS 2 documentation](<https://index.ros.org/doc/ROS> 2/Contributing/).

## License

MIT License

## FAQ

### What units are used?

We follow [REP 103](https://www.ros.org/reps/rep-0103.html) for unit conventions.
In particular, we use meters for length, seconds for time, and radians for angles.

### What is a common workflow for developing in the Dev Container?

First, build the Docker containers, ROS packages, and Python packages:

```bash
# On the host machine
tt-compose build
```

Then bring up the relevant Docker containers for testing in simulation:

```bash
# On the host machine
tt-compose --profile sim up
```

Now, open the Dev Container in VSCode (as above) and launch tasks
or other ROS 2 nodes directly:

```bash
# In the dev container or the host machine
tt-launch tasks task:=foraging_ordered robot_mode:=mock
```

If you make major changes (e.g. adding new files or packages), rebuild with:

```bash
# In the dev container
tt-build --clean
```

(You may need to restart the other docker containers)

```bash
# On the host machine
tt-compose --profile sim restart
```

## Troubleshooting

See [musings.md](musings.md) for a thoroughly disorganized and incomplete list of troubleshooting tips.
