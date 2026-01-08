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
    - [Starting Docker Containers](#starting-docker-containers)
    - [Interacting with the noVNC GUI](#interacting-with-the-novnc-gui)
    - [Choosing Launch Command](#choosing-launch-command)
    - [Container Development](#container-development)
* [Project Structure](#project-structure)
* [CLI Commands](#cli-commands)
    - [Host Commands](#host-commands-binhost)
    - [Container Commands](#container-commands-bincontainer)
    - [Common Commands](#common-commands-bincommon)
    - [Build Command Options](#build-command-options)
    - [Launch Command Options](#launch-command-options)
* [Python CLI Tools](#python-cli-tools)
* [Configuration](#configuration)
    - [Environment Setup](#environment-setup-setupbash)
    - [Environment Variables](#environment-variables-env)
    - [Robot Configuration](#robot-configuration-env_files)
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
want to permit the container GPU access, you can comment out the `runtime: nvidia`
line in the `rig` service in the `compose.yaml` file (see [Starting Docker
Containers](#starting-docker-containers) for more information on the different
containers and their associated services).

## Setup

### Minimal Installation

1. Create a new ROS 2 workspace directory:
    ```bash
    mkdir -p ~/ws/src
    ```

2. Clone the TableTop repository:
    ```bash
    cd ~/ws/src
    git clone https://github.com/jazlab/tabletop.git
    ```

3. Download the submodules:
    ```bash
    git submodule update --init --recursive
    ```

4. Clone the [`moveit2` fork](https://github.com/jazlab/moveit2):
    ```bash
    ./scripts/moveit_download.sh
    ```

### Teensy Micro-Controller Setup
This is only required if you want to use the real Teensy micro-controller.
If you intend only to simulate the Teensy, you can skip this section.

1. Update udev rules:
    ```bash
    ./scripts/udev_update.sh
    ```

2. Install PlatformIO Core:
    ```bash
    ./scripts/install_platformio.sh
    ```

    **Note**: This script will add PlatformIO to your `PATH`. You may need to
    restart your shell or open a new terminal session to use it.

3. Build and upload the Teensy firmware:
    ```bash
    ./scripts/teensy_build.sh
    ```

    **Note**: This requires PlatformIO Core to be installed. See [step 3](#optional-install-platformio-core)
    for more information.


    **Note (again)**: The build may fail with permission errors. If this is the case,
    you can use the mighty `sudo chown -R $USER:$USER .` command to change the
    ownership of the files to your user account. If you are not so bold (or if
    that doesn't work), you can run `./scripts/upload_teensy.sh --clean` to
    clean the build directory and try again. Requires `sudo` permissions.

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
./scripts/robot_network.sh
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
./scripts/robot_scp.sh
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

### Starting Docker Containers

To run the entire software stack using Docker:

1. Make sure Docker is installed on your system and the Docker daemon is
    running.

    For macOS, make sure Rosetta is enabled in Docker settings (see
    [above](#requirements)).

2. Navigate to the package root directory:

    ```bash
    cd ~/ws/src/tabletop
    ```

3. [Optional] Clean up your docker environment and ROS 2 workspace:

    ```bash
    ./scripts/docker_prune.sh [-a]
    ./scripts/clean_ws.sh
    ```
    **Warning**: The `-a` flag in `docker_prune.sh` will remove all containers,
    networks, images, and build cache associated with the TableTop project
    (except the ursim image).


4. Build the Docker containers:
    ```bash
    docker compose build [--no-cache]
    ```
    Use `--no-cache` to force a rebuild of the Docker images and install the
    latest versions of the dependencies.

5. Build the tabletop and moveit packages:
    ```bash
    docker compose --env-file env_files/build.env up ...
    ```
    This will build the tabletop packages and install any dependencies in your
    [colcon workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html.
    See [here](#choosing-launch-command) for more information on using environment
    files to set variables.

7. Start the Docker containers:

    All at once:
    ```bash
    docker compose up [--force-recreate] ursim|ursim_novnc rig|rig_novnc
    ```

    Individually:
    ```bash
    docker compose up [--force-recreate] ursim
    docker compose up [--force-recreate] ursim_novnc
    docker compose up [--force-recreate] rig
    docker compose up [--force-recreate] rig_novnc
    ```

    Running the `ursim` or `rig` containers will use your host machine's
    X server, in which case you may first need to run:
    ```bash
    xhost +
    ```
    to allow the containers to connect to the X server.

    **Note**: This has only been tested on Ubuntu 24.04. The compose file
    may need to be modified for other operating systems.
    Using `xhost +` also poses a security risk, as it allows any process to
    connect to the X server. **TODO: Find a more secure way to do this.**

    Use `--force-recreate` to make sure that the containers are recreated if
    they already exist (ensures consistent behavior across runs).
    Alternatively, you can call `docker compose down` before running the above
    commands to destroy any existing containers.

This will build the Docker images and start the containers. There are 3 primary
containers (and their variants):
- `novnc`: The noVNC container, which exposes a web interface to interact
    with the GUIs in any of the running docker containers. Includes a dynamic
    window manager ([dwm](https://dwm.suckless.org/)) for multiple GUI windows
    to be displayed at once. A list of commonly used keyboard shortcuts can be
    found [here](https://wiki.gentoo.org/wiki/Dwm#Keys_and_key_functions:~:text=the%20window%20to.-,Default%20shortcuts,-Those%20shortcuts%20are).
- `rig*`: The rig container for the TableTop project, which
    runs all the local ROS 2 nodes (including the Universal Robots driver nodes,
    the MoveIt nodes, and the TableTop nodes). On startup, the `rig`
    container will optionally install any dependencies and build the project.
    It will then run the command specified in the `LAUNCH_COMMAND` environment
    variable or the default in `compose.yaml` if `LAUNCH_COMMAND` is not set. The
    difference between `rig` and `rig_novnc` is that the former uses the
    host machine's X server to display the GUI, while the latter uses the noVNC
    web interface.
- `ursim*`: The container for the Universal Robots simulator, which simulates the
    safety constraints of the real robot. The difference between `ursim` and
    `ursim_novnc` is that the former uses the host machine's X server to display
    the simulator GUI, while the latter uses the noVNC web interface.

**Note**: Be careful not to run more than one of each type of container at
once. This means you cannot use `docker compose up` without any service
arguments.

If you are using the noVNC GUI, you can access the web interface at
`http://localhost:8080/vnc.html` in your web browser.

### Interacting with the noVNC GUI

The noVNC web interface provides a desktop-like interface for interacting with
the Universal Robots simulator and the ROS 2 visualization nodes.

To make sure that you can see the whole screen:
* Click the drawer icon on the left of the screen to expand it
* Click the gear icon
* Under **Scaling Mode**, select **Local Scaling**

To get started:

1. Make sure any previous containers have been destroyed (call `docker compose
    down` in the root directory of the project).
2. Start the `ursim_novnc` container, as above.
    - It is currently necessary to do this before the `rig_novnc` in order
      to give the URSim time to spin up.
3. Start the `rig_novnc` container, as above.
    * **Note**: If you are running MacOS, you may need to comment out the
        following lines in the `compose.yaml` file:
        ```yaml
        services:
            ...
            rig_novnc:
                # depends_on:
                #   novnc:
                #     condition: service_healthy
                ...
        ```
        This will prevent the novnc container from restarting when the `rig_novnc`
        container is started. This seems to be an issue exclusive to MacOS.

This will power on the simulated robot and initiate communication with the
ROS 2 driver.

### Choosing Launch Command

The default behavior of the `rig` container is to sleep indefinitely, which
is useful if you want to interactively launch the ROS processes and inspect the
container state. To change this default behavior, you can set the `LAUNCH_COMMAND`
environment variable to your desired bash command. You can do this by:
* [Preferred] Using an environment file (commonly used such files in
    `env_files/`):
    ```bash
    docker compose --env-file env_files/robot.env --env-file env_files/launch_tasks.env up ...
    ```
    **Note**: The order of the environment files matters. Here, the `robot.env`
    file sets variables that are used by the `launch_tasks.env` file.

    **Note**: You may not use an environment file that depends on the default
    environment variables in the compose file. For example, if your environment
    file sets `LAUNCH_COMMAND` whose value depends on `ROBOT_IP` and you do not
    first provide another environment file that sets `ROBOT_IP` (like `robot.env`),
    the `compose` command will fail.
* Setting the variable from the command line for a single command:
    ```bash
    LAUNCH_COMMAND="ros2 launch tabletop_tasks run_tasks.launch.py" docker compose up --build --force-recreate
    ```
* Editing the `compose.yaml` file (make sure to edit the default value so that
    you can overwrite `LAUNCH_COMMAND` from the command line later):

    ```yaml
    services:
        ...
        rig:
            ...
            environment:
                # Edit the default value so that you can overwrite it from the command line later
                - LAUNCH_COMMAND=${LAUNCH_COMMAND:-ros2 launch tabletop_moveit_config tabletop_moveit.launch.py}

                # Don't do the below, you will lose the ability to overwrite LAUNCH_COMMAND
                # - LAUNCH_COMMAND=ros2 launch tabletop_moveit_config tabletop_moveit.launch.py
    ```

### Container Development

To be able to develop within the ROS 2 environment (giving you syntax
highlighting, intellisense, debugging, etc.) you can use the VSCode Dev
Container extension. To open VSCode in the development container:

1. Install the "Remote - Containers" extension in VSCode.
2. **Important**: You must add `tt-docker` and `tt-compose` somewhere to your path
    (by adding `source <repo-dir>/setup.bash` to your .bashrc), then go to VSCode
    settings and set `dev.containers.dockerPath` to `tt-docker` and
    `dev.containers.dockerComposePath` to `tt-compose` (*TODO*, fix this with env
    files)
2. Open the project folder in VSCode.
3. When prompted, click "Rebuild and Reopen in Container" or use the command
    palette (F1 or Ctrl+Shift+P) and select "Remote-Containers: Rebuild and
    Reopen in Container".

VSCode will build the Dev Container and provide you with a fully configured
development environment.

You may also want to install the recommended extensions for the Dev Container,
which you should be prompted to do when you first open the Dev Container.

**Note**: In order to command the robot from the Dev Container, you will need
to set the host IP and host name in the UR Dashboard to the Dev Container's
IP address. This setting can be found in *Installation->URCaps->External
Control->Host IP* and *Installation->URCaps->External Control->Host Name*.
If you then save the installation, you will not have to do this again.


You can now run any of the scripts in the `scripts/` directory. A few useful
ones are:
* `scripts/bashrc_update.sh [--display novnc|x11]` to update your bashrc file to
    with the correct ROS 2 environment variables and optionally set the `DISPLAY`
    variable to the correct value for the noVNC or X11 display.
* `scripts/build.sh ...` to build the ROS 2 workspace (including the MoveIt and
    TableTop packages).
* `scripts/clean_ws.sh` to clean the workspace directory.
* `scripts/teensy_build.sh` to build and upload the Teensy firmware.

You can also run any ROS 2 commands as you normally would. For example, to
launch the `tabletop_tasks` node, you can run:
```bash
ros2 launch tabletop_tasks run_tasks.launch.py --task_config:=<path_to_task_config> use_mock_teensy:=<true|false>
```
*The above commands will also work for the `rig` and `rig_novnc` containers if
you choose to interact with them through the terminal (i.e. call `docker exec -it
<container_name> bash` after starting the container `docker compose up rig` with
the default launch command that sleeps indefinitely).*

### Uploading Teensy Firmware from the Dev Container

Attempting to upload the Teensy firmware from the dev container requires you
to explicitly mount `/dev` from the host machine and run the docker container
in `privileged` mode. This can be achieved by modifying (or uncommenting) the
following line in `.devcontainer/compose.devcontainer.yaml`:
```yaml
services:
    ...
    devcontainer:
        ...
        volumes:
            - /dev:/dev
        privileged: true
```
**Note**: Running the container in `privileged` mode may pose security risks,
as it gives the container root access to the host machine.
This security risk is not imposed by other containers in the project and is only
required for the Teensy upload functionality while developing in the dev
container.

To avoid this, simply upload the Teensy firmware from your host machine
following the instructions in [Optional Teensy Micro-Controller Setup](#optional-teensy-micro-controller-setup)).

## Project Structure

The TableTop meta-package consists of the following ROS 2 packages, located in
the repository's root directory:

- `tabletop_description`: TableTop URDF description
- `tabletop_moveit_config`: TableTop MoveIt configurations
- `tabletop_interfaces`: TableTop message definitions
- `tabletop_rig`: TableTop rig nodes and launch files
- `tabletop_tasks`: TableTop task nodes and launch files
- `tabletop_teensy`: TableTop Teensy nodes and launch files
- `tabletop_utils`: TableTop utility nodes and launch files

Additional non-ROS 2 packages/directories (also located in the repository's root directory):
- `novnc`: Context for building and running noVNC Docker container
- `ur_robot`: Contains the URCAPs and programs for starting the Universal Robots
    Simulator and interfacing with the simulator or physical robot
- `bin/`: CLI commands organized by execution context (see [CLI Commands](#cli-commands))
- `src/tabletop_py/`: Python utilities for gaze estimation, Flic buttons, and mesh processing


## CLI Commands

The TableTop project provides a set of `tt-*` commands that are automatically added
to your PATH when you source `setup.bash`. Commands are organized by where they
should be executed:

### Host Commands (`bin/host/`)

These commands are available on the host machine (outside Docker containers):

| Command | Description |
|---------|-------------|
| `tt-env-gen` | Generate `.env` file from `.env.example` with dynamic hardware detection |
| `tt-compose` | Wrapper for `docker compose` with TableTop-specific defaults |
| `tt-docker` | Wrapper for `docker` with TableTop environment variables |
| `tt-devcontainer` | Build and manage the VSCode Dev Container |
| `tt-dev-attach` | Attach to a running Dev Container |
| `tt-buildkit` | Configure Docker BuildKit for optimized builds |
| `tt-robot-network` | Create network interface for physical UR5e communication |
| `tt-flicd` | Start the Flic daemon container with auto-restart on disconnect |
| `tt-bluetooth-reset` | Reset Bluetooth module (useful for Flic issues) |
| `tt-udev-configure` | Configure udev rules for hardware devices |
| `tt-usbfs-configure` | Configure USB filesystem for FLIR cameras |
| `tt-cpu-speed-scaling-disable` | Disable CPU frequency scaling (for real-time performance) |
| `tt-port-forward` | Forward ports for remote access |

### Container Commands (`bin/container/`)

These commands are available inside Docker containers (rig, devcontainer):

| Command | Description |
|---------|-------------|
| `tt-build` | Build ROS 2 packages with colcon |
| `tt-launch` | Launch ROS 2 nodes (commander, rig, tasks, etc.) |
| `tt-clean-ws` | Clean the colcon workspace |
| `tt-clean-logs` | Clean ROS and colcon log files |
| `tt-dep-install` | Install ROS 2 dependencies via rosdep |
| `tt-display-set` | Configure DISPLAY variable for GUI (novnc or x11) |
| `tt-calibrate` | Run UR5e calibration and save to config file |
| `tt-create-graph` | Generate ROS 2 node/topic graph visualization |
| `tt-kill-ros` | Kill all running ROS 2 processes |
| `tt-teensy-connect` | Connect to Teensy serial port for debugging |

### Common Commands (`bin/common/`)

These commands work on both host and container:

| Command | Description |
|---------|-------------|
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
  commander     Launch the Commander node only
  rig           Launch the full rig (all hardware interfaces)
  tasks         Launch the task runner
  ur            Launch UR driver only
  teensy        Launch Teensy interface only
  flic          Launch Flic button interface only
  flir          Launch FLIR camera driver
  optitrack     Launch OptiTrack motion capture
  rviz          Launch RViz visualization
  foxglove      Launch Foxglove bridge
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
|---------|-------------|
| `gaze-calibrate` | Run the full gaze calibration pipeline |
| `gaze-preprocess` | Preprocess eye tracking and marker data |
| `gaze-train` | Train gaze estimation neural network |
| `gaze-visualize` | Visualize calibration data and predictions |

### Gaze Calibration Pipeline

```bash
# Run full calibration pipeline
gaze-calibrate -d /path/to/session --visualize

# Preprocess data only
gaze-preprocess -d /path/to/session --visualize

# Train model only (assumes preprocessed data exists)
gaze-train -d /path/to/session -c config/gaze_estimation.yaml

# Visualize results
gaze-visualize -d /path/to/session
```

### Additional Python Modules

These modules can be run directly with `python -m`:

```bash
# Flic button client
python -m tabletop_py.flic.client scan      # Scan for buttons
python -m tabletop_py.flic.client listen    # Listen for button events
python -m tabletop_py.flic.client info      # Get server info

# Flic piano (musical keyboard using Flic buttons)
python -m tabletop_py.flic.piano --key C --scale HarmonicMinor

# Mesh utilities
python -m tabletop_py.utils.mesh model.stl --preview
python -m tabletop_py.utils.mesh model.stl --simplification convex_hull -o simplified.stl

# EDF file conversion
python -m tabletop_py.gaze.edf recording.edf -o recording.csv
```


## Configuration

### Environment Setup (`setup.bash`)

The `setup.bash` file configures the shell environment for TableTop development.
Source it in your `.bashrc` or run it manually:

```bash
source /path/to/tabletop/setup.bash
```

This script:
- Sets `TABLETOP_DIR` to the repository root
- Configures ROS 2 environment variables (`RMW_IMPLEMENTATION`, `ROS_LOG_DIR`, etc.)
- Sets robot IP addresses (`ROBOT_IP`, `REVERSE_IP`, `SIM_ROBOT_IP`, `SIM_REVERSE_IP`)
- Activates the Python virtual environment (`.venv/`)
- Sources the colcon workspace (`install/setup.bash`)
- Adds `bin/common/` and context-specific `bin/` directories to PATH

### Environment Variables (`.env`)

The `.env` file contains Docker Compose configuration. Generate it using:

```bash
tt-env-gen        # Generate from .env.example
tt-env-gen --clean # Regenerate from scratch
```

#### Required Variables (set in `.env.example`)

| Variable | Description | Default |
|----------|-------------|---------|
| `USER_NAME` | Container username (should match host) | `mules` |
| `USER_UID` | Container user ID (should match host) | `1000` |
| `USER_GID` | Container group ID (should match host) | `1000` |
| `NOVNC_DISPLAY` | Display number for noVNC server | `:20.0` |
| `CUDA_VERSION` | CUDA version suffix for PyTorch | `130` |
| `BIND_CONSISTENCY` | Docker bind mount consistency mode | `cached` |

#### Auto-Generated Variables (by `tt-env-gen`)

The `tt-env-gen` script automatically detects and configures:

- **NVIDIA GPU**: Sets `COMMANDER_RUNTIME=nvidia` and configures CUDA
- **FLIR Cameras**: Detects `/dev/flir/*` devices and maps them to `FLIR_DEV_*`
- **Teensy**: Detects `/dev/ttyACM0` and sets `TEENSY_DEV`
- **PulseAudio**: Configures audio socket mounting for sound playback
- **Kitty Terminal**: Configures Kitty remote control socket

### Robot Configuration (`env_files/`)

Environment files for different robot configurations:

| File | Description |
|------|-------------|
| `env_files/robot.env` | Real robot IP configuration |
| `env_files/sim.env` | Simulator IP configuration |
| `env_files/launch_*.env` | Preset launch configurations |

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
[ROS 2 documentation](https://index.ros.org/doc/ROS 2/Contributing/).

## License
MIT License

## FAQ

### What units are used?
We follow [REP 103](https://www.ros.org/reps/rep-0103.html) for unit conventions.
In particular, we use meters for length, seconds for time, and radians for angles.

### What is a common workflow for developing in the Dev Container?
After starting the Dev Container, make sure to update your bashrc file if
you are using the noVNC display:
```bash
./scripts/bashrc_update.sh --display novnc
```
Then, you can build the project:
```bash
./scripts/build.sh [--clean]
```
You must then open a new terminal to see the changes take effect.

You can now run any ROS 2 commands as you normally would. For example, to
launch the `tabletop_tasks` node, you can run:
```bash
ros2 launch tabletop_tasks run_tasks.launch.py [--task_config_file <path_to_task_config>] [use_mock_teensy:=<true|false>] ...
```

If you make major changes to the software (e.g. adding new files or folders),
you may need to rebuild the project:
```bash
./scripts/build.sh --clean
```

## Troubleshooting

See [musings.md](musings.md) for a thoroughly disorganized and incomplete list of troubleshooting tips.
