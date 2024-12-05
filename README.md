# TableTop

## Description

TableTop is a meta-package for the TableTop project, containing multiple ROS2
packages and utilities. This repository provides ROS2 control over one or more
Universal Robots UR5e robots in a monkey electrophysiology rig. It also includes
a virtual tabletop environment for simulation and system testing purposes.
Additionally, the package supports recording sensor and robot state space data
using ROS2 Bag files. We additionally provide post-processing tools for motion
correction, spike sorting, NWB file conversion, etc.

## Table of Contents

* [Design Choices](#design-choices)
* [Requirements](#requirements)
* [Installation](#installation)
* [Usage](#usage)
    - [Starting Docker Containers](#starting-docker-containers)
    - [Choosing Launch Files](#choosing-launch-file)
    - [VSCode Dev Container](#vscode-dev-container)
* [Project Structure](#project-structure)
* [Contributing](#contributing)
* [License](#license)


## Design Choices

The following decisions were made when designing the TableTop project:

### ROS2
The project uses ROS2 as the main framework for controlling the UR5e robot
as well as recording and compiling sensor and state space data. There were
several reasaons for choosing to use ROS2 over a bespoke solution, some of
which are delineated below:
* ROS2 provides a powerful and flexible framework for building complex
  distributed software systems with many interdependent components.
* ROS2's message-based architecture allows for easy communication between
  different nodes in the system. This allows for modularity and clear
  separation of various functionality within the codebase, as well as
  efficient communication between robot, hardware, and sensor I/O.
* ROS2 has built-in recording and playback capabilities via ROS2 Bag files.
* ROS2 provides many built-in packages for customizing each stage of the robot
  control pipeline.
* Universal Robots has a robust ROS2 driver for the UR5e robot, making it
  easy to integrate with existing ROS2 pipelines.
* ROS2 has support for real-time kernels, which can be used to ensure that
  critical tasks are executed in time (e.g., closed-loop motion control).

A bespoke solution would require a significant amount of development time
and would limit the ability to incorporate new features and functionality,
such as incorporating feedback data into robot control.

### Moveit
The project uses [MoveIt 2](https://moveit.picknik.ai/main/index.html) for
planning and control of the UR5e robot.
Reasons for using MoveIt 2 include:
* MoveIt 2 provides utilities for planning and controlling the robot, as well
  as for visualizing the robot and environment state spaces in real-time.
* MoveIt 2 supports a variety of motion controllers and planning algorithms,
  each with its own customization options.
* The Universal Robots ROS2 driver comes with Moveit 2 functionality
  pre-configured.

The alternative (sending URScript commands directly to the robot) limits the
control capabilities to those provided by the robot's software, which do not
support complex scenario planning and feedback control.

### Docker
The project runs entirely in [Docker](https://www.docker.com/) containers,
which provide:
* OS-agnostic development and deployment environments, making it accessible
  to users regardless of hardware constraints.
* An isolated environment for each component of the software stack, each with
  its own dependencies and configurations already set up.
* A consistent and reproducible environment for development and deployment.
* A quick and easy way to run software without having to install and configure
  dependencies manually.

Developing and deploying on bare metal would require manual dependency management
and configuration, which can be time-consuming, error-prone, and often
system-breaking. It also limits the platform compatibility, as ROS2 is intended
and optimized for Ubuntu. The Universal Robot Simulator also cannot be
installed on Apple Silicon and must be run in a Docker container (with
modifications made in the `ursim/` directory and `compose.yaml` file).

## Requirements

This package requires:

* [Docker](https://docs.docker.com/get-docker/)
* [Visual Studio Code](https://code.visualstudio.com/) (optional, for Dev
  Container usage)

Follow the installation instructions in the links above.

If you are using an Apple device with Apple Silicon (All M-series chips),
you should enable the **Use Rosetta for x86/amd64 for emulation on Mac Silicon** option in the Docker settings:
* Open Docker Desktop
* In the menu bar, click the gear (⚙) icon
* Go to General
* Make sure **Use Rosetta x86/amd64 for emulation on Mac Silicon** is enabled

You may experience issues with the Universal Robots Simulator otherwise.


## Installation

1. Create a new ROS2 workspace directory:

    ```bash
    mkdir -p ~/ws/src
    ```

2. Clone the TableTop repository:

    ```bash
    cd ~/ws/src
    git clone https://github.com/jazlab/tabletop.git
    ```

## Usage

### Starting Docker Containers

To run the entire software stack using Docker:

1. Make sure Docker is installed on your system and the Docker daemon is
running.
    1. For macOS, make sure Rosetta is enabled in Docker settings (see 
        [above](#requirements)).
2. Navigate to the package root directory:

    ```bash
    cd ~/ws/src/tabletop
    ```

3. Build and start the Docker containers:

    ```bash
    docker compose up --build --force-recreate
    ```

This will build the Docker images and start the containers. There are 3 primary
containers:
- `server`: The server container for the TableTop project, which
    runs all the local ROS2 nodes (including the Universal Robots driver nodes,
    the MoveIt nodes, and the TableTop nodes). On startup, the `server`
    container will install any dependencies and build the project. It will then
    run the launch file specified in the `LAUNCH_FILE` environment variable or
    the default in `compose.yaml` if `LAUNCH_FILE` is not set.
- `robot`: The container for the Universal Robots simulator, which
    simulates the physical robot's teach pendant UI, as well as the safety
    constraints.
- `novnc`: The noVNC container, which acts as a window manager for the
   `server` and `robot` containers through a web interface.

Once the containers are started, you can access the web interface at
http://localhost:8080/vnc.html in your web browser.

### Interacting with the GUI

The web interface provides a desktop-like interface for interacting with the
Universal Robots simulator and the ROS2 visualization nodes. 

To make sure that you can see the whole screen:
* Click the drawer icon on the left of the screen to expand it
* Click the gear icon

To get started,
after opening the web interface, you will need to:
* Click the 
* Bring the URSim to the main panel of the window manager (focus it with your
  mouse, then type `alt+enter` (`command+enter` on macOS))
* Click the red button in the lower left corner indicating robot status
* Click the start button twice, once to prep the robot, once to release the brakes.
* Click the exit button
* In the programs tab, load a program (you are looking for the
  `external_control-1.0.5.jar` program in the `/urcaps` directory)
* Press the play button

This will power on the simulated robot and initiate communication with the
ROS2 driver.

If you are using the `server.launch.py` file, the TableTop `commander` node
will start the TableTop program and the robot should start moving according
to the task designation.

If you are using the `moveit.launch.py` file, you should now be able to set
goal positions using the RViz GUI and send planning and execution requests
to the MoveGroup.


### Choosing Launch File

There are 2 main launch files in the `tabletop_server/launch` directory:
- `server.launch.py`: The main launch file for the TableTop project.
   This will
- `moveit.launch.py`: The MoveIt launch file for the TableTop project.

To use either file, you need to edit the `compose.yaml` file and set the
environment variable `LAUNCH_FILE` to the name of the launch file you want to
use. For example, to use the `moveit.launch.py` file, you would set the
following in the `compose.yaml` file:
```yaml
services:
  server:
    environment:
      - LAUNCH_FILE=moveit.launch.py
```
followed by initializing the Docker containers as in the previous section.

Alternatively, you can use the `LAUNCH_FILE` environment variable in the Docker
command line. For example, to use the `moveit.launch.py` file, you would run
the:
```bash
LAUNCH_FILE=moveit.launch.py docker compose up --build --force-recreate
```

### VSCode Development using docker Dev Containers

To be able to develop within the ROS2 environment (giving you syntax highlighting,
intellisense, debugging, etc.) you can use the VSCode Dev Container extension.
To open VSCode in the development container:

1. Install the "Remote - Containers" extension in VSCode.
2. Open the project folder in VSCode.
3. When prompted, click "Reopen in Container" or use the command palette (F1 or
    Ctrl+Shift+P) and select "Remote-Containers: Reopen in Container".

VSCode will build the Dev Container and provide you with a fully configured
development environment.

## Project Structure

The TableTop meta-package consists of the following ROS2 packages, located in
the repository's root directory:

- `tabletop_msgs`: TableTop message definitions
- `tabletop_moveit_interface`: TableTop MoveIt interface
- `tabletop_moveit_config`: TableTop MoveIt configurations
- `tabletop_server`: TableTop server nodes and launch files
- `tabletop_teensy`: TableTop Teensy nodes and launch files

Additional non-ROS2 packages (also located in the repository's root directory):
- `novnc`: Context for building and running noVNC Docker container
- `ursim`: Contains the external control URCAP for the host machine
   to interface with the robot.
- `scripts`: Utility scripts for setting up the environment (locally and
   in Docker)

## Contributing

Contributions are welcome! To contribute, follow these steps:

1. Fork the repository to your GitHub account by clicking the "Fork"
   button.
2. Clone the forked repository to your local machine using the command
   `git clone <url>`.
3. Create a new branch for your changes using the command
   `git checkout -b <branch-name>`.
4. Make your changes, commit them using the command `git commit -am "<commit-message>"`,
   and push them to your forked repository using the command
   `git push origin <branch-name>`.
5. Create a pull request to the original repository by clicking the
   "New pull request" button.

Please follow the coding standards and best practices described in the
[ROS2 documentation](https://index.ros.org/doc/ros2/Contributing/).

## License
MIT License

