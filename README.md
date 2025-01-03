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

### ROS 2
The project uses [ROS 2](https://docs.ros.org/en/jazzy/index.html) as the main 
framework for controlling the UR5e robot as well as recording and compiling 
sensor and state space data. There were several reasons for choosing to use 
ROS 2 over a bespoke solution, some of which are delineated below:
* ROS 2 provides a powerful and flexible framework for building complex 
    distributed software systems with many interdependent components.
* ROS 2's message-based architecture allows for easy communication between 
    different nodes in the system. This allows for modularity and clear 
    separation of various functionality within the codebase, as well as 
    efficient communication between robot, hardware, and sensor I/O.
* ROS 2 has built-in recording and playback capabilities via ROS 2 Bag files.
* ROS 2 provides many built-in packages for customizing each stage of the robot 
    control pipeline.
* Universal Robots has a robust ROS 2 driver for the UR5e robot, making it 
    easy to integrate with existing ROS 2 pipelines.
* ROS 2 has support for real-time kernels, which can be used to ensure that 
    critical tasks are executed in time (e.g., closed-loop motion control).

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
    to users regardless of hardware constraints.
* An isolated environment for each component of the software stack, each with 
    its own dependencies and configurations already set up.
* A consistent and reproducible environment for development and deployment.
* A quick and easy way to run software without having to install and configure 
    dependencies manually.

Developing and deploying on bare metal would require manual dependency 
management and configuration, which can be time-consuming, error-prone, and 
often system-breaking. It also limits the platform compatibility, as ROS 2 is 
intended and optimized for Ubuntu. The Universal Robot Simulator also cannot be 
installed on Apple Silicon and must be run in a Docker container (with 
modifications made in the `ursim/` directory and `compose.yaml` file).

## Requirements

This package requires:

* [Docker](https://docs.docker.com/get-docker/)
* [Visual Studio Code](https://code.visualstudio.com/) (optional, for Dev 
    Container usage)

Follow the installation instructions in the links above.

**Note**: If you are running MacOS on Apple Silicon (All M-series chips), 
you should enable the **Use Rosetta for x86/amd64 for emulation on Mac Silicon** 
option in the Docker settings:
* Open Docker Desktop
* In the menu bar, click the gear (⚙) icon
* Go to General
* Make sure **Use Rosetta x86/amd64 for emulation on Mac Silicon** is enabled

You may experience issues with the Universal Robots Simulator otherwise.


## Installation

1. Create a new ROS 2 workspace directory:

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

    All at once:
    ```bash
    docker compose up --build --force-recreate
    ```
    
    Individually:
    ```bash
    docker compose up --build --force-recreate novnc
    docker compose up --build --force-recreate robot
    docker compose up --build --force-recreate server
    ```

4. Open a web browser and navigate to http://localhost:8080/vnc.html

This will build the Docker images and start the containers. There are 3 primary 
containers:
- `novnc`: The noVNC container, which exposes a web interface to interact 
    with the GUIs in any of the running docker containers. Includes a dynamic 
    window manager ([dwm](https://dwm.suckless.org/)) for multiple GUI windows 
    to be displayed at once.
- `server`: The server container for the TableTop project, which 
    runs all the local ROS 2 nodes (including the Universal Robots driver nodes, 
    the MoveIt nodes, and the TableTop nodes). On startup, the `server` 
    container will install any dependencies and build the project. It will then 
    run the launch file specified in the `LAUNCH_FILE` environment variable or 
    the default in `compose.yaml` if `LAUNCH_FILE` is not set.
- `robot`: The container for the Universal Robots simulator, which 
    simulates the real robot's teach pendant UI, as well as the physical and 
    safety constraints of the real robot.

Once the containers are started, you can access the web interface at 
http://localhost:8080/vnc.html in your web browser.

### Interacting with the GUI

The web interface provides a desktop-like interface for interacting with the 
Universal Robots simulator and the ROS 2 visualization nodes. 

To make sure that you can see the whole screen:
* Click the drawer icon on the left of the screen to expand it
* Click the gear icon
* Under **Scaling Mode**, select **Local Scaling**

To get started:

1. Make sure any previous containers have been destroyed (call `docker compose 
    down` in the root directory of the project).
2. Start the docker containers for the `novnc` and `robot` containers, as above.
    - It is currently necessary to do these before the `server` in order to 
      manually enable **Remote Control Mode** for the robot first (follow 
      instructions below).
3. Enable **Remote Control Mode** for the robot. 
    1. Bring the URSim to the main panel of the window manager, if it is not 
        already there (focus it with your mouse, then type `alt+enter` 
        (`command+enter` on macOS)).
    2. Click the "hamburger" (menu) icon in the top right corner of the URSim 
        window.
    3. Click **Settings**.
    4. Under **System->Remote Control**, click **Enable**.
    5. Click **Exit** in the lower left corner of the menu.
    6. Click the **Local** button in the top right corner of the URSim window.
    7. Select **Remote Control** from the dropdown.
4. Start the `server` container, as above. 
    * **Note**: If you are running MacOS, you may need to comment out the 
        following lines in the `compose.yaml` file:
        ```yaml
        services:    
            ...
            server:
                # depends_on:
                #   novnc:
                #     condition: service_healthy
                ...
        ```
        This will prevent the novnc container from restarting when the `server` 
        container is started. This seems to be an issue exclusive to MacOS.

This will power on the simulated robot and initiate communication with the 
ROS 2 driver.

If you are using the `server.launch.py` file, the TableTop `commander` node 
will start the TableTop program and the robot should start moving according 
to the task designation.

If you are using the `moveit.launch.py` file, you should now be able to set 
goal positions using the RViz GUI and send planning and execution requests 
to the MoveGroup.

### Choosing Launch File

There are 2 main launch files in the `tabletop_server/launch` directory:
- `server.launch.py`: The main launch file for the TableTop project. This will 
    start the Universal Robots ROS 2 driver nodes, the MoveIt nodes, and the 
    TableTop nodes, including the commander node.
- `moveit.launch.py`: The MoveIt launch file for the TableTop project. This 
    will start an Universal Robots ROS 2 driver nodes, the MoveIt nodes, and 
    a GUI to plan and execute simple motion plans. This is good for testing 
    and debugging your planning scene and motion planners.

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
the following:
```bash
LAUNCH_FILE=moveit.launch.py docker compose up --build --force-recreate
```

### VSCode Development using docker Dev Containers

To be able to develop within the ROS 2 environment (giving you syntax 
highlighting, intellisense, debugging, etc.) you can use the VSCode Dev 
Container extension. To open VSCode in the development container:

1. Install the "Remote - Containers" extension in VSCode.
2. Open the project folder in VSCode.
3. When prompted, click "Reopen in Container" or use the command palette (F1 or 
    Ctrl+Shift+P) and select "Remote-Containers: Reopen in Container".

VSCode will build the Dev Container and provide you with a fully configured 
development environment.

## Project Structure

The TableTop meta-package consists of the following ROS 2 packages, located in 
the repository's root directory:

- `tabletop_msgs`: TableTop message definitions
- `tabletop_moveit_interface`: TableTop MoveIt interface
- `tabletop_moveit_config`: TableTop MoveIt configurations
- `tabletop_server`: TableTop server nodes and launch files
- `tabletop_teensy`: TableTop Teensy nodes and launch files

Additional non-ROS 2 packages (also located in the repository's root directory):
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
[ROS 2 documentation](https://index.ros.org/doc/ROS 2/Contributing/).

## License
MIT License

