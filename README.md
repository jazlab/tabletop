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

- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
  - [Docker](#docker)
  - [VSCode Dev Container](#vscode-dev-container)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)

## Requirements

This package requires:

* [Docker](https://docs.docker.com/get-docker/)
* [Visual Studio Code](https://code.visualstudio.com/) (optional, for Dev
  Container usage)

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

### Starting Docker containers

To run the entire software stack using Docker:

1. Make sure Docker is installed on your system and the Docker daemon is
   running.
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
- **server**: The server container for the TableTop project, which 
   runs all the local ROS2 nodes (including the Universal Robots driver nodes,
   the MoveIt nodes, and the TableTop nodes). On startup, the **server**
   container will install any dependencies and build the project. It will then 
   run the launch file specified in the `LAUNCH_FILE` environment variable or
   the default in `compose.yaml` if `LAUNCH_FILE` is not set.
+ **robot**: The container for the Universal Robots simulator, which
   simulates the physical robot's teach pendant UI, as well as the safety
   constraints.
+ **novnc**: The noVNC container, which acts as a window manager for the
   **server** and **robot** containers through a web interface.

Once the containers are started, you can access the web interface at
http://localhost:8080/vnc.html in your web browser.

### Choosing launch files

There are 2 main launch files in the `tabletop_server/launch` directory:
- `tabletop_server.launch.py`: The main launch file for the TableTop project.
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

### VSCode Dev Container

To use the project with VSCode Dev Container:

1. Install the "Remote - Containers" extension in VSCode.
2. Open the project folder in VSCode.
3. When prompted, click "Reopen in Container" or use the command palette (F1)
   and select "Remote-Containers: Reopen in Container".

VSCode will build the Dev Container and provide you with a fully configured
development environment.

## Project Structure

The TableTop meta-package consists of the following ROS2 packages:

- `tabletop_msgs/`: TableTop message definitions
- `tabletop_moveit_interface/`: TableTop MoveIt interface
- `tabletop_moveit_config/`: TableTop MoveIt configurations
- `tabletop_server/`: TableTop server nodes and launch files
- `tabletop_teensy/`: TableTop Teensy nodes and launch files

Additional non-ROS2 packages:
- `novnc/`: Context for building and running noVNC Docker container
- `ursim/`: Contains the external control URCAP for the host machine
   to interface with the robot.
- `scripts/`: Utility scripts for setting up the environment (locally and
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

