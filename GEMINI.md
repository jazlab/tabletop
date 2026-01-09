# Gemini Code Companion Context

This document provides a comprehensive overview of the `tabletop` project, designed to be used as a context file for the Gemini Code Companion.

## Project Overview

`tabletop` is a ROS 2 meta-package for controlling one or more Universal Robots UR5e robots in a monkey electrophysiology rig. It includes a virtual tabletop environment for simulation and testing, as well as tools for recording and post-processing sensor and robot state data.

The project is architected around ROS 2, leveraging its modularity and message-passing capabilities. Motion planning is handled by MoveIt 2, and the entire system is designed to run within Docker containers for portability and reproducibility.

### Key Technologies

- **ROS 2:** The core framework for communication and control.
- **MoveIt 2:** For robot motion planning and visualization.
- **Docker:** For containerization of the entire software stack.
- **Python:** For scripting, utilities, and ROS 2 nodes.
- **UR5e:** The primary robot platform.

## Building and Running

The project uses a Docker-based workflow. The main commands are managed through `docker-compose` and a set of helper scripts.

### Building the Project

The primary build command is `tt-build`, which is a wrapper around `colcon build`.

- **Build all packages:**
  ```bash
  tt-build --all
  ```
- **Build only the tabletop packages:**
  ```bash
  tt-build
  ```
- **Clean and build:**
  ```bash
  tt-build --clean
  ```

### Running the Project

The project is launched using `docker-compose` with different profiles for simulation (`sim`) and real hardware (`real`).

- **Launch the simulation environment:**
  ```bash
  docker-compose --profile sim up
  ```
- **Launch with real hardware:**
  ```bash
  docker-compose --profile real up
  ```

The `rig.launch.py` file is the main entry point for launching the robot control system. It is highly configurable through launch arguments, allowing for different combinations of hardware and software components to be launched.

## Development Conventions

The project follows standard ROS 2 development practices. Python code is formatted with `black` and linted with `ruff`, enforced by pre-commit hooks.

### Source Code Structure

The source code is organized into two main directories:

- **`src/ros`:** Contains the ROS 2 packages for the project.
- **`src/tabletop_py`:** Contains ROS-independent Python utilities.

The ROS 2 packages are further organized by functionality:

- `tabletop_description`: URDF and robot description files.
- `tabletop_moveit_config`: MoveIt 2 configuration.
- `tabletop_rig`: The main package for controlling the rig, including launch files and nodes.
- `tabletop_tasks`: For defining and running experimental tasks.
- `tabletop_interfaces`: Custom ROS 2 message definitions.

### Testing

The project has a `tests` directory, but the testing strategy is not explicitly documented in the files I've reviewed.

## Key Files

- **`README.md`:** The main entry point for understanding the project. It provides a detailed overview of the project, setup instructions, and usage examples.
- **`compose.yaml`:** The Docker Compose file that defines the services for the project. It's well-structured and uses profiles to manage different configurations.
- **`pyproject.toml`:** Defines the Python dependencies and project metadata.
- **`src/ros/tabletop/tabletop_rig/launch/rig.launch.py`:** The main launch file for the robot control system. It's a good starting point for understanding how the different components are launched and configured.
- **`.devcontainer/devcontainer.json`:** Defines the development container for VS Code, which is the recommended development environment.
