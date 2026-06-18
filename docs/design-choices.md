# Design Choices

The major architectural decisions behind TableTop, and the rationale for each.

## ROS 2

The project uses [ROS 2](https://docs.ros.org/en/jazzy/index.html) as the main
framework for controlling the UR5e robot and for recording/compiling sensor and
state-space data. Reasons for choosing ROS 2 over a bespoke solution:

* ROS 2 provides a powerful, flexible framework for building complex
    distributed systems with many interdependent components.
* Its message-based architecture makes communication between nodes
    straightforward.
* It has built-in recording and playback via ROS 2 Bag files.
* It provides many packages for customizing each stage of the robot-control
    pipeline.
* Universal Robots ships a robust ROS 2 driver for the UR5e, making integration
    with existing ROS 2 pipelines easy.
* It supports real-time kernels, useful for ensuring critical tasks execute on
    time (e.g. closed-loop motion control).

A bespoke solution would require significant development time and would limit
the ability to incorporate new features such as feedback-driven robot control.

## MoveIt 2

The project uses [MoveIt 2](https://moveit.picknik.ai/main/index.html) for
planning and control of the UR5e:

* MoveIt 2 provides utilities for planning and control, plus real-time
    visualization of the robot and environment state.
* It supports a variety of motion controllers and planning algorithms, each
    with its own customization options.
* The Universal Robots ROS 2 driver ships with MoveIt 2 functionality
    pre-configured.

The alternative (sending URScript commands directly to the robot) limits control
to what the robot's software provides, which does not support complex scenario
planning or feedback control.

## Docker

The project runs entirely in [Docker](https://www.docker.com/) containers, which
provide:

* OS-agnostic development and deployment environments, accessible regardless of
    host hardware.
* An isolated environment for each component of the software stack, each with
    its own dependencies and configuration already set up.
* A consistent, reproducible environment for development and deployment.
* A quick way to run the software without installing and configuring
    dependencies by hand.

Developing on bare metal would require manual dependency management and
configuration, which is time-consuming, error-prone, and often system-breaking.
It also limits platform compatibility, since ROS 2 is optimized for Ubuntu. The
UR Simulator additionally cannot be installed on Apple Silicon and must be run
in a container.
