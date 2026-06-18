# TableTop

TableTop is a [ROS 2](https://docs.ros.org/en/jazzy/index.html)-based robotics
platform for controlling one or more Universal Robots UR5e arms in a monkey
electrophysiology research rig. It runs entirely in [Docker](https://www.docker.com/)
containers and uses [MoveIt 2](https://moveit.picknik.ai/) for motion planning.

The system presents physical objects to a subject, measures their responses
(button presses, eye gaze), delivers rewards, and records synchronized sensor
and robot-state data (via ROS 2 Bag files) for offline analysis. It also ships a
virtual tabletop environment for simulation and system testing, plus
post-processing tools for motion correction, spike sorting, NWB conversion, and
gaze estimation.

## Documentation

**Full documentation lives at <https://jazlab.github.io/tabletop/>.** Start there
for the project overview, setup, usage, and troubleshooting:

- **[Setup](https://jazlab.github.io/tabletop/getting-started/setup/)** — clone,
  configure the host, and bring up the containers.
- **[Real Hardware Setup](https://jazlab.github.io/tabletop/getting-started/real-hardware/)**
  — robot network, URCaps, and remote control.
- **[Usage](https://jazlab.github.io/tabletop/getting-started/usage/)** — build,
  start containers, and run a task.
- **[Design Choices](https://jazlab.github.io/tabletop/design-choices/)** — why
  ROS 2, MoveIt 2, and Docker.
- **[Architecture](https://jazlab.github.io/tabletop/architecture/)** — the
  conceptual dependency map and where to look when something breaks.
- **[Guide](https://jazlab.github.io/tabletop/guide/cli/)** — CLI tooling,
  configuration, the task system, hardware & safety, and troubleshooting.
- **[API Reference](https://jazlab.github.io/tabletop/reference/)** —
  auto-generated for `tabletop_py`, `tabletop_rig`, and `tabletop_tasks`.

The same pages live as Markdown under [`docs/`](docs/) if you prefer to read them
in the repository.

## Quick start

```bash
# Clone with submodules
git clone --recurse-submodules https://github.com/jazlab/tabletop.git
cd tabletop

# Source the environment (puts the tt-* commands on your PATH)
source setup.bash
```

See [Setup](https://jazlab.github.io/tabletop/getting-started/setup/) for the
full host configuration (Docker, `uv`, `.env` generation) and
[Usage](https://jazlab.github.io/tabletop/getting-started/usage/) for building
and launching.

## Contributing

Contributions are welcome. Fork the repo, install the dev dependencies
(`uv sync`) and pre-commit hooks (`pre-commit install`), make your changes on a
branch, and open a pull request. Python follows `ruff` formatting (79-char
lines, Google-style docstrings); pre-commit runs the linters automatically on
commit. Please follow the
[ROS 2 contribution guidelines](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing.html).

## License

[MIT](LICENSE)
