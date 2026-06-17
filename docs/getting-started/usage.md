# Usage

## Building

All builds happen inside the container; the `tt-*` wrappers handle that for you.

```bash
tt-compose build                 # Docker images + full ROS 2 workspace
tt-build                         # tabletop packages only (most common)
tt-build --clean-tabletop        # clean rebuild of tabletop packages
tt-build -p tabletop_rig         # one package + its dependencies
tt-build --all                   # everything, including external modules (moveit2)
```

See [CLI & Tooling](../guide/cli.md) for the full option list.

## Starting containers (profiles)

Services are grouped by Compose **profiles**. Bring up a whole session with
`tt-compose --profile=<name> up`:

| Profile | Brings up | Use case |
| --- | --- | --- |
| `sim` | mock UR, teensy-sim, flic-sim, eyelink-sim, rviz, foxglove, novnc | development with mock hardware |
| `ursim` | UR simulator + novnc | virtual teach pendant |
| `real` | ur, teensy, flic, eyelink, optitrack, flir, rviz, foxglove, novnc | real hardware |

```bash
# Simulation (most common for development)
tt-compose --profile=sim up [--detach]

# Real hardware
tt-compose --profile=real up

# Status / logs / teardown
tt-compose ps
tt-compose --profile=sim logs -f [<service>]
tt-compose --profile=sim down
```

Open the noVNC desktop (RViz, UR sim teach pendant) at
<http://localhost:8080/vnc.html> (replacing 8080 with
whatever port is set by `NOVNC_PORT` in `.env`) or
connect Foxglove to `ws://localhost:8765`.

## Dev Container (VS Code)

Install the **Dev Containers** extension, run `tt-env-gen`, open the folder, and
choose **Reopen in Container**. Inside, `tt-build` and `tt-launch` work directly.

!!! note
    Source changes for nodes running in *other* containers require restarting
    or rebuilding those containers to take effect.

## Running a task

Tasks are the behavioral experiments. Launch them from the host (which spins up
a temporary `commander` container) or from inside a container:

```bash
# Default foraging task, mock robot
tt-launch tasks

# Specific task + robot mode (must match the profile you started)
tt-launch tasks task:=foraging_ordered robot_mode:=mock
tt-launch tasks task:=smooth_pursuit_random robot_mode:=real
```

`Ctrl-C` stops a running task. The `task` argument is a config filename (without
`.yaml`) from `tabletop_tasks/config/`; `robot_mode` is `mock`, `ursim`, or
`real`. See [Tasks & Experiments](../guide/tasks.md) for the available tasks and
how to configure them.
