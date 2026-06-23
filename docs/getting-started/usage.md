# Usage

## Building

All builds happen inside the container; the `tt-*` wrappers handle that for you.
`tt-build` takes a required component — `colcon`, `microros`, or `foxglove`:

```bash
tt-compose --profile='*' pull     # pull the prebuilt Docker images from Docker Hub
tt-build colcon --all             # full workspace incl. external modules (moveit2) — run this first
tt-build colcon                   # tabletop packages only (most common, after the first build)
tt-build colcon --clean-tabletop  # clean rebuild of tabletop packages
tt-build colcon -p tabletop_rig   # one package + its dependencies
tt-build microros                 # Teensy & Flic firmware (PlatformIO)
tt-build foxglove                 # Foxglove MoveIt plugin (.foxe -> $TABLETOP_DIR)
```

Run `tt-build colcon --all` the **first** time so the external modules (moveit2,
etc.) are built too. After that you can drop `--all` and just run
`tt-build colcon`, as long as you are only changing the tabletop packages.

See [CLI & Tooling](../guide/cli.md) for the full option list.

## Starting containers (profiles)

Services are grouped by Compose **profiles**. Bring up a whole session with
`tt-compose --profile=<name> up`:

| Profile | Brings up | Use case |
| --- | --- | --- |
| `sim` | mock UR, teensy-sim, flic-sim, eyelink-sim, rviz, foxglove, novnc | development with mock hardware |
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

## Foxglove visualization

The `foxglove` service exposes a [Foxglove](https://foxglove.dev/) WebSocket
bridge at `ws://localhost:8765`. Open the Foxglove app (web or desktop), choose
**Open connection → Foxglove WebSocket**, and enter that URL (replace the port
if you changed `FOXGLOVE_PORT` in `setup.bash`). See the
[Foxglove documentation](https://docs.foxglove.dev/docs) for the full
visualization workflow.

### MoveIt converter plugin (`.foxe`)

To visualize MoveIt planning scenes, install the bundled Foxglove extension.
Build and package it with:

```bash
tt-build foxglove                            # writes to $TABLETOP_DIR
tt-build foxglove -o ~/plugins               # write into a directory (keeps the packaged name)
tt-build foxglove -o ~/plugins/moveit.foxe   # write to a specific filename
```

This packages the `foxglove_moveit_msg_converter` extension and writes the
resulting `.foxe` file to `$TABLETOP_DIR` (the repository root) by default, or to
the `-o/--output` path — a directory (the plugin keeps its packaged name) or a
path ending in `.foxe` (the plugin is renamed to it). Install it into the
Foxglove app by opening the extensions settings and adding the local `.foxe`; see
the
[Foxglove extensions guide](https://docs.foxglove.dev/docs/visualization/extensions/introduction)
for the exact steps for your Foxglove version.

## Dev Container (VS Code)

Install the **Dev Containers** extension, run `tt-env-gen`, open the folder, and
choose **Reopen in Container**. Inside, `tt-build` and `tt-launch` work directly.
To open a shell in a non-dev service instead, use `tt-attach <service>` (see
[CLI & Tooling](../guide/cli.md)).

!!! note
    Source changes for nodes running in *other* containers require restarting
    or rebuilding those containers to take effect.

## Running a task

Tasks are the behavioral experiments. `tt-launch` runs inside a container. The
recommended way is to open a shell in the `commander` service with `tt-attach`
(or use the Dev Container) and run `tt-launch` from there:

```bash
tt-attach commander       # open a shell in a fresh commander container, then:
tt-launch tasks                                              # default foraging task, mock robot
tt-launch tasks task:=foraging_ordered robot_mode:=mock     # specific task + robot mode
tt-launch tasks task:=smooth_pursuit_random robot_mode:=real
```

Alternatively, run a task as a one-shot from the host — this spins up a temporary
`commander` container and tears it down when the task exits:

```bash
tt-compose run --rm commander tt-launch tasks task:=foraging_ordered robot_mode:=mock
```

`Ctrl-C` stops a running task. The `task` argument is a config filename (without
`.yaml`) from `tabletop_tasks/config/`; `robot_mode` is `mock` or `real`. See
[Tasks & Experiments](../guide/tasks.md) for the available tasks and how to
configure them.
