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
tt-build microros                 # Teensy firmware (PlatformIO)
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

### Layout configs

Foxglove lets you save and restore panel arrangements as **layout** JSON files.
To import a layout:

1. Open Foxglove (web or desktop) and connect to the WebSocket bridge.
2. Click the **layout** icon (top-left panel selector) → **Import from file…**
3. Select a `.json` layout file and confirm.

Two sample layout JSON files have been provided in the `share/foxglove/`
directory: `planning-scene-layout.json` and `cameras-layout.json`.
The former includes a panel for the planning scene as well as six
small panels for each Flir camera feed. The latter has the same
six cameras but also a plotting panel to verify that the Teensy
sync pulse is being seen on the camera meta-data topics.

### MoveIt converter plugin (`.foxe`)

To visualize MoveIt planning scenes, install the bundled Foxglove extension.
Build and package it with:

```bash
tt-build foxglove                            # writes to $TABLETOP_DIR
tt-build foxglove -o ~/plugins               # write into a directory (keeps the packaged name)
tt-build foxglove -o ~/plugins/moveit.foxe   # write to a specific filename
```

This packages the `src/foxglove_moveit_msg_converter` extension and writes the
resulting `.foxe` file to `$TABLETOP_DIR` (the repository root) by default, or to
the `-o/--output` path — a directory (the plugin keeps its packaged name) or a
path ending in `.foxe` (the plugin is renamed to it). Install it into the
Foxglove app by opening the extensions settings and adding the local `.foxe`; see
the
[Foxglove extensions guide](https://docs.foxglove.dev/docs/visualization/extensions/introduction)
for the exact steps for your Foxglove version.

!!! warning "Keep Foxglove open and focused when a task starts"
    The MoveIt planning scene is pushed as a burst of messages at task
    initialisation.  If Foxglove is **not open and focused** (e.g. the browser
    tab is backgrounded or the app is minimised) when the task starts, it will
    miss those planning-scene updates and the 3-D scene will appear empty or
    incomplete.  Re-open the connection **before** launching a task, or trigger
    a scene refresh after connecting.

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

Pass `rosbag:=true` to also record the session to an MCAP bag (topics from
`rosbag.yaml`); recording is off by default. See below for turning a recorded
bag into analysis-ready files.

## Converting recorded bags

Sessions are recorded as MCAP rosbags. Two exporters turn a bag into per-topic
CSVs and decoded image files. They share the same generic flattening (nested
fields → `pose.position.x` columns, sequences → `name[0]` columns; image topics
are decoded to image files), so you can use whichever fits your environment.

**`tabletop_unbag` (`unbag`, recommended)** — a standalone C++ exporter with no
Python/pandas/rclpy runtime dependency. It is multithreaded, streams the bag
(bounded memory, so it handles very large camera-heavy bags), and is resumable
(an interrupted run leaves valid partial output and picks up where it left off).
It is built by the normal `tt-build colcon`:

```bash
# Export everything into <parent of BAG_DIR>/unbag/
ros2 run tabletop_unbag unbag /path/to/session/bag

# Choose the output directory; only CSV (skip image topics)
ros2 run tabletop_unbag unbag BAG_DIR -o /path/to/out --handlers csv

# Restrict to some topics; save every image as lossless PNG
ros2 run tabletop_unbag unbag BAG_DIR --topics /joint_states /eyelink/sample
ros2 run tabletop_unbag unbag BAG_DIR --handlers image --image-format png

# Re-run from scratch instead of resuming
ros2 run tabletop_unbag unbag BAG_DIR --overwrite
```

CSV output is byte-for-byte identical to the Python converter, with one
intentional difference: fixed-size primitive arrays (e.g. `CameraInfo.k`) are
expanded to indexed columns (`k[0]..k[8]`). The full flag set (handler
selection, `--jobs`, `--csv-batch-size`, `--image-encoding`, resume/overwrite
semantics) is documented in `src/ros/tabletop/tabletop_unbag/README.md`.

**Python converter (`rosbag_convert`)** — the original `rosbag_to_csv` module,
still available as a launch target. Requires rclpy + pandas (already present in
the commander image):

```bash
tt-launch rosbag_convert        # ≡ ros2 run tabletop_rig rosbag_to_csv
```
