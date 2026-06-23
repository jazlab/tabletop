# Deprecated / archived components

This directory is an **archive**. Nothing here is built, imported, launched, or
on any runtime code path — `colcon`/`tt-build` ignore it and no active config
references it. Files are kept only so a curious maintainer can see how a
superseded subsystem worked, or revive it if needed.

Generated build artifacts (e.g. the firmware `.pio/` tree, `compile_commands.json`)
and runtime state (daemon logs, databases) were stripped before archiving; only
source-of-reference files remain.

## What moved here, and where it came from

| Archived path | Original location | Why retired |
| --- | --- | --- |
| `flic-button/flicd/` | `docker/flicd/` | `flicd` daemon superseded by the scapy BLE sniffer |
| `flic-button/client.py` | `src/tabletop_py/flic/client.py` | flicd socket client → `scapy_client.py` |
| `flic-button/piano.py` | `src/tabletop_py/flic/piano.py` | demo built on the flicd client |
| `flic-button/flic_node.py` | `tabletop_rig/.../nodes/deprecated/flic.py` | old ROS flic node (flicd-based) |
| `flic-button/tabletop_flic_micro/` | `src/ros/tabletop/tabletop_micro/tabletop_flic_micro/` | incomplete ESP32 firmware |
| `ursim/entrypoint.sh` | `docker/ursim/entrypoint.sh` | UR simulator support removed |
| `moveit-config/kinematics.yaml` | `tabletop_moveit_config/config/kinematics.yaml` | old single-arm variant |
| `moveit-config/tabletop.xrdf` | `tabletop_moveit_config/xrdf/tabletop.xrdf` | cuMotion/Isaac description, unused |
| `ros-graph/tt-create-graph` | `bin/container/tt-create-graph` | graph tooling no longer maintained |
| `ros-graph/ros_graph_style.yaml` | `config/ros_graph_style.yaml` | styling for `tt-create-graph` |
| `docker/buildkitd.toml` | `docker/buildkitd.toml` | BuildKit GC config, unreferenced |
| `compose-services.yaml` | extracted from `compose.yaml` | removed Compose service definitions |

## `flic-button/` — the old `flicd`-based Flic button stack

The Flic buttons are the subject's response device. The original stack ran the
proprietary **`flicd`** daemon, which connected to each button over BLE GATT to
read its click-type characteristic. That connection added tens of milliseconds
of latency between the press and the emitted event.

It was replaced by an in-process **scapy BLE sniffer**
(`src/tabletop_py/flic/scapy_client.py`, driven by
`src/ros/tabletop/tabletop_rig/tabletop_rig/nodes/flic.py`). The sniffer opens a
raw HCI socket and reports the button's wake-up advertisement directly, so it
catches the press at the contact-closure edge with no GATT round-trip and no
daemon. The `flic` Compose service simply needs `NET_ADMIN`.

- **`flicd/`** — the `flicd` binary plus its Docker assets (`entrypoint.sh`,
  `healthcheck.sh`, and the binary's `LICENSE`). It ran as the `flicd` Compose
  service (see [`compose-services.yaml`](compose-services.yaml)), exposing a
  socket server on port `5551`.
- **`client.py`** — the asyncio client that spoke the `flicd` socket protocol.
  It was exposed as the `tt-flic-client` entry point (removed from
  `pyproject.toml`). `scapy_client.py` deliberately mirrors its
  `asyncio.Protocol` structure.
- **`piano.py`** — the "FlicPiano" demo: plays musical notes from button presses
  (needs the `mingus` package from the `dev` dependency group). Built on
  `client.py`.
- **`flic_node.py`** — the earlier ROS node that used the flicd client (it lived
  at `tabletop_rig/.../nodes/deprecated/flic.py`). The active
  `tabletop_rig/.../nodes/flic.py` uses `scapy_client` instead.
- **`tabletop_flic_micro/`** — incomplete **ESP32 (Feather ESP32)** firmware that
  read Flic BLE advertisements on-device and published over micro-ROS. It was
  never finished: `resetButtonAds()` connected/disconnected on the main loop and
  could block long enough to stall the executor and drop the micro-ROS agent. It
  had no launch file. It was built with `tt-build microros -t flic_micro` (that
  target was removed from `bin/container/tt-build`).

  Only the files unique to this firmware are kept here — `src/main.cpp` (the
  firmware), `platformio.ini` (ESP32 board/deps), and `colcon.meta`. This is
  therefore **not a buildable PlatformIO package on its own**: to actually build
  it, copy the full package layout from `src/microros/tabletop_teensy/` (its
  `.vscode/`, `include/`, `lib/`, `test/`, `pio_compiledb.py`, `.gitignore`, and
  the `extra_packages/tabletop_interfaces` symlink) and drop these files in,
  then build it the way the Teensy firmware is built.

## `ursim/` — UR5e simulator support

`entrypoint.sh` was the entrypoint for the **`ursim`** Compose service
(`universalrobots/ursim_e-series` image), which ran a virtual UR controller and
teach pendant rendered to the noVNC display. The simulator was dropped:
`robot_mode` now supports only `mock` and `real`, and the `ursim` Compose
profile was removed. The service definition is preserved in
[`compose-services.yaml`](compose-services.yaml).

## `moveit-config/` — MoveIt configs that are no longer loaded

- **`kinematics.yaml`** — the old single-`manipulator:` IK config, with several
  alternative solvers (KDL, cached KDL, `pick_ik`) commented out. The active
  `tabletop_moveit_config/config/kinematics.yaml` uses per-arm
  `left_manipulator` / `right_manipulator` TRAC-IK blocks.
- **`tabletop.xrdf`** — a single-arm cuRobo/Isaac **XRDF** robot description for
  the `isaac_ros_cumotion` planning pipeline (also retired). The package's whole
  `xrdf/` directory was removed, including from its CMake `install(DIRECTORY …)`.

## `ros-graph/` — ROS graph generation tooling

- **`tt-create-graph`** wrapped `ros2_graph` to render the live ROS 2
  node/topic graph to `graph.md` (run after `tt-launch`, while nodes are up).
  Note: it `source`s `../../setup.bash` and `../utils.sh` relative to its old
  home `bin/container/`; to revive it, restore it there so those paths resolve.
- **`ros_graph_style.yaml`** was the graph styling config it consumed.

## `docker/buildkitd.toml`

BuildKit daemon configuration (garbage-collection policies and cache-size
limits). No longer referenced by the build setup.

## `compose-services.yaml`

The Compose **service definitions** removed from the top-level `compose.yaml`,
kept verbatim under a `deprecated` profile for reference (no user profile starts
them):

- **`ursim`** — the UR5e simulator (see above).
- **`flicd`** — the `flicd` daemon bridge (see above).
- **`flir-no-sync`** — the unsynchronized FLIR camera driver (one device per
  `$FLIR_DEV_*`), superseded by the hardware-synchronized `flir` service. (The
  `flir_no_sync` *launch target* still exists and is used by `tt-flir-reset`; it
  is only the standalone Compose service that was removed.)
