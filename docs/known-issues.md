# Known Issues Found During Documentation Review

Discrepancies between code, configuration, and documentation found
while building the architecture docs (2026-06). **No code was changed**
— these are flagged for maintainer review.

## Likely bugs

1. **`multi_ur.launch.py` references a missing config file.**
   `tabletop_rig/launch/multi_ur.launch.py:135` defaults
   `controllers_file` to `config/multi_controllers.yaml`, which does
   not exist (only `dual_controllers.yaml` does). The launch file will
   fail at runtime unless the argument is overridden. Either add the
   file, point the default at `dual_controllers.yaml`, or delete the
   launch file if `dual_ur.launch.py` superseded it.

2. **`flir_synchronized.launch.py` mis-parses `use_sim_time`.**
   Line ~103: `use_sim_time = bool(LaunchConfiguration("use_sim_time")
   .perform(context))` — `bool("false")` is `True` because any
   non-empty string is truthy. Passing `use_sim_time:=false` (the
   default!) sets the driver parameter to `True`. Should compare
   against the string, e.g. `perform(context).lower() == "true"`.

3. **`AIOExecutor.spin_until_future_complete` leaks
   `ConditionReachedException`.** The exception suppression in
   `tabletop_rig/executors.py:_spin_context_manager` is commented out,
   so callers crash with an `ExceptionGroup` when the awaited future
   completes. `nodes/system_check.py` works around it with a local
   `except*`; other future callers will hit the same trap.

## Outdated documentation (fixed where doc-only; listed for awareness)

4. **`CLAUDE.md` architecture section is stale.** It lists
   `interfaces/dashboard.py` (actual file: `interfaces/ur.py`) and
   places `tabletop_teensy` under `src/ros/tabletop/` (actual:
   `src/ros/tabletop/tabletop_micro/tabletop_teensy`, COLCON_IGNOREd
   PlatformIO firmware). It also omits `interfaces/moveit/requests.py`
   siblings `trajectory_cache_kdtree.py` / `trajectory_cache_lmdb.py`.

5. **`musings.md` references commands that no longer exist** (or never
   did under these names): `tt-display-set`, `tt-usbfs-configure`,
   `tt-udev-configure`, `tt-teensy-build`, `tt-teensy-connect`,
   `tt-calibrate`, `tt-cpu-speed-scaling-disable`, `tt-docker`, plus
   `scripts/docker_prune.sh`, `scripts/build.sh`, `scripts/piano.sh`.
   Current equivalents: `scripts/configure/usbfs-configure.sh`,
   `scripts/configure/udev-configure.sh`,
   `scripts/configure/cpu-speed-scaling-disable.sh`,
   `tt-microros-build` (for teensy builds), `tt-compose`/`tt-build`.
   The gaze CLI entry points are `tt-gaze-*` (not `gaze-*`).

## Minor

6. **`bin/container/tt-create-graph` is broken.** The
   `nodes_to_ignore` list is fully commented out, leaving
   `nodes_to_ignore=""`. An empty pattern in `grep -vE ""` matches
   every line, so the inverted grep outputs *nothing* — zero nodes are
   passed to `ros2_graph` (and `set -o pipefail` + grep's exit 1 may
   abort the script outright). Guard the grep behind a non-empty
   check, or restore at least one pattern.

7. **`commander_pretty.yaml`** appears to be an unconsumed alternate of
   `commander.yaml` (no launch file references it). Confirm whether it
   is still needed.
