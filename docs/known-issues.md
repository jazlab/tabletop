# Known Issues Found During Documentation Review

Discrepancies between code, configuration, and documentation found
while building the architecture docs (2026-06), flagged for maintainer
review. Items that have since been fixed in this branch have been removed;
the remaining ones are open.

## Likely bugs

1. **`AIOExecutor.spin_until_future_complete` leaks
   `ConditionReachedException`.** The exception suppression in
   `tabletop_rig/executors.py:_spin_context_manager` is commented out,
   so callers crash with an `ExceptionGroup` when the awaited future
   completes. `nodes/system_check.py` works around it with a local
   `except*`; other future callers will hit the same trap.

## Likely bugs (continued)

1. **Copy-paste parameter bug in
    `interfaces/moveit/plan_and_execute.py:1031`.**
    `allowed_duration_margin = self.param("execution.allowed_duration_scaling")`
    reads the *scaling* parameter for the *margin* variable (the
    validation error message right below names the correct
    `allowed_duration_margin` key). The configured margin is ignored.

2. **Dropped `use_cache=False` in
    `interfaces/moveit/object_manipulation.py:1175-1178`.** A copy of
    `config.reset_request` is made and `use_cache` set to False, but
    `plan_and_execute` is then called with the *original*
    `config.reset_request` — the no-cache intent is silently lost.

3. **Arm-lock safety check disabled in `interfaces/teensy.py`.**
    `_msg_safe_to_execute` (line ~206) has the arm-lock condition
    (`is_left_arm_locked and is_right_arm_locked`) commented out and
    returns only `not msg.is_safety_laser_broken`. So robot motion is
    gated solely on the safety laser; the subject's arms being seated in
    the restraints is published by the firmware (`TeensySensor`) but NOT
    enforced. The method's docstring previously claimed both arms were
    checked — corrected to match the implementation. Confirm whether the
    arm-lock gate should be re-enabled (safety-relevant).

## Config review (config documentation pass)

1. **Typo'd joint name in `commander.yaml` `test_object_attached`.**
    `left_manipulation_interface.test_object_attached.joint_name` is
    `left_eblow_joint` and the right arm is `right_eblow_joint`
    (`commander.yaml:369`, `:444`). The correct UR joint is
    `*_elbow_joint` (cf. `tabletop_description` initial-position configs
    and `dual_view_robot.launch.py`). `object_manipulation.py:1422`
    reads this `test_object_attached` config, so the attach self-test
    would reference a non-existent joint. The config now carries an
    inline NOTE; the value itself was left unchanged.

2. **Unread config parameters in `commander.yaml`.**
    `*_manipulation_interface.trajectory_cache.base_link_name` and
    `common_manipulation_interface.execution.moved_tolerance` are never
    read in `tabletop_rig` (no matching `self.param(...)`). Either dead
    config or a missing code read — confirm intent.

3. **Duplicate Flic button MAC in `commander.yaml`.** `flic.bd_addrs`
    maps `big_object_3`, `big_object_7`, and `small_object_0` all to
    `90:88:a9:50:5f:b6` (`commander.yaml:40,44,45`). If these are meant
    to be distinct physical buttons this is a copy-paste error; if
    intentional (spare/unassigned), ignore.

4. **`config/gaze_estimation_geometric.yaml` visualize keys don't match
    `visualize.py`.** The file uses `visualize.eyelink_range` /
    `visualize.markers_range`, but `gaze/visualize.py` reads
    `visualize.animate_2d_dots` / `visualize.animate_3d_dots` (as in
    `gaze_estimation.yaml`). The preprocess section was realigned to the
    nested structure in this branch, but the visualize wrappers are still
    stale. The same file also uses a `data:` block where the shared pipeline
    (`gaze/utils.py::init_dataloaders`) reads `dataloaders:` (as in
    `gaze_estimation.yaml`) — realign if this config is ever fed to the MLP
    training pipeline.

## Firmware review (firmware documentation pass)

1. **Potential pin conflict in Teensy firmware.**
    `src/microros/tabletop_teensy/src/main.cpp`: `LEFT_ARM_LOCK_STATE_PIN`
    is `38` with a `// TODO: change back to 36`, but `BUTTON_STATE_PIN` is
    already `36`. Acting on the TODO without relocating the button pin would
    map both to pin 36.

2. **Misspelled enum `UNCRECOVERABLE_ERROR` in Teensy firmware.**
    The `agent_states` member is spelled `UNCRECOVERABLE_ERROR` (used
    consistently, so functionally harmless) while a nearby LED blink-pattern
    comment refers to `UNRECOVERABLE_ERROR`. Cosmetic; rename for clarity.

## Code smells / API warts

1. **Typo'd public API method: `Commander.manually_atatch_object`**
   (`nodes/commander.py`). Renaming it is NOT doc-safe: it is called
   under the typo'd spelling by `tabletop_tasks/tasks/smooth_pursuit.py:335`
   and `tabletop_tasks/tasks/dummy.py:380`. Fix requires renaming the
   method and both call sites together (or adding an alias).

2. **`interfaces/ur.py` `stop_program()`** fires `call_async` and
   never awaits or checks the returned future — failures are silent.

3. **`executors.py` `_queue_producer`** reports exceptions via bare
    `print` instead of the node/ROS logger.

## Outdated documentation (fixed where doc-only; listed for awareness)

1. **`CLAUDE.md` architecture section (corrected).** It previously listed
   `interfaces/dashboard.py` (actual file: `interfaces/ur.py`), placed
   `tabletop_teensy` directly under `src/ros/tabletop/` (actual:
   `src/microros/tabletop_teensy`, COLCON_IGNOREd
   PlatformIO firmware), and omitted the `interfaces/moveit/` cache siblings
   (`trajectory_cache_kdtree.py` / `trajectory_cache_lmdb.py`) and
   `requests.py`. These have since been corrected.

2. **`musings.md` references commands that no longer exist** (or never
   did under these names): `tt-display-set`, `tt-usbfs-configure`,
   `tt-udev-configure`, `tt-teensy-build`, `tt-teensy-connect`,
   `tt-calibrate`, `tt-cpu-speed-scaling-disable`, `tt-docker`, plus
   `scripts/docker_prune.sh`, `scripts/build.sh`, `scripts/piano.sh`.
   Current equivalents: host configuration (USB buffer size, udev rules, CPU
   governor) is now documented in
   [Real Hardware Setup](getting-started/real-hardware.md) rather than shipped
   as scripts; `tt-build microros` (for teensy builds); `tt-compose`/`tt-build`.
   The gaze CLI entry points are `tt-gaze-*` (not `gaze-*`).

## Resolved in the deprecation/cleanup branch

The following previously flagged items are no longer open — the relevant code
was removed or retired to `deprecated/` (see `deprecated/README.md`):

- **`commander_pretty.yaml`** (an unconsumed alternate of `commander.yaml`) was
  removed.
- **`bin/container/tt-create-graph`** — besides its earlier fixes (it referenced
  a non-existent style-config path and dropped all nodes when the ignore-list
  was empty), the script and its `ros_graph_style.yaml` were retired to
  `deprecated/ros-graph/`.
- **`trial_generators/blocked_cup_drawer.py`** (the inverted "correct" counter)
  was removed along with the non-alternating `ordered_choice` / `random_choice`
  generators; the surviving `*_alternating` variants are the supported path.
- **`tabletop_flic_micro` firmware** (the `resetButtonAds()` main-loop block)
  was retired to `deprecated/flic-button/`, superseded by the in-process scapy
  BLE sniffer.
