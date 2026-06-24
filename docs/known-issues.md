# Known Issues Found During Documentation Review

Discrepancies between code, configuration, and documentation found
while building the architecture docs (2026-06), flagged for maintainer
review. Items that have since been fixed in this branch have been removed;
the remaining ones are open.

> **2026-06-24 triage.** Every open item below was re-verified against
> `main` (commit `3e9d697`) — all are still present; none were removed as
> fixed. The list was then expanded with substantive `TODO` comments found
> in tracked source and with the pending work captured in the maintainer's
> `todo.md` (see the new sections: *Functional issues flagged in `todo.md`*,
> *Source TODOs worth tracking*, *Build / developer-environment cleanups*,
> and *Documentation gaps and inconsistencies*). A prioritized,
> worktree-parallelizable fix plan lives in `docs/fix-plan.md`.

## Likely bugs

1. **`AIOExecutor.spin_until_future_complete` leaks
   `ConditionReachedException`.** `spin_until_future_complete`
   (`tabletop_rig/executors.py:416`) drives `_spin_impl`, which raises
   `ConditionReachedException` (`executors.py:823`) when its wait condition
   is met but never catches it — so the exception propagates to the caller,
   wrapped as an `ExceptionGroup` by the spin `TaskGroup`.
   (`_spin_context_manager` no longer contains the once-commented-out
   suppression at all.) `nodes/system_check.py:433` works around it with a
   local `except* ConditionReachedException`; other future callers will hit
   the same trap.

## Likely bugs (continued)

1. **Copy-paste parameter bug in
    `interfaces/moveit/plan_and_execute.py:1033-1034`.**
    `allowed_duration_margin = self.param("execution.allowed_duration_scaling")`
    reads the *scaling* parameter for the *margin* variable (the
    validation error message right below names the correct
    `allowed_duration_margin` key). The configured margin is ignored.

2. **Dropped `use_cache=False` in
    `interfaces/moveit/object_manipulation.py:1229-1232`.** A copy of
    `config.reset_request` is made and `use_cache` set to False
    (`reset_request = copy(...)`), but `plan_and_execute` is then called
    with the *original* `config.reset_request` — the no-cache intent is
    silently lost (the call passes `cache_trajectories=False`, which
    suppresses *writing* the cache but not *reading* a stale entry).

3. **Arm-lock safety check disabled in `interfaces/teensy.py`.**
    `_msg_safe_to_execute` (`teensy.py:198-217`) has the arm-lock condition
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
    (`commander.yaml:382`, `:457`). The correct UR joint is
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
    `90:88:a9:50:5f:b6` (`commander.yaml:48,52,53`). If these are meant
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
   under the typo'd spelling by `tabletop_tasks/tasks/smooth_pursuit.py:370`
   and `tabletop_tasks/tasks/dummy.py:439`. Fix requires renaming the
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

## Functional issues flagged in `todo.md` (2026-06-24 triage)

Pulled from the maintainer's `todo.md`. None were fixed at triage time;
listed here so they live alongside the rest of the open issues.

1. **Teensy spuriously reports the safety laser broken on the first
    trial.** Right after the Teensy node starts (or on a task's first
    trial) `is_safety_laser_broken` reads true until the laser is manually
    broken and un-broken once. Suspected debounce-initialisation bug in the
    firmware (`main.cpp`); if so, the same flaw affects the other debounced
    sensors. The commander-side `safety_laser_last_time_broken` handling may
    also need revisiting. Safety-relevant — see the ISR note below.

2. **Debounce ISRs read epoch-synchronised time.** The debounced-sensor
    ISRs in `main.cpp` timestamp using the ROS-epoch-synchronised clock
    (`rmw_uros_epoch_nanos`), which is not interrupt-safe — the epoch sync
    can be mid-update when the ISR fires. Latch a monotonic time in the ISR
    and convert outside it if confirmed.

3. **UR `safety_restart` recovery doesn't finish.** The `safety_restart`
    dashboard service works, but the rest of the reset sequence fails once
    safety mode returns to normal (`/left/dashboard_client/is_in_remote_control
    service call timed out!`). The driver likely has to reconnect afterwards
    (`interfaces/ur.py` recovery state machine).

4. **Unverified bringup parameter warnings.** rviz/UR bringup logs
    `joint_state_broadcaster.fallback_controllers is not initialized`
    (ros2_control) and `publish_robot_description_semantic is not
    initialized` (rviz). Believed benign — the rviz node is likely missing
    the full MoveIt config in its parameters — but confirm and silence.

5. **`group_name` vs `robot_name` conflation.** Tasks, trial specs, and
    trial generators pass `group_name` where they should pass `robot_name`
    (the commander looks up a manipulation context manager by robot name,
    not MoveIt group name; they happen to be equal today). Make `robot_name`
    the task-facing parameter and keep `group_name` as a property of the
    controlled robot. Touches `tabletop_tasks` plus the
    `ObjectManipulationInterface` / `PlanAndExecuteInterface` plumbing — a
    larger refactor.

6. **Retire `rig.launch.py`; flatten `tasks.launch.py`.** Each launch file
    now runs in its own compose service, so `rig.launch.py` (which bundles
    `commander.launch.py` + others) should move to `deprecated/`, and
    `tasks.launch.py` should include `commander.launch.py` /
    `rosbag.launch.py` directly instead of via `rig.launch.py`. The
    `# TODO: … unscoped …` markers at `rig.launch.py:310` and
    `tasks.launch.py:99` are part of this.

## Source TODOs worth tracking (2026-06-24 scan)

Substantive `TODO`s found in tracked source (trivial "decide later" markers
are bundled at the end). All still present.

1. **`tasks/foraging.py:154` `# TODO: Remove!!!`** sits directly above an
    unconditional `await self.commander.release_arm(arm)` at the top of the
    response phase — looks like temporary/debug behaviour that should not
    ship. Confirm intended response-phase arm handling.

2. **`tasks/smooth_pursuit.py:368` `# TODO: FIX!!!`** — the real
    `await manipulator.fetch_object(...)` is commented out and replaced by
    the typo'd `manually_atatch_object` workaround (see *Code smells* #1).
    So smooth-pursuit never actually picks the object up; it teleports it
    onto the gripper. The follow-on `plan_and_move("fetched")` carries a
    `# TODO: Maybe remove`.

3. **Eyelink node TODOs (`nodes/eyelink.py`).** `:397`
    `# TODO: Fix callback groups` (possible concurrency-correctness issue),
    `:915` discard stale samples before collection (data quality), `:926`
    "verify this fix", `:213-214` / `:717-718` commented tuning, and a
    content-free `:1402 # TODO: Something is fucking wrong, help me` at
    end-of-file that should be removed or turned into a real ticket.

4. **`nodes/flic.py:323` `# TODO: Change to custom message`** — the Flic
    response is published on a generic message type.

5. **Low-priority "decide later" markers** (investigate or delete, no known
    breakage): `compose.yaml:203` `SYS_NICE` cap, `moveit_controllers.yaml:13`,
    the duplicated `moveit_config.to_dict()` "which one to use" TODOs in
    `moveit.launch.py:212` / `rviz.launch.py:114`,
    `interfaces/moveit/moveit.py:465`,
    `interfaces/moveit/plan_and_execute.py:628` ("maybe revalidate"),
    `interfaces/ur.py:542`, `object_manipulation.py:1759` ("check
    presentation region"), and `tabletop_py/gaze/preprocess.py:627`.

## Build / developer-environment cleanups (2026-06-24, from `todo.md`)

1. **`mingus` is in the wrong dependency group.** `pyproject.toml:49` lists
    `mingus` under `[dependency-groups].dev`, but it is imported at runtime
    by `SoundInterface` — move it into `[project].dependencies`.

2. **Devcontainer is missing the GitHub CLI feature.** Add
    `"ghcr.io/devcontainers/features/github-cli:1": {}` to
    `.devcontainer/devcontainer.json` `features`.

3. **`docker-bake.hcl` is single-platform.** Add multi-platform targets
    (`linux/amd64` + `linux/arm64`) and review the image tags (the
    `jazlabtabletop/*` tags have no `:latest`).

4. **`.vscode/c_cpp_properties.json` IntelliSense paths are stale.** Point
    the `pio_teensy` configuration at the correct path and remove the
    `pio_sniffer` configuration (the BLE-sniffer firmware was retired to
    `deprecated/`).

## Documentation gaps and inconsistencies (2026-06-24, from `todo.md`)

1. **`TeensySensor.msg` header is inaccurate.** It credits the
    "tabletop_micro firmware" (the firmware package is `tabletop_teensy` in
    `src/microros/`) and hard-codes the "100 Hz" publish rate, which is
    configurable. Fix the package reference and drop the rate.

2. **`deprecated/README.md` references a stale path.** It cites
    `src/ros/tabletop/tabletop_micro/tabletop_flic_micro/`; reconcile with
    the actual original path / current `deprecated/` location.

3. **`bin/` scripts have no `-h|--help`.** `tt-build`, `tt-compose`,
    `tt-launch`, … print no usage. Add a help option to each.

4. **Per-node/interface parameters aren't in the published docs.** Most are
    documented only inline in the `config/*.yaml` files; surface them in
    `docs/` (descriptions can be copied from the configs).

5. **Missing FLIR GenICam reference.** Link the BFS-U3-23S3 node reference
    (<https://softwareservices.flir.com/BFS-U3-23S3/latest/Model/public/index.html>)
    from the camera-config docs, tying it to `blackfly_s.yaml` /
    `flir_synchronized.yaml`.

6. **Foxglove layouts aren't in the repo or docs.** Export the layout
    configs, commit them (the two examples currently live only in a local
    `share/` dir), document how to import them, and add the note that
    Foxglove must be open and focused at task start to receive all
    planning-scene updates.

7. **Setup-doc gaps.** Add `git-lfs` and `jq` to the prerequisites, note
    that a full reboot may be needed after adding the user to the `docker`
    group, and note that the `platformio-core` Docker volume is owned by the
    first user to create it (fix: delete the volume and re-run
    `tt-build microros`).

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
- **`musings.md`** (which referenced many commands/scripts that no longer exist)
  was removed; its still-relevant troubleshooting content lives in
  [Troubleshooting](guide/troubleshooting.md) with the current command set.
