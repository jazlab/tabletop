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

## Likely bugs (continued)

11. **Copy-paste parameter bug in
    `interfaces/moveit/plan_and_execute.py:1031`.**
    `allowed_duration_margin = self.param("execution.allowed_duration_scaling")`
    reads the *scaling* parameter for the *margin* variable (the
    validation error message right below names the correct
    `allowed_duration_margin` key). The configured margin is ignored.

12. **Dropped `use_cache=False` in
    `interfaces/moveit/object_manipulation.py:1175-1178`.** A copy of
    `config.reset_request` is made and `use_cache` set to False, but
    `plan_and_execute` is then called with the *original*
    `config.reset_request` — the no-cache intent is silently lost.

13. **Inverted "correct" counter in
    `trial_generators/blocked_cup_drawer.py:133`.** `_num_correct` is
    incremented when `feedback.timeout` is True, but in the tasks that
    produce feedback (`tasks/foraging.py:177-181`) `timeout=True`
    means the subject did NOT respond. Blocks therefore switch after N
    *failed* trials, contradicting the `correct_trials_per_block`
    name. Docstrings now describe the implemented behavior; the logic
    needs review.

## Config review (config documentation pass)

14. **Typo'd joint name in `commander.yaml` `test_object_attached`.**
    `left_manipulation_interface.test_object_attached.joint_name` is
    `left_eblow_joint` and the right arm is `right_eblow_joint`
    (`commander.yaml:369`, `:444`). The correct UR joint is
    `*_elbow_joint` (cf. `tabletop_description` initial-position configs
    and `dual_view_robot.launch.py`). `object_manipulation.py:1422`
    reads this `test_object_attached` config, so the attach self-test
    would reference a non-existent joint. The config now carries an
    inline NOTE; the value itself was left unchanged.

15. **Unread config parameters in `commander.yaml`.**
    `*_manipulation_interface.trajectory_cache.base_link_name` and
    `common_manipulation_interface.execution.moved_tolerance` are never
    read in `tabletop_rig` (no matching `self.param(...)`). Either dead
    config or a missing code read — confirm intent.

16. **Duplicate Flic button MAC in `commander.yaml`.** `flic.bd_addrs`
    maps `big_object_3`, `big_object_7`, and `small_object_0` all to
    `90:88:a9:50:5f:b6` (`commander.yaml:40,44,45`). If these are meant
    to be distinct physical buttons this is a copy-paste error; if
    intentional (spare/unassigned), ignore.

## Code smells / API warts

8. **Typo'd public API method: `Commander.manually_atatch_object`**
   (`nodes/commander.py`). Renaming it is NOT doc-safe: it is called
   under the typo'd spelling by `tabletop_tasks/tasks/smooth_pursuit.py:335`
   and `tabletop_tasks/tasks/dummy.py:380`. Fix requires renaming the
   method and both call sites together (or adding an alias).

9. **`interfaces/ur.py` `stop_program()`** fires `call_async` and
   never awaits or checks the returned future — failures are silent.

10. **`executors.py` `_queue_producer`** reports exceptions via bare
    `print` instead of the node/ROS logger.

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
