# Known Issues

Discrepancies between code, configuration, and documentation, plus tracked
`TODO`s and pending functional work, flagged for maintainer review.

> **2026-06-29 reconciliation.** All of **Wave 1** of the fix plan
> (`docs/fix-plan.md`) has been merged into `main` (PRs #19–#30), and the
> `tabletop_unbag` bag→CSV/image converter (PR #17) was added. This file has been
> reconciled against that state:
>
> - Items fixed in Wave 1 moved to [Resolved in Wave 1](#resolved-in-wave-1-2026-06)
>   with their PR numbers.
> - The items that remain **open** — by design, or scheduled for Wave 2 — are
>   listed first.
> - Earlier deprecation-branch history is unchanged at the bottom.
>
> Each formerly-open item was re-verified by *content* against `main`, not just
> by commit title.
>
> **2026-06-30 update (PR review).** The three items that were awaiting hardware
> validation — UR `safety_restart` recovery, the Teensy safety firmware (#30),
> and the Eyelink stale-sample drain — have been **confirmed on real hardware**
> and are now resolved. The `tasks.launch.py` `rosbag` default was confirmed
> intended `false` and the docstring fixed. Two new Wave-2 items were added
> (Open §B5/§B6), and `tt-launch` was reduced to a pure `ros2 launch` wrapper
> (the `rosbag_convert` and `discovery` targets were removed).

---

## Open issues

### A. Open by design / deferred

1. **smooth-pursuit teleports the object onto the gripper (design decision).**
    `tasks/smooth_pursuit.py` still calls `manually_attach_object` instead of
    `await manipulator.fetch_object(...)` (the real fetch is commented out, with
    the `# TODO: FIX!!!` and follow-on `plan_and_move("fetched")`
    `# TODO: Maybe remove` deliberately retained). Whether to keep the
    smooth-pursuit object in the grid and physically pick it, or manually attach
    it at task start, is an unresolved design decision — kept open intentionally.

2. **`nodes/flic.py:323` publishes the response on a generic message type**
    (`# TODO: Change to custom message`). Deferred from the PR #26 cleanup: it is
    a feature addition needing a new `tabletop_interfaces` `.msg` and a colcon
    rebuild, out of scope for a comment cleanup.

### B. Larger refactors (Wave 2)

3. **`group_name` vs `robot_name` conflation.** Tasks, trial specs, and trial
    generators pass `group_name` where they should pass `robot_name` (the
    commander looks up a manipulation context manager by robot name, not MoveIt
    group name; they happen to be equal today). Make `robot_name` the task-facing
    parameter and keep `group_name` a property of the controlled robot. Touches
    `tabletop_tasks` plus the `ObjectManipulationInterface` /
    `PlanAndExecuteInterface` plumbing — see `fix-plan.md` WT-I.

4. **Arm-lock terminology is mechanism-specific.** The rig currently uses a
    **button per hand** (unpressed = the hand is free) plus a per-arm buzzer to
    cue which hand to use; a physical arm lock may be added later. The wording
    "electromagnetic arm lock" (e.g. `interfaces/teensy.py`) and the
    `SetArmLock` / `*_arm_lock*` naming across `tabletop_interfaces`, firmware,
    `commander.yaml`, and docs is inaccurate. The safety gate is now
    *configurable* (`safe_to_execute.require_arm_locks`, see Resolved §B3) but
    still uses this naming. Reword repo-wide to be agnostic to the hold/detect
    mechanism — see `fix-plan.md` WT-N.

5. **Safe-execution should gate on the presentation region, not the manipulation
    state.** The current safety stop checks whether the arm is in the `PRESENT`
    manipulation state to decide whether motion must be prevented. But while the
    object is actually being presented the arm is still in the `FETCHED` state,
    so the subject can be within reach during that window. Gate safe-execution on
    whether the arm has **acquired the presentation region** (cf. the
    `object_manipulation.py:1759 # TODO: Check if robot is in presentation
    region` marker in §C), not on the manipulation-state label alone.
    Safety-relevant — see `fix-plan.md` WT-O.

6. **Retire the legacy Python bag converter.** Move
    `tabletop_rig/utils/rosbag.py` (`rosbag_to_csv`) to `deprecated/`, and stop
    the gaze-estimation calibration scripts from importing it to convert bags
    themselves. Instead, the operator unbags the gaze/marker data with
    `tabletop_unbag` (`unbag`) first, and calibration consumes the resulting CSVs.
    (The `rosbag_convert` `tt-launch` target was already removed; the entry point
    and the calibration import remain until this lands.) See `fix-plan.md` WT-P.

### C. Low-priority "decide later" markers (P4)

Investigate or delete; no known breakage. Still present after Wave 1:

- `compose.yaml:203` — `SYS_NICE # TODO: see if needed` on the commander service.
- `interfaces/moveit/moveit.py:465` — `# TODO: Should probably use this`.
- `interfaces/moveit/plan_and_execute.py:628` — `# TODO: Maybe revalidate`.
- `interfaces/ur.py:542` — `# TODO: See if this is necessary`.
- `interfaces/moveit/object_manipulation.py:1759` — `# TODO: Check if robot is
  in presentation region`.

(The `moveit_controllers.yaml`, `moveit.launch.py` / `rviz.launch.py` `to_dict()`,
and `gaze/preprocess.py` markers from the previous list were resolved — see
Resolved §F.)

---

## Resolved in Wave 1 (2026-06)

Every item below was open at the 2026-06-24 triage and is now fixed on `main`.
Grouped by the original section, with the merging PR.

### A. Likely bugs

- **Executor leaked `ConditionReachedException`** → **#25.**
  `AIOExecutor.spin_until_future_complete` now catches the signal internally
  (bare or `ExceptionGroup`-wrapped) and returns cleanly; the
  `system_check.py` `except*` workaround was removed.
- **Copy-paste parameter bug** (`plan_and_execute.py`) → **#19.**
  `allowed_duration_margin` now reads `execution.allowed_duration_margin`.
- **Dropped `use_cache=False` on reset** (`object_manipulation.py`) → **#19.**
  The `use_cache=False` copy of `reset_request` is now the one passed to
  `plan_and_execute`, so the reset path no longer reads a stale cached
  trajectory.

### B. Config

1. **Typo'd joint name** in `commander.yaml` `test_object_attached` → **#22.**
   `left_eblow_joint` / `right_eblow_joint` corrected to `*_elbow_joint`.
2. **Unread config parameters** → **#22.** `trajectory_cache.base_link_name` and
   `execution.moved_tolerance` were confirmed to have no reader and were deleted
   as dead config.
3. **Arm-lock safety check** (`teensy.py::_msg_safe_to_execute`) → **#30 / #22.**
   Per the maintainer decision, the gate was **not** force-re-enabled; instead it
   is now configurable via `safe_to_execute.require_arm_locks` in
   `commander.yaml`, defaulting to `false` (laser-only) to preserve existing
   behaviour. (Validated on real hardware; mechanism-agnostic rename is Open §B4.)
4. **Duplicate Flic MAC** for `big_object_3` / `big_object_7` / `small_object_0`
   → **intentional.** Confirmed by the maintainer as deliberately shared
   (spare/unassigned); left unchanged.
5. **`gaze_estimation_geometric.yaml` keys out of sync** → **#23.** Realigned to
   `gaze_estimation.yaml`: `data:` → `dataloaders:`, and the `visualize:` block
   now uses `animate_2d_dots` / `animate_3d_dots`. The two configs are now
   identical except the `model:` block (model-creation parameters).

### C. Firmware

1. **Teensy pin "conflict"** (`LEFT_ARM_LOCK_STATE_PIN`) → **#30.** Resolved as
   *intentional*: pin 38 is deliberate because pin 36 is `BUTTON_STATE_PIN`; the
   stale `// TODO: change back to 36` was removed and the rationale documented.
2. **Misspelled enum `UNCRECOVERABLE_ERROR`** → **#30.** Renamed to
   `UNRECOVERABLE_ERROR` throughout.

### D. Code smells / API warts

1. **`Commander.manually_atatch_object`** → **#21.** Renamed to
   `manually_attach_object`; both call sites (`smooth_pursuit.py`, `dummy.py`)
   updated in the same commit; the deprecated alias was not kept.
2. **`ur.py::stop_program()` ignored its future** → **#20.** Now tracks the
   rclpy future across calls (skip while pending, retry on failure, suppress
   until `reset()`).
3. **`executors.py` bare `print`** → **#25.** Exceptions now go through the
   node/ROS logger.

### E. Functional issues from `todo.md`

1. **First-trial false "laser broken"** → **#30** (debounce-init fix; **validated
   on real hardware** 2026-06-30).
2. **Debounce ISRs read epoch-synced time** → **#30** (latched monotonic time;
   validated with the firmware bench check above).
3. **UR `safety_restart` recovery** → **#20** + recovery-state-machine reconnect
   (`ur.py::_reconnect`); **validated on the real robot** 2026-06-30.
4. **Unverified bringup parameter warnings** → **#29.** `fallback_controllers`
   silenced via an explicit empty list in `dual_controllers.yaml`;
   `publish_robot_description_semantic` set `False` on the rviz/moveit
   visualization nodes (and `True` only on the commander).
5. **`group_name` vs `robot_name`** → still open, Wave 2 (Open §B3).
6. **Retire `rig.launch.py`; flatten `tasks.launch.py`** → **#29.**
   `rig.launch.py` moved to `deprecated/launch/`; `tasks.launch.py` now includes
   `commander.launch.py` / `rosbag.launch.py` directly; the `tt-launch rig`
   target and the unscoped-group `TODO`s were removed.

### F. Source TODOs and P4 markers

- **`foraging.py` response-phase `release_arm`** → **#21.** Per decision, the
  unconditional `release_arm(arm)` was kept and the `# TODO: Remove!!!` removed.
- **Eyelink node TODOs** (`:397` callback groups, `:915` stale samples, `:926`
  "verify this fix", `:213/717` tuning, `:1402` content-free) → **#26.** All
  resolved (callback-group rationale documented; stale-sample drain implemented
  and **validated on a live Eyelink** 2026-06-30; dead comments removed).
- **`moveit_controllers.yaml:13`**, **`moveit.launch.py` / `rviz.launch.py`
  `to_dict()`** → **#29** (kept with documented rationale).
- **`gaze/preprocess.py` marker** → resolved (no TODO remains in that file).

### G. Build / developer-environment

All → **#24:** `mingus` moved to `[project].dependencies`; the GitHub CLI
devcontainer feature added; `docker-bake.hcl` made multi-platform
(`linux/amd64` + `linux/arm64`); `.vscode/c_cpp_properties.json` `pio_teensy`
path fixed and the stale `pio_sniffer` config removed.

### H. Documentation gaps

- **`TeensySensor.msg` header** (wrong package, hard-coded "100 Hz") → **#28.**
- **`deprecated/README.md` stale paths** → **#28 / #29** (flic-micro path and the
  new `launch/rig.launch.py` entry).
- **`bin/` scripts had no `-h|--help`** → **#27.**
- **Per-node/interface parameters not in published docs** → **#28**
  (`docs/guide/parameters.md`).
- **Missing FLIR GenICam reference**, **Foxglove layouts not in repo**, **setup
  gaps** (git-lfs/jq, reboot after `usermod -aG docker`, platformio-core volume
  ownership) → **#28** (`share/foxglove/*.json`, setup/usage docs).
- **`tabletop_unbag` (PR #17) undocumented globally** → resolved in this PR:
  added to `CLAUDE.md`, `architecture.md` (§5.5), `cli.md`, and
  `usage.md` (*Converting recorded bags*); package detail stays in
  `tabletop_unbag/README.md`.
- **`CLAUDE.md` / docs stale after the launch refactor** (the retired
  `tt-launch rig` target, the "arm-lock commented out" safety note, the
  "Teensy & Flic firmware" label) → resolved in this PR.
- **`tasks.launch.py` `rosbag` default doc/code drift** → resolved in this PR:
  the intended default is `false`, so the docstring was corrected to match the
  `default_value="false"` declaration (recording off unless `rosbag:=true`).
- **`tt-launch` simplified to a pure `ros2 launch` wrapper** → resolved in this
  PR (per review): the `rosbag_convert`/`rosbag_to_csv` target and the unused
  FastDDS `discovery` target were removed, and with them the `EXEC`/`RUN`
  dispatch machinery. Docs (`CLAUDE.md`, `cli.md`, `usage.md`, `architecture.md`)
  updated to match. (`discovery` was removed outright, not deprecated.)

---

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
