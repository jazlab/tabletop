# TODO / Known-Issues Fix Plan (2026-06-24)

This plan triages every open `TODO` comment in tracked source, every entry
in `docs/known-issues.md`, and every item in the maintainer's `todo.md`,
then groups the fixes into **independent worktrees** that can be tackled in
parallel. It is the companion to the expanded `docs/known-issues.md`.

The work was triaged against `main` (commit `3e9d697`).

---

## 1. Triage summary

| Source | Items | Already fixed | Still open |
| --- | --- | --- | --- |
| `docs/known-issues.md` (pre-existing) | 14 | **0** | 14 |
| `TODO`/`FIXME` comments in tracked source | ~28 | 0 | all (pending by nature) |
| `todo.md` (functional + docs) | ~21 | 0 | all |

**Nothing was found already-fixed**, so no comments or known-issue entries
were removed. Every pre-existing known-issue was re-verified by *content*
(the cited line numbers had drifted in several cases — they were refreshed
in `known-issues.md`). All newly-found substantive issues were added to
`known-issues.md` under four new sections.

> The only edits made so far are to `docs/known-issues.md` (re-verify +
> expand) and this new file. No code has been changed yet — that is the
> work below.

---

## 2. Priority tiers

Per the request, ordering favours **quick/important bug fixes and
documentation inconsistencies** over feature work.

- **P1 — quick, high-confidence bug fixes & doc inconsistencies.** Small,
  isolated, low-risk, no design decision required.
- **P2 — important but gated.** Needs a maintainer decision, hardware
  (firmware flash), or a bit more code; plus quick non-bug wins.
- **P3 — feature updates / larger refactors.**
- **P4 — optional cleanup** (cosmetic / "decide later" markers).

Priority is independent of *wave*: wave is about file-conflict
scheduling (Section 4), priority is about what to spend effort on first.

---

## 3. Worktrees (each = one branch, disjoint file set)

Every worktree below edits a **non-overlapping** set of files, so all of
Wave 1 can run concurrently. Suggested branch names are `fix/<slug>`.

### WT-A · moveit interface bug fixes — **P1**

Files: `tabletop_rig/interfaces/moveit/plan_and_execute.py`,
`tabletop_rig/interfaces/moveit/object_manipulation.py`

- **KI "copy-paste param"** (`plan_and_execute.py:1033-1034`): change
  `allowed_duration_margin = self.param("execution.allowed_duration_scaling")`
  to read `"execution.allowed_duration_margin"`.
- **KI "dropped `use_cache=False`"** (`object_manipulation.py:1231-1233`):
  pass the `reset_request` copy (which has `use_cache=False`) to
  `plan_and_execute(...)` instead of the original `config.reset_request`.
- No decisions needed. ~2 small edits, ideally with a regression check that
  the reset path no longer pulls a cached trajectory.

### WT-B · executor robustness — **P1**

Files: `tabletop_rig/executors.py`, `tabletop_rig/nodes/system_check.py`

- **KI-1**: make `spin_until_future_complete` (or `_spin_impl`) catch the
  `ConditionReachedException` it raises at `executors.py:823` so callers
  don't receive an `ExceptionGroup`; then remove the
  `except* ConditionReachedException` workaround at `system_check.py:433`
  and confirm `system_check` still exits its wait cleanly.
- **KI "code smell #3"**: replace the bare `print(...)` in `_queue_producer`
  (`executors.py:485`) — and the `print(e)` at `:734` — with the node/ROS
  logger.

### WT-C · UR interface reliability — **P1 / P2**

Files: `tabletop_rig/interfaces/ur.py`

- **P1 — KI "code smell #2"**: `stop_program()` (`ur.py:659-665`) fires
  `call_async` and never awaits/checks the future. This is an rclpy future,
  which is distinct from asyncio futures in that it is thread-safe to call
  `call_async` while it is not threadsafe to call the asyncio wrapped version
  of the service call. The reason we use the native rclpy async service call
  here and the asyncio-wrapped service calls elsewhere is because we want
  stop_program to be both thread-safe and fast since it is called by the
  teensy sensor message callback and is used to quickly stop the robot in
  the event that the safety laser is broken and the robot is in the vicinity
  of the subject. The result of this future can be checked on subsequent calls
  to this function: if the future is not complete, do not send another stop_program request; if the future is complete and but it did not complete
  successfully, then the service should be called again; if it did complete successfully, then the service should not be called again until the `reset`
  method has been called.
- **P2 — `todo.md` functional**: `safety_restart` recovery doesn't finish
  (`is_in_remote_control` times out after safety returns to normal);
  likely needs a dashboard reconnect in the recovery state machine.
  Needs real-hardware reproduction. (I think I implemented this, make sure
  I did but don't remove this from the known-issues or the fix-plan until
  I've tested it on the real robot)

### WT-D · API typo rename + task-logic TODOs — **P1 / P3**

Files: `tabletop_rig/nodes/commander.py`,
`tabletop_tasks/tasks/smooth_pursuit.py`,
`tabletop_tasks/tasks/dummy.py`, `tabletop_tasks/tasks/foraging.py`

- **P1 — KI "code smell #1"**: rename `Commander.manually_atatch_object`
  → `manually_attach_object` and update the two call sites
  (`smooth_pursuit.py:370`, `dummy.py:439`) in the same commit. (Get rid of deprecated alias.)
- **P3 — `smooth_pursuit.py:368` `# TODO: FIX!!!`**: decide whether to
  restore the real `await manipulator.fetch_object(...)` (currently
  commented out in favour of the manual-attach workaround). Needs a
  decision + bench test. Also the `plan_and_move("fetched")`
  `# TODO: Maybe remove`.
- **P3 — `foraging.py:154` `# TODO: Remove!!!`**: confirm whether the
  unconditional `release_arm(arm)` at the top of the response phase should
  stay; remove if it was debug scaffolding.

> Owns **all** `tabletop_tasks/tasks/*` edits and `commander.py` in Wave 1
> to keep those files single-writer.

### WT-E · `commander.yaml` config decisions — **P2**

Files: `tabletop_rig/config/commander.yaml`

- **KI config #1** (`:382`, `:457`): fix `left_eblow_joint` /
  `right_eblow_joint` → `*_elbow_joint` (confirm against
  `tabletop_description` joint names first).
- **KI config #2**: `trajectory_cache.base_link_name` (`:375`, `:450`) and
  `execution.moved_tolerance` (`:227`) have no reader — decide: delete as
  dead config, or wire up the intended `self.param(...)` read.
- **KI config #3** (`:48,52,53`): `big_object_3` / `big_object_7` /
  `small_object_0` share one Flic MAC — confirm distinct buttons (then
  supply real MACs) or document as intentionally-shared/spare.

> All three need a maintainer decision; pure config, no code.

### WT-F · gaze geometric config alignment — **P1**

Files: `config/gaze_estimation_geometric.yaml`

- **KI config #4**: realign the `visualize:` block to the keys the code
  reads (`animate_2d_dots` / `animate_3d_dots`, not `eyelink_range` /
  `markers_range`) and rename the `data:` block to `dataloaders:` to match
  `gaze/utils.py::init_dataloaders` — matching `gaze_estimation.yaml`.
  Only needed if this config is fed to the visualize / MLP pipeline; verify
  intended use before deleting vs. realigning.

### WT-G · Teensy firmware + safety gate — **P2 (safety)**

Files: `src/microros/tabletop_teensy/src/main.cpp`,
`tabletop_rig/interfaces/teensy.py`

- **SAFETY DECISION — KI bug "arm-lock disabled"** (`teensy.py:198-217`):
  decide whether to re-enable the `is_left_arm_locked && is_right_arm_locked`
  gate in `_msg_safe_to_execute`. Motion is currently gated on the safety
  laser only.
- **`todo.md` functional — debounce false "laser broken" on first trial**:
  fix the debounce-initialisation in `main.cpp` (affects all debounced
  sensors), and revisit the commander-side `safety_laser_last_time_broken`
  handling (see follow-up note in WT-D's owner — coordinate, it lives in
  `commander.py`).
- **`todo.md` functional — ISR clock**: stop reading
  `rmw_uros_epoch_nanos` inside the debounce ISRs; latch monotonic time.
- **KI firmware #1** (`main.cpp:83`): resolve the `LEFT_ARM_LOCK_STATE_PIN`
  `38 → 36` TODO vs `BUTTON_STATE_PIN 36` conflict (relocate one pin).
- **KI firmware #2**: rename the `UNCRECOVERABLE_ERROR` enum →
  `UNRECOVERABLE_ERROR` (cosmetic, do all uses at once).

> Requires PlatformIO flashing + bench validation. The commander-side
> safety-laser-time tweak touches `commander.py`; sequence it **after**
> WT-D merges, or hand that one-line change to WT-D.

### WT-J · build / developer-environment — **P1 (mingus) / P2**

Files: `pyproject.toml`, `.devcontainer/devcontainer.json`,
`docker-bake.hcl`, `.vscode/c_cpp_properties.json`, `compose.yaml`

- **P1**: move `mingus` from `[dependency-groups].dev` (`pyproject.toml:49`)
  into `[project].dependencies` — it's imported at runtime by
  `SoundInterface`, so non-dev installs break today.
- **P2**: add `"ghcr.io/devcontainers/features/github-cli:1": {}` to the
  devcontainer `features`.
- **P2**: make `docker-bake.hcl` multi-platform (`linux/amd64` +
  `linux/arm64`); review the `jazlabtabletop/*` tags (no `:latest`).
- **P2**: fix the `pio_teensy` path and drop the `pio_sniffer` config in
  `.vscode/c_cpp_properties.json`.
- **P4**: investigate the `SYS_NICE` cap `# TODO: see if needed`
  (`compose.yaml:203`).

> Each item is its own file — this worktree can be split further if desired.

### WT-K · `bin/` script help — **P2 (quick win)**

Files: `bin/host/*`, `bin/common/*`, `bin/container/*`

- Add a `-h|--help` usage block to each `tt-*` script. Establish one shared
  helper/pattern (e.g. in `bin/common`) and apply it consistently.

### WT-L · documentation — **P1 (inconsistencies) / P2 (additive)**

Files: `docs/**`, `src/ros/tabletop/tabletop_interfaces/msg/TeensySensor.msg`,
`deprecated/README.md`, `share/**`

- **P1 — `TeensySensor.msg`**: header credits "tabletop_micro firmware"
  (real package: `tabletop_teensy` in `src/microros/`) and hard-codes
  "100 Hz" (configurable). Fix the reference; drop the rate.
- **P1 — `deprecated/README.md`**: reconcile the
  `src/ros/tabletop/tabletop_micro/tabletop_flic_micro/` path reference.
- **P2 — FLIR GenICam**: link the BFS-U3-23S3 node reference from the
  camera-config docs (tie to `blackfly_s.yaml` / `flir_synchronized.yaml`).
- **P2 — node/interface parameter docs**: surface the per-node parameters
  (currently inline in `config/*.yaml`) into `docs/`.
- **P2 — Foxglove**: export + commit the layout configs (two examples
  currently only in a local `share/` dir), document import, add the
  "must be open & focused at task start" planning-scene note.
- **P2 — setup gaps**: add `git-lfs` + `jq` prerequisites; note possible
  reboot after `usermod -aG docker`; note the `platformio-core` volume
  ownership fix (delete volume, re-run `tt-build microros`).

### WT-M · `tabletop_rig` node TODOs (eyelink + flic) — **P2 / P3**

Files: `tabletop_rig/nodes/eyelink.py`, `tabletop_rig/nodes/flic.py`

- **P2 (trivial)**: delete the content-free `eyelink.py:1402

  # TODO: Something is fucking wrong, help me`, or convert to a real issue

- **P2/P3**: investigate `eyelink.py:397 # TODO: Fix callback groups`
  (concurrency correctness) and `:915` discard-stale-samples-before-collection.
- **P3**: `flic.py:323 # TODO: Change to custom message`.

### WT-H · launch refactor + bringup warnings — **P3**

Files: `tabletop_rig/launch/rig.launch.py`,
`tabletop_tasks/launch/tasks.launch.py`,
`tabletop_rig/launch/rviz.launch.py`,
`tabletop_moveit_config/launch/moveit.launch.py`,
`tabletop_moveit_config/config/moveit_controllers.yaml`

- Retire `rig.launch.py` to `deprecated/`; have `tasks.launch.py` include
  `commander.launch.py` / `rosbag.launch.py` directly (resolves the
  `# TODO: … unscoped …` markers at `rig.launch.py:310`,
  `tasks.launch.py:99`).
- Silence the benign bringup warnings (`fallback_controllers`,
  `publish_robot_description_semantic`) — likely add the full MoveIt config
  to the rviz node params.
- Resolve the `moveit_config.to_dict()` "which one to use" TODOs
  (`moveit.launch.py:212`, `rviz.launch.py:114`) and
  `moveit_controllers.yaml:13`.

> File-independent of Wave 1, so it *can* run alongside — kept P3 as it's a
> refactor, not a bug fix.

### WT-I · `robot_name` vs `group_name` refactor — **P3 (Wave 2)**

Files: `tabletop_tasks/**` (task code, trial specs, trial generators),
`tabletop_rig/nodes/commander.py`,
`tabletop_rig/interfaces/moveit/object_manipulation.py`,
`tabletop_rig/interfaces/moveit/plan_and_execute.py`

- Make `robot_name` the task-facing parameter; keep `group_name` as a
  property of the controlled robot (independent of the name).
- **Conflicts with WT-A, WT-D (and WT-H's tasks usage).** Must run in
  **Wave 2**, rebased after those merge — see Section 4.

---

## 4. Execution strategy (waves)

**Wave 1 — run all in parallel** (disjoint files, no cross-conflicts):
`WT-A, WT-B, WT-C, WT-D, WT-E, WT-F, WT-G, WT-J, WT-K, WT-L, WT-M` (and
optionally `WT-H`). Recommended order to *merge* by value:

1. P1 bug fixes: WT-A, WT-B, WT-C(P1 part), WT-D(rename), WT-F, WT-J(mingus)
2. P1 doc inconsistencies: WT-L(`TeensySensor.msg`, `deprecated/README.md`)
3. P2: WT-E, WT-G, WT-K, WT-J(rest), WT-L(rest), WT-M, WT-C(reconnect)
4. P3: WT-D(task-logic), WT-H, WT-M(flic)

**Wave 2 — after Wave 1 merges:** `WT-I` (rebase on the merged
`commander.py` / moveit-interface / tasks changes), then mop up the **P4**
"decide later" markers (Section 5) by riding along with whichever worktree
owns each file.

### Shared-file caveats (the only things that break naive parallelism)

- `commander.py`: Wave-1 owner is **WT-D** (rename). WT-G's optional
  commander-side safety-laser tweak and WT-I both also touch it — defer
  both to after WT-D.
- `object_manipulation.py` / `plan_and_execute.py`: WT-A (Wave 1) then
  WT-I (Wave 2). WT-A is tiny, so WT-I rebases trivially.
- `tabletop_tasks/tasks/*`: WT-D owns it in Wave 1; WT-I edits it in Wave 2.

---

## 5. P4 — optional "decide later" markers

Low-priority TODOs with no known breakage; fold into whichever worktree
already owns the file, or sweep last:
`compose.yaml:203` (SYS_NICE), `moveit_controllers.yaml:13`,
`moveit.launch.py:212` / `rviz.launch.py:114` (`to_dict` choice),
`interfaces/moveit/moveit.py:465`,
`interfaces/moveit/plan_and_execute.py:628` (maybe revalidate),
`interfaces/ur.py:542`, `object_manipulation.py:1759`
(check presentation region), `gaze/preprocess.py:627`.

---

## 6. Decisions needed from the maintainer (blockers)

These gate their worktrees and can be answered up front:

1. **WT-G**: re-enable the arm-lock safety gate in `_msg_safe_to_execute`? (safety)
2. **WT-E**: `base_link_name` / `moved_tolerance` — delete or wire up? Are
   the three shared Flic MACs intentional?
3. **WT-D**: restore real `fetch_object` in smooth-pursuit, or keep the
   manual-attach workaround? Should `foraging.py`'s response-phase
   `release_arm` stay?
4. **WT-F**: is `gaze_estimation_geometric.yaml` still fed to the
   visualize / MLP pipeline (realign) or dead (delete)?

Answers:

1. Do not re-enable the arm-lock safety gate. Instead, make this configurable via a parameter to in commander.yaml, and set it to false so that current
behavior is maintained.
2. I think the base_link_name and moved_tolerance are no longer used, make
sure that is the case, but then you can delete. The shared Flic addresses
are intentional, leave those alone.
3. Fetch object should not be restored for right now, but this should remain
in the known-issues and the TODO comment should stay since this is a design
decision whether or not to keep the smooth_pursuit_object in the grid or to
manually attach at the beginning of the smooth pursuit task. The release_arm
should be kept, and the TODO comment removed.
4. The gaze_estimation_geometric config should match the MLP version in
everything except the parameters relevant to model creation: Otherwise,
everything else should be identical. Make sure that this is the case, then
align gaze_estimation_geometric with gaze_estimation.
