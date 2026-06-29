# TODO / Known-Issues Fix Plan

This plan triaged every open `TODO` comment in tracked source, every entry in
`docs/known-issues.md`, and every item in the maintainer's `todo.md`, then
grouped the fixes into **independent worktrees** runnable in parallel. It is the
companion to `docs/known-issues.md`.

Original triage was against `main` @ `3e9d697` (2026-06-24).

---

## 1. Status (updated 2026-06-29)

**Wave 1 is fully merged into `main`** (PRs #19–#30), and the `tabletop_unbag`
converter (PR #17, not originally in the plan) was added. The
docs/CLAUDE.md/known-issues reconciliation pass (this PR) is what closes Wave 1.

| Worktree | PR | Status |
| --- | --- | --- |
| WT-A · moveit interface bug fixes | #19 | ✅ merged |
| WT-C · UR `stop_program` future (P1) | #20 | ✅ merged |
| WT-D · rename + task-logic decisions | #21 | ✅ merged |
| WT-E · `commander.yaml` config | #22 | ✅ merged |
| WT-F · gaze geometric config | #23 | ✅ merged |
| WT-J · build / dev-env | #24 | ✅ merged |
| WT-B · executor robustness | #25 | ✅ merged |
| WT-M · eyelink/flic node TODOs | #26 | ✅ merged |
| WT-K · `bin/` script help | #27 | ✅ merged |
| WT-L · documentation | #28 | ✅ merged |
| WT-H · launch refactor | #29 | ✅ merged |
| WT-G · Teensy firmware + safety gate | #30 | ✅ merged |

All Section 6 maintainer decisions were answered and applied. The per-worktree
detail is preserved in [Section 4](#4-wave-1-worktrees-completed) for provenance.

---

## 2. Remaining work (re-prioritized)

Wave 1 closed the quick bug fixes and doc inconsistencies. What's left is
**maintainer validation**, two **Wave-2 refactors**, one deferred feature, and
P4 cleanup. Re-prioritized by what unblocks the most / carries the most risk:

### P1 — Maintainer validation (no further code expected, but must be verified)

These have code on `main` but need real hardware before
`docs/known-issues.md` can close them. **Do not delete the known-issue entries
until verified.**

1. **UR `safety_restart` recovery on the real robot.** Reconnect-in-recovery is
   implemented (`interfaces/ur.py::_reconnect`); run a real safety event and
   confirm the reset sequence completes (no `is_in_remote_control` timeout).
   → known-issues Open §A1.
2. **Teensy firmware #30 on the bench.** Flash and check: no first-trial false
   "laser broken" (debounce init), ISR monotonic clock, and the configurable
   arm-lock gate (`require_arm_locks`). → known-issues Open §A3.
3. **Eyelink stale-sample drain on a live unit.** Confirm the
   `tracker.resetData()` drain at retrieval start behaves as intended on
   hardware (sim-only so far). → known-issues Open §A2.

### P2 — Wave 2 refactors (cross-cutting; coordinate with each other)

4. **WT-I · `robot_name` vs `group_name`.** Make `robot_name` the task-facing
   parameter; keep `group_name` a property of the controlled robot. Touches
   `tabletop_tasks/**`, `nodes/commander.py`,
   `interfaces/moveit/{object_manipulation,plan_and_execute}.py`. Now unblocked
   (its Wave-1 dependencies merged); rebase on current `main`.
5. **WT-N · arm-lock terminology (mechanism-agnostic rename).** Reword the
   inaccurate "electromagnetic arm lock" wording and the `SetArmLock` /
   `*_arm_lock*` naming repo-wide (`interfaces/teensy.py`, `commander.py`,
   `tabletop_interfaces/**`, firmware comments, `commander.yaml`, `docs/**`) to
   be agnostic to the hold/detect mechanism (currently a button per hand).
   Large cross-cutting rename — **coordinate with WT-I** to avoid churn on the
   shared files (`commander.py`, tasks, interfaces).

> Both WT-I and WT-N edit `commander.py` and the manipulation interfaces. Run
> them back-to-back (or together) and rebase the second on the first; don't run
> them blindly in parallel.

### P3 — Deferred feature

6. **`nodes/flic.py:323` → custom message.** Publish the Flic response on a
   dedicated `tabletop_interfaces` message instead of a generic type. Needs a new
   `.msg` + colcon rebuild. → known-issues Open §B5.

### Quick doc/code reconciliation (gated on a trivial decision)

7. **`tasks.launch.py` `rosbag` default.** The arg defaults to `"false"` but the
   docstring/PR #29 say `true`. Decide the intended default, then fix the
   `DeclareLaunchArgument` *or* the docstring so source is self-consistent (the
   global docs already describe the code's actual behaviour). → known-issues
   Open §E8.

### P4 — "Decide later" markers (sweep with whichever worktree owns the file)

`compose.yaml:203` (`SYS_NICE`), `interfaces/moveit/moveit.py:465`,
`interfaces/moveit/plan_and_execute.py:628` (maybe revalidate),
`interfaces/ur.py:542`, `interfaces/moveit/object_manipulation.py:1759`
(check presentation region). → known-issues Open §D.

### Not scheduled — open design decision

- **smooth-pursuit fetch vs. manual-attach.** `smooth_pursuit.py` keeps the
  `manually_attach_object` workaround and its `# TODO: FIX!!!` intentionally;
  whether to physically pick the object is a design call, not a bug. Revisit
  only if the experiment design changes. → known-issues Open §B4.

---

## 3. Priority tiers (original definitions)

- **P1** — quick, high-confidence bug fixes & doc inconsistencies (Wave 1, done).
- **P2** — important but gated (decision / hardware / more code).
- **P3** — feature updates / larger refactors.
- **P4** — optional cleanup (cosmetic / "decide later" markers).

---

## 4. Wave 1 worktrees (completed)

Each worktree below was one branch over a disjoint file set, merged in the PR
shown in Section 1. Retained for provenance; see the PRs and
`docs/known-issues.md` "Resolved in Wave 1" for the exact changes.

- **WT-A** (`#19`) — `plan_and_execute.py` margin param; `object_manipulation.py`
  reset honours `use_cache=False`.
- **WT-B** (`#25`) — `spin_until_future_complete` no longer leaks
  `ConditionReachedException`; `executors.py` uses the logger, not `print`.
- **WT-C** (`#20`) — `stop_program()` tracks its rclpy future across calls (P1).
  The P2 `safety_restart` recovery is implemented but pending real-robot test
  (now Section 2 · P1·1).
- **WT-D** (`#21`) — `manually_atatch_object` → `manually_attach_object` (+ call
  sites); `foraging.py` `release_arm` kept, its `# TODO: Remove!!!` removed;
  `smooth_pursuit.py` fetch decision kept open by design.
- **WT-E** (`#22`) — `*_elbow_joint` fix; deleted dead `base_link_name` /
  `moved_tolerance`; added `safe_to_execute.require_arm_locks` (default false);
  shared Flic MACs confirmed intentional.
- **WT-F** (`#23`) — `gaze_estimation_geometric.yaml` realigned to
  `gaze_estimation.yaml` (model block aside).
- **WT-G** (`#30`) — configurable arm-lock gate, debounce-init fix, ISR
  monotonic clock, pin-38 rationale, `UNRECOVERABLE_ERROR` rename. Bench
  validation pending (Section 2 · P1·2).
- **WT-H** (`#29`) — retired `rig.launch.py` to `deprecated/`; flattened
  `tasks.launch.py` to include `commander`/`rosbag` directly; silenced the
  `fallback_controllers` / `publish_robot_description_semantic` bringup warnings;
  resolved the `to_dict()` and `execution_duration_monitoring` TODOs.
- **WT-J** (`#24`) — `mingus` → runtime deps; gh-cli devcontainer feature;
  multi-platform `docker-bake.hcl`; `c_cpp_properties.json` cleanup.
- **WT-K** (`#27`) — `-h|--help` for the `tt-*` scripts.
- **WT-L** (`#28`) — `TeensySensor.msg` header; `deprecated/README.md`;
  `docs/guide/parameters.md`; FLIR GenICam reference; committed Foxglove layouts;
  setup-doc gaps.
- **WT-M** (`#26`) — eyelink TODOs resolved (stale-sample drain pending live
  test, Section 2 · P1·3); `flic.py` custom-message TODO deferred (Section 2 · P3).

---

## 5. Maintainer decisions (answered & applied)

1. **Arm-lock safety gate** — not force-re-enabled; made configurable via
   `safe_to_execute.require_arm_locks` (default `false`). ✅ applied (#30/#22).
2. **`base_link_name` / `moved_tolerance`** — confirmed unused, deleted. Shared
   Flic MACs intentional, left alone. ✅ applied (#22).
3. **smooth-pursuit `fetch_object`** — not restored; TODO + known-issue kept as a
   design decision. `foraging.py` `release_arm` kept, its TODO removed.
   ✅ applied (#21).
4. **`gaze_estimation_geometric.yaml`** — aligned with `gaze_estimation.yaml`
   except model-creation parameters. ✅ applied (#23).
