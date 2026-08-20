# Long-running manipulation reliability

This page documents the long-running manipulation reliability and safety
changes. It is both an operator guide and a record of the behavior that differs
from the earlier implementation.

The overall policy is:

- validate mount-specific kinematic branches in simulation before hardware use;
- keep successful motion paths fast and direct;
- recover bounded, planning-only failures without losing an object;
- recover laser stops only after the physical and UR-controller state are safe;
- stop both arms on a real execution or UR safety fault whose state is uncertain.

## Manipulation preflight

`tt-preflight` performs a planning-only commissioning pass over the physical
object grid. The search is hard-disabled in real mode: it cannot execute a
trajectory on hardware.

For each reachable mount, preflight:

1. deterministically enumerates distinct PRE_FETCH inverse-kinematics branches;
2. filters joint-limit and collision-invalid representatives;
3. plans the complete chain from IDLE through PRE_FETCH, attach, clearance, and
   FETCHED while applying the same temporary collision allowances as runtime;
4. models the object attachment in the planning scene for attached-object
   segments;
5. rejects excessive per-waypoint joint wrapping;
6. requires the selected branch to pass three consecutive full-chain replans;
7. records the shortest valid branch and the reason an unavailable object
   failed.

The report is checkpointed after every object, so an interrupted run can
resume. Results are reused only if a SHA-256 fingerprint still matches the
object pose, grid index, arm, joint set, Cartesian manipulation goals, fetched
goal, collision allowances, planner chain, and relevant planning parameters.
Moving a mount or changing those settings therefore makes the result stale
instead of silently reusing it.

Run and inspect preflight with:

```bash
tt-preflight run
tt-preflight results
```

After reviewing a fresh report, create ignored hardware-run configurations:

```bash
tt-preflight prepare-real
```

This produces a two-trial smoke test, a finite all-accessible test, and a
long-running configuration under `.cache/tabletop/`. Real startup accepts only
a `mock_planning_only` report, requires an explicit `allow_real_use` setting,
rechecks every fingerprint, installs only compatible PASS branches, and removes
unavailable, stale, or missing objects from the trial generator. It refuses to
start if preflight leaves either arm with no objects.

A separate mock-cycle harness can execute complete fetch/return cycles and
exercise recovery branches without allowing real mode.

## Planning and trajectory execution

The motion path changes are deliberately concentrated in planning and failure
handling; successful paths retain their configured velocity scaling.

- The default planning-attempt limit is five instead of three. FETCHED and the
  recovery waypoint also receive five attempts.
- If normal PRE_FETCH or FETCHED planning is exhausted, the arm gets one
  stage-level recovery. PRE_FETCH transits through the configured SRDF `idle`
  state; FETCHED first moves the attached object through a configured
  outward/upward clearance pose. The original stage is then tried once more.
- A failed POST_FETCH plan may still skip directly to FETCHED, while preserving
  a state from which the object can be returned if execution is interrupted.
- Cached trajectories are collision-validated against the current scene before
  use. Invalid candidates are rejected individually, with normal planning used
  when necessary.
- Time-parameterized output is revalidated after TOTG/Ruckig processing, so the
  exact trajectory submitted to a controller—not only the planner's geometric
  path—must remain valid.
- Collision padding participates in the planning-scene fingerprint, preventing
  cache reuse after padding changes.
- MoveIt planning and validation are serialized across the two arms because
  MoveItPy shares native planning-scene state. Controller execution remains
  concurrent.
- A joint-position PRE_FETCH override is not reused as POST_RETURN; return uses
  the normal Cartesian goal and avoids invalid reverse-branch assumptions.
- Normal returns skip the unnecessary final IDLE pose. This removes a long
  extra motion without changing fetch, presentation, unpresentation, or return
  velocity scaling.
- The configured maximum acceleration was reduced from `30.0` to `20.0` for
  each UR joint. Velocity limits were not reduced; this bounds acceleration at
  transitions rather than deliberately slowing the complete path.

## Trial-level recovery and object availability

Planning recovery is bounded at two levels:

- A manipulation stage has the single conditional recovery described above.
- If a trial still exits because of planning errors only, the exact same trial
  is restarted immediately once. It is not moved to the back of the queue.

If that immediate retry also fails, the trial reports failed feedback and the
experiment continues. The object is not blacklisted and remains eligible for a
later normal trial. Controller, execution, and safety failures are not treated
as planning misses and do not enter this retry path.

## Laser interruption and UR controller restoration

The Teensy laser signal is fail-safe. When it becomes unsafe while a presented
motion is executing, Commander stops the affected UR external-control program
once; the 100 Hz sensor stream cannot repeatedly flood the stop service.

Recovery now follows this sequence:

1. lock the arms and wait until all configured safety conditions are
   continuously valid for `teensy_interface.safe_to_execute.required_time`
   (currently **0.5 seconds**);
2. record the current laser-break generation;
3. verify the UR external-control program and reset/replay it if needed;
4. verify controller readiness again;
5. verify that safety is still valid and no newer laser edge occurred;
6. reapply the safety gate immediately before the motion retry.

Raw laser edges are recorded for both arms even when neither arm has an active
trajectory. This closes the race in which the beam could be broken while the
UR program was being restored and the event would otherwise be forgotten after
the beam cleared.

If a newer break occurs during restoration, the newly restored program is
stopped and the full recovery cycle restarts after another stable-clear period.
At most `interruptions.max_controller_recovery_attempts` cycles are allowed
(currently five) for one interruption. Holding the beam broken does not consume
attempts; the system simply waits. Exhausting the limit latches a rig-wide fault
instead of retrying indefinitely.

A later, successfully separated laser event gets a fresh recovery allowance.
Explicit reset/service exceptions can still fail immediately rather than
consuming all five readiness cycles.

## Fatal safety faults and Ctrl-C

The following UR safety modes are treated as fatal for the complete dual-arm
session: protective stop, safeguard stop, system or robot emergency stop,
violation, fault, automatic-mode safeguard stop, and three-position-enabling
stop when supplied by the driver.

A non-laser execution error on real hardware is also fatal because the measured
arm state can no longer be assumed to match the planned state. Commander latches
the first fault, cancels both active goals, stops both UR programs, prevents
automatic cleanup motion, and requires explicit operator inspection and a new
Commander session.

Ctrl-C remains intentionally fast: both active controller goals are cancelled
before task cancellation unwinds, and no return or IDLE motion is started. This
can leave the current object and a prefetched next object attached; physically
restore them before the next run.

By contrast, natural completion of a finite task occludes the glass, locks both
arms, returns staged objects, and stops without an extra IDLE move.

## Teensy and service resilience

The micro-ROS firmware no longer uses a very short clock-sync call as its
connection-liveness test. It pings the agent with a realistic timeout, performs
clock synchronization separately as best-effort maintenance, tears down a lost
session best-effort, and returns to the reconnect state instead of becoming
permanently unrecoverable because one cleanup call failed.

Commander retries one timed-out call for idempotent Teensy setters: arm lock,
smartglass state, and solenoid state. Repeating these setters is safe and lets a
request survive the narrow interval in which the micro-ROS session reconnects.

Laser diagnostic logs include raw state transitions, debounced last-break
timestamp changes, sensor delay, and arm-lock state. The diagnostics do not
weaken or bypass the safety gate.

## Development-environment changes

The build wrapper safely parses the selected uv extra and defaults to CPU
PyTorch if the environment has not yet been generated. The current environment
workflow uses the explicit `COMMANDER_RUNTIME` setting from `main`: `runc`
selects CPU dependencies and `nvidia` selects the configured CUDA extra.

## Configuration summary

| Setting | Current value | Purpose |
| --- | ---: | --- |
| `planning.default_max_attempts` | 5 | Normal planning retries |
| `planning.fetched_max_attempts` | 5 | Difficult attached-object FETCHED planning |
| `fetch_recovery.enable` | `true` | Enable one conditional stage recovery |
| `fetch_recovery.max_attempts` | 5 | Planning attempts for the recovery waypoint |
| `fetch_recovery.pre_fetch_transit_goal` | `idle` | Safe transit before retrying PRE_FETCH |
| `fetch_recovery.fetched_clearance_offset` | `[0.22, 0.0, -0.18]` | Attached-object clearance before retrying FETCHED |
| `skip_idle_on_return` | `true` | Remove the unnecessary final IDLE motion |
| `interruptions.max_attempts` | 3 | Motion attempts after an interruption |
| `interruptions.max_controller_recovery_attempts` | 5 | UR restoration cycles for one laser interruption |
| `safe_to_execute.required_time` | 0.5 s | Required continuous clear/safe period |
| joint `max_acceleration` | 20.0 | Bound transition acceleration without lowering velocity limits |

## Validation evidence

The implementation is covered by deterministic tests for preflight
fingerprints and branch loading, planning serialization, trajectory validation,
fetch-stage recovery, immediate trial retry, cleanup behavior, fatal UR faults,
laser edge tracking, re-breaks during controller reset, pre-retry safety gates,
and bounded restoration. The complete rig/task suite currently contains 55
passing tests.

Two extended checks were completed on 2026-08-19:

- **Simulation:** 500 trials over approximately 1 hour 18 minutes using all 25
  accessible objects; 500 presentations, no object misses, two recovered laser
  interruptions, and no fatal fault.
- **Real rig:** 1,179 presented trials over approximately 3 hours 2 minutes
  using the 25-object preflight set; no retrieval or presentation misses. All
  12 laser-interrupted motions recovered, including 10 cases where restoration
  correctly restarted after another beam break. Three `big_object_5` fetch
  stages used the bounded recovery and succeeded. The run ended cleanly by
  operator Ctrl-C with no fatal rig fault or software crash.

These results exercise the recovery paths but do not replace preflight or the
operator's physical rig inspection after mounts, calibration, or attached
objects change.
