# Tasks & Experiments

A **task** is a behavioral experiment. Tasks live in `tabletop_tasks` and drive
the rig through the `Commander`; they never touch devices directly.

## How a task runs

```text
tasks.launch.py  (task:=<name>  ⇒  coro_config = config/<name>.yaml)
  └─ run.py: run_tasks(commander, config_file)
       loads tasks: [{class, kwargs}, …] from the YAML
         └─ tasks/<paradigm>.py   (BaseObjectInteractionTask subclass)
              trial_generator → TrialSpec → run_trial() → TrialFeedback
                 └─ trial_generators/<generator>.py
```

`run_tasks` instantiates each task class with its `kwargs` (plus the injected
`Commander`) and awaits its run loop. The generic trial loop pulls a
`TrialSpec` from a **trial generator**, runs it, and feeds a `TrialFeedback`
back into the generator via its `send()` method — so the next trial can depend
on the subject's last response (e.g. alternating or blocked designs).

## Task paradigms

| Task class | Config prefix | Behavior |
| --- | --- | --- |
| `ForagingTask` | `foraging_*` | Subject selects among presented objects |
| `InvariantAffordanceTask` | `invariant_affordance` | Alternating designated objects with a bounded occluded pose change and timed go cues |
| `PresentTask` | `present_*` | Passive object presentation |
| `SmoothPursuitTask` | `smooth_pursuit_*` | Object follows a trajectory for the subject to track |
| `DummyTask` | `dummy` | Diagnostic scratchpad (latency / motion / component checks) |

## Trial generators

Generators implement an iterator + `send(trial_spec, feedback)` protocol:
`BaseTrialGenerator` plus `OrderedChoiceAlternating` and
`RandomChoiceAlternating`, which alternate between robot groups (left/right).
The config's `trial_generator` key selects one and supplies its kwargs (object
groups, poses, occlusion probability, …).

## Configuring a task

Task configs are YAML files in `tabletop_tasks/config/`. Each lists one or more
`{class, kwargs}` entries; the kwargs map directly to the task class
constructor. Available configs:

| Config | Task | Notes |
| --- | --- | --- |
| `foraging_ordered.yaml` / `foraging_random.yaml` | ForagingTask | ordered / randomized trials |
| `invariant_affordance.yaml` | InvariantAffordanceTask | Designated left/right objects, 1 s reveal, brisk bounded reposition, then sound + both buzzers |
| `present_ordered.yaml` / `present_random.yaml` | PresentTask | ordered / randomized presentation |
| `smooth_pursuit_random.yaml` | SmoothPursuitTask | random waypoints |
| `smooth_pursuit_spiral{,_test}.yaml` | SmoothPursuitTask | helical trajectory |
| `smooth_pursuit_sin.yaml` | SmoothPursuitTask | sinusoidal trajectory |
| `smooth_pursuit_sin_one_lap.yaml` | SmoothPursuitTask | same sinusoid, exactly one lap |
| `smooth_pursuit_sin_static.yaml` | SmoothPursuitTask | one sinusoidal lap followed by 27 stationary fixation points |
| `smooth_pursuit_sin_static_forward.yaml` | SmoothPursuitTask | experimental: lap and 27 half-second holds shifted 6 in toward the subject; mock planning did not pass |
| `smooth_pursuit_sin_static_forward_five.yaml` | SmoothPursuitTask | experimental: lap and 27 half-second holds shifted 5 in toward the subject; mock planning was not repeatable |
| `smooth_pursuit_sin_static_forward_five_gentle_turns.yaml` | SmoothPursuitTask | **Preferred for collecting gaze training data:** five-inch-closer sinusoid with eased turns, then 27 held-out fixations |
| `smooth_pursuit_sin_two_laps_static.yaml` | SmoothPursuitTask | one extended sinusoid (~5 minutes moving) that samples new positions in the second half, then one 27-point fixation block |
| `smooth_pursuit_sin_two_laps_static_forward_five.yaml` | SmoothPursuitTask | ~5 continuous minutes at the five-inch-closer distance, then 27 half-second holds |
| `smooth_pursuit_depth_sweep.yaml` | SmoothPursuitTask | one 150 s depth-balanced sweep followed by the same 27-point block |
| `smooth_pursuit_bearing_depth_forward_five.yaml` | SmoothPursuitTask | 150 s repeated-bearing depth calibration at the five-inch-closer position, then 27 half-second holds |
| `smooth_pursuit_static_grid.yaml` | SmoothPursuitTask | five-minute static-only gaze calibration, holding each point for 500 ms |
| `dummy.yaml` | DummyTask | diagnostic scratchpad |

To create a new task, copy an existing config and adjust the kwargs. Every
config is commented inline; the class definitions in
`tabletop_tasks/tabletop_tasks/tasks/` and the generators in
`trial_generators/` are the source of truth for available parameters (see the
[API Reference](../reference/tabletop_tasks.md)).

```bash
tt-launch tasks task:=foraging_ordered robot_mode:=mock
```

### Preferred gaze calibration protocol

Use `smooth_pursuit_sin_static_forward_five_gentle_turns` to collect training
data for the pupil-corrected binocular neural network. The observer follows
the LED, not the marker above it. Train on the full moving lap (about 159 s
on the tested rig), then evaluate on the 27 separate 500 ms static holds.
Do not use those holds to fit, normalize, or select the model. The two
blocks are in the same session; this tests moving-to-static accuracy, not
generalization to a different participant or head position. The static grid
alone, the longer sinusoid, and the depth-sweep paths are alternatives for
specific experiments, not the preferred collection preset. This recommendation
does not change any launcher default or override local clearance checks.

A standalone right-arm smooth-pursuit task can start with the arm overlapping
its virtual presentation wall. Startup still validates both arms and rechecks
after the fixture mesh is attached. The presentation safety gate covers the
approach and the pursuit trajectory. Other tasks use the normal startup check.

On normal completion, gaze calibration closes the smartglass and returns
the attached fixture to the same ceiling-facing `fetched` pose used before
the gaze path. This also applies to the standalone static grid, and the
return is outside the timed calibration block. The return uses the existing
presentation safety gate and is not started after cancellation or an aborted
acquisition. If there is no valid return route, the task reports that the
return was skipped rather than commanding an alternate parking move.

The one- and two-lap variants leave `smooth_pursuit_sin.yaml` unchanged. In the
static variants, follow the LED during pursuit and keep looking at it as the robot
steps through a 3×3×3 grid around the sinusoid's center. The original-distance
one- and two-lap presets hold each target for one second; the forward-shifted
presets use 500 ms holds. All 27 transitions are planned before
pursuit begins. The double-duration preset keeps all three oscillations going
through the midpoint without returning to center or restarting their phases.
The path covers more of the same volume, at the original waypoint spacing and
speed setting. The glass stays transparent into the static block, and the
pursuit tone and reward stop after the final lap. With `rosbag:=true`, the
`/rosout` topic records `STATIC_HOLD_START` and `STATIC_HOLD_END` messages
with a point number. The start message also reports the planned TCP position
in meters; use measured OptiTrack positions and the marker-to-LED offset for
gaze accuracy, not the commanded TCP coordinates. Omit incomplete holds.

The forward preset shifts both the one-lap sinusoid and the entire
27-point grid by 15.24 cm along world −Y (toward the subject). It keeps
the original amplitudes, periods, orientation, and speed settings. Each
stationary hold is 500 ms. **Do not use this preset on the real rig yet.**
The mock planner rejected the fixed-orientation path before revealing the
stimulus, including when approached from the original center. A successful
simulation and a physical fixture-clearance check are still needed before
running the shifted workspace with a participant.

The five-inch alternative translates the same waypoints by 12.7 cm,
retaining the 500 ms holds and original orientation. One mock startup
found a valid plan on attempt 13, but a separate startup exhausted all
20 attempts. The task does not reveal the LED until the lap and all
holds have valid plans. Do not treat this as a validated participant-run
preset; reliable mock planning and a physical fixture-clearance check
are still needed.

The gentle-turn variant keeps the five-inch sinusoid and fixation grid.
Only the moving lap is time-stretched: the arm gradually slows where the
LED changes direction quickly and resumes its usual pace after each bend,
without adding stops or speeding up other sections. Expect several extra
seconds for the moving lap. Startup planning is capped at 40 attempts
before the LED is revealed; this does not retry a lap after motion starts.
Use `CORNER_EASE_START` and `CORNER_EASE_END` to identify the moving lap
in the recorded task log.

The extended five-inch preset continues the same sinusoid for about five
minutes without restarting its phases halfway through. It keeps the same
12.7 cm shift, orientation, and 500 ms holds as the shorter five-inch preset.
It plans the entire longer path and all 27 holds before revealing the LED.
The five minutes refer to the moving portion, excluding the approach,
stationary holds, and parking. Mock startup can still fail to find a plan;
check fixture clearance with an empty presentation area before involving a
participant.

The opt-in depth sweep uses nine viewing directions, a 15 cm near–far range,
and a small continuous angular ellipse. This changes depth while repeatedly
revisiting similar directions, without a dwell or abrupt corner between
neighborhoods. The fixed-orientation fixture stays within the standard
pursuit envelope. The path starts and ends at the original center, is
stretched to 150 planned seconds without increasing any joint speed or
acceleration, and then runs the same 27 holds. `DEPTH_SWEEP_START` and
`DEPTH_SWEEP_END` are recorded with the other task log messages. The 5 s
baseline and the static block are outside the 150 s sweep; real duration can
increase if the controller speed slider is below full scale. No EyeLink or
pursuit-detection settings differ from the sinusoidal task.

The repeated-bearing preset is a separate experiment at the five-inch-closer
position. It draws the same apparent figure six times while varying the LED's
distance at a different rhythm. The tool orientation stays fixed, and the LED
returns to the center before the standard 27 half-second holds.
The depth envelope blends gradually where the workspace bounds change,
avoiding the sharp acceleration peaks of the original tighter blend.

Set `motion_kwargs.viewpoint_offset_from_center_led_m` to the displacement from
the LED at the center pose to the midpoint of the participant's eyes, in rig
world XYZ meters. The default is an estimate from the previous five-inch-closer
recording, not a measurement of the current setup. Recheck the path in mock
mode after changing this setting or the mounting geometry. The 150 seconds
refer only to the planned moving part; the planner retimes the continuous
curve, and the task will not speed up an over-budget plan to force it to fit.
Use `BEARING_DEPTH_START` and `BEARING_DEPTH_END` in the task log to delimit
the motion, and `STATIC_HOLD_START` / `STATIC_HOLD_END` for the fixation block.

The static-only gaze preset uses the same manually mounted right-arm LED.
It interleaves eight 27-point passes through a uniformly spaced 6×6×6
grid within ±7 cm of the usual center. Each pass spans the workspace; in
a partial final pass some grid points may get fewer visits. The five-minute
budget starts with the first visible hold and includes the moves. The task
does not start a transition whose planned duration plus the full 500 ms
hold won't fit. Slower motion can reduce the point count, and a pause can
extend a move already underway. Setup and preplanning are outside the budget.
The glass remains transparent during the block; no pursuit reward runs.

The bag contains `STATIC_GRID_BLOCK_START` / `STATIC_GRID_BLOCK_END`
and the same `STATIC_HOLD_START` / `STATIC_HOLD_END` events as the
one-second preset. For analysis, pair completed events and use a window
appropriate for 500 ms. The prior benchmark's 400 ms start trim plus
50 ms end trim leaves only ~50 ms per hold. Its session-specific replay
also expects 27 points after a moving phase; don't use it unchanged here.

From the repository on the real rig, with the usual services up and the
LED fixture mounted, the source preset can be used without rebuilding an
editable install:

```bash
source ./setup.bash
tt-compose run --rm commander tt-launch tasks \
  task:=/tabletop/src/ros/tabletop/tabletop_tasks/config/smooth_pursuit_static_grid \
  robot_mode:=real rosbag:=true
```

The invariant-affordance task uses `small_object_28`, directly behind the
front-right mount, and `small_object_0` on the front-left mount. It presents
each at the standard
center pose, reveals it for 1 second, and re-occludes the glass. Target
positions are sampled uniformly from the configured presentation box.
Orientations use shuffled, equal-area tilt strata so every three accepted trials
per arm span the full range from upside-down through sideways to upright. After
a 0.5-second glass-settle interval, a general collision planner may use a
complex path anywhere on that arm's half of the rig, including toward its
storage area; normal collision checking remains active, and a small centerline
overlap lets either arm reach presentation. Invalid or over-budget candidates
are discarded before the object is revealed. The go tone and both hand buzzers
occur exactly 3 seconds after occlusion; plans
reserve at least 0.25 seconds
between expected motion completion and that cue. Both hand buzzers run for
their firmware-bounded 1-second interval, and the robot holds the final pose
until the
2-second trial window ends. After that behavioral window, it follows a
separately planned and validated path to the center while the glass remains
opaque so normal
unpresentation starts from its established pose. It does not read or score a
response button.

Launch it with `tt-launch tasks task:=invariant_affordance robot_mode:=mock`.

## Recording readiness at task startup

With `rosbag:=true`, the task waits for this launch's combined camera/recorder
preflight and EyeLink recorder discovery before enabling task sync pulses,
publishing `TASK_STARTED`, or running a trial. A missing, failed, stale, or
camera-only preflight report cannot release the task. The wait is bounded and
cancellable; it is not a fixed sleep and does not change trajectory speed or ITI.
Unrecorded tasks keep their existing startup behavior.

Commander starts from a delayed robot-description event. Its recording-gate
flags and launch timestamp are therefore captured into the process's explicit
environment before the temporary task-launch scope ends. This prevents the
readiness checks from silently disappearing when the event fires later.

The normal launch command is unchanged. For a recorded run, expect
`Waiting for FLIR recording preflight` followed by
`FLIR recording preflight passed` before task sync pulses are enabled.

## Manipulation preflight and real commissioning

After moving or recalibrating mounts, regenerate the branch report in mock
mode only:

```bash
tt-preflight run
```

The command prints a physical-grid PASS / UNAVAILABLE summary when it finishes.
To display the most recently saved report again without repeating the search:

```bash
tt-preflight results
```

The report under `.cache/tabletop/` is reused while every object pose and
manipulation-setting fingerprint remains unchanged. Task startup excludes
`UNAVAILABLE`, stale, and missing objects. Branch search and the cycle harness
always refuse `robot_mode:=real`.

After verifying a fresh mock report, prepare separate, ignored
real-commissioning configs with:

```bash
tt-preflight prepare-real
```

This creates a two-trial smoke test, a finite all-accessible test, and the
normal long-running task under `.cache/tabletop/`. Re-run it only after the
foraging configuration or preflight results change. Real-mode loading rejects
reports that are not marked `mock_planning_only` or whose current fingerprints
do not match. Generated real configurations explicitly install compatible PASS
branches and exclude unavailable, stale, or missing mounts. PRE_FETCH can
therefore reuse the deterministic branch and trajectory cache used by the
stable baseline; a cache miss still runs normal collision-checked planning.

When a finite object-interaction task ends, it occludes the glass, locks both
arms, returns staged objects to their mounts, and skips the unnecessary final
IDLE motion. Ctrl-C remains different: it cancels both active controller goals
immediately and deliberately starts no cleanup motion.

!!! tip "Robot already holding an object?"
    If the arm starts a session holding a grid object, tell the commander which
    grid index it holds: `tt-launch tasks initial_object:=5,0 …` (or just put
    the object back yourself).
