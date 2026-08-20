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
| `present_ordered.yaml` / `present_random.yaml` | PresentTask | ordered / randomized presentation |
| `smooth_pursuit_random.yaml` | SmoothPursuitTask | random waypoints |
| `smooth_pursuit_spiral{,_test}.yaml` | SmoothPursuitTask | helical trajectory |
| `smooth_pursuit_sin.yaml` | SmoothPursuitTask | sinusoidal trajectory |
| `dummy.yaml` | DummyTask | diagnostic scratchpad |

To create a new task, copy an existing config and adjust the kwargs. Every
config is commented inline; the class definitions in
`tabletop_tasks/tabletop_tasks/tasks/` and the generators in
`trial_generators/` are the source of truth for available parameters (see the
[API Reference](../reference/tabletop_tasks.md)).

```bash
tt-launch tasks task:=foraging_ordered robot_mode:=mock
```

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
