# Tasks & Experiments

A **task** is a behavioral experiment. Tasks live in `tabletop_tasks` and drive
the rig through the `Commander`; they never touch devices directly.

## How a task runs

```
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
|---|---|---|
| `ForagingTask` | `foraging_*` | Subject selects among presented objects |
| `PresentTask` | `present_*` | Passive object presentation |
| `SmoothPursuitTask` | `smooth_pursuit_*` | Object follows a trajectory for the subject to track |
| `DummyTask` | `dummy` | Minimal no-op task (smoke test / diagnostics) |

## Trial generators

Generators implement an iterator + `send(feedback)` protocol:
`BaseTrialGenerator` and the `{ordered,random}_choice[_alternating]` and
`blocked_cup_drawer` variants. The config's `trial_generator` key selects one
and supplies its kwargs (object groups, poses, occlusion probability, block
sizes, …).

## Configuring a task

Task configs are YAML files in `tabletop_tasks/config/`. Each lists one or more
`{class, kwargs}` entries; the kwargs map directly to the task class
constructor. Available configs:

| Config | Task | Notes |
|---|---|---|
| `foraging_ordered.yaml` / `foraging_random.yaml` | ForagingTask | ordered / randomized trials |
| `present_ordered.yaml` / `present_random.yaml` | PresentTask | ordered / randomized presentation |
| `smooth_pursuit_random.yaml` | SmoothPursuitTask | random waypoints |
| `smooth_pursuit_spiral{,_test}.yaml` | SmoothPursuitTask | helical trajectory |
| `smooth_pursuit_sin.yaml` | SmoothPursuitTask | sinusoidal trajectory |
| `dummy.yaml` | DummyTask | smoke test |

To create a new task, copy an existing config and adjust the kwargs. Every
config is commented inline; the class definitions in
`tabletop_tasks/tabletop_tasks/tasks/` and the generators in
`trial_generators/` are the source of truth for available parameters (see the
[API Reference](../reference/tabletop_tasks.md)).

```bash
tt-launch tasks task:=foraging_ordered robot_mode:=mock
```

!!! tip "Robot already holding an object?"
    If the arm starts a session holding a grid object, tell the commander which
    grid index it holds: `tt-launch tasks initial_object:=5,0 …` (or just put
    the object back yourself).
