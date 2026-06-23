# tabletop_tasks

Behavioral experiment definitions. A task config (YAML) names a task class and
its kwargs; `run_tasks` injects a `Commander` and drives the trial loop. Trial
generators implement an iterator + `send(trial_spec, feedback)` protocol so the next trial
can depend on the subject's last response. See
[Tasks & Experiments](../guide/tasks.md) and
[Architecture §5.3](../architecture.md).

## Entry point

::: tabletop_tasks.run

## Tasks — `tabletop_tasks.tasks`

::: tabletop_tasks.tasks.base
::: tabletop_tasks.tasks.foraging
::: tabletop_tasks.tasks.present
::: tabletop_tasks.tasks.smooth_pursuit
::: tabletop_tasks.tasks.dummy

## Trial generators — `tabletop_tasks.trial_generators`

::: tabletop_tasks.trial_generators.base
::: tabletop_tasks.trial_generators.ordered_choice_alternating
::: tabletop_tasks.trial_generators.random_choice_alternating
