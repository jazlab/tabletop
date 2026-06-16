# TableTop

**TableTop** is a ROS 2 platform for controlling Universal Robots UR5e arms in
a monkey electrophysiology research rig. It runs entirely in Docker containers
and uses [MoveIt 2](https://moveit.picknik.ai/) for motion planning.

The system presents physical objects to a subject, measures their responses
(button presses, eye gaze), delivers rewards, and records synchronized sensor
and robot-state data for offline analysis.

## What's here

<div class="grid cards" markdown>

- :material-rocket-launch: **[Getting Started](getting-started/setup.md)** —
  clone, set up the environment, and bring up the containers.
- :material-sitemap: **[Architecture](architecture.md)** — the conceptual
  dependency map: who talks to whom and where to look when something breaks.
- :material-book-open-variant: **[Guide](guide/cli.md)** — CLI tooling,
  configuration, the task system, hardware & safety, and troubleshooting.
- :material-api: **[API Reference](reference/index.md)** — auto-generated
  from the source for `tabletop_py`, `tabletop_rig`, and `tabletop_tasks`.

</div>

## The system at a glance

The platform is layered; each layer only reaches *down*:

```
┌──────────────────────────────────────────────────────────────────┐
│ EXPERIMENTS   tabletop_tasks: ForagingTask, SmoothPursuitTask, …  │
├──────────────────────────────────────────────────────────────────┤
│ ORCHESTRATION tabletop_rig Commander node (aggregates interfaces) │
├──────────────────────────────────────────────────────────────────┤
│ DEVICE NODES  ur driver, teensy, flic, eyelink, flir, optitrack   │
├──────────────────────────────────────────────────────────────────┤
│ ROS PLUMBING  tabletop_interfaces, tabletop_description, moveit    │
├──────────────────────────────────────────────────────────────────┤
│ PURE PYTHON   tabletop_py: gaze ML, flic protocol, utils          │
├──────────────────────────────────────────────────────────────────┤
│ INFRA         bin/ scripts → compose.yaml services → Docker       │
└──────────────────────────────────────────────────────────────────┘
```

Every container shares `network_mode: host` and `ipc: host`, so all ROS nodes
share one DDS domain regardless of which container they run in. See
[Architecture](architecture.md) for the full picture.

!!! note "Conventions"
    Units follow [REP 103](https://www.ros.org/reps/rep-0103.html): meters,
    seconds, radians. Python targets 3.12 and is formatted with `ruff`
    (79-char lines, Google-style docstrings).
