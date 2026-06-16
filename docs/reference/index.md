# API Reference

Auto-generated from the in-source Google-style docstrings via
[mkdocstrings](https://mkdocstrings.github.io/) (static analysis — no ROS or
hardware imports required). Private members (leading underscore) are hidden.

The platform is split into three importable Python packages:

| Package | Role | Depends on |
|---|---|---|
| [`tabletop_py`](tabletop_py.md) | ROS-independent utilities: gaze ML, Flic clients, helpers | — |
| [`tabletop_rig`](tabletop_rig.md) | ROS 2 nodes & device interfaces (incl. in-process MoveIt) | `tabletop_py`, `tabletop_interfaces` |
| [`tabletop_tasks`](tabletop_tasks.md) | Experiment task definitions & trial generators | `tabletop_rig`, `tabletop_py` |

See [Architecture](../architecture.md) for how these fit together at runtime.
