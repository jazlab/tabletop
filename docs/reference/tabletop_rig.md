# tabletop_rig

The core rig-control package: ROS 2 nodes and the layered device interfaces
aggregated by the `Commander`. See [Architecture §5.2](../architecture.md) for
the interface inheritance chain and how the Commander composes them.

## Nodes — `tabletop_rig.nodes`

One ROS 2 node per process. `Commander` is the orchestrator; the rest are
device nodes and mock stand-ins for simulation.

::: tabletop_rig.nodes.base
::: tabletop_rig.nodes.commander
::: tabletop_rig.nodes.eyelink
::: tabletop_rig.nodes.flic
::: tabletop_rig.nodes.system_check
::: tabletop_rig.nodes.mock_teensy
::: tabletop_rig.nodes.mock_dashboard_client
::: tabletop_rig.nodes.mock_robot_state_helper

## Device interfaces — `tabletop_rig.interfaces`

Each interface wraps one subsystem and is owned by the `Commander`.

::: tabletop_rig.interfaces.base
::: tabletop_rig.interfaces.teensy
::: tabletop_rig.interfaces.ur
::: tabletop_rig.interfaces.flic
::: tabletop_rig.interfaces.eyelink
::: tabletop_rig.interfaces.sound

## MoveIt interfaces — `tabletop_rig.interfaces.moveit`

The motion-planning stack: planning scene, plan-and-execute (with fuzzy
trajectory caching), the pick/present/return state machine, and the unified
top-level `MoveItInterface`.

::: tabletop_rig.interfaces.moveit.moveit
::: tabletop_rig.interfaces.moveit.plan_and_execute
::: tabletop_rig.interfaces.moveit.object_manipulation
::: tabletop_rig.interfaces.moveit.requests
::: tabletop_rig.interfaces.moveit.trajectory_cache
::: tabletop_rig.interfaces.moveit.trajectory_cache_kdtree
::: tabletop_rig.interfaces.moveit.trajectory_cache_lmdb

## Async executors & exceptions

::: tabletop_rig.executors
::: tabletop_rig.exceptions

## Utilities — `tabletop_rig.utils`

::: tabletop_rig.utils.ros
::: tabletop_rig.utils.logging
::: tabletop_rig.utils.rosbag
