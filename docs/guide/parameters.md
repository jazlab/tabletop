# Node & Interface Parameters

This page summarises the main ROS 2 parameters accepted by the `Commander` node
and its interfaces.  The canonical, always-up-to-date source is the config file
itself:

```
src/ros/tabletop/tabletop_rig/config/commander.yaml
```

Every parameter in that file carries an inline comment.  The sections below are
a first-pass overview, grouped by node/interface; refer to the YAML for the
exact types, defaults, and caveats.

## Common / override pattern

`commander.yaml` is loaded by `commander.launch.py` and merged at runtime with a
per-session `/tmp/commander_overrides.yaml` (populated by `tt-launch` from
launch arguments such as `robot_mode`).

Lookups use `BaseInterface.param(name)`: the per-interface key takes precedence,
then falls back to the corresponding `common_*` section (dict values are
deep-merged).  For example, `left_ur_interface.namespace` overrides
`common_ur_interface.namespace`.

---

## Commander (nodes/commander.py)

| Parameter | Description |
| --- | --- |
| `simulate` | When `true` (set from `robot_mode:=mock`), skips the post-grasp effort check and relaxes certain UR mode assertions.  Consumed only by the per-arm manipulation stack. |
| `shutdown_timeout` | Maximum seconds to wait for all interfaces to shut down cleanly (default `3.0`). |
| `robot_interface_names` | Maps logical manipulator names to the three per-arm interface instance names (`ur_interface_name`, `manipulation_interface_name`, `manipulation_context_interface_name`). |
| `smooth_pursuit.reward_duration` | Duration (s) of each individual reward pulse during smooth-pursuit. |
| `smooth_pursuit.reward_interval` | Minimum interval (s) between consecutive reward pulses. |
| `smooth_pursuit.reward_threshold_ratio` | Minimum ratio of "pursing" feedback messages needed to activate reward. |
| `flic.bd_addrs` | Map of `object_id` → Bluetooth MAC address for each Flic 2 button. |

---

## SoundInterface (interfaces/sound.py)

| Parameter | Description |
| --- | --- |
| `enable` | Set `false` to silence all audio output. |
| `soundfont_path` | Absolute path to a SoundFont `.sf2` file. |
| `instrument` | General MIDI instrument number (0–127); default `62` (Synth Brass 1). |
| `default_note.{name,octave,velocity,channel}` | Note played by `SoundInterface.play()` when no note is specified. |
| `default_duration` | Default playback duration (s) for `SoundInterface.play()`. |

---

## TeensyInterface (interfaces/teensy.py)

| Parameter | Description |
| --- | --- |
| `safe_to_execute.required_time` | Seconds the safety conditions must be continuously met before `safe_to_execute` returns `True`. |
| `safe_to_execute.max_sensor_delay` | Maximum age (s) of the last `TeensySensor` message before the sensor is considered stale and `safe_to_execute` returns `False`. |
| `spin_period` | Poll interval (s) in `lock_arms_and_wait` / `start_reward_and_wait` busy-wait loops. |
| `sensor_delay_warn_threshold` | Log a warning when ROS receive latency of a `TeensySensor` message exceeds this value (s). |

---

## URInterface (interfaces/ur.py)

Shared defaults live in `common_ur_interface`; per-arm sections
(`left_ur_interface`, `right_ur_interface`) override individual keys.

| Parameter | Description |
| --- | --- |
| `namespace` | ROS topic/node namespace prefix for this arm's UR driver nodes (`left` / `right`). |
| `installation` | UR controller installation file name loaded on the teach pendant. |
| `program` | UR controller program file name to load and play for `external_control`. |
| `max_reset_attempts` | Maximum total recovery attempts before giving up. |
| `reset_retry_delay` | Seconds between consecutive reset attempts. |
| `safety_restart_enable` | If `true`, automatically restarts the safety controller on `VIOLATION`/`FAULT`; if `false`, waits for operator intervention. |
| `safety_restart_delay` | Seconds to wait before calling `restart_safety` (gives the operator a window to abort). |
| `safety_restart_timeout` | Seconds to wait for the robot to complete a safety restart. |
| `check_remote_control_delay` | Seconds between polling for remote-control mode to become active. |
| `check_safety_mode_delay` | Seconds between polling for safe safety mode after a protective stop. |
| `post_reset_delay` | Seconds to wait after a successful reset before returning control. |

---

## ObjectManipulationInterface / PlanAndExecuteInterface

Shared defaults live in `common_manipulation_interface`; per-arm sections
(`left_manipulation_interface`, `right_manipulation_interface`) override them.

### Planning

| Parameter | Description |
| --- | --- |
| `planning.fast_pipeline` | Primary planning pipeline (e.g. `ptp` for Pilz point-to-point). |
| `planning.fallback_pipeline` | Fallback pipeline when the fast path fails (e.g. `aps_rrt_star`). |
| `planning.default_max_attempts` | Default maximum planning attempts per request. |
| `planning.default_exp_backoff_factor` | Exponential backoff multiplier applied to planning timeouts on retry. |
| `planning.max_backoff_time` | Cap on the backoff delay (s) between retries. |
| `planning.default_pose_link` | Link used to evaluate end-effector poses (per-arm; e.g. `left_eef`). |

### Trajectory cache

| Parameter | Description |
| --- | --- |
| `trajectory_cache.base_dir` | Root directory for cached trajectories. |
| `trajectory_cache.use_cached_trajectories` | If `false`, always plans from scratch. |
| `trajectory_cache.freeze_cache` | If `true`, reads from cache but never writes new entries. |
| `trajectory_cache.backend` | Cache backend: `lmdb` (persistent), `kdtree`, `dict`, or `linear`. |
| `trajectory_cache.kwargs` | Backend-specific keyword arguments (e.g. `position_tolerance`, `orientation_tolerance`, `sort_by`). |

### Execution

| Parameter | Description |
| --- | --- |
| `execution.allowed_duration_scaling` | Multiplier on nominal trajectory duration for the execution timeout window. |
| `execution.allowed_duration_margin` | Additional seconds added to the scaled duration window. |
| `execution.allowed_start_tolerance` | Maximum joint-position error (rad) between current state and trajectory start before execution is blocked. |
| `execution.allowed_end_tolerance` | Maximum joint-position error (rad) at the end of execution before the result is considered a failure. |
| `execution.joint_trajectory_controller` | ROS 2 controller action name used to send trajectories (per-arm). |

### Object attachment / manipulation

| Parameter | Description |
| --- | --- |
| `test_object_attached.enable` | If `false`, skip the post-grasp effort check. |
| `test_object_attached.joint_name` | Joint whose effort is sampled for attachment detection. |
| `test_object_attached.effort_threshold` | Effort threshold (Nm) compared with `greater_than` to confirm attachment. |
| `detach_velocity_scaling_factor` | Velocity scaling (0–1) applied when detaching an object from its mount. |
| `skip_idle_on_return` | If `true`, skip moving to the idle pose when returning from the presented state. |
| `skip_fetched_on_fetch` | If `true`, skip the fetched pose when the object is already beyond the pre-fetch waypoint. |
| `persistent_state_path` | File path where the arm's persistent manipulation state is pickled across sessions. |
| `attach_link` | Robot link to which the grasped object collision mesh is attached. |
| `touch_links` | Links allowed to touch the attached object without collision error. |
| `mount_collision_ids` | Collision IDs of the physical rig mount, excluded from collision checking during manipulation. |
| `manipulation_state_goals` | Named waypoint goals for each stage of the fetch/present/return state machine (type: `named_target_state` or `offset`). |
| `reset_config_map` | Per-object reset YAML paths; `null` means the object cannot be reset via `reset_object`. |

---

## ManipulationContextManager (nodes/commander.py)

Shared defaults live in `common_manipulation_context_interface`.

| Parameter | Description |
| --- | --- |
| `interruptions.max_attempts` | Maximum times a manipulation action is retried after an interruption (e.g. safety stop) before raising. |
| `reset.max_attempts` | Maximum times the reset coroutine is retried before giving up. |

---

## MoveItInterface (interfaces/moveit/moveit.py)

| Parameter | Description |
| --- | --- |
| `current_state_wait_time` | Seconds to wait for the planning scene monitor to report a fresh robot state. |
| `link_padding` | Per-link collision padding (m) added to the robot's collision geometry. |
| `exclusive_regions` | Named collision regions that only one arm may occupy at a time; `collision_ids` lists the wall IDs that gate access. |
| `planning_scene.cache_dir` | Directory where the serialised scene and collision matrix are cached. |
| `planning_scene.use_saved_scene` | If `true`, load scene from cache when the config+mesh hash matches. |
| `planning_scene.rig.primitives` | Static box/plane collision objects for the physical rig structure. |
| `planning_scene.rig.meshes` | Static mesh collision objects (rig body, reset stations). |
| `planning_scene.grid_objects` | Dynamic experiment objects placed on the 10×3 presentation grid. |

---

## Other config files

The table in [Configuration](configuration.md#parameter-files-config--consumer)
lists the remaining config files.  Key ones:

- `tabletop_rig/config/flir_synchronized.yaml` — camera list, serial numbers,
  trigger/chunk settings, exposure controller, and camera poses.  See also the
  [FLIR GenICam node reference](configuration.md#parameter-files-config--consumer)
  tip in that page for the BFS-U3-23S3 node reference link.
- `tabletop_rig/config/blackfly_s.yaml` — mapping from ROS parameter names to
  GenICam node paths, shared by the unsynchronised FLIR driver.
- `tabletop_tasks/config/<task>.yaml` — task class, kwargs, and trial generator.
  See [Tasks](tasks.md).
- `tabletop_moveit_config/config/*.yaml` — MoveIt planners, limits, and
  controllers.  Follow standard MoveIt conventions; see the
  [MoveIt docs](https://moveit.picknik.ai/).
