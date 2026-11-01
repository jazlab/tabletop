# Configuration

The virtual tabletop reads the rig's own configuration from the repository
(never copied). Its own files, in `virtual_tabletop/config/`, hold only what
the rig doesn't record. Rebuild after changing any of them:
`uv run vtt-build`.

## `config/rig.yaml`: sim-only rig details

| Key | Meaning | Read by |
|---|---|---|
| `smartglass.center`, `.size` | the occluder pane (world, m). Assumed to fill the presentation opening | `assets/parts.py` (rig model) |
| `smartglass.occluded_rgba`, `.revealed_rgba` | its colour when opaque / clear | `replay/player.py`, `task/sim.py` |
| `part_colors` | STEP node name → rgba overrides for `rig.step` parts | `assets/parts.py` |

## `config/subject.yaml`: the monkey

| Key | Meaning | Read by |
|---|---|---|
| `arm_file` | the right-arm MJCF (`monkey_arm/…`); the left arm is mirrored from it | `assets/parts.py`, `assets/subject.py` |
| `placements.rig.<side>` | `shoulder_position` (world, m) and `yaw` (rad) of each arm in the **rig model** (replay, foraging sim) | `assets/rig_model.py` |
| `placements.rl.<side>` | the same for the **RL model**, tuned for reach | `assets/rl_model.py` |
| `rl_actuation.kp` | position-servo stiffness (N·m/rad), critically damped | `assets/parts.py` (RL model) |
| `rl_actuation.max_torque` | servo torque limit (N·m) | `assets/parts.py` |
| `rl_actuation.gravity_compensation` | cancel the arms' weight | `assets/parts.py` |

## `config/buttons.yaml`: response buttons (RL model)

| Key | Meaning | Read by |
|---|---|---|
| `buttons.<geometry>.position` | the button base's centre, object frame (m) | `buttons/model.py`, `rl/scene.py` |
| `.normal` | its outward normal (pressing is along −normal) | same |
| `.radius`, `.height` | size (m) | `buttons/model.py` |
| `.rgba` | colour | `buttons/model.py` |
| `hollow.<geometry>` | collision primitives replacing a geometry's convex hull (which would fill the cup and hide the button): `base` cylinder (`radius`, `z` range) and a `wall` ring of `segments` boxes (`inner_radius`, `outer_radius`, `z`). Tested against the mesh | `buttons/model.py` |

`<geometry>` is an object mesh stem: `small_object` (the cup) or
`big_object` (the drum). See `vtt-object-sheets` for drawings in the object
frame.

## `config/training/*.yaml`: RL training runs

One file per run (e.g. `touch_right_arm.yaml`): `name`, `seed`,
`eval_every`, `eval_trials`, and groups `env` (task, reward, trials,
response window, trial order, released and controlled arms, control),
`wrappers` (delta-action speed, shaping scale) and `ppo`
(hyperparameters, length, parallel envs). Every key is documented in
`src/virtual_tabletop/training/config.py`.

## Rig sources read directly

| Source | For |
|---|---|
| `tabletop_description/urdf/dual_tabletop.urdf.xacro` + `config/*_calibration.yaml` | robot kinematics |
| `tabletop_description/launch/dual_rsp.launch.py` | robot base poses |
| `tabletop_moveit_config/srdf/` | named robot poses, home keyframe |
| `tabletop_rig/config/commander.yaml` | planning scene, object grid, mounts |
| `tabletop_rig/meshes/` | rig CAD, object meshes |
| `tabletop_tasks/config/*.yaml` | task timings, trial generators (sim and RL) |
| `.cache/tabletop/manipulation_preflight.yaml` (repo root) | which objects are available |

Environment variables: `TABLETOP_DIR` (repo root; default the parent of
`virtual_tabletop/`), `VTT_CACHE_DIR` (default `virtual_tabletop/.cache`).
