# Architecture

The virtual tabletop is a ROS-free Python package. It reads the rig's own
sources (xacro, launch files, SRDF, `commander.yaml`, task YAMLs, CAD) and
decodes MCAP bags with the schemas embedded in them. It never imports ROS.

## Two models, two uses

```text
                       rig sources (xacro, SRDF, commander.yaml, rig.step)
                       config/rig.yaml, config/subject.yaml
                                        │
                       assets/  (parts.py: shared pieces)
                 ┌──────────────────────┴──────────────────────┐
     assets/rig_model.py                              assets/rl_model.py
     tabletop.xml                                     rl_tabletop[_rig].xml
     robots, grid objects, smartglass,                monkey (servos, rl placement),
     monkey (static, rig placement)                   2 stimulus bodies + buttons
                 │                                             │
   ┌─────────────┴─────────────┐                     rl/  (env, scene, trials,
 replay/                    task/                         policies, press_paths)
 bag → playback          foraging on sim time,                 │
 (vtt-replay)            parity vs bags                  vtt-rl-demo; training/
                         (vtt-clips, vtt-parity)         (PPO: vtt-rl-train, vtt-rl-eval)
                         (vtt-clips, vtt-parity)
   bags/: MCAP reading (vtt-bag-info), used by replay/ and task/

 design/: tools for deciding the RL setup (arm explorer, arm report,
          object sheets, button reach). Nothing in the runtime imports them.
```

| | Rig model (`tabletop.xml`) | RL model (`rl_tabletop.xml`) |
|---|---|---|
| Used by | bag replay, foraging sim, parity | RL env, scripted policies, design tools |
| Robots | both UR5e arms, calibrated | none |
| Objects | the 10×3 grid (mocap) | one body per geometry (`stim/small_object`, `stim/big_object`), placed per trial |
| Buttons | no | yes (`config/buttons.yaml`) |
| Smartglass | yes | no |
| Monkey placement | `placements.rig` (where the animal sits) | `placements.rl` (tuned for reach) |
| Monkey actuation | none used (static) | position servo per joint (`rl_actuation`) |
| Simulation | kinematic (frames set `qpos`) | physics by default (`mj_step`, contact forces); kinematic option |

## Packages

| Package | Role | CLIs |
|---|---|---|
| `assets/` | model generation: `robots.py` (xacro → URDF), `rig_config.py` (`commander.yaml`), `named_poses.py` (SRDF), `meshes.py` (mesh conversion), `subject.py` (the monkey, left-arm mirroring), `sim_config.py` (config loaders), `parts.py` (shared pieces), `rig_model.py`, `rl_model.py`, `build.py` | `vtt-build` |
| `bags/` | MCAP reading; joint states, task events, Teensy, Flic | `vtt-bag-info` |
| `replay/` | grasp inference, bag → timeline, viewer and video | `vtt-replay` |
| `task/` | the rig's foraging task on simulated time: config, trial generators (ported), motion clips from bags, `ForagingSim`, simulated responses, parity checks | `vtt-clips`, `vtt-parity` |
| `buttons/` | button config and model additions (incl. the cup's hollow collision) | |
| `rl/` | `PresentationEnv` (Gymnasium) and its scene, trial schedule, scripted policies, press-path planner | `vtt-rl-demo` |
| `design/` | arm explorer, arm report, object sheets, button reach | `vtt-arm-explorer`, `vtt-arm-report`, `vtt-object-sheets`, `vtt-button-reach` |
| `training/` | RL training (optional `rl` extra): wrappers (delta actions, reach shaping), run configs, PPO, evaluation | `vtt-rl-train`, `vtt-rl-eval` |
| `viewer.py` | MuJoCo passive-viewer loop (and its locking rule), macOS `mjpython` re-launch, playback keys | |
| `ik.py` | damped least-squares IK (adapted from dm_control) | |

Also `scripts/record_sim_trials.sh` (record N trials from the Docker ROS
sim; how-to 2).

## Commands

| Command | Does |
|---|---|
| `vtt-build [--clean]` | build the rig and RL models |
| `vtt-bag-info BAG` | list a bag's topics; show messages |
| `vtt-replay BAG [--video F]` | replay a bag (viewer or video) |
| `vtt-clips BAG… [-o F]` | cut robot motion clips from sim bags |
| `vtt-parity BAG --task CFG` | run the MuJoCo foraging task and compare with a bag |
| `vtt-rl-demo [--reward …]` | watch a scripted monkey do an RL task |
| `vtt-arm-explorer` | pose the arms with sliders |
| `vtt-arm-report` | sweep videos, workspace, joint ablation |
| `vtt-object-sheets` | drawings of each object geometry |
| `vtt-button-reach [--offset F U] [--scan]` | button reachability (every pose, both arms) vs shoulder placement |
| `vtt-rl-train CONFIG` | train PPO on an RL task |
| `vtt-rl-eval RUN [--baselines] [--video F]` | evaluate a trained policy (and baselines) |

## Conventions

- World frame, metres, radians (REP 103). Object frames: origin at the base
  centre, +z up; at the presentation pose +x faces the monkey.
- Geom groups: 2 visual, 3 collision (robots, the cup's hollow shape), 4
  planning volumes, frames and mount sites (hidden).
- Only the monkey makes contacts: with the robots and objects, and bone with
  bone (joint neighbours filtered).
- Names: monkey elements `monkey/<side>/…`; stimulus bodies `stim/<geometry>`;
  buttons `stim/<geometry>/button`.
- macOS: interactive viewers re-launch themselves under `mjpython`. In viewer
  code, never call viewer methods while holding its lock (`viewer.py`).

## Generated files (`virtual_tabletop/.cache/`, git-ignored)

| Path | What | Remade by |
|---|---|---|
| `generated/` | the MJCF models | `vtt-build` (cleared by `--clean`) |
| `press_paths/` | scripted button-press paths, keyed by the RL model | first button-task run (cleared by `--clean`) |
| `meshes/` | converted and split meshes, mirrored left-arm STLs | `vtt-build`, on demand |
| `ur_description/` | the UR description package, pinned commit | `vtt-build`, on demand |
| `motion/library.npz` | robot motion clips | `vtt-clips` |
| `arm_report/`, `object_sheets/` | design-tool outputs | `vtt-arm-report`, `vtt-object-sheets` |
| `runs/<name>/` | training runs (config, logs, policy, evaluation, video) | `vtt-rl-train`, `vtt-rl-eval` |

## Known gaps

- The monkey isn't driven during replay (needs OptiTrack data; sim bags have
  none). Its rig placement is an estimate.
- The smartglass pane's extent is assumed (it isn't in the CAD).
- Objects left at the reset stations, and failed grasps, aren't tracked in
  replay.
- `foraging_ordered` presents at three poses (centred; 5 cm either side,
  tilted). Its own order cycles them only every ~78 trials, so RL training
  samples trials randomly (`trial_order="random"`). Tools showing "the"
  pose use the first (`rl.scene.presentation_pose`); press paths and
  reachability cover all three.
- `tabletop_tasks/config/foraging_random.yaml` lists objects that aren't on
  the grid in `commander.yaml`; they're dropped as unavailable.
- RL actuation (servo gains, torque limit, gravity compensation) and the
  arm's mass (77 g) are provisional.
