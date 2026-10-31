# Virtual TableTop

A MuJoCo model of the TableTop rig, generated from the rig's own sources, for
replaying sessions recorded on the physical rig and, later, for running the
tasks with a simulated monkey arm. It never imports ROS: it reads the rig's
xacro, launch, SRDF and `commander.yaml` files directly, and decodes MCAP bags
with the schemas embedded in them.

It is a uv project separate from the repository root (whose environment is the
rig's).

## Quick start

```bash
cd virtual_tabletop
uv sync --all-extras        # first time; also run `git lfs pull` at the repo root
uv run vtt-build            # writes .cache/generated/tabletop.xml
uv run pytest tests

# Look at the model (on macOS, MuJoCo's viewer needs mjpython)
uv run mjpython -m mujoco.viewer --mjcf=.cache/generated/tabletop.xml

# Inspect a bag, then replay it
uv run vtt-bag-info ../bags/latest/bag
uv run mjpython -m virtual_tabletop.replay.player ../bags/latest/bag
uv run vtt-replay ../bags/latest/bag --video replay.mp4 --camera subject
```

Replay keys: space pause, ←/→ ±2 s, N/P next/previous trial, `=`/`-`
faster/slower, R restart.

## What the model contains, and where it comes from

| Part | Source |
|---|---|
| Both UR5e arms + end-effectors, calibrated kinematics | `tabletop_description` xacro + `config/*_calibration.yaml` |
| Arm base poses | `tabletop_description/launch/dual_rsp.launch.py` |
| Home keyframe (SRDF `idle`), named poses | `tabletop_moveit_config/srdf/` |
| Rig frame, object holders, hot mirror | `tabletop_rig/meshes/rig/rig.step` |
| Reset stations, planning volumes, 10×3 object grid | `tabletop_rig/config/commander.yaml` |
| Monkey arm (7 joints, 39 muscles; muSim-derived) | `monkey_arm/right_arm_v0.xml` |
| Subject placement, smartglass pane, part colours | `config/sim_rig.yaml` (sim-only; not recorded anywhere in the rig) |

The UR description package is cloned into `.cache/` at a pinned commit.
Generated files live in the git-ignored `.cache/`.

Conventions: geom group 2 is visual, 3 robot collision meshes, 4 planning
volumes/frames/mount sites (hidden by default). Only the monkey makes contacts
(with the robots and task objects). Robots have gravity compensation and
joint position actuators. Task objects are mocap bodies, moved kinematically.

## Package layout

```text
src/virtual_tabletop/
├── paths.py              # repo locations; ROS package map (replaces ament index)
├── ik.py                 # damped least-squares IK (from dm_control)
├── assets/               # model generation: vtt-build
│   ├── robots.py         #   xacro -> URDF, no ROS
│   ├── rig_config.py     #   commander.yaml planning scene
│   ├── named_poses.py    #   SRDF named states
│   ├── meshes.py         #   DAE/STEP -> cached STL parts
│   └── build.py          #   MjSpec assembly
├── bags/                 # MCAP reading, no ROS: vtt-bag-info
│   ├── reader.py         #   generic reader, joint states
│   └── topics.py         #   task events, Teensy states, button presses
└── replay/               # bag -> model playback: vtt-replay
    ├── objects.py        #   infer held objects from end-effector motion
    ├── timeline.py       #   bag -> per-frame model state (no viewer)
    └── player.py         #   viewer and offscreen video
```

## Known gaps

- The monkey isn't driven during replay yet (needs OptiTrack data, which sim
  bags don't have).
- The subject placement and smartglass pane in `config/sim_rig.yaml` are
  provisional. From the provisional placement, the presented object is about
  0.36 m away, beyond the arm model's ~0.30 m reach.
- Objects left at the reset stations, and failed grasps, aren't tracked.

## Recording simulated sessions

With the Docker ROS sim running (`tt-compose --profile=sim up -d`) and a
manipulation preflight report (`tt-preflight run`), record N foraging trials:

```bash
RESPONSE_TIMEOUT=5 virtual_tabletop/scripts/record_sim_trials.sh N [TASK]
```

The session lands in `bags/latest`. `RESPONSE_TIMEOUT` is optional: without
it, sim trials time out before the simulated button press.
