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
| Monkey arm (7 arm + 16 finger joints, joint motors, no muscles) | `monkey_arm/right_arm_fingers_v0.xml` (alternative: `right_arm_v0.xml`, 39 muscles, no fingers; set `subject.arm_file`) |
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

## The foraging task in MuJoCo (`task/`)

A re-implementation of the rig's `ForagingTask` on simulated time (same task
YAMLs, generators and trial logic; robot motion replayed from recorded clips
instead of MoveIt). Build the clip library from sim sessions once, then:

```bash
uv run vtt-clips ../bags/session_<...>/bag            # -> .cache/motion/library.npz
uv run vtt-parity ../bags/session_<...>/bag --task ../.cache/tabletop/<config>.yaml
```

```python
from virtual_tabletop.task.env import ForagingEnv, run_scripted_episode
env = ForagingEnv("foraging_ordered")       # Gymnasium env; reward 1 per rewarded trial
print(run_scripted_episode(env, seed=0))    # IK reaching baseline
```

The env's action space (kinematic joint targets) is provisional while the
joint-control design is being decided.

## The simplified RL environment (`rl/`)

`PresentationEnv`: no robots or object grid (rig frame optional), both monkey
arms, the smartglass, and one presented object per trial (intertrial →
stimulus → delay → response; ends at the timeout, or on a touch with
`reward_on_touch=True`). About 5× faster than the full foraging env.

```bash
uv run vtt-rl-demo                       # watch a scripted monkey (viewer)
uv run vtt-rl-demo --video rl.mp4 --reward-on-touch
```

The left arm is generated by mirroring the right arm file
(`assets/subject.py`); arm placements are in `config/sim_rig.yaml`.
