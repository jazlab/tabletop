# Virtual TableTop

A MuJoCo copy of the TableTop experiment, generated from the rig's own
sources, for:

- **replaying sessions** recorded on the rig (or the Docker ROS sim) from
  their ROS bags;
- **re-running the foraging task** on simulated time and checking it against
  bags (parity);
- **RL**: a virtual monkey (two arms with fingers) doing simplified
  presentation tasks: no reward, reward for touching the object, or for
  pressing a button on it.

It never imports ROS: it reads the rig's xacro, launch, SRDF, `commander.yaml`
and task YAMLs directly, and decodes MCAP bags with the schemas embedded in
them. It is a uv project separate from the repository root.

## Quick start

```bash
cd virtual_tabletop
uv sync --all-extras        # first time; also `git lfs pull` at the repo root
uv run vtt-build            # generate the models into .cache/generated/
uv run pytest tests         # or -m "not slow" for the quick ones

uv run vtt-replay ../bags/latest/bag          # replay a bag (viewer)
uv run vtt-rl-demo --reward touch             # watch the touch task
```

## Documentation

See [`documentation/`](documentation/README.md):

- [architecture.md](documentation/architecture.md): the two models (rig vs
  RL), what each package does, every command, generated files, known gaps;
- [configuration.md](documentation/configuration.md): the `config/` files
  and the rig sources read;
- how-tos: run the ROS sim and RViz on a Mac; record and replay bags; the RL
  tasks; the arm-design tools.

## Layout

```text
config/            rig.yaml, subject.yaml, buttons.yaml (sim-only facts)
monkey_arm/        the right-arm MJCF and bone meshes
documentation/     architecture, configuration, how-tos
scripts/           record_sim_trials.sh (Docker ROS sim → bag)
src/virtual_tabletop/
├── assets/        model generation (vtt-build): rig model, RL model
├── bags/          MCAP reading (vtt-bag-info)
├── replay/        bag playback (vtt-replay)
├── task/          foraging on sim time, parity (vtt-clips, vtt-parity)
├── buttons/       response buttons on the objects (RL model)
├── rl/            the RL env, scene, trials, scripted policies (vtt-rl-demo)
├── design/        tools for deciding the RL setup (vtt-arm-explorer, …)
├── viewer.py      shared interactive-viewer loop
└── ik.py          IK (adapted from dm_control)
tests/             pytest; fixtures/ holds a small motion library
```
