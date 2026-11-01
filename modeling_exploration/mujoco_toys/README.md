# MuJoCo toys: reward-free exploration

Small MuJoCo worlds for studying agents that learn to do interesting things
with **no task reward**, only intrinsic motivation (curiosity, novelty,
empowerment, skill discovery). The agents are deliberately simpler than the
virtual monkey's hand: a ball, or a 3-joint arm.

## Implemented environments

Chosen 2026-11-01: **the playpen** (concept 1) and **the 3D arm with blocks**
(concept 13). Gymnasium environments (`import mujoco_toys` registers
`MujocoToys/Playpen-v0` and `MujocoToys/ArmBlocks-v0`). **Reward is always
0**; `info` reports diagnostics (objects touched by the agent, objects that
moved, spawns) for analysis, not for optimising.

```bash
cd modeling_exploration/mujoco_toys
uv sync
uv run pytest tests
uv run toys-demo playpen --video playpen.mp4 --spawn-rate 0.3 --drop 0.6
uv run toys-demo arm --video arm.mp4 --camera top
```

```python
from mujoco_toys import PlaypenEnv, ArmBlocksEnv

env = PlaypenEnv(n_objects=6, max_objects=12, spawn_rate=0.2,
                 obs_mode="state")          # or "pixels" / "both"
obs, info = env.reset(seed=0)
obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
```

| | Playpen (`PlaypenEnv`) | Arm and blocks (`ArmBlocksEnv`) |
|---|---|---|
| Agent | a rolling ball (r = 12 cm) in a 3 m walled arena | a 3-joint arm on a pedestal: base yaw, shoulder, elbow; sphere tip, no hand |
| Action | 2: horizontal force on the ball, in [-1, 1] (× 10 N) | 3: joint-angle targets across each range (position servos), or torques (`control="torque"`) |
| Objects | boxes, spheres, cylinders, rolling capsules; random size, mass (0.1–1 kg), colour (`object_kinds`, `object_scale`, `object_mass`) | boxes 7–12 cm, 0.05–0.4 kg, within reach; `towers` stacked at the start (default one tower of 3) |
| Agent state | ball position (3), velocity (3) | joint angles (3), velocities (3), tip position (3) |
| Cameras | `overview`, `top`, `agent` (follows the ball) | `overview`, `top`, `side` |
| Speed (state obs) | about 3,200 steps/s (each step 40 ms of simulation) | about 2,800 steps/s |

Shared options (`ToyEnv`): `n_objects` at the start, `max_objects` slots,
`spawn_rate` (new objects per second; the oldest is recycled when the slots
are full), `drop_height` (spawned objects fall in), `episode_seconds`
(default 30), `control_dt` (default 0.04 s), `obs_mode` (`"state"`,
`"pixels"` 96×96 by default, `"both"`), `camera`. Each episode draws new
random object shapes. The state vector is the agent state followed by
`max_objects` slots of 22 numbers (active flag, pose, velocities, size,
shape one-hot, mass; zeros when not in the scene);
`env.observation_layout` names the slices.

Code: `src/mujoco_toys/` (`scene.py` floor/walls/lights, `objects.py` object
slots and spawning, `envs/base.py` the reward-free env, `envs/playpen.py`,
`envs/arm_blocks.py`, `demo.py`). Physics notes: friction is 0.5 for every
surface (MuJoCo uses the larger of two geoms' values, so a stickier floor
pins heavy objects); joint ranges are in radians (`compiler.degree =
False`).

## Candidate environments (the original menu)

All renders are real MuJoCo scenes built by
`concepts/render_concepts.py` (`concepts/images/`, overview in
`concepts/images/contact_sheet.png`). Objects spawn at random positions
(and, where noted, keep spawning).

| # | Environment | Agent | What there is to discover | Good for testing |
|---|---|---|---|---|
| 1 | **Playpen**: walled arena, random objects (boxes, balls, cylinders, capsules; varied size, mass, colour) | ball (2 DOF: push in x, y) | pushing, rolling vs sliding, heavy vs light, corners | the baseline; any method (RND, ICM, disagreement, count-based) |
| 2 | **Object rain**: the playpen, but new objects keep falling from the sky at random times and places | ball | a never-ending stream of novelty; catching or avoiding falling things | non-stationary novelty; whether curiosity chases new arrivals |
| 3 | **Toggle room**: floor pads that switch coloured lamps on/off (with a few objects) | ball | controllable effects at a distance; discrete state changes | empowerment, "controllable novelty", causal discovery |
| 4 | **Noisy TV**: the playpen plus a wall screen of randomly flickering colours | ball | (a trap) unpredictable but uncontrollable pixels | the classic failure of prediction-error curiosity; RND/disagreement vs ICM |
| 5 | **Hills and ramp**: bumpy terrain and a ramp; objects roll downhill | ball | gravity, rolling, heights; pushing things uphill | exploration in 3D state; dynamics models with gravity |
| 6 | **Sticky ball**: an adhesion actuator lets the ball pick things up and drop them | ball + "stick" action (3 actions) | carrying, collecting, stacking against walls | rich skills without a hand; learning a discrete-ish action's effect |
| 7 | **Tethered toy**: the ball is tied by a rope to a heavy box | ball | dragging, swinging, using the toy as a tool | delayed/indirect consequences |
| 8 | **Billiards**: low-friction table of balls | ball (cue) | chain reactions, collisions | long-horizon, chaotic dynamics; prediction-based curiosity |
| 9 | **Two rooms**: objects are behind a doorway | ball | navigation then interaction | hard exploration (count-based, Go-Explore-like) |
| 10 | **Planar arm + pucks**: 3-joint arm sweeping a walled table with pucks | 3-joint planar arm | pushing with different links, sweeping, trapping | skill discovery (DIAYN, DADS, LSD/METRA), goal-conditioned play |
| 11 | **Planar arm + gadgets**: a drawer, a door, a spinning wheel, a lever, a push button | 3-joint planar arm | articulated objects with distinct affordances | affordance/"play" learning; disentangled skills per gadget |
| 12 | **3D arm + toy bin**: a 3-joint arm (yaw, shoulder, elbow) next to a bin of balls; objects fall in | 3-joint spatial arm | scooping, spilling, knocking things in/out | 3D control without a hand; closest to the TableTop arm |
| 13 | **3D arm + blocks**: blocks to topple (and a small tower) | 3-joint spatial arm | toppling, pushing, (rarely) stacking | object-centric curiosity; long-tail events |

### Agents (any environment can take either)

- **Ball:** 2 DOF (force in x and y), optionally an adhesion "stick" action
  (6) or a jump. Tiny action space, fast simulation (thousands of
  steps/s), easy to visualise.
- **3-joint planar arm:** hinges about the vertical, sweeping a table.
  Same action count as a ball plus one, but morphology matters (which link
  touches what).
- **3-joint spatial arm:** base yaw + shoulder + elbow, sphere tip. The
  natural bridge to the monkey arm (which uses 7 joints and a hand).

### Intrinsic-motivation methods these could compare

- prediction-error curiosity (ICM), random network distillation (RND),
  ensemble disagreement (Plan2Explore-style);
- count/density-based novelty;
- empowerment and skill discovery (DIAYN, DADS, LSD/METRA);
- goal-conditioned self-play (Skew-Fit, LEXA-style "set your own goals").

### Suggested starting points

- **Simplest informative:** 1 (playpen) with the ball, plus 4 (noisy TV) as
  a control.
- **Most "play"-like:** 11 (gadgets) with the planar arm, or 6 (sticky
  ball).
- **Closest to the monkey work:** 12 (3D arm + toy bin).
