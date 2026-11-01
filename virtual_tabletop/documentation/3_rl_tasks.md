# The RL tasks: run the demo, use the env

The RL environment (`PresentationEnv`) is a stripped-down version of the
task for training: both monkey arms (with fingers) and one presented object
per trial. There are no robot arms, object grid or smartglass; the rig
frame is optional. It runs **MuJoCo physics**: the arms are driven by joint
position servos and real contact forces act between the bones and the
object, so the hand can't pass through anything.

It needs **no Docker, no ROS and no recorded bags**. It runs natively on the
Mac.

Verified on this laptop on 2026-10-31.

---

## The three tasks

| `--reward` | What is rewarded | A trial ends |
|---|---|---|
| `none` (default) | nothing | at the response timeout |
| `touch` | a released hand touching the object | on the touch, or at the timeout |
| `button` | a released hand pressing the object's red **button** | on the press, or at the timeout |

The buttons are on the small cup's inner floor and on the big drum's top
(`virtual_tabletop/config/buttons.yaml`). In the button task, touching the
object elsewhere is not rewarded.

---

## Run the demo

```bash
cd ~/Desktop/tabletop/tabletop_current/tabletop/virtual_tabletop
uv run vtt-rl-demo                    # no-reward task
uv run vtt-rl-demo --reward touch
uv run vtt-rl-demo --reward button
```

A MuJoCo window opens (on macOS it re-launches itself under `mjpython`,
which MuJoCo's viewer needs) and plays 10 trials with a **scripted
monkey**: during each response window the released arm reaches for the
object (touch/none) or comes down onto the button from above (button).
Episodes restart automatically. In the button task, the first trial with
each object and arm pauses for about 1.5 s while the scripted press path
is planned.

Top-left text: the task, time, trial number and object, which arm is
released (`arm:left/right/both`), the phase, whether the stimulus is
visible (`glass:clear/opaque`, as the rig's glass would be), and
`TOUCH(side)`, `PRESS(side)`, `REWARD` when they happen.

| Key | Action |
|---|---|
| space | pause / resume |
| `=` / `-` | faster / slower (×2) |
| R | restart the episode |

### Options
```bash
uv run vtt-rl-demo --response-timeout 3   # response window in s
uv run vtt-rl-demo --rig                  # show the rig frame
uv run vtt-rl-demo --policy still         # monkey doesn't move
uv run vtt-rl-demo --trials 4             # episode length
uv run vtt-rl-demo --task foraging_random # a different task YAML
```

The response window defaults to the task config's (the rig's 0.5 s), or
3 s for the button task: the scripted press takes about 1.5 s.

### Save a video instead
```bash
uv run vtt-rl-demo --reward button --video rl_button.mp4 --trials 6
```
It prints how many trials were touched, pressed and rewarded.

---

## What a trial is

Timings, objects, poses and released arms come from the task YAML
(`src/ros/tabletop/tabletop_tasks/config/foraging_ordered.yaml` by default):

1. **intertrial** (0.5 s): the object is already in place.
2. **stimulus** (`stimulus_duration`): the stimulus is visible.
3. **delay** (`delay_duration`): still visible unless the trial says
   `occlude`.
4. **response** (`response_timeout`): the trial's arm(s) are released.

The next trial starts immediately with a new object; nothing is returned or
reset. Objects alternate between the left and right robot arms' object
lists, as the rig alternates arms.

---

## Use it from Python

```python
from virtual_tabletop.rl.env import PresentationEnv
from virtual_tabletop.rl.policies import scripted_policy

env = PresentationEnv(num_trials=20, reward="button", response_timeout=3.0)
obs, info = env.reset(seed=0)
policy = scripted_policy(env)       # or your agent
done = False
while not done:
    obs, reward, terminated, truncated, info = env.step(policy(obs))
    done = terminated or truncated
print([(t.spec.object_id, t.pressed_by, t.rewarded) for t in env.trials])
```

`env.observation_layout` maps each observation part's name to its slice
(e.g. `obs[env.observation_layout["object_position"]]`).

- **Action:** joint-angle targets for all 46 monkey joints (per arm: 7 arm
  and 16 finger joints), sent to position servos. `joints=[...]` (e.g.
  `"right/elbow_flexion"`) controls a subset. The servo gains, torque limit
  and gravity compensation are provisional
  (`rl_actuation` in `config/subject.yaml`).
- **`control="kinematic"`:** sets the angles directly with no dynamics
  (about 7× faster, but bodies can pass through each other).
- **Observation:** joint angles and velocities, both fingertip positions,
  object position, button position, released flags (left, right), stimulus
  visible, time left in the phase (`env.observation_layout` names the
  slices).
- **`info`:** trial number, object, released arm, phase, `touched_by`,
  `pressed_by`, `trial_ended`.
- **Speed (no rendering):** about 5,900 env steps/s with physics (about
  117× real time at dt = 0.02 s); about 40,000 kinematic.

---

## Related tools

```bash
uv run vtt-object-sheets         # drawings of each object geometry (cm grid)
uv run vtt-button-reach          # can each button be pressed without going
uv run vtt-button-reach --scan   #   through the object? (and vs shoulders)
```

## Troubleshooting

| Symptom | Fix |
|---|---|
| `launch_passive requires that the Python script be run under mjpython` | Use `uv run vtt-rl-demo` (it re-launches itself), not `python -m …`. |
| Model looks out of date after changing `config/*.yaml` | `uv run vtt-build` rebuilds all models, including the RL ones. |
| Button task never presses | Use a longer window (`--response-timeout 3`); check `uv run vtt-button-reach`. |
