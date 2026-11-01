# Record N simulated trials to a ROS bag, then replay them in MuJoCo

Two parts:
1. Record a session of exactly N foraging trials from the Docker ROS sim.
2. Replay the bag in the MuJoCo rig model, interactively or as a video.

Replay needs no ROS or Docker; it works on any bag from the rig, real or
simulated.

Verified on this laptop on 2026-10-31: a 4-trial and a 2-trial session,
replayed, with video rendered.

---

## Part 1: record N trials

### Prerequisites
- The simulated rig is up: steps 1–3 of
  `1_run_foraging_in_sim_and_view_in_rviz.md` (Docker Desktop running, then
  `tt-compose --profile=sim up -d`).
- The one-time manipulation preflight exists (already done on this laptop).

### Record
From the repo root, in zsh or bash:

```bash
cd ~/Desktop/tabletop/tabletop_current/tabletop
RESPONSE_TIMEOUT=5 virtual_tabletop/scripts/record_sim_trials.sh 10
```

- `10` is N, the number of trials. The task stops by itself after N.
- Optional second argument: the task config, default `foraging_ordered`
  (e.g. `... 10 foraging_random`).
- **`RESPONSE_TIMEOUT=5` is recommended.** The shipped config allows 0.5 s
  for a response, but the simulated button presses after 1–3 s, so without
  it every trial times out with no button press or reward in the bag.
  Leave it off to record exactly the shipped behaviour.

The script:
1. writes a copy of the config with your N (and timeout) into the
   git-ignored `.cache/tabletop/`;
2. starts the recorder;
3. runs the task with mock robots;
4. stops the recorder cleanly.

It prints the session folder at the end, e.g.

```text
Recording stopped cleanly: /tabletop/bags/session_2026-10-31_16-16-20/bag
Session: .../tabletop/bags/session_2026-10-31_16-16-20
```

- **Where it goes:** `bags/session_<UTC time>/` in the repo (git-ignored).
  `bags/latest` always points at the newest session. Each session also gets
  `session_integrity.json`.
- **Time and size:** about 20 s of startup, then about 5–15 s per trial in
  mock mode. Bags grow at about 4 MB/s (joint states are recorded at
  500 Hz), so 10 trials is about 0.5 GB.
- **To watch while it records,** open RViz as in file 1, step 5.
- **To stop early,** press Ctrl-C. The script still stops the recorder
  cleanly; wait for `Recording stopped cleanly`.

### What a sim bag contains
Joint states, task events (`/tabletop/task_event`), Teensy state (glass,
reward, arm locks), Flic presses, EyeLink sim samples, IO, and FT wrenches.

**Not included:** TCP poses (disabled in mock mode), OptiTrack (so no
monkey hand), or camera images.

---

## Part 2: replay in MuJoCo

All commands run from `virtual_tabletop/`:

```bash
cd ~/Desktop/tabletop/tabletop_current/tabletop/virtual_tabletop
```

### Inspect the bag (optional)
```bash
uv run vtt-bag-info ../bags/latest/bag
uv run vtt-bag-info ../bags/latest/bag --show /tabletop/task_event -n 5
```

### Interactive replay
```bash
uv run vtt-replay ../bags/latest/bag
```
A MuJoCo window opens and plays the session (on macOS it re-launches itself
under `mjpython`, which MuJoCo's viewer requires). Long sessions take a few
seconds to load first: about 8 s for a 30-trial, 1.5 GB bag. The terminal
prints the trial list when loading is done.

| Key | Action |
|---|---|
| space | pause / resume |
| → / ← | jump +2 s / −2 s |
| N / P | next / previous trial |
| `=` / `-` | faster / slower (×2) |
| R | restart |

The top-left text shows the time, speed, current trial and object, the
glass state, and **REWARD** / **PRESS** when they happen.

Mouse: the usual MuJoCo viewer controls. To switch to the `subject` or
`overview` camera, use the Rendering panel on the left (Camera
dropdown). Geom groups 3/4 toggle robot collision meshes and planning
volumes.

### Render a video instead
```bash
uv run vtt-replay ../bags/latest/bag --video replay.mp4 --camera subject
uv run vtt-replay ../bags/latest/bag --video trial0.mp4 --start 25 --end 40
```
- Cameras: `overview` (default) and `subject` (over the monkey's shoulder,
  into the presentation window).
- `--start`/`--end` are seconds from the start of the bag. The trial times
  are printed by `vtt-replay` and visible in the interactive overlay.

### Replay a different session
Point it at any session's `bag` folder (or a single `.mcap` file), e.g.
`../bags/session_2026-10-31_16-12-05/bag`. Bags copied from the rig work
the same way.

---

## What you will see, and current limits

- **Arms:** joint angles straight from the bag.
- **Objects:** inferred from the arm motion. An object is "picked up" when
  the end-effector reaches its mount, and put back when it returns. On the
  test sessions every inferred grasp matched the task's trial objects.
  Objects left at the reset stations aren't handled yet.
- **Smartglass:** opaque or clear following the Teensy state. Its exact
  size and position are an assumption (it isn't in the rig CAD); see
  `virtual_tabletop/config/rig.yaml`.
- **Monkey arms (left and right):** shown but **static**, at the rig
  placement (`placements.rig` in `virtual_tabletop/config/subject.yaml`;
  the RL model uses its own, further forward). Hand replay needs OptiTrack
  data, which sim bags lack.
- **Trial order:** trials appear in presentation order, which isn't always
  trial-number order. Both arms fetch in parallel, so trial 1 can be
  presented before trial 0. That's what the rig really did.

---

## Housekeeping
- **Delete old sessions** with `rm -rf bags/session_<...>` (they're large).
- **After changing rig configs or anything in `virtual_tabletop/config/`,**
  rebuild the model: `cd virtual_tabletop && uv run vtt-build`.
- **Tests:** `cd virtual_tabletop && uv run pytest tests`.
