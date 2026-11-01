# Run the foraging task in simulation and watch it in RViz

On this MacBook, the whole TableTop ROS stack runs in Docker with mock
hardware: mock UR arms, simulated Teensy, Flic button and EyeLink. The real
Commander and foraging task code drive it, and you watch in RViz through a
browser (noVNC).

Verified on this laptop on 2026-10-31.

---

## Every time

### 1. Start Docker Desktop
Open **Docker Desktop** and wait until it says it is running.

### 2. Open a bash shell in the repo
The `tt-*` commands need bash (your terminal's default is zsh):

```bash
cd ~/Desktop/tabletop/tabletop_current/tabletop
bash                      # Homebrew bash 5; type `exit` to return to zsh
source setup.bash         # puts the tt-* commands on PATH
```

### 3. Start the simulated rig
```bash
tt-compose --profile=sim up -d
tt-compose ps             # ~11 services, all "Up", none "Restarting"
```
(The `ifconfig: interface scope does not exist` lines are harmless, a macOS
quirk in the noVNC certificate script.)

### 4. Start the browser relay (once per Docker Desktop session)
Docker Desktop isn't forwarding the rig's ports to the Mac, so a tiny relay
container does it, on `localhost` only:

```bash
docker start vtt-novnc-proxy 2>/dev/null || docker run -d --name vtt-novnc-proxy \
  --restart unless-stopped -p 127.0.0.1:18080:8080 -p 127.0.0.1:18765:8765 \
  --entrypoint sh alpine/socat -c \
  'socat TCP-LISTEN:8080,fork,reuseaddr TCP:172.17.0.1:8080 & exec socat TCP-LISTEN:8765,fork,reuseaddr TCP:172.17.0.1:8765'
```
It restarts by itself with Docker Desktop, so normally you only run this once.

### 5. Open RViz in your browser
**<http://localhost:18080/vnc.html?autoconnect=true>**

- Note **http**, not https, and port **18080**. The repo docs mention https
  and port 8080 with a password; the noVNC image on Docker Hub predates
  that change and serves plain HTTP.
- If it does ask for a password: `tt-novnc-security show`.
- The desktop has numbered workspaces along the top bar. RViz is on **1**;
  the IO, TTL and session dashboards are on 5, 6 and 7.

You should see the rig, the object grid and both arms standing still.

### 6. Run the foraging task
```bash
tt-compose run --rm commander tt-launch tasks task:=foraging_ordered robot_mode:=mock
```
After about 10–20 s of startup the arms start fetching, presenting and
returning objects in RViz, alternating left and right.

- **This runs until you stop it** (`num_trials: 100000`). Press **Ctrl-C
  once** and let it shut down.
- For a fixed number of trials (and a recording), use the script in
  `2_record_n_trials_and_replay_in_mujoco.md`.
- In mock mode the trials are quick: the shipped config has a 0.5 s
  response window and the simulated button presses after 1–3 s, so trials
  time out with no reward. That's expected.

Other tasks work the same way (`task:=present_ordered`,
`task:=smooth_pursuit_spiral`, …; see `src/ros/tabletop/tabletop_tasks/config/`).

### 7. Shut down when done
```bash
tt-compose --profile=sim down
```
Quit Docker Desktop to free its memory (about 16 GB reserved).

---

## One-time setup (already done on this laptop)

Only needed again after a fresh clone or a clean rebuild.

1. **Docker Desktop:** Rosetta emulation on, about 16 GB+ memory.
2. **Build the workspace:** `tt-compose --profile='*' pull`, then
   `tt-build colcon --all -w 3` (about 25 min).
   - **Gotcha:** `moveit_py` fails to link under emulation (a GCC LTO
     crash). Build it once with LTO off:
     ```bash
     docker compose run --rm builder bash -c "source /tabletop/setup.bash && cd /tabletop && \
       colcon --log-base log/colcon build --base-paths src/ros --packages-select moveit_py \
       --cmake-args -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF \
       --mixin build-testing-off ccache mold compile-commands release --symlink-install"
     ```
     Then re-run `tt-build colcon --all -w 3`. Repeat this after any
     `--clean-cmake`/`--clean-all`.
3. **Manipulation preflight** (the task refuses to start without it):
   `tt-preflight run`.
   - Mock planning only; it takes about 30 min here.
   - It can segfault under emulation; just re-run it, since it resumes
     where it stopped.
   - Result on this laptop: 24/30 objects PASS. The 6 UNAVAILABLE objects
     are left out of tasks automatically.
   - Re-run only after mounts or manipulation settings change.
     `tt-preflight results` shows the table again.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `service "rviz-cameras" depends on undefined service "camera-preview"` | The `compose.override.yaml` fix (`profiles: !override [real]`) is missing. It's an uncommitted change on `watters_mujoco`. |
| Task dies with `No compatible manipulation preflight report` | Run `tt-preflight run` (see one-time setup). |
| Browser can't connect | Docker Desktop running? `docker ps` shows `tabletop-novnc` and `vtt-novnc-proxy`? Use `http://` and port `18080`. |
| `tt-*: command not found` / `declare: -n: invalid option` | You're in zsh or old bash: run `bash`, then `source setup.bash`. |
| PulseAudio / fluidsynth errors in the task log | Harmless: no audio passthrough on the Mac. |
| `No 3D sensor plugin(s) defined for octomap` | Harmless MoveIt message. |
