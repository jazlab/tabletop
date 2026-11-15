# noVNC workspace layout

The real-hardware Compose profile keeps one standard view on each DWM workspace:

| Workspace | View |
| --- | --- |
| 1 | Full-window six-camera preview grid, no 3D rig rendering |
| 2 | Color left-back preview and full rig |
| 3 | Camera exposure-trigger monitor |
| 4 | I/O check dashboard |
| 5 | TTL monitor |
| 6 | Session-health dashboard, including Stop |

`novnc-layout` repairs the assignments once when it starts and watches for a
managed window to be recreated. This prevents concurrent GUI services from
placing two windows on the same workspace. `rviz-cameras` and
`rviz-left-back` keep the camera views alive independently of the optional main
RViz service, so a profile restart repopulates workspaces 1 and 2. The legacy
service name `rviz-cameras` now runs a camera-only viewer rather than RViz.
It reads only the bounded `/cam_preview/*/image_raw` streams and redraws at
10 Hz; full-rate acquisition and recording are unchanged. The unused former
rig-only workspace has been removed and the views renumbered. DWM is built
with exactly six tags and matching Alt+1 through Alt+6 shortcuts; there are no
unused trailing workspace buttons. The session Stop button is now on tab 6.

The main `rviz` service is opt-in (`rviz-manual` profile) instead of starting
with the real/sim stack. Its configuration is retained for manual diagnostics.

## Hidden-view CPU savings

Only visible windows draw. The camera grid and dashboards skip image composition
and redraws on inactive workspaces; selecting the workspace resumes with the
latest data, without replaying old frames. ROS subscriptions, counters, session
history, output timers and the session Stop handler remain active on every tab.
Full-rate camera acquisition, previews and recording are not paused or changed.
Hidden views still use some CPU for receiving/processing data.

For workspace 2, the `tabletop_visualization/VisibilityRenderGuard` RViz display
disables only OGRE render-window targets while hidden. It does **not** stop the
RViz update loop, scene subscriptions or TF processing. An independent 50 ms
timer restores rendering when visible. The optional manual RViz configuration
is not modified.

The layout supervisor publishes window visibility into
`/tabletop/.cache/tabletop/novnc-visibility.json` on changes and once per second.
Missing, invalid or more-than-three-second-old status falls back to normal
rendering, preventing an absent supervisor from freezing the views. Set
`TABLETOP_NOVNC_VISIBILITY_FILE` consistently in the GUI/layout containers if
overriding this shared path. Visibility checks run at 50 ms; dashboard redraw
intervals remain 50–100 ms. This detects DWM workspace selection, not whether a
remote browser has disconnected.

The RViz guard is a small compiled ROS package; build it through the normal
workspace build after updating the code. No driver or motion configuration is
changed by this optimization.

On the rig workstation, a short idle-streaming check with workspace 1 selected
measured the seven GUI/layout services at approximately 4.15 CPU-core equivalents
before gating and 1.66 afterward (about 60% lower). Render-resume notifications
arrived 10–119 ms after workspace selection across repeated switches through all
six views; actual visible paint can take a little longer. These are local
measurements, not guaranteed latency or whole-machine CPU reductions.

To repair or verify the standard views without restarting any robot service:

```bash
cd /path/to/tabletop
docker compose --profile real up -d novnc-layout rviz-cameras rviz-left-back
docker compose ps novnc-layout rviz-cameras rviz-left-back \
  camera-trigger-monitor io-dashboard ttl-dashboard session-dashboard
```

The supervisor only manages X11 windows. It does not initialize Commander or
send commands to either robot.
