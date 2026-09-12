# noVNC workspace layout

The real-hardware Compose profile keeps one standard view on each DWM workspace:

| Workspace | View |
| --- | --- |
| 1 | Main full-rig RViz view |
| 2 | Six camera previews and full rig |
| 3 | Color left-back preview and full rig |
| 4 | Camera exposure-trigger monitor |
| 5 | I/O check dashboard |
| 6 | TTL monitor |
| 7 | Session-health dashboard |

`novnc-layout` repairs the assignments once when it starts and watches for a
managed window to be recreated. This prevents concurrent GUI services from
placing two windows on the same workspace. `rviz-cameras` and
`rviz-left-back` keep the auxiliary RViz views alive independently of the main
RViz service, so a profile restart repopulates workspaces 2 and 3.

To repair or verify the standard views without restarting any robot service:

```bash
cd /path/to/tabletop
docker compose --profile real up -d novnc-layout rviz-cameras rviz-left-back
docker compose ps novnc-layout rviz rviz-cameras rviz-left-back \
  camera-trigger-monitor io-dashboard ttl-dashboard session-dashboard
```

The supervisor only manages X11 windows. It does not initialize Commander or
send commands to either robot.
