# Explore the monkey arm (joint-control design tools)

Two tools for deciding the monkey's joints and ranges: an interactive
slider viewer, and a report of videos, plots and reach numbers. No Docker,
ROS or bags are needed.

Verified on this laptop on 2026-10-31.

All commands run from `virtual_tabletop/`:

```bash
cd ~/Desktop/tabletop/tabletop_current/tabletop/virtual_tabletop
```

---

## 1. The arm explorer (interactive)

```bash
uv run vtt-arm-explorer
```

A MuJoCo window opens (it re-launches itself under `mjpython`) with both
monkey arms and the presented object (the RL scene has no smartglass).

- **Sliders:** the **Control** panel on the right of the window has one
  slider per joint, **in degrees**, spanning that joint's range. If the
  panel is hidden, press Tab (left panel) / Shift-Tab (right panel).
- **The left arm mirrors the right**, so there are 23 sliders (7 arm and
  16 finger joints). For separate sliders per arm:
  `uv run vtt-arm-explorer --independent`.
- **Posing is kinematic:** no gravity or physics; the sliders set the
  angles directly.
- **Top-left text:** each fingertip's position, and its gap to the object
  in cm. Negative means the hand overlaps the object, which counts as a
  touch (`TOUCH`).

| Key | Action |
|---|---|
| P | print the current posture to the terminal (YAML, degrees): paste it to Claude |
| 0 | back to rest (all joints 0°: arm hanging, fingers straight) |
| O | switch the object (small / big) |
| H | hide / show the object |

Options: `--rig` shows the rig frame.

**A posture that touches the small object:** shoulder_abduction 74,
shoulder_flexion 88, shoulder_rotation 30, elbow_flexion 18,
wrist_flexion 27, everything else 0.

---

## 2. The arm report (videos, plot, numbers)

```bash
uv run vtt-arm-report            # about 15 s
open .cache/arm_report/          # Finder
```

Writes to `virtual_tabletop/.cache/arm_report/`:

| File | What |
|---|---|
| `arm_sweep.mp4` | each arm joint through its range (rest → min → max → rest), others at rest; subject view and side view; joint name and angle in the caption |
| `finger_sweep.mp4` | thumb abduction, each finger's flexion (all three segments together), then all fingers; close-up of the right hand |
| `workspace.png` | where the right fingertip can be (100,000 random arm postures, fingers at rest), top and side views, with the object and (for reference) the rig's smartglass plane |
| `report.md` | joint ranges, mass, reach numbers, the best-reach posture, and a **joint ablation** (which joints can be frozen at 0° and still touch) |

Options: `--no-video` (fast, numbers and plot only), `--samples N`,
`--out DIR`.

Everything uses the right arm; the left is its exact mirror image.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| No sliders | Shift-Tab shows the right-hand panel; open its **Control** section. |
| Window frozen | It shouldn't (fixed); close it with the window button or Ctrl-C in the terminal, and tell Claude. |
| Model out of date after editing `config/*.yaml` | `uv run vtt-build` |
