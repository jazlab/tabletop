# Pupil-corrected 3D gaze calibration

**Preferred collection task:** `smooth_pursuit_sin_static_forward_five_gentle_turns`. The observer looks at the LED below the right-arm marker, follows a continuous five-inch-closer sinusoid with eased turns for approximately 159 seconds, then fixates 27 stationary positions. See [task instructions and physical clearance limitations](tasks.md). This is a calibration for the current observer, head position, rig geometry and EyeLink setup, not a pretrained universal gaze estimator.

## Offline model

Fit on valid moving samples only. The robust calibration first estimates how measured pupil centers shift with pupil size while accounting for known LED position; at prediction time it removes **only the pupil-size contribution**. Four small 4–16–32–16–3 networks with different seeds average their continuous 3D predictions. A prediction needs left and right pupil XY and both pupil sizes, in their original raw units. It needs no target identity, robot position, or time. The output of `predict_m` is XYZ in the rig's `world` frame, **meters**.

Export `/rosout`, `/eyelink/sample_array`, `/markers`, and `/right_tcp_pose_broadcaster/pose` to CSV. For the preferred task, create the aligned session and then train:

```bash
mkdir -p saved_weights
python -m tabletop_py.gaze.pursuit_alignment /path/to/export aligned.npz
python -m tabletop_py.gaze.pupil_corrected aligned.npz saved_weights/pupil-corrected.npz
```

The input `x` has six columns (left X, left Y, right X, right Y, left pupil size, right pupil size); `y` is LED position in **world millimeters**, not raw OptiTrack marker position. The 1 ms time vector `g` is in seconds relative to moving onset; `duration` is the lap's length. `valid` flags usable eye/LED rows, `bad` excludes rows for movement training only, and `ids > 0` marks the settled scoring windows of the static holds. Keep the marker-to-LED offset and coordinate transforms tied to the physical setup. Do not use an assumed fixed marker slot if there are reflections or additional markers.

Use held-out static rows to score Euclidean 3D RMSE and median in cm; average per-session scores equally. On three exploratory same-observer runs this model achieved 3.42 cm mean static RMSE and 2.03 cm mean per-session median error. These figures are not independent validation on new observers. Do not train on the stationary test points. Keep weights and all participant data on approved private storage; there is deliberately no session checkpoint in Git.

## Preview in noVNC workspace 2

Use the separate **idle-rig** preview, not the older four-input `/predicted_markers` model. Rebuild/install the ROS package normally if necessary. With the workstation's standard Docker setup and the model file generated above, reload **only** tab 2 to pick up its Marker display. From the host:

```bash
docker restart tabletop-rviz-left-back
docker exec -it -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 tabletop-eyelink bash -lc 'source /opt/ros/jazzy/setup.bash && source /tabletop/install/setup.bash && exec /tabletop/.venv.container/bin/python -m tabletop_rig.nodes.gaze_preview --model /tabletop/saved_weights/pupil-corrected.npz --seconds 120'
```

Open workspace 2 and look between known targets. The orange sphere on `/gaze_preview/point` is the latest valid predicted point, not a measured fixation. The preview uses a separate EyeLink scratch capture; it does not command robots. Stop with Ctrl+C before starting any experiment. Missing binocular data should make the point disappear, not freeze at an old location. Never use a raw sample-topic subscription as an idle diagnostic: the EyeLink node starts recording when a sample subscriber connects.
