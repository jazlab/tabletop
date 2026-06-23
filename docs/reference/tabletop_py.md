# tabletop_py

ROS-independent Python utilities: eye-gaze estimation/tracking, the Flic
button client, and shared helpers. This package never imports ROS — it is
wrapped by `tabletop_rig` nodes and also exposes the `tt-gaze-*` / `tt-flic-*`
command-line tools.

## Gaze estimation — `tabletop_py.gaze`

The offline gaze calibration/estimation pipeline (preprocess → train →
predict → visualize) plus EDF parsing and synchronization helpers.

::: tabletop_py.gaze.calibrate
::: tabletop_py.gaze.preprocess
::: tabletop_py.gaze.train
::: tabletop_py.gaze.predict
::: tabletop_py.gaze.visualize
::: tabletop_py.gaze.models
::: tabletop_py.gaze.edf
::: tabletop_py.gaze.utils
::: tabletop_py.gaze.syncer
::: tabletop_py.gaze.sync_aligner
::: tabletop_py.gaze.optitrack_to_sync
::: tabletop_py.gaze.calibrate_camera
::: tabletop_py.gaze.roi_bound_finder

## Flic buttons — `tabletop_py.flic`

Client for the Flic Bluetooth buttons used as the subject's response device.
`scapy_client` is a raw BLE sniffer that reports button presses with minimal
latency. (The older `flicd`-daemon client and the `piano` demo were retired —
see `deprecated/flic-button/`.)

::: tabletop_py.flic.scapy_client

## Shared utilities — `tabletop_py.utils`

::: tabletop_py.utils.common
::: tabletop_py.utils.mesh
::: tabletop_py.utils.dbm_sqlite3
