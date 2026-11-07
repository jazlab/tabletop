# NWB conversion plan

Status: **plan only — nothing implemented.**
Scope: convert one recorded TableTop session (rosbag + EyeLink EDF + camera
frames + SpikeGLX/Neuropixels) into four NWB files that share one timeline:

| File | Contents |
| --- | --- |
| `task.nwb` | everything that is not raw ephys or raw video: trials, every task/trial event, rewards and other I/O, eye tracking, robot state, OptiTrack, Teensy sensors, sync data, configuration |
| `video.nwb` | the six FLIR camera streams (external video files + per-frame timestamps) and per-frame camera metadata |
| `ephys.nwb` | full raw Neuropixels 2.0 data (one wideband stream per probe), probe geometry and site map, the SpikeGLX sync channel, optional derived LFP |
| `spike_sorting.nwb` | sorted units (spike times on the shared timeline), waveforms, quality metrics; produced later from `ephys.nwb` |

## Decision log

| # | Decision | Status |
| --- | --- | --- |
| D1 | Ephys system | **Decided:** SpikeGLX + Neuropixels. Teensy pin 31 will be wired to the sync input on the acquisition chassis. |
| D2 | Master clock | **Tentatively decided:** host ROS time, which is the clock the Teensy uses to timestamp its sync pulses. Revisit if R7 shows the host clock being slewed during sessions (§2.4). |
| D4 | Video encoding / archiving | **Open:** you will test video sizes first. |
| D5 | Raw ephys in `ephys.nwb` | **Decided:** full raw. Probes are **Neuropixels 2.0**, so this is one wideband 30 kHz stream per probe, and LFP is contained in it (§4.3). |
| D6 | Trial definition | **Decided:** a trial starts when its object reaches the presentation pose (`TRIAL_PRESENTED`). Every trial event is recorded (§4.1). |
| — | Post-conversion test suite | **Planned** (§10) |

Where to find things:

- **§8: Questions for you.** Open decisions with my recommendations, plus facts only you can supply.
- **§9: Uncertainties.** Things I couldn't verify from the code, each with how and when it gets resolved.
- **§10: Post-conversion test suite.**

---

## 1. What a session contains today

`rosbag.launch.py` creates `bags/session_<YYYY-MM-DD_HH-MM-SS>/` and points
`bags/latest` at it. After a task run, the directory holds:

| Path | Produced by | Contents |
| --- | --- | --- |
| `bag/*.mcap` + `metadata.yaml` | `diagnostic_rosbag_recorder` (topics from `tabletop_rig/config/rosbag.yaml`) | all ROS topics below; split at 100 GB |
| `eyelink/` (MCAP) | EyeLink node's own `rosbag2_py` writer | `~/sample` (`tabletop_interfaces/Eyelink`) |
| `eyelink_received/last.edf` (+ CSV) | EyeLink node at stop | **authoritative** tracker recording |
| `eyelink_integrity.json` | EyeLink node | retrieval stats |
| `flir_preflight.yaml` | recorder `--flir-preflight` | camera readiness + one `CameraInfo` per camera |
| `session_integrity.json` | `utils/session_integrity.py` | per-stream PASS/WARN/ERROR summary |

Recorded topics that need to be converted:

| Topic | Type | Rate | Timestamp source |
| --- | --- | --- | --- |
| `/teensy/sensor` | `TeensySensor` | 100 Hz | Teensy, micro-ROS-synchronized ROS epoch time; includes `sync_pulse_state`, `sync_pulse_last_time_on/off`, `safety_laser_last_time_broken` |
| `/teensy/log` | `String` | low | bag receive time |
| `/teensy/set_{reward,smartglass,arm_lock,buzzer,solenoid}/_service_event` | service events | event | host (request + response) |
| `/tabletop/task_event` | `TaskEvent` | event | Commander `now()` |
| `/flic/button_pressed_time` | `Header` | event | first BLE advertisement (host) |
| `/eyelink/sample_array` | `EyelinkBatch` | ~1 kHz | `header` = host dequeue time; `eyelink_time_ms` = tracker clock; `input` = DB25 port incl. sync TTL |
| `/predicted_markers` | gaze model output | | host |
| `/joint_states`, `/dynamic_joint_states` | | ~500 Hz | ros2_control (host) |
| `/{left,right}_tcp_pose_broadcaster/pose` | `PoseStamped` | ~500 Hz | host (**0 messages in the 2026-10-31 bags**) |
| `/{left,right}_force_torque_sensor_broadcaster/wrench` | `WrenchStamped` | ~500 Hz | host |
| `/left_io_and_status_controller/io_states` | `ur_msgs/IOStates` | ~425 Hz | no header, so only the bag receive time (U5 in §9) |
| `/markers`, `/rigid_bodies` | mocap4r2 | Motive rate | host `now()` minus the latency NatNet reports |
| `/cam_sync/<cam>/image_raw/compressed` ×6 | JPEG q80 of BayerRG8 1920×1200 | 120 Hz | synchronized-driver estimate (host) |
| `/cam_sync/<cam>/meta` ×6 | `ImageMetaData` | 120 Hz | + `camera_time`, `camera_frame_id`, `line_status`, exposure, gain |
| `/cam_sync/<cam>/*_unmatched` | | rare | frames the synchronizer could not group |
| `/rosout` | `Log` | | |

**Task events published today** (`TaskEvent`, each with `trial_number`, `group_name`, `object_id`):

| Event | Where | When |
| --- | --- | --- |
| `TASK_STARTED` / `TASK_ENDED` | `run.py` | around the whole task list; the sync pulses are enabled and disabled at the same points |
| `TRIAL_PRESENTED` | `tasks/base.py` | after `present_object` **and** the final `plan_and_move` to `trial_spec.object_pose`, i.e. when the object has reached its presentation pose |
| `TRIAL_DEPRESENTING` | `tasks/base.py` | after `run_trial` returns and the glass is occluded and the arms locked, before `unpresent_object` |
| `OBJECT_MISS` | `tasks/base.py` | a trial failed planning twice and never reached presentation |
| `STIMULUS_REVEALED`, `MOTION_STARTED`, `GO_CUE` | `invariant_affordance.py` only | phase boundaries within `run_trial` |

**Not recorded anywhere today, but needed for a complete `task.nwb`** (the rig changes are in §7):

- The task config YAML: it is only printed to stdout.
- The full `TrialSpec` (object pose, response arm, occlude, orientation stratum, …) and `TrialFeedback` (reaction time, timeout); only `trial_number`/`group_name`/`object_id` reach the bag.
- **The Foraging/Present phase boundaries** (stimulus, delay, response window, reveal). `ForagingTask.run_trial` emits no `TaskEvent`s. Some of them can be reconstructed from smartglass and reward edges, but not reliably.
- **Audio cues**: `play_sound` (fluidsynth) leaves no trace in the bag.
- Manipulation lifecycle steps (fetch start, present start, unpresent done, reset, planning retry).
- Subject/session metadata, code version, container image versions.

Existing code to reuse:

- `virtual_tabletop/src/virtual_tabletop/bags/reader.py`: a ROS-free MCAP reader (`mcap` + `mcap-ros2-support`).
- `tabletop_unbag --handlers video`: JPEG Bayer → demosaiced H.264 segments plus per-frame timestamp sidecars.
- `tabletop_py/gaze/edf.py`: EDF → CSV with the `-input` port column.
- `tabletop_py/gaze/sync_aligner.py`: a prototype of inter-pulse-interval matching. Reimplement it vectorized.
- `nodes/ttl_dashboard.py`: per-stream TTL decoders to port to offline use.

---

## 2. Synchronization

### 2.1 The sync signal

| Teensy pin | Signal | Details |
| --- | --- | --- |
| **31** | experiment sync pulse | 100 ms HIGH. A base timer fires every 1000 ms and each onset is delayed by a uniform random 50–200 ms, so inter-pulse intervals are random in ~850–1150 ms. Enabled only between `TASK_STARTED` and `TASK_ENDED`. |
| **33** | FLIR exposure trigger | 120 Hz square wave from a hardware PIT timer, running from boot |

Because the inter-pulse intervals are random, ~10–20 consecutive pulses
identify a unique position in the train. Streams can therefore be aligned
despite different start times or dropped pulses, with no barcode needed.

### 2.2 Which streams receive it

| Stream | Where the pulse appears | Native clock | Edge resolution |
| --- | --- | --- | --- |
| Teensy (pulse source) | `/teensy/sensor.sync_pulse_last_time_on/off` | **host ROS time** (via micro-ROS time sync, re-synced every 10 s) | software timestamp right after the pin write |
| **Neuropixels (SpikeGLX)** | **to be wired (D1):** pin 31 → chassis sync input → recorded on every probe's SY channel (and the NI-DAQ stream, if used) | each probe's own sample clock | 1 sample (33 µs at 30 kHz) |
| EyeLink | DB25 status pin 12 → `input` (127 idle / 95 asserted, mask `0x20`) | tracker clock | 1 ms |
| FLIR | Line3 → bit 3 of `line_status`, **only on `left_back_top_cam`**. All cameras share the pin-33 trigger, so one camera suffices. | frame index | 1 frame (8.3 ms) |
| UR robot 1 (left) | DI3, active-low, in `io_states` | bag receive time | DDS jitter (several ms); QC only |
| OptiTrack | solenoid moves a marker 3.44 mm | host (driver estimate) | 1 mocap frame |

### 2.3 Alignment method

For each stream S with native clock `t_S`:

1. **Detect** pulse onsets (and offsets) in S's native clock.
2. **Match** S's onset train to the reference train by cross-correlating the inter-pulse-interval sequences. Then require N consecutive interval matches within a tolerance (~1 ms for hardware streams, ~5–10 ms for software streams).
3. **Fit** `t_ref = a·t_S + b` with a robust regression on the matched pairs. If the residuals show steps (clock resync, NTP slew, restart), use a piecewise-linear fit.
4. **QC**: residual RMS and max, matched/unmatched counts, pulse coverage of the recording, drift `a−1` in ppm, and a residual plot. Fail if RMS exceeds the threshold for that stream class.
5. **Apply** the map to every timestamp from S before writing.

Stream specifics:

- **Neuropixels**: each probe is a separate stream with its own clock, so each gets its own fit. The NI-DAQ stream (if used) is fitted the same way. SpikeGLX's nominal `imSampRate` is only a starting value; the fit determines the true rate.
- **Video**: map *frame index* → reference time. The frame index comes from `camera_frame_id`, which also reveals dropped frames; the fit uses the Line3 onset frames. The ±4 ms quantization averages out over thousands of pulses.
- **EyeLink**: the EDF (tracker clock + `input`) is the primary source; the ROS samples are a fallback.
- **ROS-native streams** (task events, robot, Teensy sensors, Flic, OptiTrack) are already in host ROS time, so they need no map under D2.

### 2.4 Master clock (D2): what it means

The master clock is **not** a clock that gets transmitted to the other devices.
It is only the timebase chosen, during offline conversion, to express every
timestamp in the NWB files.

The physical setup is the same regardless: the Teensy *emits* pulses, and every
device *records* them with its own clock. That gives, for each device, a list
of the same pulses as seen by that device. From two such lists you can fit
`t_A = f(t_B)` and, equally, its inverse `t_B = f⁻¹(t_A)`. The mapping works in
both directions, so any stream that recorded the pulses can serve as master.
The "ephys master" option meant converting the Teensy, EyeLink, and video
times *into* ephys time offline. Nothing needs to be sent to the ephys system
beyond the pulses themselves.

Note that the Teensy has no independent clock in the data. It timestamps its
pulses in **host ROS time**, kept in step by micro-ROS. "Teensy master"
therefore means "host ROS time master", and that is what D2 now uses.

Trade-off, for the record:

| | Host ROS time master (chosen) | Ephys master |
| --- | --- | --- |
| Behavior-only sessions | same convention | need a different convention |
| Task/robot/event data | timestamps unchanged | mapped |
| Raw ephys | `starting_time = b`, `rate = fs/a`. This is exact for a linear map. | native `starting_time`/`rate` |
| If the host clock is slewed by NTP mid-session | the ephys map becomes piecewise; `rate` is then a linear approximation, and the exact piecewise map is stored in `processing/sync` and used for spike times | unaffected |
| Precision | identical: the pulses determine the map in both cases | |

**t = 0 / `session_start_time` (D3, proposed default; §8):** the earliest
first sample of any stream after mapping. This is usually the start of the
SpikeGLX recording, since ephys starts before the task (R8). As a result, no
NWB time is negative.

### 2.5 How the four files end up synchronized

Every file gets the **identical** `session_start_time`, `timestamps_reference_time`,
`session_id` and `subject`, and all of its timestamps are expressed in host
ROS time relative to that start. A time `t` therefore means the same instant
in all four files.

Each file also stores, in `processing/sync`, its streams' raw pulse times in
their native clocks plus the fitted parameters, so the alignment can be audited
or redone. A `sync_report.json` and QC plots are written next to the files.

---

## 3. Pipeline overview

```text
session dir + SpikeGLX run folder + session_metadata.yaml
        │
[0] inventory & validate → inventory.json (fail early on fatal gaps)
[1] extract   MCAP → parquet cache; EDF → samples/events; unbag → video segments;
              SpikeGLX meta + SY-channel edges (spikeinterface / CatGT-style)
[2] sync      detect → match → fit → QC → sync_report.json + plots
[3] write     task.nwb | video.nwb | ephys.nwb (neuroconv)  — independent
[4] validate  post-conversion test suite (§10) on task/video/ephys.nwb
              → nwb/validation/report.{json,md}; non-zero exit on any FAIL
[5] sort      SpikeInterface → Kilosort4 → curation → metrics → spike_sorting.nwb
[6] validate  test suite again, now including the spike_sorting.nwb checks
```

Principles: stages are idempotent and resumable; the MCAP is read once; no ROS
is needed except for `unbag`; metadata is declarative (YAML); raw sources are
never modified.

---

## 4. File structure

Notation: `├──` = NWB group/object; *(Type)* = NWB neurodata type; `[cols]`
= table columns; `(N,k)` = dataset shape. Times are seconds from
`session_start_time`, in host ROS time.

### 4.0 Fields shared by all four files

```text
<file>.nwb
├── identifier              unique UUID per file
├── session_id              e.g. "session_2026-10-31_20-16-48" (same in all 4)
├── session_start_time      same in all 4 (§2.4)
├── timestamps_reference_time = session_start_time
├── session_description     one line, e.g. "InvariantAffordance, task data"
├── general/
│   ├── subject (Subject)   subject_id, species, sex, date_of_birth|age, weight, description
│   ├── experimenter, lab, institution, protocol, notes, keywords
│   ├── source_script       pipeline version + rig git commit
│   └── devices/            only the devices relevant to that file
└── processing/sync/        that file's pulse trains + fit table (see each file)
```

### 4.1 `task.nwb`

```text
task.nwb
├── general/devices/
│   teensy, ur5e_left, ur5e_right, eyelink, optitrack, flic_button,
│   reward_solenoid, smartglass, speaker
│
├── intervals/
│   ├── epochs (TimeIntervals)
│   │     [start_time, stop_time, tags, task_class, task_index, task_config_name]
│   │     one row per task class run (TASK_STARTED→TASK_ENDED; per-class
│   │     boundaries need R2c)
│   ├── trials (TimeIntervals)   — one row per trial that reached presentation
│   │     (column list below)
│   └── aborted_trials (TimeIntervals or table)
│         [time, trial_number, group_name, object_id, reason]   (OBJECT_MISS, etc.)
│
├── acquisition/
│   ├── EyeTracking (EyeTracking)
│   │   ├── left_eye_raw   (SpatialSeries, (N,2) EyeLink camera units, timestamps)
│   │   └── right_eye_raw  (SpatialSeries, (N,2))
│   ├── PupilTracking (PupilTracking)
│   │   ├── left_pupil     (TimeSeries, (N,) arbitrary units)
│   │   └── right_pupil
│   ├── eyelink_parser_events (TimeIntervals)
│   │     [start_time, stop_time, eye, kind(fixation|saccade|blink),
│   │      start_x, start_y, end_x, end_y, amplitude, peak_velocity, mean_pupil]
│   ├── robot_left_joint_position  (TimeSeries, (N,6) rad; joint names in description/comments)
│   ├── robot_left_joint_velocity  (TimeSeries, (N,6) rad/s)
│   ├── robot_left_joint_effort    (TimeSeries, (N,6) units as reported by ur_robot_driver)
│   ├── robot_left_tcp_position    (SpatialSeries, (N,3) m, reference_frame=base)
│   ├── robot_left_tcp_orientation (TimeSeries, (N,4) quaternion wxyz)
│   ├── robot_left_wrench          (TimeSeries, (N,6) [Fx Fy Fz N, Tx Ty Tz N·m])
│   ├── robot_right_* (same six)
│   ├── robot_left_io              (TimeSeries, (N,18) digital inputs; QC/provenance)
│   ├── teensy_hand_sensors        (TimeSeries, (N,2) bool [left, right], 100 Hz)
│   ├── teensy_glove_left          (TimeSeries, (N,5) 10-bit ADC)
│   ├── teensy_glove_right         (TimeSeries, (N,5))
│   ├── teensy_outputs             (TimeSeries, (N,4) bool [reward, smartglass, sync, solenoid])
│   ├── teensy_safety_laser        (TimeSeries, (N,) bool)
│   ├── optitrack_rigid_body_<name>_position    (SpatialSeries, (N,3) m)
│   ├── optitrack_rigid_body_<name>_orientation (TimeSeries, (N,4))
│   └── optitrack_markers (DynamicTable, ragged: [timestamp, frame_number, marker_index[], x[], y[], z[]];
│                          sync-solenoid marker tagged/excluded — D10)
│
├── events  (ndx-events EventsTables; D9)   — every discrete event, one table per kind,
│   │                                          each row tagged with trial_number (−1 between trials)
│   ├── task_events      [timestamp, event_type(name), trial_number, group_name, object_id]
│   │                     ALL TaskEvent messages, verbatim
│   ├── reward           [timestamp(on), duration, trial_number, source(service|sensor)]
│   ├── smartglass       [timestamp, state(revealed|occluded), trial_number]
│   ├── arm_lock         [timestamp, arm, state(locked|released), trial_number]
│   ├── hand_sensor      [timestamp, hand, state(held|released), trial_number]
│   ├── buzzer           [timestamp, side, duration, trial_number]
│   ├── solenoid         [timestamp, state, trial_number]
│   ├── safety_laser     [timestamp(break, interrupt-latched), restored_time, trial_number]
│   ├── flic_press       [timestamp, button_address, trial_number]
│   └── sound            [timestamp, sound_name, duration, trial_number]     (needs R2b)
│
├── processing/
│   ├── behavior/
│   │   ├── gaze_rig_coordinates (SpatialSeries, (N,3) m — from /predicted_markers or recalibration)
│   │   └── fk_tcp_pose_{left,right} (if the TCP topics are empty)
│   ├── sync/
│   │   ├── teensy_sync_pulses   (TimeIntervals [start_time=onset, stop_time=offset])
│   │   ├── eyelink_sync_pulses_native, ur_left_sync_pulses_native, optitrack_sync_pulses_native
│   │   └── sync_fits (DynamicTable)
│   │         [stream, native_clock, model(linear|piecewise), slope, intercept,
│   │          breakpoints, n_pulses_matched, n_unmatched, residual_rms_ms,
│   │          residual_max_ms, drift_ppm, coverage_start, coverage_stop]
│   │         — rows for ALL streams in the session (ephys, video too)
│   ├── configuration/
│   │   └── files (DynamicTable [name, format, content])  task YAML, commander.yaml,
│   │         rosbag.yaml, flir_synchronized.yaml, UR calibrations, URDF,
│   │         session_integrity.json, eyelink_integrity.json, launch args
│   └── logs/
│       ├── rosout     (DynamicTable [timestamp, node, level, message])
│       └── teensy_log (DynamicTable [timestamp, message])
```

**`trials` columns** (D6: trial start = object reaches its presentation pose):

| Column | Source | Available today? |
| --- | --- | --- |
| `start_time` | `TRIAL_PRESENTED` | yes |
| `stop_time` | `TRIAL_DEPRESENTING` (the stimulus is over and the arms are locked; D6b) | yes |
| `trial_number`, `task_class`, `group_name`, `object_id` | `TaskEvent`, epoch | yes |
| `retried` | a second planning attempt before presentation | no (R2c) |
| `response_arm`, `occlude`, `object_position` (3), `object_orientation` (4), task-specific spec fields (e.g. tilt stratum) | `TrialSpec` | **no (R2a)** |
| `stimulus_revealed_time`, `motion_started_time`, `go_cue_time` | InvariantAffordance `TaskEvent`s | yes (NaN for other tasks) |
| `stimulus_start_time`, `stimulus_stop_time`, `delay_start_time`, `response_start_time`, `response_stop_time`, `reveal_start_time` | Foraging/Present phases | **no (R2b)**; partially reconstructible from smartglass edges |
| `smartglass_reveal_times`, `smartglass_occlude_times` (ragged) | Teensy edges in [start, stop] | yes |
| `reward_times`, `reward_durations` (ragged) | reward events | yes |
| `hand_release_times_left/right`, `hand_hold_times_left/right` (ragged) | Teensy hand-sensor edges | yes |
| `flic_press_times` (ragged) | Flic | yes |
| `buzzer_times`, `sound_times` (ragged) | service events / R2b | buzzer yes, sound no |
| `reaction_time`, `timeout`, `outcome` | `TrialFeedback` | **no (R2a)** |

How events are assigned to trials: `TaskEvent`s carry `trial_number`, so they
are assigned directly. Untagged events (Teensy edges, rewards, Flic, buzzer)
are assigned by time window `[start_time, stop_time]`. This is unambiguous
because `_trial_lock` lets only one trial be presented at a time. The next
trial's preparation (fetch) runs on the other arm *during* the current trial,
so preparation events must never be assigned by time. Events outside every
trial window stay in the `events` tables with `trial_number = −1`. Every event
is therefore always in the file, whether or not it belongs to a trial.

Time resolution notes: Teensy state edges (smartglass, reward, hand sensors)
come from the 100 Hz sensor message, so they have 10 ms resolution. Safety-laser
breaks are latched by interrupt and are exact. Service events give the host
time of the *command*, which may differ from the physical change.

### 4.2 `video.nwb`

```text
video.nwb
├── general/devices/
│   left_front_top_cam … right_bottom_cam (Device: model Blackfly S, serial, lens)
├── acquisition/
│   ├── left_front_top_cam (ImageSeries)
│   │     format = "external"
│   │     external_file = ["video/left_front_top_cam/000.mp4", "…/001.mp4", …]  (relative)
│   │     starting_frame = [0, 36000, 72000, …]       (first frame index of each segment)
│   │     timestamps (n_frames,)                       per-frame, host ROS time, from §2.3 fit
│   │     dimension = [1920, 1200], device → camera, description (pixel format, demosaic,
│   │     JPEG q80 source, encoder + quality, trigger settings)
│   └── … one per camera (6)
└── processing/
    ├── camera_metadata/
    │   └── <camera> (DynamicTable, one row per frame)
    │         [timestamp, frame_index, camera_frame_id, camera_time, exposure_time_us,
    │          gain_db, brightness, line_status, synchronized_timestamp_valid, header_stamp]
    ├── dropped_frames (TimeIntervals) [start_time, stop_time, camera, n_missing]
    ├── unmatched_frames (DynamicTable) [timestamp, camera, camera_frame_id]   (D4b)
    ├── camera_calibration (DynamicTable)
    │     [camera, width, height, K(9), D, R(9), P(12), distortion_model,
    │      extrinsic_position(3), extrinsic_orientation(4), reference_frame]
    └── sync/
        ├── line3_onset_frames (left_back_top_cam frame indices + native times)
        └── sync_fits (the video row(s))
```

Video files sit next to the NWB file at relative paths. Raw frames are not
stored in NWB (~1.6 GB/s raw). Encoding and archiving are decision D4.

### 4.3 `ephys.nwb` (SpikeGLX + Neuropixels, written with neuroconv)

**D5: does "full raw" include LFP?** The probes are **Neuropixels 2.0**. NP2.0
records one **wideband** stream per probe (30 kHz, no on-probe 300 Hz
high-pass). SpikeGLX writes only `*.imec<N>.ap.bin`; there is no `.lf.bin`.
The raw stream therefore already **contains** the LFP band, and "full raw" is
a single `ElectricalSeries` per probe.

A separate LFP series would be **derived** data: low-pass ~300 Hz and downsample
to ~2.5 kHz, stored in `processing/ecephys/LFP`. It is optional (D5c). It adds
~8% to file size and saves every LFP analysis from re-filtering hours of 30 kHz
data.

Other NP2.0 specifics:

- **Site map**: each probe records 384 channels chosen from 1280 sites (single-shank) or 5120 sites (4-shank). Which sites were recorded is defined by the IMRO table in the SpikeGLX `.meta` file. It must go into the electrodes table (`shank_id`, `contact_id`, `rel_x`, `rel_y`), and it can differ between sessions.
- **4-shank probes**: one `ElectrodeGroup` per shank.
- **Spike sorting** must high-pass the wideband data first; this is already part of the §4.4 preprocessing.

Approximate size per probe, uncompressed: ≈ 23 MB/s ≈ **83 GB/h** (+ ~7 GB/h
if a derived LFP is stored). HDF5 compression gains are modest (on the order of
1.5–2×).

```text
ephys.nwb
├── general/
│   ├── devices/
│   │   NeuropixelsImec0 (Device: probe type, serial, headstage serial, PXIe slot/port)
│   │   [NeuropixelsImec1 …], [NIDQ (if used)]
│   └── extracellular_ephys/
│       ├── electrode groups: one per probe (per shank for multi-shank probes)
│       │     [name, description, location, device, (position)]
│       └── electrodes (DynamicTable, one row per recorded channel)
│             [id, location (brain area), group, group_name, probe, shank_id,
│              contact_id, channel_name ("AP0"…), rel_x, rel_y (µm, probe geometry),
│              inter_sample_shift, gain_to_uV, offset_to_uV, imp/bad_channel flag,
│              x, y, z (stereotaxic/atlas if known)]
├── acquisition/
│   ├── ElectricalSeriesImec0 (ElectricalSeries)  — wideband raw, NP2.0
│   │     data (n_samples, 384) int16 — chunked ~1 s × 64 ch, compressed (D5b)
│   │     conversion/offset → volts, electrodes → region of table,
│   │     starting_time = b, rate = fs/a  (from the sync fit; §2.4)
│   ├── [ElectricalSeriesImec1 … one per additional probe]
│   └── [TimeSeries for NIDQ analog/digital channels, if used]
└── processing/
    ├── sync/
    │   ├── imec0_sync_pulses (TimeIntervals: onset/offset; native seconds + sample index)
    │   ├── [one per probe / NIDQ]
    │   └── sync_fits (per-stream rows; includes piecewise map if needed)
    ├── ecephys/LFP/  (optional, derived — D5c)
    │   └── LFPImec0 (ElectricalSeries, (n_samples, 384) @ ~2.5 kHz, low-pass ~300 Hz;
    │                 filter parameters in description)
    └── ecephys/imro (DynamicTable: the raw IMRO entries per probe, for provenance)
```

The SY (sync) channel is not part of the `ElectricalSeries`; its edges go to
`processing/sync`.

### 4.4 `spike_sorting.nwb`

```text
spike_sorting.nwb
├── general/
│   ├── devices/, extracellular_ephys/ (copy of ephys.nwb's groups + electrodes,
│   │   so the file stands alone; ephys.nwb's identifier recorded in notes)
├── units (Units)   — one row per curated unit
│     [id, spike_times (ragged, host ROS time, exact piecewise sync map),
│      electrodes (region), electrode_group, probe, sorter_unit_id,
│      waveform_mean (n_samples, n_channels), waveform_sd, peak_channel,
│      unit_x, unit_y (µm on probe), depth,
│      firing_rate, presence_ratio, isi_violations_ratio, amplitude_cutoff,
│      amplitude_median, snr, isolation_distance, l_ratio, d_prime,
│      drift_ptp, quality (good|mua|noise), curation_source (auto|manual)]
└── processing/ecephys/
    ├── motion_estimate (TimeSeries, depth × time drift), if motion correction ran
    └── sorting_provenance (DynamicTable [step, tool, version, parameters_json])
```

Sorting (SpikeInterface): load the recording (`ephys.nwb` or the native `.bin`;
sample indices are identical either way) → phase shift, high-pass, bad-channel
detection, common reference, motion correction → **Kilosort4** → `SortingAnalyzer`
(waveforms, templates, amplitudes, PCs, metrics) → curation (Bombcell/UnitRefine
+ optional Phy) → spike sample index → probe seconds → host ROS time via the
probe's sync map → `spike_sorting.nwb`. Sorter output is archived alongside.

---

## 5. Proposed code layout (inside `nwb_conversion/`)

```text
nwb_conversion/
├── PLAN.md
├── pyproject.toml             # uv project; no ROS dependency
├── metadata/                  # lab.yaml, devices.yaml, subjects/<id>.yaml
├── src/tabletop_nwb/
│   ├── cli.py                 # tt-nwb inventory|extract|sync|convert|validate|sort
│   ├── session.py
│   ├── readers/               # mcap.py, edf.py, video.py, spikeglx.py
│   ├── sync/                  # detect.py, match.py, fit.py, report.py
│   ├── writers/               # common.py, task.py, video.py, ephys.py, sorting.py
│   ├── trials.py              # events → trials table + trial assignment
│   └── validation/            # post-conversion test suite (§10)
│       ├── expectations.yaml  # which streams are required/optional, thresholds
│       ├── conftest.py        # --nwb-dir / --session fixtures, lazy file handles
│       └── test_*.py          # one module per check category (§10.3)
├── fixtures/                  # synthetic-session generator (§10.6)
└── tests/                     # unit tests of the converter code itself
```

Key dependencies: `pynwb`, `hdmf`, `neuroconv[spikeglx]`, `spikeinterface`,
`probeinterface`, `ndx-events`, `nwbinspector`, `mcap`, `mcap-ros2-support`,
`numpy`, `pandas`/`pyarrow`, `scipy`, `dandi` (optional). Kilosort4 goes in a
separate GPU environment.

Output per session:

```text
<session_dir>/nwb/
├── task.nwb  video.nwb  ephys.nwb  spike_sorting.nwb
├── video/<camera>/<segment>.mp4
├── sorting/                  # sorter + curation outputs
└── sync/                     # sync_report.json, residual plots
```

---

## 6. Build order

0. **Synthetic-session generator + test-suite skeleton** (§10.6). The file-level, cross-file, and sync checks come first, so every later milestone is validated as it lands.
1. **Sync** for Teensy, EyeLink EDF, FLIR Line3, and UR DI3 on an existing bag. This also measures Teensy timestamp quality and sets the initial sync thresholds (§10.4).
2. **`task.nwb`** with today's data: trials from `TaskEvent`, all event tables, Teensy, robot, sync, config.
3. **EyeLink** from the EDF.
4. **`video.nwb`** (after D4 testing).
5. **SpikeGLX test recording** with pin 31 wired: verify SY edges, fit per probe, then **`ephys.nwb`**. Cross-check the ephys fit against EyeLink and video independently.
6. **Spike sorting → `spike_sorting.nwb`**.
7. The trial-record columns as R2 lands; then OptiTrack, FK poses, logs, DANDI packaging.

End-to-end check: a short session in which one physical event is seen by several
streams, e.g. a reward click or an LED/photodiode visible to a camera and
wired to an NI-DAQ input.

---

## 7. Rig-side changes this plan depends on (not to be done now)

| # | Change | Why |
| --- | --- | --- |
| R1 | Wire Teensy pin 31 → SpikeGLX chassis sync input; configure SpikeGLX so the signal lands on each probe's SY channel (and the NI-DAQ, if used) | ephys alignment (D1). See U1–U2 (§9). |
| R2a | Record each trial's full `TrialSpec` + `TrialFeedback` + outcome (new `/tabletop/trial_record` topic or `trials.jsonl` in the session dir) | trials table columns |
| R2b | Publish a `TaskEvent` (or richer event message) at **every** phase boundary in every task, including Foraging/Present stimulus/delay/response/reveal, plus every sound played | "all trial events in the NWB" (D6) |
| R2c | Publish manipulation lifecycle events (fetch/present/unpresent/reset start+end, planning retry) and per-task-class start/end | trial context, epochs |
| R3 | Copy the resolved task config, launch args, git commit, and image digests into the session dir | provenance |
| R4 | Add `session_metadata.yaml` (subject, experimenter, probe insertion sites/depths, notes) | NWB metadata, electrode locations |
| R5 | Run sync pulses for the whole recording, not only the task | coverage at both ends of the ephys/video |
| R6 | Fix the empty TCP pose topics (or accept FK) | robot pose |
| R7 | Check host clock discipline (chrony slewing/stepping) during sessions; log chrony tracking | linearity of ROS time (§2.4) |
| R8 | Start SpikeGLX before the task and stop it after; check levels and grounding between the Teensy and the chassis | coverage, signal integrity |

---


## 8. Questions for you

### 8.1 Open decisions

Each has a recommendation, which the pipeline will use as its default
unless you say otherwise.

| # | Decision | Options | Recommendation | Needed by |
| --- | --- | --- | --- | --- |
| D2 | Confirm host ROS time as the master clock | ROS time / ephys clock | ROS time (§2.4); revisit only if R7 shows the host clock being slewed | milestone 1 |
| D3 | t = 0 / `session_start_time` | earliest sample of any stream / `TASK_STARTED` / bag start | earliest sample of any stream (no negative times) | milestone 2 |
| **D4** | Video encoding and archiving | H.264 proxy (CRF ~18–23) / lossless / also keep MCAP images | **open: you will test video sizes first** | milestone 4 |
| D4b | `*_unmatched` camera frames | drop + log / separate series | drop, log in `unmatched_frames` | milestone 4 |
| D5b | Ephys HDF5 compression | gzip / Blosc-zstd | gzip (portable) unless write speed is a problem | milestone 5 |
| D5c | Derived LFP in `ephys.nwb` | yes / no | yes, if you will analyze LFP (~7 GB/h per probe) | milestone 5 |
| D6b | Trial `stop_time` | `TRIAL_DEPRESENTING` / next `TRIAL_PRESENTED` / unpresent done | `TRIAL_DEPRESENTING` | milestone 2 |
| D7 | EyeLink source | EDF / ROS samples | EDF primary, ROS fallback | milestone 3 |
| D8 | File names | `task.nwb` etc. / DANDI-style | your names on disk; `dandi organize` at upload | before any upload |
| D9 | NWB extensions | core only / `ndx-events` / custom | `ndx-events` for event tables | milestone 2 |
| D10 | OptiTrack unlabeled markers | ragged table / padded / skip | ragged table, solenoid marker tagged | milestone 7 |
| D11 | Spike sorter + curation | Kilosort4 + Bombcell/UnitRefine (+ Phy) / other | Kilosort4; auto-curation + optional Phy | milestone 6 |
| D12 | Where the pipeline runs | uv env / rig container / cluster | uv env; `unbag` in the ROS container | milestone 1 |
| D13 | Storage backend | HDF5 / Zarr | HDF5 | milestone 2 |
| D14 | Include `/rosout` and Teensy logs | yes / no | yes | milestone 2 |
| D15 | Robot detail | joint states + wrench + pose / + dynamic extras | without the dynamic extras | milestone 2 |
| D16 | Test-suite strictness | which checks FAIL vs WARN (§10.2) | defaults in §10.3; tune after the first real sessions | milestone 0 |

### 8.2 Facts I need from you

1. **NP2.0 variant:** single-shank or 4-shank? How many probes per session? (These can also be read from the `.meta` files once a test recording exists.)
2. **Will you use an NI-DAQ (NIDQ) stream** in SpikeGLX (e.g. for a photodiode, juice line, or as a fallback sync input; U1 in §9)?
3. **Subject metadata:** species, subject IDs, sex, DOB/age, and how weight is tracked. Also the lab/institution/protocol strings for the NWB header.
4. **Probe localization:** what will you have, and when (planned coordinates, insertion depth, post-hoc histology/atlas registration)? This determines the electrodes table's `location`/`x,y,z` fields, and whether they get filled in later.
5. **Behavior-only sessions:** will there be sessions with no ephys? (The design supports them; this just confirms the test expectations.)
6. **Rig changes R1–R8 (§7):** which you'll make and roughly when. In particular, **R2a/R2b** (full trial records and every phase event) gate the "all trial events" requirement for Foraging/Present.
7. **R5:** is it OK to run sync pulses for the whole recording instead of only during the task?
8. **DANDI:** is public or archive upload a goal? This affects naming (D8), extension choice (D9), and how strict the inspector checks are (§10.3 A).
9. **Compute/storage:** where conversion and sorting will run (GPU for Kilosort4), and where outputs live. A session is roughly 83 GB/probe-hour of ephys plus video (D4).
10. **Which task(s) to support first:** Foraging, Present, InvariantAffordance, SmoothPursuit. Task-specific trial columns get built in that order.

---

## 9. Uncertainties

Things I could not verify from the code. Each lists how and when it gets resolved.

| # | Uncertainty | Why it matters | How / when resolved |
| --- | --- | --- | --- |
| U1 | **SpikeGLX with an irregular external sync.** Its sync tools expect a regular square wave; the Teensy pulse has random 850–1150 ms intervals and is 100 ms long. Recording it on SY should work (our pipeline extracts the edges itself), but SpikeGLX's run-time sync check and TPrime may complain. | If it doesn't work, the wiring changes | Test recording (milestone 5). Fallback: SpikeGLX's own 1 Hz wave syncs probes↔NI, and Teensy pin 31 goes to an NI-DAQ digital input. |
| U2 | **Voltage level**: the Teensy outputs 3.3 V; the chassis sync input threshold is unchecked. | Missed edges | Check the imec/PXIe module spec before wiring (R1). |
| U3 | **Teensy timestamp accuracy** vs the host clock: software-timed, micro-ROS re-sync every 10 s, unmeasured. | It sets the floor on sync residuals and therefore the test thresholds | Milestone 1: compare against EyeLink and FLIR, which are hardware-timed. |
| U4 | **Host clock linearity** (NTP/chrony slewing) | Linear vs piecewise ephys map (§2.4) | R7; also visible as steps in the sync residuals. |
| U5 | **`ur_msgs/IOStates` has no header** (to my knowledge; not vendored in this repo) | UR DI3 timing would be bag-receive time only | Inspect a bag's schema (milestone 1). |
| U6 | **FLIR timing details**: how the driver's `TimeEstimator` behaves across dropped frames; whether `line_status` is latched at exposure start | Video timestamp accuracy | Milestone 4: compare the frame-index fit against header stamps. |
| U7 | **OptiTrack**: whether `mid_exposure_time`/`data_received_time` are populated | Motive-side clock available or not | Inspect a bag with OptiTrack data. |
| U8 | **UR joint effort units** from `ur_robot_driver` | Correct `unit` field | Driver docs / inspect values. |
| U9 | **Data volume and throughput**: no 2026-10-31 bag contains images, OptiTrack, or TCP pose data | Storage, D4, runtime | Your video tests (D4) + first full session. |
| U10 | **Trial structure differs by task**: task-specific columns are NaN elsewhere; SmoothPursuit may need its own intervals table | Trials table design | When tasks are prioritized (§8.2 Q10). |
| U11 | **Initial test thresholds** (§10.4) are educated guesses | False FAILs or missed problems | Tune after the first ~3 real sessions; recorded in `expectations.yaml`. |
| U12 | **Multiple task classes per run**: `run.py` can run several in sequence under one `TASK_STARTED`; per-class boundaries aren't evented | Epochs table | R2c. |

---

## 10. Post-conversion test suite

### 10.1 Purpose and how it runs

After every conversion, an automated suite checks that:

- every expected data stream exists and is complete,
- synchronization succeeded and is accurate,
- trials and events are consistent with the raw source,
- the four files agree with one another.

It answers the question "can I trust this session's NWB files?" and is
separate from the unit tests of the converter code (§10.6).

- **Runner:** pytest, invoked as `tt-nwb validate <session_dir>/nwb` (pipeline stages [4] and [6]). Tests are parameterized over the streams listed in `expectations.yaml`.
- **Inputs:** the NWB files, `inventory.json` (counts taken from the raw sources during stage 0), `sync_report.json`, and (for spot checks) read-only access to the raw sources: MCAP, EDF, SpikeGLX `.bin/.meta`, video files.
- **Outputs:** `nwb/validation/report.json` (machine-readable: every check, status, measured value, threshold), `report.md` (human summary at the top: PASS/WARN/FAIL counts plus each failure with an explanation), JUnit XML, and sync residual plots. Exit code is non-zero if any FAIL.
- **Speed:** heavy checks (raw-vs-NWB equality on ephys, frame decoding) use random **samples** of chunks and frames, with a fixed, recorded seed. A `--full` flag runs exhaustive versions.

### 10.2 Expectations, not hard-coding

`expectations.yaml` declares, for each stream:

- required / optional / absent
- expected rate and tolerance
- value ranges
- sync class and thresholds
- severity of each check

A per-session override (generated from `session_metadata.yaml` and
`inventory.json`) handles cases like "OptiTrack not used today" or "behavior-only
session". This distinguishes *the stream was never recorded* (WARN or skip)
from *the stream was recorded but lost in conversion* (FAIL).

Severity levels:

- **FAIL:** the data are wrong or missing. Blocks downstream use and spike sorting.
- **WARN:** usable, but needs a look (gaps, borderline residuals, an optional stream missing).
- **INFO:** measurements reported for trend tracking (drift ppm, dropped-frame counts).

### 10.3 Checks by category

**A. File-level** (each file)

| Check | Severity |
| --- | --- |
| File exists and opens with pynwb (extensions resolve) | FAIL |
| `nwbinspector`: no CRITICAL; BEST_PRACTICE_VIOLATION per D8/DANDI choice | FAIL / WARN |
| (Optional) `dandi validate` passes | WARN |
| All external video files referenced by `video.nwb` exist at their relative paths and are readable by ffprobe | FAIL |
| `source_script` records pipeline version + rig git commit | WARN |

**B. Cross-file consistency**

| Check | Severity |
| --- | --- |
| Identical `session_id`, `session_start_time`, `timestamps_reference_time`, subject fields in all files | FAIL |
| `identifier`s are distinct | FAIL |
| `spike_sorting.nwb` electrodes table equals `ephys.nwb`'s (ids, geometry, groups) and records `ephys.nwb`'s identifier | FAIL |
| `sync_fits` rows are identical wherever the same stream appears in more than one file | FAIL |
| Time spans overlap as expected (task span ⊂ ephys span ⊂ session; video covers the task) | WARN |

**C. Stream presence and completeness** (each stream in `expectations.yaml`)

| Check | Severity |
| --- | --- |
| Object exists at the expected path with the expected neurodata type | FAIL if required |
| **Count conservation**: NWB sample/row count equals the raw source count (bag `metadata.yaml` message counts × samples per message; EDF sample count; SpikeGLX `nSamples` from `.meta`; per-camera frame count from `/meta`), minus any documented, logged drops | FAIL |
| Shape / dtype / channel count (e.g. 384 per probe; 6 joints; 5 glove channels) | FAIL |
| `unit`, `conversion`, `description` set; units match `expectations.yaml` | WARN |
| Value sanity: no all-NaN columns; joints within limits; glove ADC 0–1023; quaternions unit-norm; wrench finite; pupil > 0 except where marked missing; EyeLink missing-data sentinel converted to NaN | FAIL / WARN |
| Six camera `ImageSeries`, each with frame count = sum of frames in its external files (ffprobe) and `starting_frame` consistent with segment lengths | FAIL |
| Electrodes geometry equals the IMRO-derived positions from `.meta` | FAIL |

**D. Timestamps**

| Check | Severity |
| --- | --- |
| No NaN/inf; all ≥ 0 and ≤ session duration | FAIL |
| Monotonic: strictly increasing for sampled series; non-decreasing for event tables | FAIL |
| Effective rate within tolerance of expected (video 120 Hz, Teensy 100 Hz, robot ~500 Hz, EyeLink = EDF header rate, ephys = fit rate ≈ 30 kHz ± 0.05%) | WARN |
| Gap report: largest gap and total gap time per stream vs thresholds | WARN (FAIL above hard limit) |
| Video: all six cameras' timestamps for the same trigger group are identical; dropped frames match `camera_frame_id` gaps and are listed in `dropped_frames` | FAIL |

**E. Synchronization**: the core of the suite

| Check | Severity |
| --- | --- |
| Every stream with a sync signal has a `sync_fits` row with status OK | FAIL |
| Matched pulses ≥ 95% of reference pulses within the stream's overlap; no ambiguous matches | FAIL below 80%, WARN 80–95% |
| Residual RMS and max below class thresholds (§10.4) | FAIL / WARN |
| Drift plausible: \|slope − 1\| < 200 ppm (crystals are typically < 50 ppm) | WARN |
| Pulse coverage: fraction of each stream's duration that lies inside pulse coverage; extrapolated duration at each end is reported | WARN if > 5 min extrapolated (motivates R5) |
| **Closure test**: re-detect pulses from the *written NWB data* (Teensy `teensy_outputs` sync column; video `line_status` in camera metadata; ephys/EyeLink stored native pulse trains mapped with the stored fit) and confirm they land on the reference pulse times within tolerance. This checks the *applied* timestamps, not just the fit. | FAIL |
| **Triangle test**: fit stream pairs directly, without going through the Teensy (ephys↔video, ephys↔EyeLink, video↔EyeLink), and confirm the result agrees with the composition of their fits to the reference within tolerance. This catches a bad reference or a bad single fit. | FAIL |
| Piecewise fits: breakpoints are reported, and each segment individually passes the residual limits | WARN |
| (When available) Physical-event test: an event seen by several streams (reward click, LED) has consistent times across files | WARN |

**F. Trials and events (`task.nwb`)**

| Check | Severity |
| --- | --- |
| Every `TaskEvent` in the bag appears in `task_events`: counts equal per event type | FAIL |
| `len(trials)` = number of `TRIAL_PRESENTED`; trial numbers unique; `start_time < stop_time` | FAIL |
| Trials do not overlap (guaranteed by `_trial_lock`) and fall inside an epoch | FAIL |
| Each trial's `start_time` equals its `TRIAL_PRESENTED` time and `stop_time` equals its `TRIAL_DEPRESENTING` time (D6/D6b) | FAIL |
| Phase events tagged with a trial number lie within that trial, in the expected order (e.g. `STIMULUS_REVEALED < MOTION_STARTED < GO_CUE`) | FAIL |
| Time-assigned events (reward, smartglass, hand sensors, Flic, buzzer) with trial_number ≥ 0 lie inside that trial's window; those with −1 lie outside all windows | FAIL |
| Event conservation: reward onsets = `is_reward_active` rising edges (± tolerance vs `set_reward` service calls); Flic rows = bag messages; smartglass/arm-lock edges = Teensy edges | FAIL / WARN |
| `aborted_trials` count = `OBJECT_MISS` count | FAIL |
| Trial columns that are expected for this task (from `expectations.yaml`) are non-NaN; once R2 lands, required `TrialSpec`/`TrialFeedback` fields are present | FAIL / WARN |
| Configuration table contains the task config, URDF, and calibrations | WARN |

**G. Ephys (`ephys.nwb`)**

| Check | Severity |
| --- | --- |
| Random-chunk bitwise equality: N random (time × channel) blocks of the `ElectricalSeries` equal the SpikeGLX `.bin` | FAIL |
| `conversion`/`offset` match the `.meta` gains (NP2.0) | FAIL |
| `starting_time` and `rate` equal the stored fit (`b`, `fs/a`) | FAIL |
| SY edges stored in `processing/sync` match edges re-extracted from the raw SY channel in sampled windows | FAIL |
| Derived LFP (if D5c): rate ≈ 2.5 kHz, length consistent with raw, filter parameters recorded | WARN |
| Basic signal health: per-channel RMS within plausible range; flags dead or saturated channels (cross-checked with `bad_channel`) | WARN |

**H. Spike sorting (`spike_sorting.nwb`, stage [6] only)**

| Check | Severity |
| --- | --- |
| Units table non-empty; each unit's `spike_times` sorted, within the ephys recording span | FAIL |
| Spike counts per unit equal the sorter output (after curation) | FAIL |
| Recomputation: for random units, spike sample indices from the sorter output, mapped with the stored ephys fit, reproduce `spike_times` (≤ 1 µs) | FAIL |
| Electrode references valid; `peak_channel` within the unit's electrodes | FAIL |
| Quality-metric columns present and non-NaN for all units; `quality` label set | WARN |
| `sorting_provenance` records sorter, versions, parameters | WARN |

### 10.4 Initial thresholds (to be tuned; U11)

| Stream class | Residual RMS (WARN / FAIL) | Residual max (FAIL) | Notes |
| --- | --- | --- | --- |
| Hardware-timed (ephys, EyeLink, video) vs Teensy reference | 1 ms / 3 ms | 10 ms | Floor set by Teensy software timestamps (U3); expect to tighten |
| Hardware ↔ hardware (triangle test) | 0.2 ms / 1 ms | 2 ms (video: 1 frame) | Independent of Teensy jitter |
| Software-timed (UR DI3, OptiTrack) | 5 ms / 15 ms | 50 ms | QC streams |
| Closure test | within each class's FAIL threshold | | |

### 10.5 Mutation testing of the suite itself

A test suite that never fails can't be trusted. Using the synthetic session
(§10.6), the converter output is deliberately corrupted, and the suite **must
FAIL** for each of these:

- drop a camera
- delete 5% of ephys samples
- shift one stream's timestamps by 20 ms
- add 300 ppm drift without refitting
- remove the SY edges
- drop half the reward events
- swap two trials' times
- mismatch `session_start_time` between files
- truncate a video segment
- corrupt one ephys chunk

These run in CI on every change to the converter or the suite.

### 10.6 Converter unit tests and synthetic sessions

- **Synthetic session generator** (`fixtures/`): writes a small MCAP with the rig's message types (via `mcap`), a fake EDF-equivalent, a few seconds of fake SpikeGLX `.bin/.meta` (NP2.0 geometry), and short video segments. Every stream carries a known Teensy-style jittered pulse train with **known injected** drift, offset, jitter, dropouts, and one clock step. The generator also writes the ground truth, so the converter's fits can be checked against it exactly.
- **Unit tests**: pulse detection per stream (EyeLink mask, Line3 bit, UR active-low, SY bit, OptiTrack displacement); interval matching under dropouts and different start times; robust/piecewise fitting; trial assignment edge cases (events at window boundaries, overlapping preparation on the other arm, aborted trials).
- **Golden real session**: once a short real session with all streams exists, it becomes a regression fixture. Its validation report is stored and compared across converter versions.
