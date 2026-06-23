# tabletop_unbag

A generic ROS 2 bag (MCAP) → CSV / image exporter, written in C++. It is a port
of the Python `tabletop_rig.utils.rosbag` module (`rosbag_to_csv`) and produces
the same per-topic CSV layout, but runs as a standalone, dependency-light
executable with no Python / pandas / rclpy at runtime.

## What it does

Given a *session directory* containing one or more recorded bags
(`<session>/<bag>/*.mcap`), `unbag` writes one CSV per topic into the session
directory, named after the topic (`/eyelink/sample` → `eyelink_sample.csv`).

* **Any message type** is flattened generically — there is no per-type code.
  Nested submessages become dot-separated columns (`pose.position.x`) and array
  / sequence elements become bracket-indexed columns (`name[0]`, `position[2]`).
* Each row carries a `bag_time_ns` column (the bag receive timestamp), and the
  CSV columns are the union of every field seen across all messages on that
  topic (shorter messages simply leave trailing columns empty), matching the
  Python implementation.
* **Image topics** (`sensor_msgs/msg/Image`, `sensor_msgs/msg/CompressedImage`)
  are never written to CSV. With `--image`, `CompressedImage` messages are
  decoded and saved as `<topic>/<sec>_<nanosec>.{jpg,png,tiff}`.

## How the generic flattening works

The Python version relies on rclpy's dynamically generated message classes and
`get_fields_and_field_types()` for introspection. C++ messages are statically
typed, so instead this package:

1. Loads the `rosidl_typesupport_introspection_cpp` type support for each topic
   type at runtime (cached per type), which describes every field's name, type,
   and array/sequence shape.
2. Reads the CDR payload directly with Fast CDR (ROS 2 serializes messages as
   PLAIN_CDR / XCDRv1), walking the introspection field description in
   declaration order to emit `(column, value)` pairs.

This is RMW-independent (it does not go through `rmw_deserialize`), so it works
regardless of the active middleware.

Values are formatted to match pandas' `to_csv` output: shortest round-tripping
floats with a trailing `.0` on integral values (`1.0`), `True`/`False` for
booleans, and RFC 4180 minimal quoting for strings.

### One intentional difference from the Python version

The Python flattener expands variable-length sequences (`position[0]`,
`position[1]`, …) but, due to a quirk of its `sequence<...>` type-string check,
collapses **fixed-size primitive arrays** (e.g. `CameraInfo.k`, a `float64[9]`)
into a *single* column holding a numpy string like `[600.   0. 320. ...]`. This
port instead expands fixed-size arrays the same way as sequences
(`k[0]..k[8]`), which is the more useful and presumably intended behavior. All
other output is byte-for-byte identical to the Python converter.

## Building

```bash
tt-build colcon -p tabletop_unbag
# or, directly:
colcon build --packages-select tabletop_unbag
```

## Usage

```bash
# Convert the most recent session ($ROS_BAG_DIR/latest by default)
ros2 run tabletop_unbag unbag

# Convert a specific session directory
ros2 run tabletop_unbag unbag -d /path/to/session_dir

# Convert every session directory under $ROS_BAG_DIR
ros2 run tabletop_unbag unbag --all-sessions

# Restrict / exclude topics, export images, force re-conversion
ros2 run tabletop_unbag unbag -d <session> --topics /joint_states /markers
ros2 run tabletop_unbag unbag -d <session> --exclude-topics /rosout
ros2 run tabletop_unbag unbag -d <session> --image --force
```

| Flag | Description |
| --- | --- |
| `-d, --session-dir DIR` | Session directory to convert (default `$ROS_BAG_DIR/latest`). |
| `-a, --all-sessions` | Convert all session directories in `$ROS_BAG_DIR`. |
| `--topics T [T ...]` | Whitelist of topics to include (default: all). |
| `--exclude-topics T [T ...]` | Topics to exclude. |
| `--image` | Export image topics as image files. |
| `-f, --force` | Overwrite existing CSV / image files. |
| `-v, --verbose` | Increase logging verbosity. |

A session is skipped if it already contains `.csv` files, unless `--force` is
given. If the same topic appears in more than one bag within a session, the
conversion fails with a collision error (matching the Python behavior).
