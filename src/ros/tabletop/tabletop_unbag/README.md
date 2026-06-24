# tabletop_unbag

A generic ROS 2 bag (MCAP) → CSV / image exporter, written in C++. It is a port
of the Python `tabletop_rig.utils.rosbag` module, but runs as a standalone,
dependency-light executable (`unbag`) with no Python / pandas / rclpy at
runtime, and is built to stream very large bags without exhausting memory.

## What it does

Given a *bag directory* (the directory that holds the `.mcap` files and
`metadata.yaml`), `unbag` writes one output per topic into an output directory
(by default `<parent of BAG_DIR>/unbag_output`):

* **Normal messages** are flattened into a per-topic CSV
  (`/eyelink/sample` → `eyelink_sample.csv`). Any message type works — there is
  no per-type code. Nested submessages become dot-separated columns
  (`pose.position.x`) and array / sequence elements become bracket-indexed
  columns (`name[0]`, `position[2]`). Every row carries a `bag_time_ns` column,
  and the columns are the union of every field seen across the topic's messages
  (shorter messages leave trailing columns empty).
* **Image topics** (`sensor_msgs/msg/Image`,
  `sensor_msgs/msg/CompressedImage`) are decoded and saved as image files under
  a per-topic subdirectory (`<topic>/<sec>_<nanosec>.{jpg,png,tiff}`).

## Usage

```bash
# Unbag everything into <parent of BAG_DIR>/unbag_output
ros2 run tabletop_unbag unbag /path/to/session/bag

# Choose the output directory
ros2 run tabletop_unbag unbag /path/to/session/bag -o /path/to/out

# Only the CSV handler (skip image topics), or only the image handler
ros2 run tabletop_unbag unbag BAG_DIR --handlers csv
ros2 run tabletop_unbag unbag BAG_DIR --handlers image --image-encoding mono8

# Restrict / exclude topics (mutually exclusive)
ros2 run tabletop_unbag unbag BAG_DIR --topics /joint_states /predicted_markers
ros2 run tabletop_unbag unbag BAG_DIR --exclude-topics /rosout

# Re-run from scratch instead of resuming
ros2 run tabletop_unbag unbag BAG_DIR --overwrite
```

| Flag | Description |
| --- | --- |
| `BAG_DIR` (positional) | Directory containing the bag's `.mcap` files and `metadata.yaml`. Required. |
| `-o, --output-dir DIR` | Where to write outputs. Default: `<parent of BAG_DIR>/unbag_output`. |
| `--handlers H [H ...]` | Handlers to enable (`csv`, `image`). Topics whose type is not claimed by an enabled handler are skipped. Default: all. |
| `--topics T [T ...]` | Only unbag these topics. Mutually exclusive with `--exclude-topics`. |
| `--exclude-topics T ...` | Unbag all topics except these. Mutually exclusive with `--topics`. |
| `--overwrite` | Delete previously unbagged output for the selected topics before writing. Without it, an interrupted run resumes. |
| `--batch-size N` | Messages buffered in memory before flushing to disk (default 1000). |
| `--image-encoding ENC` | Target encoding for saved images (default `bgr8`). |
| `--storage-id ID` | Storage plugin override (default: inferred from metadata, fallback `mcap`). |
| `--serialization-format F` | Serialization override (default: inferred, fallback `cdr`). |
| `-v, --verbose` | Log the handler chosen for each topic and topics skipped. |

The storage plugin and serialization format are inferred from the bag's
`metadata.yaml`; the `--storage-id` / `--serialization-format` flags are only
needed when that inference is unavailable or wrong.

## Architecture

Each message type is routed to a **handler**. Handlers live in `handlers/` and
derive from `MessageHandler` (`handlers/handler.hpp`); a small dispatch in
`unbagger.cpp` picks, per topic, the first handler that claims the message type
(the image handler is checked before the CSV catch-all). Adding support for a
new output type is a matter of adding a handler and registering it.

```text
include/tabletop_unbag/
├── options.hpp            # UnbagOptions, TopicInfo
├── progress_bar.hpp       # tqdm-style progress bar (header-only, no deps)
├── unbagger.hpp           # unbag(): metadata inference + dispatch + the passes
└── handlers/
    ├── handler.hpp        # MessageHandler base (the handler lifecycle)
    ├── csv_handler.hpp
    └── image_handler.hpp
src/
├── main.cpp               # CLI
├── unbagger.cpp           # orchestration
└── handlers/
    ├── csv_handler.cpp    # flattening + pandas-compatible value formatting
    └── image_handler.cpp  # cv_bridge / Bayer decode
```

### The handler lifecycle (two passes, batched, resumable)

The orchestrator drives every handler through the same lifecycle, which is what
lets it stay type-agnostic:

1. **Preprocess pass** — `preprocess()` is called once per message, but only for
   handlers whose `needs_preprocess()` is true. The CSV handler needs it to
   learn the column set: a `sequence<>` field's length is only known per
   message, so the column union cannot be known without reading every message
   first. It stores only column *names* (O(#columns) memory), not rows. The
   image handler does not need it (each image is independent), so its
   `preprocess()` is a no-op and — when no enabled handler needs the pass — the
   whole pass is skipped.
2. **`begin_write()`** — handlers apply the overwrite/resume policy.
3. **Write pass** — `write()` is called once per message; handlers buffer and
   flush in batches of `--batch-size`, so memory stays bounded no matter how
   many messages a topic has.
4. **`finish()`** — flush and close.

**Interruption / resume.** Because output is flushed incrementally, an
interrupted run leaves valid partial output. Re-running (without `--overwrite`)
picks up where it left off: the CSV handler counts the rows already on disk
(repairing a torn final line) and skips that many messages before appending; the
image handler skips timestamps whose file already exists. `--overwrite` instead
deletes the selected topics' prior output first — the CSV handler removes its
file, and the image handler clears the topic's directory so a re-run with a
different `--image-encoding` cannot leave a mix of old and new formats behind.

### Generic flattening

The Python version relies on rclpy's dynamically generated message classes and
`get_fields_and_field_types()`. C++ messages are statically typed, so the CSV
handler instead loads the `rosidl_typesupport_introspection_cpp` description of
each type at runtime and reads the CDR payload directly with Fast CDR (ROS 2
serializes messages as PLAIN_CDR / XCDRv1). This is RMW-independent (it does not
go through `rmw_deserialize`), so it works regardless of the active middleware.
Values are formatted to match pandas' `to_csv` output: shortest round-tripping
floats with a trailing `.0` on integral values, `True`/`False` for booleans, and
RFC 4180 minimal quoting for strings.

### Image decoding

Color conversion goes through **cv_bridge** for uncompressed `Image` messages
and for non-Bayer `CompressedImage` messages, so any target encoding cv_bridge
supports works (set with `--image-encoding`). The one case cv_bridge cannot
handle is a **compressed Bayer** image: the FLIR cameras JPEG-compress the raw
single-channel Bayer mosaic (format e.g. `bayer_rggb8; jpeg compressed
bayer_rggb8`), and cv_bridge infers a compressed image's encoding from its
channel count, so it would mistake the mosaic for `mono8` and never demosaic.
For that case the handler parses the `format` string and demosaics with the
Bayer-aware OpenCV conversion codes (ROS and OpenCV name Bayer patterns from
opposite corners, hence the swapped-looking codes). Uncompressed Bayer is fine
through cv_bridge, which knows the source encoding from `msg.encoding`.

### One intentional difference from the Python version

The Python flattener expands variable-length sequences but, due to a quirk of
its `sequence<...>` type-string check, collapses **fixed-size primitive arrays**
(e.g. `CameraInfo.k`, a `float64[9]`) into a single column holding a numpy
string like `[600. 0. 320. ...]`. This port expands fixed-size arrays the same
way as sequences (`k[0]..k[8]`), which is the more useful and presumably
intended behavior. All other CSV output is byte-for-byte identical to the Python
converter (verified by diffing both on the same bag).

## Building

```bash
tt-build colcon -p tabletop_unbag
# or, directly:
colcon build --packages-select tabletop_unbag
```
