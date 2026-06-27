# tabletop_unbag

A generic ROS 2 bag (MCAP) → CSV / image **or** HDF5 exporter, written in C++.
It is a port of the Python `tabletop_rig.utils.rosbag` module, but runs as a
standalone, dependency-light executable (`unbag`) with no Python / pandas /
rclpy at runtime, and is built to stream very large bags without exhausting
memory.

Two output backends are available, selected with `--format`:

* **`csv`** (default) — one CSV per non-image topic plus a directory of
  per-frame image files. Backwards-compatible with the Python converter.
* **`hdf5`** — the whole run in a single `unbag.h5`: one group per topic, one
  dataset per flattened column, and one stacked `(N,H,W,C)` dataset per image
  topic. Same flattening and debayering, different container. The HDF5 backend
  reads the bag **once** (no preprocess pass), which is the only effective fix
  for the cold-cache, bandwidth-bound first run — see [docs/performance.md](docs/performance.md).

It is **multithreaded**: a single reader thread feeds one consumer thread per
CSV topic and a shared pool of worker threads that decode images in parallel, so
the (CPU-bound) image transcoding scales across all cores. On a 24-core machine
this is roughly a 10× end-to-end speedup over the single-threaded version, while
peak memory stays bounded by back-pressure on the work queues.

## What it does

Given a *bag directory* (the directory that holds the `.mcap` files and
`metadata.yaml`), `unbag` writes one output per topic into an output directory
(by default `<parent of BAG_DIR>/unbag`). With the default `csv` backend that
means one file/directory per topic, as described below; the `hdf5` backend
writes the same flattened columns and decoded frames into a single
`<output-dir>/unbag.h5` instead (see [HDF5 backend](#hdf5-backend)):

* **Normal messages** are flattened into a per-topic CSV
  (`/eyelink/sample` → `eyelink_sample.csv`). Any message type works — there is
  no per-type code. Nested submessages become dot-separated columns
  (`pose.position.x`) and array / sequence elements become bracket-indexed
  columns (`name[0]`, `position[2]`). Every row carries a `bag_time_ns` column,
  and the columns are the union of every field seen across the topic's messages
  (shorter messages leave trailing columns empty).
* **Image topics** (`sensor_msgs/msg/Image`,
  `sensor_msgs/msg/CompressedImage`) are decoded and saved as image files under
  a per-topic subdirectory (`<topic>/<sec>_<nanosec>.{jpg,png,tiff}`). The `sec`
  and `nanosec` fields are zero-padded to a fixed width (10 and 9 digits) so the
  files sort chronologically under a plain lexicographic sort. When more
  than one frame shares a header stamp (e.g. synchronized cameras), the extra
  frames are kept under a zero-padded suffix
  (`<sec>_<nanosec>_<NNN>.<ext>`) assigned in bag-receive order, so no frame is
  lost and the files still sort chronologically. `--image-format` controls the
  output file format.

## Usage

```bash
# Unbag everything into <parent of BAG_DIR>/unbag
ros2 run tabletop_unbag unbag /path/to/session/bag

# Choose the output directory
ros2 run tabletop_unbag unbag /path/to/session/bag -o /path/to/out

# Only the CSV handler (skip image topics), or only the image handler
ros2 run tabletop_unbag unbag BAG_DIR --handlers csv
ros2 run tabletop_unbag unbag BAG_DIR --handlers image --image-encoding mono8

# Save every image as PNG (lossless) instead of keeping the source format
ros2 run tabletop_unbag unbag BAG_DIR --handlers image --image-format png

# Export everything to a single HDF5 file instead of CSVs + image files
ros2 run tabletop_unbag unbag BAG_DIR --format hdf5
ros2 run tabletop_unbag unbag BAG_DIR --format hdf5 --hdf5-gzip-level 6

# Restrict / exclude topics (mutually exclusive)
ros2 run tabletop_unbag unbag BAG_DIR --topics /joint_states /predicted_markers
ros2 run tabletop_unbag unbag BAG_DIR --exclude-topics /rosout

# Re-run from scratch instead of resuming
ros2 run tabletop_unbag unbag BAG_DIR --overwrite
```

Handler-specific options are namespaced by handler (`--csv-*`, `--image-*`) so it
is clear which handler each one affects; run-wide options have no prefix.

**Common options**

| Flag | Description |
| --- | --- |
| `BAG_DIR` (positional) | Directory containing the bag's `.mcap` files and `metadata.yaml`. Required. |
| `-o, --output-dir DIR` | Where to write outputs. Default: `<parent of BAG_DIR>/unbag`. |
| `--format FMT` | Output backend: `csv` (per-topic CSV files + image directories, the default) or `hdf5` (a single `<output-dir>/unbag.h5` holding every topic). |
| `--handlers H [H ...]` | Handlers to enable (`csv`, `image`). Topics whose type is not claimed by an enabled handler are skipped. Default: all. |
| `--topics T [T ...]` | Only unbag these topics. Mutually exclusive with `--exclude-topics`. |
| `--exclude-topics T ...` | Unbag all topics except these. Mutually exclusive with `--topics`. |
| `--overwrite` | Delete previously unbagged output for the selected topics before writing. Without it, an interrupted `csv` run resumes; the `hdf5` backend rewrites the whole file, so it requires `--overwrite` to replace an existing `unbag.h5`. |
| `--jobs N` | Worker threads in the shared image-decoding pool (default: number of hardware threads). Each CSV topic additionally runs on its own consumer thread. |
| `--opencv-threads N` | Threads OpenCV uses internally per image decode (default `1`). We already parallelize across images via `--jobs`, so `1` avoids oversubscribing cores; `0` lets OpenCV choose. Tune against `--jobs` for your machine. |
| `--storage-id ID` | Storage plugin override. Default: inferred from `metadata.yaml`; if that file is missing the bag is reindexed first and the id read back from the rebuilt metadata, then falling back to the installed default plugin (`mcap` on a stock Jazzy install). |
| `-v, --verbose` | Log the handler chosen for each topic, topics skipped, and per-topic success/failure counts in the end-of-run summary. |

**`csv` handler options**

| Flag | Description |
| --- | --- |
| `--csv-batch-size N` | Rows buffered in memory before flushing to disk (default 1000). |

**`image` handler options**

| Flag | Description |
| --- | --- |
| `--image-encoding ENC` | Target *color* encoding for decoded images (default `bgr8`; e.g. `rgb8`, `mono8`). |
| `--image-format FMT` | Output *file* format: `keep` \| `png` \| `jpg` \| `tiff` (default `keep`). `keep` preserves the source container (a `CompressedImage` keeps its compression, a raw `Image` is written as PNG); a specific format applies to every image topic (`png` avoids a lossy re-encode of compressed Bayer topics). CSV backend only — the HDF5 backend stores decoded frames. |

**`hdf5` backend options** (only with `--format hdf5`)

| Flag | Description |
| --- | --- |
| `--hdf5-gzip-level N` | gzip/deflate level `0`–`9` for the image and column datasets (default `4`; `0` disables compression). Higher is smaller but slower to write. |

The storage plugin is inferred from the bag's `metadata.yaml`; if that file is
missing or corrupted it is rebuilt with the rosbag2 reindexer and the storage id
read back from the result. `--storage-id` is only needed when that inference is
unavailable or wrong (and seeds the reindexer when no metadata exists). There is
no `--serialization-format` flag: the reader determines each message's input
serialization from the per-topic metadata, and the converter always targets CDR
for the deserializers, so the format is not something the user picks.

After the write pass, `unbag` prints a one-line summary of how many messages were
successfully unbagged versus dropped (a decode/flatten/write failure), with a
per-topic breakdown for any topic that had failures (and for every topic under
`--verbose`). Failures are usually all-or-nothing per topic, so a partial count
is a useful signal that something is off with that topic.

## Architecture

Each message type is routed to a **handler**. Handlers live in `handlers/` and
derive from `MessageHandler` (`handlers/handler.hpp`); a small dispatch in
`unbagger.cpp` picks, per topic, the first handler that claims the message type
(the image handler is checked before the CSV catch-all). Adding support for a
new output type is a matter of adding a handler and registering it.

```text
include/tabletop_unbag/
├── options.hpp            # UnbagOptions, TopicInfo, OutputFormat
├── progress_bar.hpp       # tqdm-style progress bar (header-only, no deps)
├── concurrent_queue.hpp   # bounded blocking queue (reader <-> workers)
├── flatten.hpp            # MessageFlattener: generic message -> flat columns
├── image_decode.hpp       # shared cv_bridge / Bayer decode helpers
├── hdf5_writer.hpp        # Hdf5Writer: the single-file HDF5 sink (pimpl)
├── unbagger.hpp           # unbag(): metadata inference + dispatch + the passes
└── handlers/
    ├── handler.hpp        # MessageHandler base (the handler lifecycle)
    ├── csv_handler.hpp
    ├── image_handler.hpp
    └── hdf5_handlers.hpp  # Hdf5CsvHandler + Hdf5ImageHandler
src/
├── main.cpp               # CLI
├── unbagger.cpp           # orchestration + the threaded pipeline
├── flatten.cpp            # flattening + pandas-compatible value formatting
├── image_decode.cpp       # cv_bridge / Bayer decode (shared by both backends)
├── hdf5_writer.cpp        # HDF5 C API: lazy columns, extendable datasets, gzip
└── handlers/
    ├── csv_handler.cpp    # CSV rows (flatten) + resume / torn-line repair
    ├── image_handler.cpp  # per-frame image files (thread-safe, atomic writes)
    └── hdf5_handlers.cpp  # append rows / decoded frames into the shared Hdf5Writer
```

Both backends share the same `MessageFlattener` (`flatten.*`) and image
decode/debayer code (`image_decode.*`); only the sink differs (per-topic files
vs. the single `Hdf5Writer`).

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
   whole pass is skipped. This pass also pushes a topic filter into the storage
   reader so only the CSV topics are read: the bulk of a bag is usually image
   data, and there is no reason to read (or decode) it just to learn columns.
2. **`begin_write()`** — handlers apply the overwrite/resume policy.
3. **Write pass** — `write()` is called once per message; handlers buffer and
   flush in batches (the CSV handler's batch is sized by `--csv-batch-size`), so
   memory stays bounded no matter how many messages a topic has.
4. **`finish()`** — flush and close.

**Interruption / resume.** Because output is flushed incrementally, an
interrupted run leaves valid partial output. Re-running (without `--overwrite`)
picks up where it left off: the CSV handler counts the rows already on disk
(repairing a torn final line) and skips that many messages before appending; the
image handler skips timestamps whose file already exists. `--overwrite` instead
deletes the selected topics' prior output first — the CSV handler removes its
file, and the image handler clears the topic's directory so a re-run with a
different `--image-encoding` cannot leave a mix of old and new formats behind.

### Parallelism and back-pressure

A bag is read once per pass by a single reader thread (rosbag2's reader is
sequential), but the per-message work is fanned out:

* **Across topics.** Each CSV topic gets its own bounded queue and consumer
  thread, so different topics' files are flattened and written concurrently.
  Rows stay in bag order because a topic's messages flow through one FIFO queue
  to one consumer.
* **Across messages, for images.** Image messages from every image topic go to a
  single shared queue drained by a pool of `--jobs` worker threads. Each image is
  decoded, demosaiced and encoded independently and written to its own file, so
  this scales across cores — and image transcoding is by far the most expensive
  part of unbagging a camera-heavy bag. (CSV rows are *not* parallelized within a
  topic: they share one file and must stay ordered, and CSV work is cheap enough
  that one thread per topic already keeps up with the reader.)

The queues are **bounded**, which does double duty: it caps memory (the reader
cannot race ahead and buffer the whole bag), and it makes the reader
self-throttle to the slowest consumer — when the image pool is saturated the
reader simply blocks on `push()` instead of piling up work.

Shutdown is the same on success and on Ctrl-C. A `SIGINT`/`SIGTERM` handler sets
a stop flag; the reader stops, closes the queues, and the workers drain whatever
is already queued and exit before the handlers flush and close. Combined with
atomic image writes (each image is encoded to memory, written to a temp file,
then `rename`d into place) and the CSV torn-line repair, an interrupted run never
corrupts an output file and always resumes cleanly. The input bag is opened
read-only and is never written to.

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

### HDF5 backend

With `--format hdf5` the per-topic file handlers are swapped for handlers that
write into one shared `Hdf5Writer` (`hdf5_writer.cpp`, pimpl so `<hdf5.h>` stays
out of the headers). The flattening and the image decode/debayer are *identical*
to the CSV/JPEG backend — only the destination changes:

* **Layout.** One HDF5 group per topic, named like the CSV files
  (`/eyelink/sample` → `eyelink_sample`) and tagged with `topic` and `ros_type`
  string attributes. Non-image topics get one 1-D dataset per flattened column,
  each in its native HDF5 type (`bool`→`uint8`, `int64`/`uint64`,
  `float`/`double`, variable-length UTF-8 `string`) plus a `bag_time_ns`
  dataset. Image topics get one chunked, gzip-compressed 4-D `(N,H,W,C)`
  `uint8` dataset of the decoded frames plus `stamp_sec` / `stamp_nanosec`.
* **No preprocess pass.** The CSV backend reads the whole bag once just to learn
  each topic's column union (sequence lengths vary per message). The HDF5 writer
  instead creates a column dataset the first time a column is seen and back-fills
  earlier rows via `H5Dset_extent` (numeric columns fill with NaN/0, strings with
  `""`), so the union is discovered *during* the single write pass. Reading the
  bag once rather than twice is the whole point — the cold first-run read is
  bandwidth-bound, so halving the passes is the only lever that helps (see
  [docs/performance.md](docs/performance.md)).
* **Thread-safety.** The serial HDF5 C library is not thread-safe, so every HDF5
  call funnels through a single mutex in `Hdf5Writer`. The expensive per-message
  work (flatten, decode, debayer) still happens in the handlers on the shared
  pool *before* calling in; only the cheap append serializes. Image frames are
  assigned a bag-order row index single-threaded on the reader thread
  (`note_for_write`), so frames decoded out of order on the pool still land at
  the correct row (any gap is left as fill until written).
* **Overwrite, not resume.** The HDF5 file is written whole; it is not resumable
  like the CSV output. Re-running over an existing `unbag.h5` requires
  `--overwrite`.

CMake links the **serial** HDF5 (`HDF5_PREFER_PARALLEL OFF`): both the serial and
the OpenMPI dev packages are typically installed, and this single-process tool
must not accidentally pull in the MPI build.

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
