# Unbag performance: why the first run is slow

This note explains a specific, reproducible observation and what actually causes
it:

> The **preprocess** phase takes as long as (or longer than) the **write**
> phase, but **only on the first unbag of a bag**. Re-running — even with
> `--overwrite` — is much faster in both phases. Clearing the OS page cache and
> re-running reproduces the slow first run exactly.

Short version: on the first run the entire bag is read off cold storage
**twice** (once to discover CSV columns, once to write), and the bag's payload
is far larger than RAM-resident state but *smaller* than RAM, so every later run
is served from the page cache. The fix is not a faster reader — it is to stop
reading the cold bag twice.

## How unbag reads a bag today

`unbag` makes up to two streaming passes over the bag (`unbagger.cpp`):

1. **Preprocess pass** — only runs if some enabled handler needs it. The CSV
   handler does: a `sequence<>` field's length is per-message, so the full set
   of flattened columns (`name[0]`, `position[2]`, …) cannot be known without
   reading every message of every CSV topic first. This pass pushes a
   **storage filter** so only the CSV topics are read — the intent being to skip
   the (large) image payloads.
2. **Write pass** — reads every selected topic and writes the output.

Both passes open the bag with `rosbag2_cpp::Reader`, whose MCAP plugin returns
messages in **log-time order** using the message index
(`mcap::IndexedMessageReader`).

## The bag we measured

`mcap info` on a representative 8-minute session
(`/bags/big_session/bag/bag_0.mcap`):

```
messages:    1,713,013
chunks:      117,821        (uncompressed, avg ~0.9 MB, max 1.28 MB)
overlaps:    max concurrent: 9
size:        ~107 GB
```

33 topics. Six of them are `sensor_msgs/msg/CompressedImage` (the FLIR cameras,
~59k frames each) and account for essentially all of the 107 GB. The rest —
`joint_states` (497 Hz), `dynamic_joint_states` (497 Hz), 6× `camera_info` and
6× `meta` (~119 Hz each), `predicted_markers` / `teensy/sensor` (100 Hz),
`eyelink/sample_array` (18 Hz), … — are the CSV topics.

## Root cause 1: the CSV filter cannot skip any bytes

The chunks are tiny and tightly time-sliced: 117,821 chunks over 495 s is
**~238 chunks/s, so each chunk spans only ~4 ms**. In any 4 ms window the
recording contains several `joint_states`, several `camera_info`/`meta`, a
`predicted_markers`, and so on. So **every one of the 117k chunks contains CSV
topic messages.**

MCAP's read unit is the chunk. To return the small filtered messages the reader
must read (and, in a compressed bag, decompress) **every chunk that contains at
least one selected message** — which is all of them. The arithmetic:

- Useful CSV payload across the whole bag: ~1.36 M messages × a few hundred
  bytes ≈ **~270 MB**.
- Bytes the filtered preprocess pass must actually read to collect them:
  **~107 GB** — the whole bag.

The storage filter narrows *which messages are handed to the handlers*; it does
**not** narrow *which bytes are read off disk*. That is why the "skip the
images" optimization does not make the preprocess pass cheap, and why the
preprocess pass costs about the same as the full write pass.

## Root cause 2: it happens twice, on cold storage, and RAM hides it later

The first run reads the full ~107 GB for the preprocess pass and again for the
write pass — **~214 GB of reads** off whatever disk the bag lives on. The
machine has 188 GiB of RAM, so after the first pass the entire 107 GB bag is
resident in the page cache; the write pass and every subsequent run (including
`--overwrite`, which only changes what is *written*, not what is *read*) are
served from RAM. Drop the page cache and the slow first run returns — exactly
the reported behavior.

So the wall-clock cost is dominated by **cold reads of the bag**, and the first
run pays for two of them.

## Why the storage medium matters (and why "use mmap" is not the fix)

A natural hypothesis is that the indexed, log-time reader hurts on a spinning
disk: with 117k chunks whose time ranges overlap (max 9 concurrent),
reconstructing log-time order might jump around the file, which is fine on an
SSD but could be brutal on an HDD. We tested that hypothesis — and, as the HDD
table below shows, **it does not hold**: the chunks are laid out in roughly time
order, so the access pattern is nearly sequential regardless of read order.

We measured a full cold scan of this bag (read only, no decode/write; cache
evicted with `posix_fadvise(POSIX_FADV_DONTNEED)`), comparing the current
rosbag2 indexed reader against a plain file-order MCAP scan. Same bag, both
disks:

### On NVMe (Samsung 990 PRO; bag staged on `/tabletop`)

| reader                                   | time   | throughput   |
| ---------------------------------------- | ------ | ------------ |
| rosbag2 indexed (log-time order)         | 44.9 s | 2270 MiB/s   |
| mcap **mmap**, file order                | 89.7 s | 1137 MiB/s   |

Two lessons here:

1. On a fast SSD the indexed reader is **not** the bottleneck — the whole bag
   reads in 45 s. The reported ~5-minute preprocess (107 GB ÷ 5 min ≈
   **357 MB/s**) is the speed of a **spinning disk**, not this NVMe. In normal
   operation the bags live on the 24 TB rotational drive.
2. **`mmap` is slower, not faster**, for a linear scan of a file this large.
   Faulting 107 GB in 4 KB pages is ~26 M minor page faults; a buffered `pread`
   loop (what rosbag2 and mcap's `FileStreamReader` already do) moves the same
   bytes with far less per-byte overhead. So "reimplement the MCAP reader with a
   memory-mapped file" does **not** help on fast storage and is counter-
   productive there. (`bag_read_bench` carries a small `MmapReadable` purely as
   the baseline these numbers compare against; the takeaway is to prefer
   buffered file-order reads, which `mcap::FileStreamReader` already provides.)

### On the 24 TB rotational HDD (`sda1` → `/bags`, where bags really live)

Full cold scan of the same bag on the spinning disk, three readers:

| reader                                   | time     | throughput   |
| ---------------------------------------- | -------- | ------------ |
| mcap buffered, **file** order            | 421.9 s  | 241.7 MiB/s  |
| mcap buffered, **log-time** order        | 422.4 s  | 241.4 MiB/s  |
| rosbag2 indexed (current unbag reader)   | 421.7 s  | 241.8 MiB/s  |

**They are identical — within 0.2% of each other.** This is the key result, and
it *refutes* the seeking hypothesis: read order does not matter. The reason is
that the bag's ~0.9 MB chunks are written in roughly time order with only 9-way
time overlap, so even "log-time order" reads them almost front-to-back; the disk
is simply streaming at its sequential ceiling (~242 MiB/s) in every case. The
earlier observation that switching to file order "doesn't change much" was
exactly right: there is nothing for it to change.

So **the reader is not the lever** — not on NVMe (already 45 s) and not on the
HDD (bandwidth-bound regardless of order or implementation). A spinning disk
reads this 107 GB bag once in ~7 min; the slow first unbag reads it **twice**.

## The fix

**Stop reading the cold bag twice.** That is the only thing that moves the
first-run number, and it is storage-independent (a flat ~2x on the first run).

The preprocess pass exists only to learn the CSV column union. The new **HDF5
output backend removes it entirely**: HDF5 stores each flattened column as its
own extendable dataset, created lazily the first time a column is seen and
back-filled for earlier rows, so columns are discovered *during* the single
write pass. One cold read instead of two.

For the legacy CSV/JPEG backend the same single-pass idea is possible (stream
rows to a per-topic temporary keyed by column, then assemble the header at the
end), but it trades a second *bag* read for a second pass over the much smaller
*output*. Tracked as a follow-up; the HDF5 backend is the recommended path.

### What does *not* help (measured, not assumed)

* **A different read order** (`FileOrder` vs `LogTimeOrder`): a wash on both
  disks — see the tables above.
* **A reimplemented mmap reader**: *slower* than buffered reads on NVMe (page-
  fault overhead over a 100+ GB scan) and no better on the HDD (bandwidth-bound
  either way). `mcap::McapReader` already does buffered file-order reads via
  `FileStreamReader`; there is nothing to gain by replacing it.
* **The CSV topic filter**: cannot reduce bytes read (it reads ~the whole file
  to extract the interleaved small messages), as shown above.

## Reproducing / measuring on your own hardware

The `bag_read_bench` tool (built with this package) isolates the storage-read
cost — no decoding, no writing — so you can measure the effect on the disk your
bags actually live on:

```bash
# Cold (page cache evicted before each run), current reader vs file-order scan:
ros2 run tabletop_unbag bag_read_bench /path/to/bag

# Just the file-order vs log-time comparison with the buffered reader:
ros2 run tabletop_unbag bag_read_bench /path/to/bag --reader mcap --io buffered --order file
ros2 run tabletop_unbag bag_read_bench /path/to/bag --reader mcap --io buffered --order logtime

# Warm (page-cache resident) for comparison:
ros2 run tabletop_unbag bag_read_bench /path/to/bag --no-evict

# Limit to a sample of the first N messages for a quick check:
ros2 run tabletop_unbag bag_read_bench /path/to/bag --max-messages 200000
```

Cache eviction uses `posix_fadvise(POSIX_FADV_DONTNEED)`, which needs no root
(unlike `echo 3 > /proc/sys/vm/drop_caches`).
