// Copyright 2026 Jazlab
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef TABLETOP_UNBAG__OPTIONS_HPP_
#define TABLETOP_UNBAG__OPTIONS_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace tabletop_unbag
{

/// A topic's name and message type, as read from the bag metadata.
struct TopicInfo
{
  std::string name;  ///< e.g. "/joint_states"
  std::string type;  ///< e.g. "sensor_msgs/msg/JointState"
};

/// Options specific to the CSV handler. Surfaced on the command line under the
/// `--csv-*` namespace (see main.cpp) and reflected here so handler-specific
/// settings are grouped together rather than mixed into the top-level options.
struct CsvOptions
{
  /// Number of rows the CSV handler buffers in memory before flushing to disk.
  /// Bounds peak memory and how much work an interruption can lose.
  std::size_t batch_size = 1000;

  /// Optional directory for temporary binary row spools. Keeping this on a
  /// fast filesystem separate from a large source/output HDD avoids competing
  /// reads and writes during the single MCAP pass. Empty uses an output-local
  /// hidden directory.
  std::string spool_dir;
};

/// Options specific to the image handler. Surfaced under the `--image-*`
/// namespace (see main.cpp).
struct ImageOptions
{
  /// Target OpenCV/ROS *color* encoding for decoded images (e.g. "bgr8",
  /// "rgb8", "mono8").
  std::string encoding = "bgr8";

  /// Output *file* format for saved images:
  ///   "keep" - preserve the source container: a CompressedImage keeps its own
  ///            compression (.jpg/.png/.tiff) and a raw Image is written as PNG
  ///            (lossless). This is the default.
  ///   "png"  - write every image as PNG (lossless; good for avoiding a
  ///            lossy-on-lossy re-encode of compressed Bayer topics).
  ///   "jpg"/"jpeg" - write every image as JPEG.
  ///   "tiff" - write every image as TIFF.
  /// A specific format applies to all image topics regardless of their source.
  std::string format = "keep";
};

/// Options for direct camera-video export.
struct VideoOptions
{
  /// FFmpeg encoder name. h264_nvenc uses the NVIDIA hardware encoder; libx264
  /// is a portable CPU fallback.
  std::string codec = "h264_nvenc";

  /// Constant-quality value passed to NVENC (lower is higher quality).
  int quality = 28;

  /// Nominal frame rate recorded in stream metadata. Per-frame timing remains
  /// timestamp-driven, and exact nanoseconds are stored in the sidecar CSV.
  int nominal_fps = 120;

  /// Close and atomically publish one MKV segment after this many seconds of
  /// camera time. Segmenting makes long exports safely resumable.
  int segment_seconds = 300;
};

/// Options controlling an unbag run. Populated from the command line (main.cpp)
/// and passed down to the handlers. Run-wide options live at the top level;
/// options that only affect one handler live in the per-handler sub-structs
/// (`csv`, `image`) so the grouping is explicit in code and on the CLI.
struct UnbagOptions
{
  /// Whitelist of topics to unbag. Mutually exclusive with exclude_topics;
  /// std::nullopt means "every topic".
  std::optional<std::vector<std::string>> topics;

  /// Topics to skip. Mutually exclusive with topics.
  std::optional<std::vector<std::string>> exclude_topics;

  /// Names of the handlers to enable (e.g. {"csv", "image"}). Empty preserves
  /// the legacy default: CSV for ordinary messages and files for image topics.
  std::vector<std::string> handlers;

  /// If true, previously unbagged output for the selected topics is deleted
  /// before writing. If false (the default), unbagging resumes where a prior
  /// interrupted run left off.
  bool overwrite = false;

  /// Number of worker threads in the shared pool that decodes images (and any
  /// other per-message-parallel handler). 0 means "auto" (hardware
  /// concurrency). Each CSV topic additionally gets its own consumer thread.
  std::size_t jobs = 0;

  /// Number of threads OpenCV may use internally for a single image decode
  /// (cv::setNumThreads). The default of 1 keeps OpenCV single-threaded because
  /// we already parallelize across images via `jobs`, so letting OpenCV spawn
  /// its own threads per decode would oversubscribe the cores. 0 lets OpenCV
  /// choose. The optimal `opencv_threads` vs `jobs` split is machine- and
  /// bag-dependent and is left for the user to tune empirically.
  int opencv_threads = 1;

  /// Override for the storage plugin id (e.g. "mcap"). std::nullopt means
  /// "infer from the bag metadata" (reindexing the bag first if metadata.yaml
  /// is missing, then falling back to the installed default storage plugin).
  std::optional<std::string> storage_id;

  /// Emit extra per-topic logging.
  bool verbose = false;

  /// Per-handler option groups.
  CsvOptions csv;
  ImageOptions image;
  VideoOptions video;

  /// Refuse to start when the destination filesystem has less free space than
  /// this. Zero disables the check.
  std::size_t minimum_free_gb = 20;
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__OPTIONS_HPP_
