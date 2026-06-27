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

/// The output backend.
///   Csv  - per-topic CSV files and per-topic image directories (the default,
///          for backwards compatibility).
///   Hdf5 - a single HDF5 file holding every topic (one group per topic; one
///          dataset per flattened column; one stacked (N,H,W,C) dataset per
///          image topic). Same flattening and debayering, different container.
enum class OutputFormat
{
  Csv,
  Hdf5,
};

/// Options specific to the HDF5 backend. Surfaced under the `--hdf5-*`
/// namespace (see main.cpp).
struct Hdf5Options
{
  /// gzip/deflate level (0-9) for image datasets and the flattened columns.
  /// 0 disables compression. Higher is smaller but slower to write.
  int gzip_level = 4;
};

/// Options controlling an unbag run. Populated from the command line (main.cpp)
/// and passed down to the handlers. Run-wide options live at the top level;
/// options that only affect one handler live in the per-handler sub-structs
/// (`csv`, `image`, `hdf5`) so the grouping is explicit in code and on the CLI.
struct UnbagOptions
{
  /// Output backend (CSV/image files vs. a single HDF5 file). Default Csv.
  OutputFormat format = OutputFormat::Csv;

  /// Whitelist of topics to unbag. Mutually exclusive with exclude_topics;
  /// std::nullopt means "every topic".
  std::optional<std::vector<std::string>> topics;

  /// Topics to skip. Mutually exclusive with topics.
  std::optional<std::vector<std::string>> exclude_topics;

  /// Names of the handlers to enable (e.g. {"csv", "image"}). Empty means
  /// "every registered handler". A topic whose handler is not enabled is
  /// skipped.
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
  Hdf5Options hdf5;
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__OPTIONS_HPP_
