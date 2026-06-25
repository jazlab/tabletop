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

/// Options controlling an unbag run. Populated from the command line (main.cpp)
/// and passed down to the handlers.
struct UnbagOptions
{
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

  /// Number of messages a handler buffers in memory before flushing to disk.
  /// Bounds peak memory and how much work an interruption can lose.
  std::size_t batch_size = 1000;

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

  /// Target OpenCV/ROS encoding for saved images (e.g. "bgr8", "rgb8",
  /// "mono8"). Only used by the image handler.
  std::string image_encoding = "bgr8";

  /// Override for the storage plugin id (e.g. "mcap"). std::nullopt means
  /// "infer from the bag metadata" (reindexing the bag first if metadata.yaml
  /// is missing, then falling back to the installed default storage plugin).
  std::optional<std::string> storage_id;

  /// Emit extra per-topic logging.
  bool verbose = false;
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__OPTIONS_HPP_
