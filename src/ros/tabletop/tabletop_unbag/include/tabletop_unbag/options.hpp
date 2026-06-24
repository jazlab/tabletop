// Copyright 2026 Jazlab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

  /// Target OpenCV/ROS encoding for saved images (e.g. "bgr8", "rgb8",
  /// "mono8"). Only used by the image handler.
  std::string image_encoding = "bgr8";

  /// Override for the storage plugin id (e.g. "mcap"). std::nullopt means
  /// "infer from the bag metadata".
  std::optional<std::string> storage_id;

  /// Override for the serialization format (e.g. "cdr"). std::nullopt means
  /// "infer from the bag metadata".
  std::optional<std::string> serialization_format;

  /// Emit extra per-topic logging.
  bool verbose = false;
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__OPTIONS_HPP_
