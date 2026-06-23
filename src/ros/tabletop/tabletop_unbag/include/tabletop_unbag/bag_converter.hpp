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

#ifndef TABLETOP_UNBAG__BAG_CONVERTER_HPP_
#define TABLETOP_UNBAG__BAG_CONVERTER_HPP_

#include <optional>
#include <string>
#include <vector>

namespace tabletop_unbag
{

/// Options controlling a conversion run, mirroring the Python CLI flags.
struct ConvertOptions
{
  /// Whitelist of topics to include. std::nullopt means "all topics".
  std::optional<std::vector<std::string>> topics;
  /// Topics to exclude from processing.
  std::vector<std::string> exclude_topics;
  /// If true, decode image topics to image files (otherwise they are skipped).
  bool convert_images = false;
  /// If true, overwrite already-saved images instead of skipping them.
  bool force = false;
  /// If true, emit per-message debug logging.
  bool verbose = false;
};

/// Convert a ROS topic name to a filename-safe basename: strip leading '/' and
/// replace remaining '/' with '_' (e.g. "/eyelink/sample" -> "eyelink_sample").
std::string topic_to_basename(const std::string& topic);

/// Convert a single bag directory (containing an .mcap and metadata.yaml).
///
/// Each non-image topic is flattened into a table and, if `save_dir` is set,
/// written to "<save_dir>/<topic_basename>.csv". Image topics are exported as
/// image files when options.convert_images is set.
///
/// \return The names of the (non-image) topics that produced a table; used by
///   the session-level converter to detect topic collisions across bags.
/// \throws std::runtime_error on failure to open the bag.
std::vector<std::string> rosbag_to_csv(const std::string& bag_dir, const ConvertOptions& options,
                                       const std::optional<std::string>& save_dir);

/// Convert every bag under a session directory, writing CSVs into the session
/// directory. Bags are discovered as "<session_dir>/*/*.mcap".
///
/// \throws std::runtime_error if no .mcap files are found, or if the same topic
///   appears in more than one bag in the session (a collision).
void rosbag_session_to_csv(const std::string& session_dir, const ConvertOptions& options);

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__BAG_CONVERTER_HPP_
