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

#ifndef TABLETOP_UNBAG__HANDLERS__CSV_HANDLER_HPP_
#define TABLETOP_UNBAG__HANDLERS__CSV_HANDLER_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "rcpputils/shared_library.hpp"
#include "rcutils/types/uint8_array.h"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "tabletop_unbag/handlers/handler.hpp"
#include "tabletop_unbag/options.hpp"

namespace tabletop_unbag
{

/// Flattens the messages of one topic into a CSV file, one row per message.
///
/// This is the C++ counterpart of the Python `gen_msg_values` / `rosbag_to_dfs`
/// path. Nested submessages become dot-separated columns (`pose.position.x`)
/// and array / sequence elements become bracket-indexed columns (`name[0]`).
/// The column set is the union of every field seen across the topic's messages,
/// in first-seen order with `bag_time_ns` first; shorter messages leave
/// trailing columns empty. Because a `sequence<>` field's length is only known
/// per message, that union is computed in a preprocess pass (needs_preprocess()
/// returns true) before any row is written.
///
/// The write pass streams rows in batches of `batch_size`, so memory stays
/// bounded regardless of how many messages a topic has. If a previous run was
/// interrupted, writing resumes after the rows already on disk (unless
/// --overwrite was given, which deletes the file first).
///
/// Note on fixed-size primitive arrays: the Python version expands variable
/// sequences (`position[0]`, `position[1]`) but, due to a quirk of its
/// `sequence<...>` type-string check, collapses fixed-size primitive arrays
/// (e.g. `CameraInfo.k`, a `float64[9]`) into a single column holding a numpy
/// string. This port expands fixed-size arrays the same way as sequences
/// (`k[0]..k[8]`), which is the more useful and presumably intended behavior.
class CsvHandler : public MessageHandler
{
public:
  /// The handler's registry name.
  static std::string handler_name()
  {
    return "csv";
  }

  /// The CSV handler is the catch-all: it can flatten any message type. The
  /// dispatch in unbagger.cpp checks the image handler first, so image topics
  /// never reach here.
  static bool handles(const std::string& ros_type)
  {
    (void)ros_type;
    return true;
  }

  CsvHandler(TopicInfo topic, const std::string& output_dir, const UnbagOptions& options);

  bool needs_preprocess() const override
  {
    return true;
  }
  void preprocess(const rcutils_uint8_array_t& data, int64_t bag_time_ns) override;
  void begin_write() override;
  void write(const rcutils_uint8_array_t& data, int64_t bag_time_ns) override;
  void finish() override;

private:
  /// Load (once) the introspection type support for this topic's type.
  void ensure_type_support_loaded();

  /// Record a column name, preserving first-seen order.
  void note_column(const std::string& column);

  /// The exact header line that this topic's columns produce (no newline).
  std::string header_line() const;

  /// Open the output file, applying the resume/append decision made in
  /// begin_write(). Called lazily on the first row so a topic with no rows
  /// produces no file (matching the Python behavior).
  void ensure_open();

  /// Append one buffered CSV line; flush the batch when it is full.
  void buffer_line(std::string line);
  void flush_batch();

  TopicInfo topic_;
  std::filesystem::path csv_path_;
  bool overwrite_;
  std::size_t batch_size_;

  // Column union, built during the preprocess pass.
  std::vector<std::string> columns_;
  std::unordered_set<std::string> seen_columns_;

  // Write-pass state.
  std::size_t skip_rows_ = 0;  ///< rows already on disk to skip (resume)
  bool append_ = false;        ///< open existing file for append vs. truncate
  bool opened_ = false;
  std::ofstream out_;
  std::vector<std::string> batch_;

  // Cached introspection type support (kept alive by the shared library).
  std::shared_ptr<rcpputils::SharedLibrary> type_support_library_;
  const rosidl_typesupport_introspection_cpp::MessageMembers* members_ = nullptr;
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__HANDLERS__CSV_HANDLER_HPP_
