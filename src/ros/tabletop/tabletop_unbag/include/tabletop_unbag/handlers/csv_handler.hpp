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
  void prepare() override;
  void preprocess(const rcutils_uint8_array_t& data, int64_t bag_time_ns) override;
  void begin_write() override;
  void write(const rcutils_uint8_array_t& data, int64_t bag_time_ns) override;
  void finish() override;

  /// Rows flattened+written vs rows dropped (a message that failed to flatten).
  /// write() runs on this topic's single consumer thread, so plain counters are
  /// safe; stats() is read after that thread has joined.
  HandlerStats stats() const override
  {
    return { succeeded_, failed_ };
  }

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

  // Write-pass tally (single-threaded per topic, so no atomics needed). A row
  // counts as succeeded once it is flattened and buffered (or skipped on
  // resume because it is already on disk); a message that fails to flatten is
  // counted as failed and dropped rather than aborting the whole run.
  std::size_t succeeded_ = 0;
  std::size_t failed_ = 0;
  bool flatten_warned_ = false;  ///< warn once per topic on flatten failure

  // Cached introspection type support (kept alive by the shared library).
  std::shared_ptr<rcpputils::SharedLibrary> type_support_library_;
  const rosidl_typesupport_introspection_cpp::MessageMembers* members_ = nullptr;
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__HANDLERS__CSV_HANDLER_HPP_
