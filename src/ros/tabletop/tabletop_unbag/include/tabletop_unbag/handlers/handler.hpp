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

#ifndef TABLETOP_UNBAG__HANDLERS__HANDLER_HPP_
#define TABLETOP_UNBAG__HANDLERS__HANDLER_HPP_

#include <algorithm>
#include <cstdint>
#include <string>

#include "rcutils/types/uint8_array.h"

namespace tabletop_unbag
{

/// Convert a ROS topic name to a filename-safe basename: strip leading '/' and
/// replace remaining '/' with '_' (e.g. "/eyelink/sample" -> "eyelink_sample").
inline std::string topic_to_basename(const std::string& topic)
{
  const auto start = topic.find_first_not_of('/');
  std::string basename = start == std::string::npos ? "" : topic.substr(start);
  std::replace(basename.begin(), basename.end(), '/', '_');
  return basename;
}

/// Abstract base for everything that turns the messages of one topic into
/// on-disk output (a CSV file, a directory of images, ...).
///
/// One handler instance is created per topic. The unbag pipeline drives every
/// handler through the same lifecycle, which is what lets the orchestrator stay
/// type-agnostic:
///
///   1. preprocess pass  -- preprocess() is called once per message (only if
///      needs_preprocess() is true), to accumulate whatever metadata the write
///      pass needs that cannot be known up front. For CSV this is the union of
///      flattened columns (an unbounded sequence makes the column set
///      message-dependent); for images it is unnecessary, so the default is a
///      no-op.
///   2. begin_write()    -- called once after the preprocess pass, before any
///      message is written. Handlers apply the overwrite/resume policy here.
///   3. write pass       -- write() is called once per message. Handlers buffer
///      and flush in batches to bound memory and survive interruptions.
///   4. finish()         -- called once after the write pass to flush and close.
class MessageHandler
{
public:
  virtual ~MessageHandler() = default;

  /// Whether this handler needs the preprocess pass (an extra read over the
  /// bag). When no enabled handler needs it, the pass is skipped entirely.
  virtual bool needs_preprocess() const
  {
    return false;
  }

  /// Phase 1: observe a message during the preprocess pass.
  virtual void preprocess(const rcutils_uint8_array_t& data, int64_t bag_time_ns)
  {
    (void)data;
    (void)bag_time_ns;
  }

  /// Apply the overwrite/resume policy and prepare outputs for writing.
  virtual void begin_write()
  {
  }

  /// Phase 2: write one message (typically buffered, flushed in batches).
  virtual void write(const rcutils_uint8_array_t& data, int64_t bag_time_ns) = 0;

  /// Flush any buffered data and close outputs.
  virtual void finish()
  {
  }
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__HANDLERS__HANDLER_HPP_
