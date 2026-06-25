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

#ifndef TABLETOP_UNBAG__HANDLERS__HANDLER_HPP_
#define TABLETOP_UNBAG__HANDLERS__HANDLER_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>

#include "rcutils/types/uint8_array.h"

namespace tabletop_unbag
{

/// Per-handler tally of how many of a topic's messages were turned into output
/// versus how many were dropped (a decode/flatten/write failure). The
/// orchestrator collects these after the write pass to print a per-topic
/// success/failure summary -- useful because a failure is usually all-or-nothing
/// for a topic, so a partial count flags something worth investigating.
struct HandlerStats
{
  std::size_t succeeded = 0;
  std::size_t failed = 0;
};

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

  /// Whether this handler's per-message work is independent and therefore safe
  /// to run on a shared pool of worker threads (true), or whether each message
  /// must be processed in order on a single dedicated thread (false).
  ///
  /// The image handler returns true: every image is decoded and written to its
  /// own file, so messages have no ordering or shared-state coupling. The CSV
  /// handler returns false: all of a topic's rows go to one file and must keep
  /// bag order, so the orchestrator gives it one consumer thread.
  ///
  /// Contract: a handler that returns true MUST make write() thread-safe, since
  /// the pool calls it concurrently for the same handler instance.
  virtual bool parallelizable_per_message() const
  {
    return false;
  }

  /// One-time setup run on the main thread before any worker thread starts.
  /// Use it to do work that is unsafe to trigger concurrently from several
  /// threads (e.g. loading a type-support shared library on first use).
  virtual void prepare()
  {
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

  /// Called by the reader thread in bag order, once per message in the write
  /// pass, immediately before the message is dispatched to write(). It returns
  /// an opaque per-message index that is handed back to write() unchanged.
  ///
  /// This exists because a parallelizable handler's write() runs on a shared
  /// pool out of bag order, so it cannot derive anything that depends on order.
  /// The image handler uses it to assign each message a per-timestamp
  /// "occurrence index" (in receive-time order) so that frames sharing a header
  /// stamp get distinct, order-preserving filenames. Handlers that need no such
  /// index leave the default, which returns 0.
  ///
  /// Runs single-threaded on the reader, so an implementation may keep
  /// unsynchronized state here; the returned value travels with the message and
  /// is the only thing write() sees.
  virtual uint64_t note_for_write(const rcutils_uint8_array_t& data, int64_t bag_time_ns)
  {
    (void)data;
    (void)bag_time_ns;
    return 0;
  }

  /// Phase 2: write one message (typically buffered, flushed in batches).
  /// `write_index` is the value note_for_write() returned for this message.
  virtual void write(const rcutils_uint8_array_t& data, int64_t bag_time_ns, uint64_t write_index) = 0;

  /// Flush any buffered data and close outputs.
  virtual void finish()
  {
  }

  /// Success/failure counts for this topic's write pass, read by the
  /// orchestrator once the pass has completed (so it may be queried from the
  /// reader thread after the workers have joined). The default reports no
  /// failures; handlers that can drop a message override this. Returns a
  /// snapshot by value -- the underlying counters may be atomics updated
  /// concurrently by a parallelized handler's write().
  virtual HandlerStats stats() const
  {
    return {};
  }

  /// Number of messages this handler wrote under a disambiguated name because
  /// they collided with an earlier message (e.g. images sharing a header
  /// stamp). 0 for handlers without such collisions. Reported in the run
  /// summary; read after the write pass has joined its workers.
  virtual std::size_t duplicate_count() const
  {
    return 0;
  }
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__HANDLERS__HANDLER_HPP_
