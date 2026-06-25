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

#ifndef TABLETOP_UNBAG__HANDLERS__IMAGE_HANDLER_HPP_
#define TABLETOP_UNBAG__HANDLERS__IMAGE_HANDLER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <string>

#include "rcutils/types/uint8_array.h"

#include "tabletop_unbag/handlers/handler.hpp"
#include "tabletop_unbag/options.hpp"

namespace tabletop_unbag
{

/// Decodes the image messages of one topic and writes them to disk as image
/// files, one per message, in a per-topic subdirectory of the output.
///
/// Files are named "<sec>_<nanosec>.<ext>". Color conversion goes through
/// cv_bridge for uncompressed Image messages and for non-Bayer CompressedImage
/// messages (so any target encoding cv_bridge supports works, not just bgr8).
/// CompressedImage payloads that carry a Bayer mosaic are decoded with a
/// Bayer-aware path, because cv_bridge infers a compressed image's encoding
/// from its channel count and would mistake a single-channel mosaic for mono8.
///
/// Each image is independent, so there is no preprocess pass. Resume is by file
/// existence (an already-saved timestamp is skipped); --overwrite deletes the
/// topic's image directory up front so a re-run cannot leave a mix of old and
/// new output formats behind.
///
/// write() is thread-safe: each call writes a distinct, timestamp-named file
/// (encoded to a buffer, written to a temp file, then atomically renamed into
/// place), so a pool of workers can decode images for the same topic in
/// parallel and an interrupted write never leaves a half-written final file.
class ImageHandler : public MessageHandler
{
public:
  static std::string handler_name()
  {
    return "image";
  }

  static bool handles(const std::string& ros_type)
  {
    return ros_type == "sensor_msgs/msg/Image" || ros_type == "sensor_msgs/msg/CompressedImage";
  }

  ImageHandler(TopicInfo topic, const std::string& output_dir, const UnbagOptions& options);

  /// Images are independent files, so the unbagger may decode them on a shared
  /// worker pool. write() is thread-safe to support that.
  bool parallelizable_per_message() const override
  {
    return true;
  }

  void begin_write() override;
  void write(const rcutils_uint8_array_t& data, int64_t bag_time_ns) override;

  /// Decode/write success vs failure counts for this topic. Read after the
  /// write pass joins its workers; the counters are atomics because write()
  /// runs concurrently on the shared image pool.
  HandlerStats stats() const override
  {
    return { succeeded_.load(), failed_.load() };
  }

private:
  void ensure_dir();

  TopicInfo topic_;
  std::filesystem::path image_dir_;
  bool overwrite_;
  std::string image_encoding_;
  std::once_flag dir_once_;
  std::atomic<bool> decode_warned_{ false };
  // write() runs on the shared pool, so these are updated concurrently. A
  // message counts as "succeeded" once its file is on disk (or was already
  // present from a prior run); a decode or write failure counts as "failed".
  std::atomic<std::size_t> succeeded_{ 0 };
  std::atomic<std::size_t> failed_{ 0 };
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__HANDLERS__IMAGE_HANDLER_HPP_
