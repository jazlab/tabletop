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

#ifndef TABLETOP_UNBAG__HANDLERS__IMAGE_HANDLER_HPP_
#define TABLETOP_UNBAG__HANDLERS__IMAGE_HANDLER_HPP_

#include <atomic>
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

private:
  void ensure_dir();

  TopicInfo topic_;
  std::filesystem::path image_dir_;
  bool overwrite_;
  std::string image_encoding_;
  std::once_flag dir_once_;
  std::atomic<bool> decode_warned_{ false };
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__HANDLERS__IMAGE_HANDLER_HPP_
