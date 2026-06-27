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

#ifndef TABLETOP_UNBAG__HANDLERS__HDF5_HANDLERS_HPP_
#define TABLETOP_UNBAG__HANDLERS__HDF5_HANDLERS_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include "rcutils/types/uint8_array.h"

#include "tabletop_unbag/flatten.hpp"
#include "tabletop_unbag/handlers/handler.hpp"
#include "tabletop_unbag/hdf5_writer.hpp"
#include "tabletop_unbag/options.hpp"

namespace tabletop_unbag
{

/// HDF5 counterpart of CsvHandler: flattens a non-image topic and appends each
/// message as a row to the shared HDF5 file (one dataset per flattened column).
///
/// Unlike the CSV handler it does **not** need a preprocess pass: the writer
/// creates a column dataset the first time a column is seen and back-fills
/// earlier rows, so the column union is discovered during the single write pass.
/// It runs on its topic's own consumer thread (not the shared pool) so its rows
/// stay in bag order; the writer serializes the actual HDF5 calls internally.
class Hdf5CsvHandler : public MessageHandler
{
public:
  static std::string handler_name()
  {
    return "csv";
  }
  static bool handles(const std::string& ros_type)
  {
    (void)ros_type;
    return true;  // catch-all, checked after the image handler
  }

  Hdf5CsvHandler(TopicInfo topic, Hdf5Writer& writer, const UnbagOptions& options);

  void prepare() override;
  void write(const rcutils_uint8_array_t& data, int64_t bag_time_ns, uint64_t write_index) override;
  void finish() override;

  HandlerStats stats() const override
  {
    return { succeeded_, failed_ };
  }

private:
  TopicInfo topic_;
  Hdf5Writer& writer_;
  MessageFlattener flattener_;
  std::size_t succeeded_ = 0;
  std::size_t failed_ = 0;
  bool flatten_warned_ = false;
};

/// HDF5 counterpart of ImageHandler: decodes/debayers each frame (on the shared
/// worker pool) and appends it to the topic's stacked (N,H,W,C) dataset.
///
/// note_for_write() hands out a monotonic per-topic frame index in bag order on
/// the reader thread, so frames decoded out of order on the pool still land at
/// the right row. The decode is identical to the JPEG backend (shared
/// image_decode); only the destination differs.
class Hdf5ImageHandler : public MessageHandler
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

  Hdf5ImageHandler(TopicInfo topic, Hdf5Writer& writer, const UnbagOptions& options);

  bool parallelizable_per_message() const override
  {
    return true;
  }

  /// Assign this frame its bag-order row index (reader thread, single-threaded).
  uint64_t note_for_write(const rcutils_uint8_array_t& data, int64_t bag_time_ns) override;

  void write(const rcutils_uint8_array_t& data, int64_t bag_time_ns, uint64_t write_index) override;

  HandlerStats stats() const override
  {
    return { succeeded_.load(), failed_.load() };
  }

private:
  TopicInfo topic_;
  Hdf5Writer& writer_;
  std::string image_encoding_;
  uint64_t next_index_ = 0;  // touched only on the reader thread
  std::atomic<std::size_t> succeeded_{ 0 };
  std::atomic<std::size_t> failed_{ 0 };
  std::atomic<bool> decode_warned_{ false };
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__HANDLERS__HDF5_HANDLERS_HPP_
