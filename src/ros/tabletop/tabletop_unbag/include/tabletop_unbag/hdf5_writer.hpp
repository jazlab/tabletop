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

#ifndef TABLETOP_UNBAG__HDF5_WRITER_HPP_
#define TABLETOP_UNBAG__HDF5_WRITER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include "tabletop_unbag/flatten.hpp"

namespace tabletop_unbag
{

/// Writes an entire unbag run into a single HDF5 file.
///
/// One group per topic (named like the CSV files: `/eyelink/sample` ->
/// `/eyelink_sample`). Non-image topics store one 1-D dataset per flattened
/// column (the same flatten as the CSV backend), created lazily the first time a
/// column appears and back-filled for earlier rows -- so no preprocess column-
/// discovery pass is needed. Image topics store one chunked, gzip-compressed
/// 4-D `(N, H, W, C)` dataset of the decoded/debayered frames plus `stamp_sec`
/// and `stamp_nanosec` datasets.
///
/// Thread-safety: the serial HDF5 C library is not thread-safe, so **every**
/// HDF5 call funnels through one internal mutex. The expensive per-message work
/// (flattening, image decode/debayer) happens in the handlers *before* calling
/// in here, so it still parallelizes across the worker pool; only the cheap
/// append serializes. All public methods are safe to call from many threads.
///
/// One Hdf5Writer is shared by every per-topic handler in an HDF5 run. It is
/// implemented with the pimpl idiom so that <hdf5.h> stays out of this header.
class Hdf5Writer
{
public:
  /// Create (truncating) the HDF5 file at `path`. `gzip_level` is the deflate
  /// level for image datasets and string columns (0 disables). `batch_size` is
  /// how many table rows are buffered per topic before a block is written.
  Hdf5Writer(const std::string& path, int gzip_level, std::size_t batch_size);
  ~Hdf5Writer();

  Hdf5Writer(const Hdf5Writer&) = delete;
  Hdf5Writer& operator=(const Hdf5Writer&) = delete;

  /// Append one flattened message row to a non-image topic's group. `bag_time_ns`
  /// becomes the `bag_time_ns` column; each FlatColumn becomes/extends its own
  /// per-column dataset. Rows are buffered and flushed in `batch_size` blocks.
  void append_row(const std::string& topic, const std::string& ros_type, int64_t bag_time_ns, const FlatRow& row);

  /// Append one decoded frame to an image topic's stack at row `frame_index`
  /// (the message's bag-order index, assigned on the reader thread). Frames may
  /// arrive out of order; unwritten rows are left as fill (zeros). `image` must
  /// be 8-bit (CV_8U); its channel count sets C.
  void append_image(const std::string& topic, const std::string& ros_type, uint64_t frame_index, int32_t sec,
                    uint32_t nanosec, const cv::Mat& image);

  /// Flush a table topic's remaining buffered rows (called from the handler's
  /// finish()). Image topics need no flush. Safe to call for any topic.
  void finish_topic(const std::string& topic);

  /// Flush and close every group/dataset and the file. Idempotent.
  void close();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__HDF5_WRITER_HPP_
