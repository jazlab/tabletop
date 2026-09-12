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
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef TABLETOP_UNBAG__HANDLERS__VIDEO_HANDLER_HPP_
#define TABLETOP_UNBAG__HANDLERS__VIDEO_HANDLER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

#include <opencv2/core/mat.hpp>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/frame.h>
#include <libswscale/swscale.h>
}

#include "rcutils/types/uint8_array.h"

#include "tabletop_unbag/handlers/handler.hpp"
#include "tabletop_unbag/options.hpp"

namespace tabletop_unbag
{

/// Convert an image topic directly into bounded, resumable Matroska segments.
/// Frames stay in bag order on a dedicated per-topic consumer thread. MKV PTS
/// values use a microsecond time base; the sidecar retains the source header
/// and bag receive timestamps at their exact nanosecond resolution.
class VideoHandler : public MessageHandler
{
public:
  static std::string handler_name()
  {
    return "video";
  }

  static bool handles(const std::string& ros_type)
  {
    return ros_type == "sensor_msgs/msg/Image" || ros_type == "sensor_msgs/msg/CompressedImage";
  }

  VideoHandler(TopicInfo topic, const std::string& output_dir, const UnbagOptions& options);
  ~VideoHandler() override;

  void begin_write() override;
  void write(const rcutils_uint8_array_t& data, int64_t bag_time_ns, uint64_t write_index) override;
  void finish() override;

  HandlerStats stats() const override
  {
    return { succeeded_, failed_ };
  }

private:
  cv::Mat decode(const rcutils_uint8_array_t& data, int64_t& header_stamp_ns);
  void discover_resume_point();
  void open_segment(const cv::Mat& image, int64_t header_stamp_ns);
  void encode_frame(const cv::Mat& image, int64_t header_stamp_ns, int64_t bag_time_ns);
  void drain_encoder(bool final);
  void close_segment(bool publish);
  void cleanup_ffmpeg();
  bool segment_due(int64_t header_stamp_ns) const;
  std::filesystem::path segment_path(std::size_t index, const std::string& suffix) const;

  TopicInfo topic_;
  std::filesystem::path video_dir_;
  bool overwrite_;
  std::string codec_name_;
  int quality_;
  int nominal_fps_;
  int64_t segment_duration_ns_;

  std::size_t segment_index_ = 0;
  std::size_t skip_frames_ = 0;
  std::size_t segment_frame_index_ = 0;
  int64_t segment_start_stamp_ns_ = 0;
  int64_t previous_stamp_ns_ = 0;
  int64_t previous_pts_us_ = -1;
  bool segment_open_ = false;
  bool fatal_ = false;
  bool warned_ = false;

  std::filesystem::path video_part_path_;
  std::filesystem::path video_final_path_;
  std::filesystem::path sidecar_part_path_;
  std::filesystem::path sidecar_final_path_;
  std::ofstream sidecar_;

  AVFormatContext* format_context_ = nullptr;
  AVCodecContext* codec_context_ = nullptr;
  AVStream* stream_ = nullptr;
  AVFrame* frame_ = nullptr;
  AVPacket* packet_ = nullptr;
  SwsContext* sws_context_ = nullptr;

  std::size_t succeeded_ = 0;
  std::size_t failed_ = 0;
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__HANDLERS__VIDEO_HANDLER_HPP_
