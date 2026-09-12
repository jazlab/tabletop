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

#include "tabletop_unbag/handlers/video_handler.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

extern "C"
{
#include <libavutil/error.h>
#include <libavutil/opt.h>
}

namespace fs = std::filesystem;
namespace enc = sensor_msgs::image_encodings;

namespace tabletop_unbag
{

namespace
{

std::string to_lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

std::string strip(const std::string& value)
{
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos)
  {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

std::tuple<std::string, std::string, std::string> parse_compressed_format(const std::string& format)
{
  const auto semicolon = format.find(';');
  if (semicolon == std::string::npos)
  {
    return { strip(format), "", "" };
  }
  const std::string original = strip(format.substr(0, semicolon));
  const std::string parameters = strip(format.substr(semicolon + 1));
  const auto first_space = parameters.find(' ');
  const std::string compression =
      first_space == std::string::npos ? parameters : strip(parameters.substr(0, first_space));
  const auto last_space = parameters.rfind(' ');
  const std::string compressed =
      last_space == std::string::npos ? parameters : strip(parameters.substr(last_space + 1));
  return { original, compressed, compression };
}

int bayer_to_bgr_code(const std::string& encoding)
{
  if (encoding == "bayer_rggb8")
  {
    return cv::COLOR_BayerBG2BGR;
  }
  if (encoding == "bayer_bggr8")
  {
    return cv::COLOR_BayerRG2BGR;
  }
  if (encoding == "bayer_gbrg8")
  {
    return cv::COLOR_BayerGR2BGR;
  }
  if (encoding == "bayer_grbg8")
  {
    return cv::COLOR_BayerGB2BGR;
  }
  return -1;
}

template <typename MessageT>
MessageT deserialize(const rcutils_uint8_array_t& data)
{
  rclcpp::SerializedMessage serialized(data.buffer_length);
  auto& serialized_message = serialized.get_rcl_serialized_message();
  std::memcpy(serialized_message.buffer, data.buffer, data.buffer_length);
  serialized_message.buffer_length = data.buffer_length;

  MessageT message;
  rclcpp::Serialization<MessageT> serializer;
  serializer.deserialize_message(&serialized, &message);
  return message;
}

std::string ffmpeg_error(int result)
{
  std::array<char, AV_ERROR_MAX_STRING_SIZE> buffer{};
  av_strerror(result, buffer.data(), buffer.size());
  return buffer.data();
}

void require_ffmpeg(int result, const std::string& operation)
{
  if (result < 0)
  {
    throw std::runtime_error(operation + ": " + ffmpeg_error(result));
  }
}

std::size_t sidecar_rows(const fs::path& path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input)
  {
    return 0;
  }
  std::size_t lines = 0;
  std::array<char, 1 << 16> buffer{};
  while (input)
  {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize count = input.gcount();
    lines += static_cast<std::size_t>(std::count(buffer.begin(), buffer.begin() + count, '\n'));
  }
  return lines > 0 ? lines - 1 : 0;
}

}  // namespace

VideoHandler::VideoHandler(TopicInfo topic, const std::string& output_dir, const UnbagOptions& options)
  : topic_(std::move(topic))
  , overwrite_(options.overwrite)
  , codec_name_(options.video.codec)
  , quality_(options.video.quality)
  , nominal_fps_(options.video.nominal_fps)
  , segment_duration_ns_(static_cast<int64_t>(options.video.segment_seconds) * 1000000000LL)
{
  video_dir_ = fs::path(output_dir) / (topic_to_basename(topic_.name) + "_video");
  if (nominal_fps_ <= 0 || segment_duration_ns_ <= 0)
  {
    throw std::runtime_error("Video frame rate and segment duration must be positive");
  }
}

VideoHandler::~VideoHandler()
{
  try
  {
    close_segment(false);
  }
  catch (...)
  {
  }
}

fs::path VideoHandler::segment_path(std::size_t index, const std::string& suffix) const
{
  std::ostringstream name;
  name << "segment_" << std::setfill('0') << std::setw(6) << index << suffix;
  return video_dir_ / name.str();
}

void VideoHandler::discover_resume_point()
{
  segment_index_ = 0;
  skip_frames_ = 0;
  while (true)
  {
    const fs::path video = segment_path(segment_index_, ".mkv");
    const fs::path sidecar = segment_path(segment_index_, "_timestamps.csv");
    if (!fs::is_regular_file(video) || !fs::is_regular_file(sidecar))
    {
      break;
    }
    const std::size_t rows = sidecar_rows(sidecar);
    if (rows == 0)
    {
      break;
    }
    skip_frames_ += rows;
    ++segment_index_;
  }

  std::error_code ec;
  for (const auto& entry : fs::directory_iterator(video_dir_, ec))
  {
    if (entry.path().extension() == ".part")
    {
      fs::remove(entry.path(), ec);
      ec.clear();
    }
  }
}

void VideoHandler::begin_write()
{
  std::error_code ec;
  if (overwrite_ && fs::exists(video_dir_, ec))
  {
    fs::remove_all(video_dir_, ec);
    if (ec)
    {
      throw std::system_error(ec, "Failed to clear video directory: " + video_dir_.string());
    }
  }
  fs::create_directories(video_dir_, ec);
  if (ec)
  {
    throw std::system_error(ec, "Failed to create video directory: " + video_dir_.string());
  }
  if (!overwrite_)
  {
    discover_resume_point();
  }
}

cv::Mat VideoHandler::decode(const rcutils_uint8_array_t& data, int64_t& header_stamp_ns)
{
  cv::Mat image;
  if (topic_.type == "sensor_msgs/msg/CompressedImage")
  {
    const auto message = deserialize<sensor_msgs::msg::CompressedImage>(data);
    header_stamp_ns = static_cast<int64_t>(message.header.stamp.sec) * 1000000000LL +
                      static_cast<int64_t>(message.header.stamp.nanosec);
    const auto [original, compressed, compression] = parse_compressed_format(message.format);
    (void)original;
    (void)compression;
    const std::string source_encoding = to_lower(compressed);
    if (enc::isBayer(source_encoding))
    {
      const cv::Mat encoded(1, static_cast<int>(message.data.size()), CV_8UC1,
                            const_cast<uint8_t*>(message.data.data()));
      const cv::Mat mosaic = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);
      const int conversion = bayer_to_bgr_code(source_encoding);
      if (mosaic.empty() || conversion < 0)
      {
        throw std::runtime_error("Unsupported or empty Bayer frame: " + source_encoding);
      }
      cv::cvtColor(mosaic, image, conversion);
    }
    else
    {
      image = cv_bridge::toCvCopy(message, "bgr8")->image;
    }
  }
  else
  {
    const auto message = deserialize<sensor_msgs::msg::Image>(data);
    header_stamp_ns = static_cast<int64_t>(message.header.stamp.sec) * 1000000000LL +
                      static_cast<int64_t>(message.header.stamp.nanosec);
    image = cv_bridge::toCvCopy(message, "bgr8")->image;
  }
  if (image.empty() || image.type() != CV_8UC3)
  {
    throw std::runtime_error("Decoded frame is not 8-bit BGR");
  }
  return image;
}

void VideoHandler::open_segment(const cv::Mat& image, int64_t header_stamp_ns)
{
  video_final_path_ = segment_path(segment_index_, ".mkv");
  video_part_path_ = segment_path(segment_index_, ".mkv.part");
  sidecar_final_path_ = segment_path(segment_index_, "_timestamps.csv");
  sidecar_part_path_ = segment_path(segment_index_, "_timestamps.csv.part");

  require_ffmpeg(avformat_alloc_output_context2(&format_context_, nullptr, "matroska", video_part_path_.c_str()),
                 "Create Matroska output");
  if (format_context_ == nullptr)
  {
    throw std::runtime_error("FFmpeg did not create a Matroska context");
  }

  const AVCodec* codec = avcodec_find_encoder_by_name(codec_name_.c_str());
  if (codec == nullptr)
  {
    throw std::runtime_error("FFmpeg encoder is unavailable: " + codec_name_);
  }
  stream_ = avformat_new_stream(format_context_, nullptr);
  codec_context_ = avcodec_alloc_context3(codec);
  if (stream_ == nullptr || codec_context_ == nullptr)
  {
    throw std::runtime_error("Failed to allocate FFmpeg stream/codec context");
  }

  codec_context_->codec_id = codec->id;
  codec_context_->codec_type = AVMEDIA_TYPE_VIDEO;
  codec_context_->width = image.cols;
  codec_context_->height = image.rows;
  codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;
  codec_context_->time_base = AVRational{ 1, 1000000 };
  codec_context_->framerate = AVRational{ nominal_fps_, 1 };
  codec_context_->gop_size = nominal_fps_ * 2;
  codec_context_->max_b_frames = 0;
  if ((format_context_->oformat->flags & AVFMT_GLOBALHEADER) != 0)
  {
    codec_context_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
  }

  AVDictionary* encoder_options = nullptr;
  const std::string quality = std::to_string(quality_);
  if (codec_name_.find("nvenc") != std::string::npos)
  {
    av_dict_set(&encoder_options, "preset", "p4", 0);
    av_dict_set(&encoder_options, "tune", "hq", 0);
    av_dict_set(&encoder_options, "rc", "vbr", 0);
    av_dict_set(&encoder_options, "cq", quality.c_str(), 0);
    av_dict_set(&encoder_options, "b", "0", 0);
    av_dict_set(&encoder_options, "bf", "0", 0);
    av_dict_set(&encoder_options, "zerolatency", "1", 0);
    av_dict_set(&encoder_options, "delay", "0", 0);
  }
  else if (codec_name_ == "libx264")
  {
    av_dict_set(&encoder_options, "preset", "veryfast", 0);
    av_dict_set(&encoder_options, "crf", quality.c_str(), 0);
  }
  const int open_result = avcodec_open2(codec_context_, codec, &encoder_options);
  av_dict_free(&encoder_options);
  require_ffmpeg(open_result, "Open video encoder " + codec_name_);

  require_ffmpeg(avcodec_parameters_from_context(stream_->codecpar, codec_context_), "Copy codec parameters");
  stream_->time_base = codec_context_->time_base;
  stream_->avg_frame_rate = AVRational{ nominal_fps_, 1 };

  if ((format_context_->oformat->flags & AVFMT_NOFILE) == 0)
  {
    require_ffmpeg(avio_open(&format_context_->pb, video_part_path_.c_str(), AVIO_FLAG_WRITE),
                   "Open video output " + video_part_path_.string());
  }
  require_ffmpeg(avformat_write_header(format_context_, nullptr), "Write Matroska header");

  frame_ = av_frame_alloc();
  packet_ = av_packet_alloc();
  if (frame_ == nullptr || packet_ == nullptr)
  {
    throw std::runtime_error("Failed to allocate FFmpeg frame/packet");
  }
  frame_->format = codec_context_->pix_fmt;
  frame_->width = codec_context_->width;
  frame_->height = codec_context_->height;
  require_ffmpeg(av_frame_get_buffer(frame_, 32), "Allocate encoded frame buffer");
  sws_context_ = sws_getContext(image.cols, image.rows, AV_PIX_FMT_BGR24, image.cols, image.rows,
                                codec_context_->pix_fmt, SWS_BILINEAR, nullptr, nullptr, nullptr);
  if (sws_context_ == nullptr)
  {
    throw std::runtime_error("Failed to allocate BGR-to-YUV converter");
  }

  sidecar_.open(sidecar_part_path_, std::ios::out | std::ios::trunc | std::ios::binary);
  if (!sidecar_)
  {
    throw std::runtime_error("Failed to open timestamp sidecar: " + sidecar_part_path_.string());
  }
  sidecar_ << "segment_frame_index,header_stamp_ns,bag_time_ns,video_pts_us,pts_adjusted\n";
  segment_start_stamp_ns_ = header_stamp_ns;
  previous_stamp_ns_ = header_stamp_ns;
  previous_pts_us_ = -1;
  segment_frame_index_ = 0;
  segment_open_ = true;
}

bool VideoHandler::segment_due(int64_t header_stamp_ns) const
{
  return segment_open_ && segment_frame_index_ != 0 && header_stamp_ns >= segment_start_stamp_ns_ &&
         header_stamp_ns - segment_start_stamp_ns_ >= segment_duration_ns_;
}

void VideoHandler::drain_encoder(bool final)
{
  if (final)
  {
    const int result = avcodec_send_frame(codec_context_, nullptr);
    if (result != AVERROR_EOF)
    {
      require_ffmpeg(result, "Flush video encoder");
    }
  }

  while (true)
  {
    const int result = avcodec_receive_packet(codec_context_, packet_);
    if (result == AVERROR(EAGAIN) || result == AVERROR_EOF)
    {
      break;
    }
    require_ffmpeg(result, "Receive encoded video packet");
    av_packet_rescale_ts(packet_, codec_context_->time_base, stream_->time_base);
    packet_->stream_index = stream_->index;
    require_ffmpeg(av_interleaved_write_frame(format_context_, packet_), "Write encoded video packet");
    av_packet_unref(packet_);
  }
}

void VideoHandler::encode_frame(const cv::Mat& image, int64_t header_stamp_ns, int64_t bag_time_ns)
{
  if (image.cols != codec_context_->width || image.rows != codec_context_->height)
  {
    throw std::runtime_error("Camera resolution changed inside a video segment");
  }
  require_ffmpeg(av_frame_make_writable(frame_), "Make video frame writable");
  const uint8_t* source[] = { image.data, nullptr, nullptr, nullptr };
  const int source_stride[] = { static_cast<int>(image.step), 0, 0, 0 };
  const int converted = sws_scale(sws_context_, source, source_stride, 0, image.rows, frame_->data, frame_->linesize);
  if (converted != image.rows)
  {
    throw std::runtime_error("Incomplete BGR-to-YUV conversion");
  }

  int64_t pts_us = (header_stamp_ns - segment_start_stamp_ns_) / 1000;
  const bool adjusted = pts_us <= previous_pts_us_;
  if (adjusted)
  {
    pts_us = previous_pts_us_ + 1;
  }
  frame_->pts = pts_us;
  require_ffmpeg(avcodec_send_frame(codec_context_, frame_), "Send frame to video encoder");
  drain_encoder(false);

  sidecar_ << segment_frame_index_ << ',' << header_stamp_ns << ',' << bag_time_ns << ',' << pts_us << ','
           << (adjusted ? 1 : 0) << '\n';
  if (!sidecar_)
  {
    throw std::runtime_error("Failed writing timestamp sidecar: " + sidecar_part_path_.string());
  }
  if ((segment_frame_index_ % 1000) == 999)
  {
    sidecar_.flush();
  }
  ++segment_frame_index_;
  previous_stamp_ns_ = header_stamp_ns;
  previous_pts_us_ = pts_us;
}

void VideoHandler::cleanup_ffmpeg()
{
  if (format_context_ != nullptr && format_context_->pb != nullptr)
  {
    avio_closep(&format_context_->pb);
  }
  sws_freeContext(sws_context_);
  sws_context_ = nullptr;
  av_packet_free(&packet_);
  av_frame_free(&frame_);
  avcodec_free_context(&codec_context_);
  avformat_free_context(format_context_);
  format_context_ = nullptr;
  stream_ = nullptr;
}

void VideoHandler::close_segment(bool publish)
{
  if (!segment_open_)
  {
    cleanup_ffmpeg();
    return;
  }

  if (publish)
  {
    drain_encoder(true);
    require_ffmpeg(av_write_trailer(format_context_), "Finalize Matroska segment");
  }
  if (sidecar_.is_open())
  {
    sidecar_.flush();
    sidecar_.close();
  }
  cleanup_ffmpeg();
  segment_open_ = false;

  if (publish)
  {
    std::error_code ec;
    fs::rename(sidecar_part_path_, sidecar_final_path_, ec);
    if (ec)
    {
      throw std::system_error(ec, "Publish timestamp sidecar: " + sidecar_final_path_.string());
    }
    fs::rename(video_part_path_, video_final_path_, ec);
    if (ec)
    {
      throw std::system_error(ec, "Publish video segment: " + video_final_path_.string());
    }
  }
}

void VideoHandler::write(const rcutils_uint8_array_t& data, int64_t bag_time_ns, uint64_t write_index)
{
  (void)write_index;
  if (skip_frames_ != 0)
  {
    --skip_frames_;
    ++succeeded_;
    return;
  }
  if (fatal_)
  {
    ++failed_;
    return;
  }

  try
  {
    int64_t header_stamp_ns = 0;
    cv::Mat image = decode(data, header_stamp_ns);
    if (segment_due(header_stamp_ns))
    {
      close_segment(true);
      ++segment_index_;
    }
    if (!segment_open_)
    {
      open_segment(image, header_stamp_ns);
    }
    encode_frame(image, header_stamp_ns, bag_time_ns);
    ++succeeded_;
  }
  catch (const std::exception& error)
  {
    ++failed_;
    fatal_ = true;
    if (!warned_)
    {
      warned_ = true;
      std::cerr << "ERROR - Video export failed for " << topic_.name << ": " << error.what() << '\n';
    }
    try
    {
      close_segment(false);
    }
    catch (...)
    {
    }
  }
}

void VideoHandler::finish()
{
  if (fatal_)
  {
    close_segment(false);
    return;
  }
  if (segment_open_)
  {
    close_segment(true);
  }
}

}  // namespace tabletop_unbag
