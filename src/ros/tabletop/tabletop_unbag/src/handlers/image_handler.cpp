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

#include "tabletop_unbag/handlers/image_handler.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <iostream>
#include <map>
#include <optional>
#include <string>
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

namespace fs = std::filesystem;
namespace enc = sensor_msgs::image_encodings;

namespace tabletop_unbag
{

namespace
{

std::string to_lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

std::string strip(const std::string& s)
{
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos)
  {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

/// OpenCV color-conversion codes for demosaicing a Bayer mosaic to a viewable
/// encoding. cv_bridge cannot do this for *compressed* images (it cannot tell a
/// single-channel mosaic from mono8), so the Bayer cases are handled here. ROS
/// and OpenCV name Bayer patterns from opposite corners (ROS bayer_rggb ==
/// OpenCV BayerBG, etc.), which is why the corners look swapped.
const std::map<std::pair<std::string, std::string>, int>& bayer_conversion_codes()
{
  static const std::map<std::pair<std::string, std::string>, int> codes = {
    // 8-bit -> BGR
    { { "bayer_rggb8", "bgr8" }, cv::COLOR_BayerBG2BGR },
    { { "bayer_bggr8", "bgr8" }, cv::COLOR_BayerRG2BGR },
    { { "bayer_gbrg8", "bgr8" }, cv::COLOR_BayerGR2BGR },
    { { "bayer_grbg8", "bgr8" }, cv::COLOR_BayerGB2BGR },
    // 8-bit -> RGB
    { { "bayer_rggb8", "rgb8" }, cv::COLOR_BayerBG2RGB },
    { { "bayer_bggr8", "rgb8" }, cv::COLOR_BayerRG2RGB },
    { { "bayer_gbrg8", "rgb8" }, cv::COLOR_BayerGR2RGB },
    { { "bayer_grbg8", "rgb8" }, cv::COLOR_BayerGB2RGB },
    // 8-bit -> grayscale
    { { "bayer_rggb8", "mono8" }, cv::COLOR_BayerBG2GRAY },
    { { "bayer_bggr8", "mono8" }, cv::COLOR_BayerRG2GRAY },
    { { "bayer_gbrg8", "mono8" }, cv::COLOR_BayerGR2GRAY },
    { { "bayer_grbg8", "mono8" }, cv::COLOR_BayerGB2GRAY },
    // 16-bit -> BGR
    { { "bayer_rggb16", "bgr16" }, cv::COLOR_BayerBG2BGR },
    { { "bayer_bggr16", "bgr16" }, cv::COLOR_BayerRG2BGR },
    { { "bayer_gbrg16", "bgr16" }, cv::COLOR_BayerGR2BGR },
    { { "bayer_grbg16", "bgr16" }, cv::COLOR_BayerGB2BGR },
    // 16-bit -> RGB
    { { "bayer_rggb16", "rgb16" }, cv::COLOR_BayerBG2RGB },
    { { "bayer_bggr16", "rgb16" }, cv::COLOR_BayerRG2RGB },
    { { "bayer_gbrg16", "rgb16" }, cv::COLOR_BayerGR2RGB },
    { { "bayer_grbg16", "rgb16" }, cv::COLOR_BayerGB2RGB },
    // 16-bit -> grayscale
    { { "bayer_rggb16", "mono16" }, cv::COLOR_BayerBG2GRAY },
    { { "bayer_bggr16", "mono16" }, cv::COLOR_BayerRG2GRAY },
    { { "bayer_gbrg16", "mono16" }, cv::COLOR_BayerGR2GRAY },
    { { "bayer_grbg16", "mono16" }, cv::COLOR_BayerGB2GRAY },
  };
  return codes;
}

/// Parse a CompressedImage.format string of the form
/// "<original_encoding>; <compression_type> <compressed_encoding>".
/// Returns (original_encoding, compressed_encoding, compression_type).
std::tuple<std::string, std::string, std::string> parse_compressed_image_format(const std::string& fmt)
{
  const auto semicolon = fmt.find(';');
  if (semicolon == std::string::npos)
  {
    return { strip(fmt), "", "" };
  }
  const std::string original_encoding = strip(fmt.substr(0, semicolon));
  const std::string params = strip(fmt.substr(semicolon + 1));

  const auto first_space = params.find(' ');
  const std::string compression_type = first_space == std::string::npos ? params : strip(params.substr(0, first_space));
  const auto last_space = params.rfind(' ');
  const std::string compressed_encoding =
      last_space == std::string::npos ? params : strip(params.substr(last_space + 1));

  return { original_encoding, compressed_encoding, compression_type };
}

std::string extension_for_compression(const std::string& compression_type, const std::string& format)
{
  if (compression_type == "jpeg")
  {
    return ".jpg";
  }
  if (compression_type == "png")
  {
    return ".png";
  }
  if (compression_type == "tiff")
  {
    return ".tiff";
  }
  std::cerr << "WARNING - Unknown CompressedImage compression in format '" << format << "'; saving as jpeg.\n";
  return ".jpg";
}

/// Deserialize a concrete ROS message from a rosbag2 serialized buffer.
template <typename MessageT>
MessageT deserialize(const rcutils_uint8_array_t& data)
{
  rclcpp::SerializedMessage serialized(data.buffer_length);
  auto& rcl_msg = serialized.get_rcl_serialized_message();
  std::memcpy(rcl_msg.buffer, data.buffer, data.buffer_length);
  rcl_msg.buffer_length = data.buffer_length;

  MessageT msg;
  rclcpp::Serialization<MessageT> serializer;
  serializer.deserialize_message(&serialized, &msg);
  return msg;
}

}  // namespace

ImageHandler::ImageHandler(TopicInfo topic, const std::string& output_dir, const UnbagOptions& options)
  : topic_(std::move(topic))
  , overwrite_(options.overwrite)
  , image_encoding_(options.image_encoding.empty() ? "bgr8" : to_lower(options.image_encoding))
{
  image_dir_ = fs::path(output_dir) / topic_to_basename(topic_.name);
}

void ImageHandler::begin_write()
{
  if (!overwrite_)
  {
    return;
  }
  // --overwrite: clear the whole topic directory so a re-run with a different
  // image encoding cannot leave a mix of old and new output formats behind.
  std::error_code ec;
  if (fs::exists(image_dir_, ec))
  {
    fs::remove_all(image_dir_, ec);
  }
}

void ImageHandler::ensure_dir()
{
  if (dir_ready_)
  {
    return;
  }
  std::error_code ec;
  fs::create_directories(image_dir_, ec);
  dir_ready_ = true;
}

void ImageHandler::write(const rcutils_uint8_array_t& data, int64_t bag_time_ns)
{
  (void)bag_time_ns;

  // Decode just enough to find the timestamp first, so a resume can skip an
  // already-saved image without paying for the pixel decode.
  std::string basename;
  cv::Mat image;
  std::string extension;

  try
  {
    if (topic_.type == "sensor_msgs/msg/CompressedImage")
    {
      const auto msg = deserialize<sensor_msgs::msg::CompressedImage>(data);
      basename = std::to_string(msg.header.stamp.sec) + "_" + std::to_string(msg.header.stamp.nanosec);

      const auto [original_encoding, compressed_encoding, compression_type] = parse_compressed_image_format(msg.format);
      (void)original_encoding;
      const std::string src = to_lower(compressed_encoding);
      extension = extension_for_compression(compression_type, msg.format);

      if (!overwrite_ && fs::exists(image_dir_ / (basename + extension)))
      {
        return;  // already saved (resume)
      }

      if (enc::isBayer(src) && src != image_encoding_)
      {
        // cv_bridge would treat the single-channel mosaic as mono8; demosaic it
        // ourselves using the Bayer-aware conversion codes.
        const cv::Mat encoded(1, static_cast<int>(msg.data.size()), CV_8UC1, const_cast<uint8_t*>(msg.data.data()));
        const cv::Mat mosaic = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);
        const auto& codes = bayer_conversion_codes();
        const auto it = codes.find({ src, image_encoding_ });
        if (it == codes.end())
        {
          std::cerr << "WARNING - Unsupported Bayer conversion " << src << " -> " << image_encoding_
                    << "; saving raw mosaic.\n";
          image = mosaic;
        }
        else
        {
          cv::cvtColor(mosaic, image, it->second);
        }
      }
      else
      {
        image = cv_bridge::toCvCopy(msg, image_encoding_)->image;
      }
    }
    else  // sensor_msgs/msg/Image
    {
      const auto msg = deserialize<sensor_msgs::msg::Image>(data);
      basename = std::to_string(msg.header.stamp.sec) + "_" + std::to_string(msg.header.stamp.nanosec);
      extension = ".png";  // lossless default for raw images

      if (!overwrite_ && fs::exists(image_dir_ / (basename + extension)))
      {
        return;  // already saved (resume)
      }
      // cv_bridge knows the source encoding from msg.encoding, so it demosaics
      // raw Bayer images correctly here.
      image = cv_bridge::toCvCopy(msg, image_encoding_)->image;
    }
  }
  catch (const std::exception& e)
  {
    if (!decode_warned_)
    {
      std::cerr << "WARNING - Failed to decode an image on " << topic_.name << " (" << e.what()
                << "); skipping further decode errors on this topic.\n";
      decode_warned_ = true;
    }
    return;
  }

  if (image.empty())
  {
    return;
  }

  ensure_dir();
  const fs::path path = image_dir_ / (basename + extension);
  if (!cv::imwrite(path.string(), image))
  {
    std::cerr << "WARNING - Failed to write image " << path.string() << "\n";
  }
}

}  // namespace tabletop_unbag
