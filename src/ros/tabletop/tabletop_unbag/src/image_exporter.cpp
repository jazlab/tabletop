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

#include "tabletop_unbag/image_exporter.hpp"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

namespace fs = std::filesystem;

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

/// Maps (src_encoding, dst_encoding) ROS encoding pairs to OpenCV color
/// conversion codes. Ported directly from the Python _CV_CONVERSION_CODES
/// table. ROS and OpenCV name Bayer patterns from opposite corners (ROS
/// bayer_rggb == OpenCV BayerBG, etc.), which is why the corners look swapped.
const std::map<std::pair<std::string, std::string>, int>& conversion_codes()
{
  static const std::map<std::pair<std::string, std::string>, int> codes = {
    // BGR <-> RGB
    { { "bgr8", "rgb8" }, cv::COLOR_BGR2RGB },
    { { "rgb8", "bgr8" }, cv::COLOR_RGB2BGR },
    { { "bgra8", "rgba8" }, cv::COLOR_BGRA2RGBA },
    { { "rgba8", "bgra8" }, cv::COLOR_RGBA2BGRA },
    // Alpha channel add/remove
    { { "bgr8", "bgra8" }, cv::COLOR_BGR2BGRA },
    { { "bgra8", "bgr8" }, cv::COLOR_BGRA2BGR },
    { { "rgb8", "rgba8" }, cv::COLOR_RGB2RGBA },
    { { "rgba8", "rgb8" }, cv::COLOR_RGBA2RGB },
    // To grayscale
    { { "bgr8", "mono8" }, cv::COLOR_BGR2GRAY },
    { { "rgb8", "mono8" }, cv::COLOR_RGB2GRAY },
    { { "bgra8", "mono8" }, cv::COLOR_BGRA2GRAY },
    { { "rgba8", "mono8" }, cv::COLOR_RGBA2GRAY },
    // From grayscale
    { { "mono8", "bgr8" }, cv::COLOR_GRAY2BGR },
    { { "mono8", "rgb8" }, cv::COLOR_GRAY2RGB },
    { { "mono8", "bgra8" }, cv::COLOR_GRAY2BGRA },
    { { "mono8", "rgba8" }, cv::COLOR_GRAY2RGBA },
    // 16-bit variants (same codes, bit depth handled by array dtype)
    { { "bgr16", "rgb16" }, cv::COLOR_BGR2RGB },
    { { "rgb16", "bgr16" }, cv::COLOR_RGB2BGR },
    { { "bgr16", "mono16" }, cv::COLOR_BGR2GRAY },
    { { "rgb16", "mono16" }, cv::COLOR_RGB2GRAY },
    { { "mono16", "bgr16" }, cv::COLOR_GRAY2BGR },
    { { "mono16", "rgb16" }, cv::COLOR_GRAY2RGB },
    // Bayer 8-bit -> BGR
    { { "bayer_rggb8", "bgr8" }, cv::COLOR_BayerBG2BGR },
    { { "bayer_bggr8", "bgr8" }, cv::COLOR_BayerRG2BGR },
    { { "bayer_gbrg8", "bgr8" }, cv::COLOR_BayerGR2BGR },
    { { "bayer_grbg8", "bgr8" }, cv::COLOR_BayerGB2BGR },
    // Bayer 8-bit -> RGB
    { { "bayer_rggb8", "rgb8" }, cv::COLOR_BayerBG2RGB },
    { { "bayer_bggr8", "rgb8" }, cv::COLOR_BayerRG2RGB },
    { { "bayer_gbrg8", "rgb8" }, cv::COLOR_BayerGR2RGB },
    { { "bayer_grbg8", "rgb8" }, cv::COLOR_BayerGB2RGB },
    // Bayer 8-bit -> grayscale
    { { "bayer_rggb8", "mono8" }, cv::COLOR_BayerBG2GRAY },
    { { "bayer_bggr8", "mono8" }, cv::COLOR_BayerRG2GRAY },
    { { "bayer_gbrg8", "mono8" }, cv::COLOR_BayerGR2GRAY },
    { { "bayer_grbg8", "mono8" }, cv::COLOR_BayerGB2GRAY },
    // Bayer 16-bit -> BGR
    { { "bayer_rggb16", "bgr16" }, cv::COLOR_BayerBG2BGR },
    { { "bayer_bggr16", "bgr16" }, cv::COLOR_BayerRG2BGR },
    { { "bayer_gbrg16", "bgr16" }, cv::COLOR_BayerGR2BGR },
    { { "bayer_grbg16", "bgr16" }, cv::COLOR_BayerGB2BGR },
    // Bayer 16-bit -> RGB
    { { "bayer_rggb16", "rgb16" }, cv::COLOR_BayerBG2RGB },
    { { "bayer_bggr16", "rgb16" }, cv::COLOR_BayerRG2RGB },
    { { "bayer_gbrg16", "rgb16" }, cv::COLOR_BayerGR2RGB },
    { { "bayer_grbg16", "rgb16" }, cv::COLOR_BayerGB2RGB },
    // Bayer 16-bit -> grayscale
    { { "bayer_rggb16", "mono16" }, cv::COLOR_BayerBG2GRAY },
    { { "bayer_bggr16", "mono16" }, cv::COLOR_BayerRG2GRAY },
    { { "bayer_gbrg16", "mono16" }, cv::COLOR_BayerGR2GRAY },
    { { "bayer_grbg16", "mono16" }, cv::COLOR_BayerGB2GRAY },
    // YUV
    { { "yuv422", "bgr8" }, cv::COLOR_YUV2BGR_UYVY },
    { { "yuv422", "rgb8" }, cv::COLOR_YUV2RGB_UYVY },
    { { "yuv422", "mono8" }, cv::COLOR_YUV2GRAY_UYVY },
    { { "yuv422_yuy2", "bgr8" }, cv::COLOR_YUV2BGR_YUY2 },
    { { "yuv422_yuy2", "rgb8" }, cv::COLOR_YUV2RGB_YUY2 },
    { { "yuv422_yuy2", "mono8" }, cv::COLOR_YUV2GRAY_YUY2 },
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

/// Decode a CompressedImage payload to an OpenCV matrix, converting to
/// `dst_encoding` if it differs from the stored (compressed) encoding.
/// Returns (image, compression_type).
std::pair<cv::Mat, std::string> compressed_imgmsg_to_cv2(const sensor_msgs::msg::CompressedImage& msg,
                                                         const std::string& dst_encoding)
{
  const cv::Mat encoded(1, static_cast<int>(msg.data.size()), CV_8UC1, const_cast<uint8_t*>(msg.data.data()));
  cv::Mat img = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);

  auto [original_encoding, compressed_encoding, compression_type] = parse_compressed_image_format(msg.format);
  (void)original_encoding;

  const std::string dst = to_lower(dst_encoding);
  const std::string src = to_lower(compressed_encoding);
  if (dst.empty() || src.empty() || dst == src)
  {
    return { img, compression_type };
  }

  const auto& codes = conversion_codes();
  const auto it = codes.find({ src, dst });
  if (it == codes.end())
  {
    std::cerr << "WARNING - Unsupported image conversion " << src << " -> " << dst << "; writing decoded image as-is\n";
    return { img, compression_type };
  }
  cv::Mat converted;
  cv::cvtColor(img, converted, it->second);
  return { converted, compression_type };
}

/// True if a file "<basename>.*" already exists in `dir`.
bool timestamped_file_exists(const fs::path& dir, const std::string& basename)
{
  std::error_code ec;
  if (!fs::is_directory(dir, ec))
  {
    return false;
  }
  for (const auto& entry : fs::directory_iterator(dir, ec))
  {
    if (entry.path().stem() == basename)
    {
      return true;
    }
  }
  return false;
}

}  // namespace

bool is_image_type(const std::string& ros_type)
{
  return ros_type == "sensor_msgs/msg/Image" || ros_type == "sensor_msgs/msg/CompressedImage";
}

void ImageExporter::save(const std::string& ros_type, const rcutils_uint8_array_t& data, const std::string& save_dir)
{
  if (ros_type != "sensor_msgs/msg/CompressedImage")
  {
    // Uncompressed Image conversion was never implemented in the Python
    // version (it raised NotImplementedError); warn once and skip.
    if (!uncompressed_warned_)
    {
      std::cerr << "WARNING - Uncompressed Image conversion is not implemented; "
                << "skipping image topics of type " << ros_type << "\n";
      uncompressed_warned_ = true;
    }
    return;
  }

  // Deserialize the concrete CompressedImage message.
  rclcpp::SerializedMessage serialized(data.buffer_length);
  auto& rcl_msg = serialized.get_rcl_serialized_message();
  std::memcpy(rcl_msg.buffer, data.buffer, data.buffer_length);
  rcl_msg.buffer_length = data.buffer_length;

  sensor_msgs::msg::CompressedImage msg;
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializer;
  serializer.deserialize_message(&serialized, &msg);

  const std::string basename = std::to_string(msg.header.stamp.sec) + "_" + std::to_string(msg.header.stamp.nanosec);

  const fs::path dir(save_dir);
  if (!force_ && timestamped_file_exists(dir, basename))
  {
    return;
  }

  auto [img, compression_type] = compressed_imgmsg_to_cv2(msg, "bgr8");

  std::string extension;
  if (compression_type == "jpeg")
  {
    extension = ".jpg";
  }
  else if (compression_type == "png")
  {
    extension = ".png";
  }
  else if (compression_type == "tiff")
  {
    extension = ".tiff";
  }
  else
  {
    std::cerr << "WARNING - Unknown CompressedImage format '" << msg.format << "'. Saving as jpeg.\n";
    extension = ".jpg";
  }

  std::error_code ec;
  fs::create_directories(dir, ec);
  const fs::path path = dir / (basename + extension);
  if (!cv::imwrite(path.string(), img))
  {
    std::cerr << "WARNING - Failed to write image " << path.string() << "\n";
  }
}

}  // namespace tabletop_unbag
