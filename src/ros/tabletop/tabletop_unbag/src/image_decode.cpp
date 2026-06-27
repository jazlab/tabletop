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

#include "tabletop_unbag/image_decode.hpp"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/image_encodings.hpp>

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

}  // namespace

bool parse_header_stamp(const rcutils_uint8_array_t& data, int32_t& sec, uint32_t& nanosec)
{
  // Layout: a 4-byte CDR encapsulation header (byte 1 selects endianness: the
  // low bit set means little-endian PLAIN_CDR), then the message body 4-byte
  // aligned. The first field is the Header's builtin_interfaces/Time
  // { int32 sec; uint32 nanosec; }, so sec is at body offset 0 and nanosec at
  // offset 4 -> absolute buffer offsets 4 and 8.
  if (data.buffer == nullptr || data.buffer_length < 12)
  {
    return false;
  }
  const uint8_t* b = data.buffer;
  const bool little_endian = (b[1] & 0x01) != 0;
  const auto read_u32 = [&](std::size_t off) -> uint32_t {
    if (little_endian)
    {
      return static_cast<uint32_t>(b[off]) | (static_cast<uint32_t>(b[off + 1]) << 8) |
             (static_cast<uint32_t>(b[off + 2]) << 16) | (static_cast<uint32_t>(b[off + 3]) << 24);
    }
    return (static_cast<uint32_t>(b[off]) << 24) | (static_cast<uint32_t>(b[off + 1]) << 16) |
           (static_cast<uint32_t>(b[off + 2]) << 8) | static_cast<uint32_t>(b[off + 3]);
  };
  sec = static_cast<int32_t>(read_u32(4));
  nanosec = read_u32(8);
  return true;
}

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

cv::Mat decode_compressed_image(const sensor_msgs::msg::CompressedImage& msg, const std::string& target_encoding)
{
  const auto [original_encoding, compressed_encoding, compression_type] = parse_compressed_image_format(msg.format);
  (void)original_encoding;
  (void)compression_type;
  const std::string src = to_lower(compressed_encoding);

  if (enc::isBayer(src) && src != target_encoding)
  {
    // cv_bridge would treat the single-channel mosaic as mono8; demosaic it
    // ourselves using the Bayer-aware conversion codes.
    const cv::Mat encoded(1, static_cast<int>(msg.data.size()), CV_8UC1, const_cast<uint8_t*>(msg.data.data()));
    const cv::Mat mosaic = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);
    const auto& codes = bayer_conversion_codes();
    const auto it = codes.find({ src, target_encoding });
    if (it == codes.end())
    {
      std::cerr << "WARNING - Unsupported Bayer conversion " << src << " -> " << target_encoding
                << "; saving raw mosaic.\n";
      return mosaic;
    }
    cv::Mat out;
    cv::cvtColor(mosaic, out, it->second);
    return out;
  }
  return cv_bridge::toCvCopy(msg, target_encoding)->image;
}

cv::Mat decode_raw_image(const sensor_msgs::msg::Image& msg, const std::string& target_encoding)
{
  return cv_bridge::toCvCopy(msg, target_encoding)->image;
}

}  // namespace tabletop_unbag
