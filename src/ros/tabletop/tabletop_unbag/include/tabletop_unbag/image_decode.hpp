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

#ifndef TABLETOP_UNBAG__IMAGE_DECODE_HPP_
#define TABLETOP_UNBAG__IMAGE_DECODE_HPP_

#include <cstdint>
#include <cstring>
#include <string>
#include <tuple>

#include <opencv2/core.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcutils/types/uint8_array.h"

namespace tabletop_unbag
{

/// Parse the std_msgs/Header stamp (sec, nanosec) straight from the front of a
/// CDR-serialized Image/CompressedImage buffer, without deserializing the
/// payload. Returns false if the buffer is too short to contain the stamp. This
/// is the cheap stamp peek used to disambiguate same-stamp frames (it never
/// touches the pixels).
bool parse_header_stamp(const rcutils_uint8_array_t& data, int32_t& sec, uint32_t& nanosec);

/// Deserialize a concrete ROS message from a rosbag2 serialized (CDR) buffer.
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

/// Parse a CompressedImage.format string of the form
/// "<original_encoding>; <compression_type> <compressed_encoding>".
/// Returns (original_encoding, compressed_encoding, compression_type).
std::tuple<std::string, std::string, std::string> parse_compressed_image_format(const std::string& fmt);

/// Decode and color-convert a CompressedImage to `target_encoding` (e.g.
/// "bgr8"). Non-Bayer payloads go through cv_bridge; a compressed Bayer mosaic
/// is demosaiced with the Bayer-aware OpenCV path, because cv_bridge infers a
/// compressed image's encoding from its channel count and would mistake a
/// single-channel mosaic for mono8. Throws std::runtime_error / cv::Exception on
/// a decode failure. Shared by the JPEG-file and HDF5 image backends so they
/// debayer identically.
cv::Mat decode_compressed_image(const sensor_msgs::msg::CompressedImage& msg, const std::string& target_encoding);

/// Color-convert a raw Image to `target_encoding` via cv_bridge (which knows the
/// source encoding from msg.encoding, so it demosaics raw Bayer correctly).
cv::Mat decode_raw_image(const sensor_msgs::msg::Image& msg, const std::string& target_encoding);

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__IMAGE_DECODE_HPP_
