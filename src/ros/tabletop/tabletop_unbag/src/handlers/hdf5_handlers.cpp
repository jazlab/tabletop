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

#include "tabletop_unbag/handlers/hdf5_handlers.hpp"

#include <algorithm>
#include <cctype>
#include <iostream>
#include <utility>

#include <opencv2/core.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "tabletop_unbag/image_decode.hpp"

namespace tabletop_unbag
{

namespace
{

std::string to_lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

}  // namespace

// --- Hdf5CsvHandler ----------------------------------------------------------

Hdf5CsvHandler::Hdf5CsvHandler(TopicInfo topic, Hdf5Writer& writer, const UnbagOptions& options)
  : topic_(std::move(topic)), writer_(writer), flattener_(topic_.type)
{
  (void)options;
}

void Hdf5CsvHandler::prepare()
{
  flattener_.prepare();
}

void Hdf5CsvHandler::write(const rcutils_uint8_array_t& data, int64_t bag_time_ns, uint64_t write_index)
{
  (void)write_index;  // rows stay in bag order on this topic's consumer thread
  FlatRow flat;
  try
  {
    flat = flattener_.flatten(data);
  }
  catch (const std::exception& e)
  {
    ++failed_;
    if (!flatten_warned_)
    {
      flatten_warned_ = true;
      std::cerr << "WARNING - Failed to flatten a message on " << topic_.name << " (" << e.what()
                << "); skipping further flatten errors on this topic.\n";
    }
    return;
  }
  writer_.append_row(topic_.name, topic_.type, bag_time_ns, flat);
  ++succeeded_;
}

void Hdf5CsvHandler::finish()
{
  writer_.finish_topic(topic_.name);
}

// --- Hdf5ImageHandler --------------------------------------------------------

Hdf5ImageHandler::Hdf5ImageHandler(TopicInfo topic, Hdf5Writer& writer, const UnbagOptions& options)
  : topic_(std::move(topic))
  , writer_(writer)
  , image_encoding_(options.image.encoding.empty() ? "bgr8" : to_lower(options.image.encoding))
{
}

uint64_t Hdf5ImageHandler::note_for_write(const rcutils_uint8_array_t& data, int64_t bag_time_ns)
{
  (void)data;
  (void)bag_time_ns;
  // Runs single-threaded on the reader, in bag order: hand out a contiguous
  // per-topic row index so out-of-order pool decodes land at the right row.
  return next_index_++;
}

void Hdf5ImageHandler::write(const rcutils_uint8_array_t& data, int64_t bag_time_ns, uint64_t write_index)
{
  (void)bag_time_ns;
  const uint64_t frame_index = write_index;

  int32_t sec = 0;
  uint32_t nanosec = 0;
  cv::Mat image;
  try
  {
    if (topic_.type == "sensor_msgs/msg/CompressedImage")
    {
      const auto msg = deserialize<sensor_msgs::msg::CompressedImage>(data);
      sec = msg.header.stamp.sec;
      nanosec = msg.header.stamp.nanosec;
      image = decode_compressed_image(msg, image_encoding_);
    }
    else  // sensor_msgs/msg/Image
    {
      const auto msg = deserialize<sensor_msgs::msg::Image>(data);
      sec = msg.header.stamp.sec;
      nanosec = msg.header.stamp.nanosec;
      image = decode_raw_image(msg, image_encoding_);
    }
  }
  catch (const std::exception& e)
  {
    failed_.fetch_add(1);
    if (!decode_warned_.exchange(true))
    {
      std::cerr << "WARNING - Failed to decode an image on " << topic_.name << " (" << e.what()
                << "); skipping further decode errors on this topic.\n";
    }
    return;
  }

  if (image.empty())
  {
    failed_.fetch_add(1);
    return;
  }

  try
  {
    writer_.append_image(topic_.name, topic_.type, frame_index, sec, nanosec, image);
  }
  catch (const std::exception& e)
  {
    failed_.fetch_add(1);
    if (!decode_warned_.exchange(true))
    {
      std::cerr << "WARNING - Failed to write an image on " << topic_.name << " (" << e.what()
                << "); skipping further write errors on this topic.\n";
    }
    return;
  }
  succeeded_.fetch_add(1);
}

}  // namespace tabletop_unbag
