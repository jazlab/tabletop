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

#include "tabletop_unbag/handlers/image_handler.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "tabletop_unbag/image_decode.hpp"

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

/// Map an --image-format value to a file extension. "keep" returns "" (the
/// caller keeps the source-derived extension). The value is assumed lowercased.
std::string extension_for_image_format(const std::string& fmt)
{
  if (fmt.empty() || fmt == "keep")
  {
    return "";
  }
  if (fmt == "png")
  {
    return ".png";
  }
  if (fmt == "jpg" || fmt == "jpeg")
  {
    return ".jpg";
  }
  if (fmt == "tiff" || fmt == "tif")
  {
    return ".tiff";
  }
  std::cerr << "WARNING - Unknown --image-format '" << fmt << "'; keeping the source format.\n";
  return "";
}

}  // namespace

ImageHandler::ImageHandler(TopicInfo topic, const std::string& output_dir, const UnbagOptions& options)
  : topic_(std::move(topic))
  , overwrite_(options.overwrite)
  , image_encoding_(options.image.encoding.empty() ? "bgr8" : to_lower(options.image.encoding))
  , image_format_(options.image.format.empty() ? "keep" : to_lower(options.image.format))
{
  image_dir_ = fs::path(output_dir) / topic_to_basename(topic_.name);
}

uint64_t ImageHandler::note_for_write(const rcutils_uint8_array_t& data, int64_t bag_time_ns)
{
  (void)bag_time_ns;
  int32_t sec = 0;
  uint32_t nanosec = 0;
  if (!parse_header_stamp(data, sec, nanosec))
  {
    // Can't read the stamp (malformed/short buffer); treat as a first
    // occurrence. write() will deserialize and decide the name from the real
    // stamp, so at worst this misses disambiguating a corrupt message.
    return 0;
  }
  // Pack the stamp into its nanosecond value as a unique per-stamp key.
  const int64_t key = static_cast<int64_t>(sec) * 1000000000LL + static_cast<int64_t>(nanosec);
  const uint64_t occurrence = stamp_counts_[key]++;
  if (occurrence >= 1)
  {
    duplicates_.fetch_add(1, std::memory_order_relaxed);
  }
  return occurrence;
}

std::string ImageHandler::make_basename(int32_t sec, uint32_t nanosec, uint64_t occurrence)
{
  // Zero-pad both stamp fields to a fixed width so a lexicographic sort of the
  // filenames matches chronological order. Without padding, std::to_string
  // renders each field at its natural width and shorter strings sort first, so
  // e.g. nanosec 9 ("9") would sort after nanosec 10 ("10"). nanosec is always
  // < 1e9 (<= 9 digits); sec is an int32 Unix epoch second, <= 10 digits for any
  // value up to INT32_MAX (2147483647).
  std::ostringstream out;
  out << std::setfill('0') << std::setw(10) << sec << "_" << std::setw(9) << nanosec;
  if (occurrence == 0)
  {
    // First (or only) frame at this stamp keeps the plain name. '.' sorts before
    // '_', so it lexicographically precedes any duplicate suffix below.
    return out.str();
  }
  out << "_" << std::setw(6) << occurrence;
  return out.str();
}

void ImageHandler::begin_write()
{
  std::error_code ec;
  if (overwrite_)
  {
    // --overwrite: clear the whole topic directory so a re-run with a different
    // image encoding cannot leave a mix of old and new output formats behind.
    if (fs::exists(image_dir_, ec))
    {
      fs::remove_all(image_dir_, ec);
    }
    return;
  }

  // Resume: sweep away any leftover ".part" temp files from a previous run that
  // was interrupted mid-write. The corresponding final files (if any) are
  // complete thanks to the atomic rename, so only the temps need cleaning.
  if (!fs::exists(image_dir_, ec))
  {
    return;
  }
  for (const auto& entry : fs::directory_iterator(image_dir_, ec))
  {
    const std::string name = entry.path().filename().string();
    if (name.size() >= 5 && name.compare(name.size() - 5, 5, ".part") == 0)
    {
      fs::remove(entry.path(), ec);
    }
  }
}

void ImageHandler::ensure_dir()
{
  // Many worker threads may reach here at once for the same topic; create the
  // directory exactly once.
  std::call_once(dir_once_, [this] {
    std::error_code ec;
    fs::create_directories(image_dir_, ec);
  });
}

namespace
{

/// Encode `image` to `extension` (".jpg"/".png"/...) and write it to `path`
/// atomically: encode to memory, write a uniquely-named temp file in the same
/// directory, then rename it into place. A rename on the same filesystem is
/// atomic, so an interrupted run never leaves a half-written final file that a
/// later resume would mistake for complete. Returns false on encode failure.
bool write_image_atomic(const fs::path& path, const std::string& extension, const cv::Mat& image)
{
  std::vector<uchar> encoded;
  if (!cv::imencode(extension, image, encoded))
  {
    return false;
  }

  // Temp name is unique per (file, thread) so concurrent workers never collide.
  std::ostringstream tmp_name;
  tmp_name << '.' << path.filename().string() << '.' << std::this_thread::get_id() << ".part";
  const fs::path tmp_path = path.parent_path() / tmp_name.str();

  {
    std::ofstream out(tmp_path, std::ios::binary | std::ios::trunc);
    if (!out)
    {
      return false;
    }
    out.write(reinterpret_cast<const char*>(encoded.data()), static_cast<std::streamsize>(encoded.size()));
    out.flush();
    if (!out)
    {
      std::error_code ec;
      fs::remove(tmp_path, ec);
      return false;
    }
  }

  std::error_code ec;
  fs::rename(tmp_path, path, ec);
  if (ec)
  {
    fs::remove(tmp_path, ec);
    return false;
  }
  return true;
}

}  // namespace

void ImageHandler::write(const rcutils_uint8_array_t& data, int64_t bag_time_ns, uint64_t write_index)
{
  (void)bag_time_ns;
  // write_index is the per-stamp occurrence index assigned by note_for_write()
  // in bag order; it disambiguates frames that share a header stamp.

  // Deserialize and read the timestamp first, so a resume can skip an already-
  // saved image without paying for the (expensive) pixel decode.
  std::string basename;
  cv::Mat image;
  std::string extension;

  try
  {
    if (topic_.type == "sensor_msgs/msg/CompressedImage")
    {
      const auto msg = deserialize<sensor_msgs::msg::CompressedImage>(data);
      basename = make_basename(msg.header.stamp.sec, msg.header.stamp.nanosec, write_index);

      const auto [original_encoding, compressed_encoding, compression_type] = parse_compressed_image_format(msg.format);
      (void)original_encoding;
      (void)compressed_encoding;
      extension = extension_for_compression(compression_type, msg.format);
      if (const std::string fmt_ext = extension_for_image_format(image_format_); !fmt_ext.empty())
      {
        extension = fmt_ext;  // --image-format overrides the source container
      }

      if (!overwrite_ && fs::exists(image_dir_ / (basename + extension)))
      {
        succeeded_.fetch_add(1);  // already saved (resume)
        return;
      }

      image = decode_compressed_image(msg, image_encoding_);
    }
    else  // sensor_msgs/msg/Image
    {
      const auto msg = deserialize<sensor_msgs::msg::Image>(data);
      basename = make_basename(msg.header.stamp.sec, msg.header.stamp.nanosec, write_index);
      extension = ".png";  // lossless default for raw images
      if (const std::string fmt_ext = extension_for_image_format(image_format_); !fmt_ext.empty())
      {
        extension = fmt_ext;  // --image-format overrides the default
      }

      if (!overwrite_ && fs::exists(image_dir_ / (basename + extension)))
      {
        succeeded_.fetch_add(1);  // already saved (resume)
        return;
      }

      image = decode_raw_image(msg, image_encoding_);
    }
  }
  catch (const std::exception& e)
  {
    // exchange() so exactly one worker prints the warning for this topic; the
    // failed_ counter still accumulates every drop so the end-of-run summary
    // shows how many messages on this topic were lost.
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
    // A successful decode that yielded no pixels still produces no file.
    failed_.fetch_add(1);
    return;
  }

  ensure_dir();
  const fs::path path = image_dir_ / (basename + extension);
  if (!write_image_atomic(path, extension, image))
  {
    failed_.fetch_add(1);
    std::cerr << "WARNING - Failed to write image " << path.string() << "\n";
    return;
  }
  succeeded_.fetch_add(1);
}

}  // namespace tabletop_unbag
