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

#ifndef TABLETOP_UNBAG__IMAGE_EXPORTER_HPP_
#define TABLETOP_UNBAG__IMAGE_EXPORTER_HPP_

#include <string>

#include "rcutils/types/uint8_array.h"

namespace tabletop_unbag
{

/// Returns true if the ROS type is an image type handled by ImageExporter
/// (sensor_msgs/msg/Image or sensor_msgs/msg/CompressedImage). Such topics are
/// exported as image files rather than flattened into CSV rows.
bool is_image_type(const std::string& ros_type);

/// Decodes image messages from a bag and writes them to disk as image files,
/// mirroring the Python `save_image_msg` helper.
///
/// CompressedImage payloads are decoded (and, if needed, color-converted to
/// bgr8) and re-encoded as .jpg/.png/.tiff based on the message's compression
/// type. Files are named "<sec>_<nanosec>.<ext>" and existing files with the
/// same timestamp are skipped unless `force` is set. Uncompressed Image
/// messages are not supported (the Python version raised NotImplementedError);
/// they are warned about once and skipped.
class ImageExporter
{
public:
  explicit ImageExporter(bool force = false) : force_(force)
  {
  }

  /// Save one serialized image message to `save_dir`.
  ///
  /// \param ros_type Fully-qualified type name (must satisfy is_image_type).
  /// \param data CDR-serialized message as handed out by the rosbag2 reader.
  /// \param save_dir Output directory (created if necessary).
  void save(const std::string& ros_type, const rcutils_uint8_array_t& data, const std::string& save_dir);

private:
  bool force_;
  bool uncompressed_warned_ = false;
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__IMAGE_EXPORTER_HPP_
