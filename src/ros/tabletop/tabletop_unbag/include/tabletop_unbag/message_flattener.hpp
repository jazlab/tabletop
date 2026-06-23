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

#ifndef TABLETOP_UNBAG__MESSAGE_FLATTENER_HPP_
#define TABLETOP_UNBAG__MESSAGE_FLATTENER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcpputils/shared_library.hpp"
#include "rcutils/types/uint8_array.h"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace eprosima
{
namespace fastcdr
{
class Cdr;
}  // namespace fastcdr
}  // namespace eprosima

namespace tabletop_unbag
{

/// An ordered list of (column_name, formatted_value) pairs for one message.
using FlatRow = std::vector<std::pair<std::string, std::string>>;

/// Flattens arbitrary ROS 2 messages into flat (column, value) pairs using
/// runtime type introspection.
///
/// This is the C++ counterpart of the Python `gen_msg_values` generator: it
/// walks a message's fields recursively, producing dot-notation names for
/// nested submessages (`pose.position.x`) and bracket-notation names for array
/// and sequence elements (`name[0]`, `position[2]`). Values are pre-formatted
/// to strings matching pandas' CSV output (see value_formatter.hpp).
///
/// Unlike the Python version -- which deserializes via rclpy's dynamically
/// generated message classes -- there is no compiled C++ type to deserialize
/// into here, so we read the CDR payload directly with Fast CDR, guided by the
/// `rosidl_typesupport_introspection_cpp` field description of the type. Type
/// support libraries are loaded once per type and cached.
///
/// Note on fixed-size primitive arrays: the Python version expands variable
/// sequences (e.g. `position[0]`, `position[1]`) but, due to a quirk of its
/// `sequence<...>` type-string check, collapses fixed-size primitive arrays
/// (e.g. `CameraInfo.k`) into a single column holding a numpy string. This
/// port instead expands fixed-size arrays the same way as sequences
/// (`k[0]..k[8]`), which is the more useful and presumably intended behavior.
class MessageFlattener
{
public:
  MessageFlattener() = default;

  /// Flatten a serialized message of the given ROS type into ordered pairs.
  ///
  /// \param ros_type Fully-qualified type name, e.g. "sensor_msgs/msg/Image".
  /// \param data The CDR-serialized message (including the encapsulation
  ///   header), as handed out by the rosbag2 reader.
  /// \throws std::runtime_error if the type support cannot be loaded or an
  ///   unsupported field type is encountered.
  FlatRow flatten(const std::string& ros_type, const rcutils_uint8_array_t& data);

private:
  /// Cached type support library + introspection description for one type.
  struct TypeSupportEntry
  {
    std::shared_ptr<rcpputils::SharedLibrary> library;
    const rosidl_typesupport_introspection_cpp::MessageMembers* members;
  };

  /// Load (or fetch from cache) the introspection description for a type.
  const rosidl_typesupport_introspection_cpp::MessageMembers* get_members(const std::string& ros_type);

  /// Recursively read all members of a (sub)message from the CDR stream.
  void flatten_members(const rosidl_typesupport_introspection_cpp::MessageMembers* members, eprosima::fastcdr::Cdr& cdr,
                       const std::string& prefix, FlatRow& out);

  /// Read a single (non-array) value of a member from the stream, either
  /// emitting a leaf (column, value) pair or recursing into a submessage.
  void read_one(const rosidl_typesupport_introspection_cpp::MessageMember& member, eprosima::fastcdr::Cdr& cdr,
                const std::string& name, FlatRow& out);

  std::unordered_map<std::string, TypeSupportEntry> cache_;
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__MESSAGE_FLATTENER_HPP_
