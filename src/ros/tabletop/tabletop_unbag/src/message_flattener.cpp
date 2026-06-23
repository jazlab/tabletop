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

#include "tabletop_unbag/message_flattener.hpp"

#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>

#include <cstdint>
#include <stdexcept>
#include <string>

#include "rclcpp/typesupport_helpers.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"

#include "tabletop_unbag/value_formatter.hpp"

namespace tabletop_unbag
{

namespace introspection = rosidl_typesupport_introspection_cpp;

namespace
{

/// Minimal wchar_t -> UTF-8 conversion (ROS wstrings are rare, but we must
/// still consume them from the stream and represent them somehow).
std::string wstring_to_utf8(const std::wstring& ws)
{
  std::string out;
  for (wchar_t wc : ws)
  {
    auto cp = static_cast<uint32_t>(wc);
    if (cp < 0x80)
    {
      out.push_back(static_cast<char>(cp));
    }
    else if (cp < 0x800)
    {
      out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
      out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    }
    else if (cp < 0x10000)
    {
      out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
      out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    }
    else
    {
      out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
      out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    }
  }
  return out;
}

}  // namespace

const introspection::MessageMembers* MessageFlattener::get_members(const std::string& ros_type)
{
  auto it = cache_.find(ros_type);
  if (it != cache_.end())
  {
    return it->second.members;
  }

  // The shared library must stay loaded for the lifetime of the handle, so it
  // is cached alongside the introspection description.
  auto library = rclcpp::get_typesupport_library(ros_type, introspection::typesupport_identifier);
  const rosidl_message_type_support_t* type_support =
      rclcpp::get_message_typesupport_handle(ros_type, introspection::typesupport_identifier, *library);
  const auto* members = static_cast<const introspection::MessageMembers*>(type_support->data);

  cache_.emplace(ros_type, TypeSupportEntry{ library, members });
  return members;
}

FlatRow MessageFlattener::flatten(const std::string& ros_type, const rcutils_uint8_array_t& data)
{
  const introspection::MessageMembers* members = get_members(ros_type);

  // rosbag2 stores ROS 2 messages as PLAIN_CDR (XCDRv1); read_encapsulation()
  // consumes the 4-byte representation header and picks up its endianness.
  eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(data.buffer), data.buffer_length);
  eprosima::fastcdr::Cdr cdr(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::CdrVersion::XCDRv1);
  cdr.read_encapsulation();

  FlatRow row;
  flatten_members(members, cdr, "", row);
  return row;
}

void MessageFlattener::flatten_members(const introspection::MessageMembers* members, eprosima::fastcdr::Cdr& cdr,
                                       const std::string& prefix, FlatRow& out)
{
  for (uint32_t i = 0; i < members->member_count_; ++i)
  {
    const introspection::MessageMember& member = members->members_[i];
    const std::string name = prefix.empty() ? member.name_ : prefix + "." + member.name_;

    if (member.is_array_)
    {
      // A fixed-size array has a known length and no length prefix on the
      // wire; a (bounded or unbounded) sequence is length-prefixed.
      const bool is_fixed_array = member.array_size_ != 0 && !member.is_upper_bound_;
      size_t count = member.array_size_;
      if (!is_fixed_array)
      {
        uint32_t sequence_length = 0;
        cdr.deserialize(sequence_length);
        count = sequence_length;
      }
      for (size_t index = 0; index < count; ++index)
      {
        read_one(member, cdr, name + "[" + std::to_string(index) + "]", out);
      }
    }
    else
    {
      read_one(member, cdr, name, out);
    }
  }
}

void MessageFlattener::read_one(const introspection::MessageMember& member, eprosima::fastcdr::Cdr& cdr,
                                const std::string& name, FlatRow& out)
{
  switch (member.type_id_)
  {
    case introspection::ROS_TYPE_MESSAGE:
    {
      const auto* sub = static_cast<const introspection::MessageMembers*>(member.members_->data);
      flatten_members(sub, cdr, name, out);
      break;
    }
    case introspection::ROS_TYPE_BOOLEAN:
    {
      bool value = false;
      cdr.deserialize(value);
      // pandas renders Python bools as "True"/"False".
      out.emplace_back(name, value ? "True" : "False");
      break;
    }
    case introspection::ROS_TYPE_FLOAT:
    {
      float value = 0.0F;
      cdr.deserialize(value);
      out.emplace_back(name, format_float(value));
      break;
    }
    case introspection::ROS_TYPE_DOUBLE:
    {
      double value = 0.0;
      cdr.deserialize(value);
      out.emplace_back(name, format_double(value));
      break;
    }
    case introspection::ROS_TYPE_LONG_DOUBLE:
    {
      long double value = 0.0L;
      cdr.deserialize(value);
      out.emplace_back(name, format_double(static_cast<double>(value)));
      break;
    }
    case introspection::ROS_TYPE_CHAR:
    case introspection::ROS_TYPE_OCTET:
    case introspection::ROS_TYPE_UINT8:
    {
      uint8_t value = 0;
      cdr.deserialize(value);
      out.emplace_back(name, std::to_string(static_cast<unsigned int>(value)));
      break;
    }
    case introspection::ROS_TYPE_INT8:
    {
      int8_t value = 0;
      cdr.deserialize(value);
      out.emplace_back(name, std::to_string(static_cast<int>(value)));
      break;
    }
    case introspection::ROS_TYPE_WCHAR:
    case introspection::ROS_TYPE_UINT16:
    {
      uint16_t value = 0;
      cdr.deserialize(value);
      out.emplace_back(name, std::to_string(value));
      break;
    }
    case introspection::ROS_TYPE_INT16:
    {
      int16_t value = 0;
      cdr.deserialize(value);
      out.emplace_back(name, std::to_string(value));
      break;
    }
    case introspection::ROS_TYPE_UINT32:
    {
      uint32_t value = 0;
      cdr.deserialize(value);
      out.emplace_back(name, std::to_string(value));
      break;
    }
    case introspection::ROS_TYPE_INT32:
    {
      int32_t value = 0;
      cdr.deserialize(value);
      out.emplace_back(name, std::to_string(value));
      break;
    }
    case introspection::ROS_TYPE_UINT64:
    {
      uint64_t value = 0;
      cdr.deserialize(value);
      out.emplace_back(name, std::to_string(value));
      break;
    }
    case introspection::ROS_TYPE_INT64:
    {
      int64_t value = 0;
      cdr.deserialize(value);
      out.emplace_back(name, std::to_string(value));
      break;
    }
    case introspection::ROS_TYPE_STRING:
    {
      std::string value;
      cdr.deserialize(value);
      out.emplace_back(name, csv_quote(value));
      break;
    }
    case introspection::ROS_TYPE_WSTRING:
    {
      std::wstring value;
      cdr.deserialize(value);
      out.emplace_back(name, csv_quote(wstring_to_utf8(value)));
      break;
    }
    default:
      throw std::runtime_error("Unsupported field type id " + std::to_string(static_cast<int>(member.type_id_)) +
                               " for field '" + name + "'");
  }
}

}  // namespace tabletop_unbag
