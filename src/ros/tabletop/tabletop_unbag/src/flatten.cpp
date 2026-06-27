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

#include "tabletop_unbag/flatten.hpp"

#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/typesupport_helpers.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"

namespace introspection = rosidl_typesupport_introspection_cpp;

namespace tabletop_unbag
{

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

void flatten_members(const introspection::MessageMembers* members, eprosima::fastcdr::Cdr& cdr,
                     const std::string& prefix, FlatRow& out);

/// Read a single (non-array) value of a member from the stream, either emitting
/// a typed leaf or recursing into a submessage. The variant alternative chosen
/// here is what downstream backends key on (see flatten.hpp).
void read_one(const introspection::MessageMember& member, eprosima::fastcdr::Cdr& cdr, const std::string& name,
              FlatRow& out)
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
      out.push_back({ name, FlatScalar{ value } });
      break;
    }
    case introspection::ROS_TYPE_FLOAT:
    {
      float value = 0.0F;
      cdr.deserialize(value);
      out.push_back({ name, FlatScalar{ value } });
      break;
    }
    case introspection::ROS_TYPE_DOUBLE:
    {
      double value = 0.0;
      cdr.deserialize(value);
      out.push_back({ name, FlatScalar{ value } });
      break;
    }
    case introspection::ROS_TYPE_LONG_DOUBLE:
    {
      long double value = 0.0L;
      cdr.deserialize(value);
      out.push_back({ name, FlatScalar{ static_cast<double>(value) } });
      break;
    }
    case introspection::ROS_TYPE_CHAR:
    case introspection::ROS_TYPE_OCTET:
    case introspection::ROS_TYPE_UINT8:
    {
      uint8_t value = 0;
      cdr.deserialize(value);
      out.push_back({ name, FlatScalar{ static_cast<uint64_t>(value) } });
      break;
    }
    case introspection::ROS_TYPE_INT8:
    {
      int8_t value = 0;
      cdr.deserialize(value);
      out.push_back({ name, FlatScalar{ static_cast<int64_t>(value) } });
      break;
    }
    case introspection::ROS_TYPE_WCHAR:
    case introspection::ROS_TYPE_UINT16:
    {
      uint16_t value = 0;
      cdr.deserialize(value);
      out.push_back({ name, FlatScalar{ static_cast<uint64_t>(value) } });
      break;
    }
    case introspection::ROS_TYPE_INT16:
    {
      int16_t value = 0;
      cdr.deserialize(value);
      out.push_back({ name, FlatScalar{ static_cast<int64_t>(value) } });
      break;
    }
    case introspection::ROS_TYPE_UINT32:
    {
      uint32_t value = 0;
      cdr.deserialize(value);
      out.push_back({ name, FlatScalar{ static_cast<uint64_t>(value) } });
      break;
    }
    case introspection::ROS_TYPE_INT32:
    {
      int32_t value = 0;
      cdr.deserialize(value);
      out.push_back({ name, FlatScalar{ static_cast<int64_t>(value) } });
      break;
    }
    case introspection::ROS_TYPE_UINT64:
    {
      uint64_t value = 0;
      cdr.deserialize(value);
      out.push_back({ name, FlatScalar{ value } });
      break;
    }
    case introspection::ROS_TYPE_INT64:
    {
      int64_t value = 0;
      cdr.deserialize(value);
      out.push_back({ name, FlatScalar{ value } });
      break;
    }
    case introspection::ROS_TYPE_STRING:
    {
      std::string value;
      cdr.deserialize(value);
      out.push_back({ name, FlatScalar{ std::move(value) } });
      break;
    }
    case introspection::ROS_TYPE_WSTRING:
    {
      std::wstring value;
      cdr.deserialize(value);
      out.push_back({ name, FlatScalar{ wstring_to_utf8(value) } });
      break;
    }
    default:
      throw std::runtime_error("Unsupported field type id " + std::to_string(static_cast<int>(member.type_id_)) +
                               " for field '" + name + "'");
  }
}

/// Recursively read all members of a (sub)message from the CDR stream.
void flatten_members(const introspection::MessageMembers* members, eprosima::fastcdr::Cdr& cdr,
                     const std::string& prefix, FlatRow& out)
{
  for (uint32_t i = 0; i < members->member_count_; ++i)
  {
    const introspection::MessageMember& member = members->members_[i];
    const std::string name = prefix.empty() ? member.name_ : prefix + "." + member.name_;

    if (member.is_array_)
    {
      // A fixed-size array has a known length and no length prefix on the wire;
      // a (bounded or unbounded) sequence is length-prefixed.
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

}  // namespace

MessageFlattener::MessageFlattener(std::string ros_type) : ros_type_(std::move(ros_type))
{
}

void MessageFlattener::prepare()
{
  if (members_ != nullptr)
  {
    return;
  }
  type_support_library_ = rclcpp::get_typesupport_library(ros_type_, introspection::typesupport_identifier);
  const rosidl_message_type_support_t* type_support =
      rclcpp::get_message_typesupport_handle(ros_type_, introspection::typesupport_identifier, *type_support_library_);
  members_ = static_cast<const introspection::MessageMembers*>(type_support->data);
}

FlatRow MessageFlattener::flatten(const rcutils_uint8_array_t& data) const
{
  if (members_ == nullptr)
  {
    throw std::runtime_error("MessageFlattener::flatten() called before prepare() for type '" + ros_type_ + "'");
  }

  // rosbag2 stores ROS 2 messages as PLAIN_CDR (XCDRv1); read_encapsulation()
  // consumes the 4-byte representation header and picks up its endianness.
  eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(data.buffer), data.buffer_length);
  eprosima::fastcdr::Cdr cdr(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::CdrVersion::XCDRv1);
  cdr.read_encapsulation();

  FlatRow row;
  flatten_members(members_, cdr, "", row);
  return row;
}

}  // namespace tabletop_unbag
