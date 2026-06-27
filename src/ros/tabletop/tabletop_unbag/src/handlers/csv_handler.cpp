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

#include "tabletop_unbag/handlers/csv_handler.hpp"

#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>

#include <array>
#include <charconv>
#include <cmath>
#include <cstring>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "rclcpp/typesupport_helpers.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"

namespace fs = std::filesystem;
namespace introspection = rosidl_typesupport_introspection_cpp;

namespace tabletop_unbag
{

namespace
{

// ---------------------------------------------------------------------------
// Value formatting (pandas `DataFrame.to_csv` compatible). Lives here because
// only the CSV handler formats values for CSV output.
// ---------------------------------------------------------------------------

/// Format a floating-point value the way pandas writes it: shortest decimal
/// that round-trips (std::to_chars, the algorithm behind Python's repr(float)),
/// a trailing ".0" kept on integral values ("1.0"), NaN as the empty string
/// (pandas' default na_rep) and +/-inf as "inf"/"-inf".
template <typename T>
std::string format_float(T value)
{
  if (std::isnan(value))
  {
    return "";
  }
  if (std::isinf(value))
  {
    return value < 0 ? "-inf" : "inf";
  }

  std::array<char, 64> buf;
  const auto result = std::to_chars(buf.data(), buf.data() + buf.size(), value);
  std::string str(buf.data(), result.ptr);

  // to_chars drops the decimal point on integral values ("1", "-3"); pandas
  // keeps it ("1.0", "-3.0"). Re-add it unless the value is in scientific form.
  if (str.find('.') == std::string::npos && str.find('e') == std::string::npos && str.find('E') == std::string::npos)
  {
    str += ".0";
  }
  return str;
}

/// Quote a CSV field per RFC 4180 / Python csv.QUOTE_MINIMAL: wrap in double
/// quotes only when the field contains a comma, double quote, or newline, and
/// double any embedded double quotes.
std::string csv_quote(const std::string& field)
{
  if (field.find_first_of(",\"\n\r") == std::string::npos)
  {
    return field;
  }
  std::string out;
  out.reserve(field.size() + 2);
  out.push_back('"');
  for (char c : field)
  {
    if (c == '"')
    {
      out += "\"\"";
    }
    else
    {
      out.push_back(c);
    }
  }
  out.push_back('"');
  return out;
}

// ---------------------------------------------------------------------------
// Generic message flattening. Lives here because only the CSV handler flattens
// messages into (column, value) pairs.
// ---------------------------------------------------------------------------

/// An ordered list of (column_name, formatted_value) pairs for one message.
using FlatRow = std::vector<std::pair<std::string, std::string>>;

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
/// a leaf (column, value) pair or recursing into a submessage.
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
      out.emplace_back(name, format_float(value));
      break;
    }
    case introspection::ROS_TYPE_LONG_DOUBLE:
    {
      long double value = 0.0L;
      cdr.deserialize(value);
      out.emplace_back(name, format_float(static_cast<double>(value)));
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

/// Flatten one serialized message into ordered (column, value) pairs.
FlatRow flatten_message(const introspection::MessageMembers* members, const rcutils_uint8_array_t& data)
{
  // rosbag2 stores ROS 2 messages as PLAIN_CDR (XCDRv1); read_encapsulation()
  // consumes the 4-byte representation header and picks up its endianness.
  eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(data.buffer), data.buffer_length);
  eprosima::fastcdr::Cdr cdr(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::CdrVersion::XCDRv1);
  cdr.read_encapsulation();

  FlatRow row;
  flatten_members(members, cdr, "", row);
  return row;
}

/// Read the first line of `path` (up to but excluding the newline).
std::string read_first_line(const fs::path& path)
{
  std::ifstream in(path, std::ios::binary);
  std::string line;
  std::getline(in, line);
  return line;
}

/// Inspect an existing CSV for resume: verify its header matches `expected`,
/// repair a torn trailing line, and return the number of complete data rows.
/// Returns std::nullopt if the header does not match (the caller should ask the
/// user to --overwrite rather than corrupt the file).
std::optional<std::size_t> inspect_existing_csv(const fs::path& path, const std::string& expected_header)
{
  if (read_first_line(path) != expected_header)
  {
    return std::nullopt;
  }

  // Count newlines and find the offset just past the last one, so a partially
  // written final line (from an interrupted run) can be truncated away.
  std::ifstream in(path, std::ios::binary);
  std::size_t newlines = 0;
  std::size_t last_newline_end = 0;
  std::size_t pos = 0;
  std::array<char, 1 << 16> chunk;
  while (in)
  {
    in.read(chunk.data(), static_cast<std::streamsize>(chunk.size()));
    const std::streamsize got = in.gcount();
    for (std::streamsize i = 0; i < got; ++i)
    {
      ++pos;
      if (chunk[static_cast<std::size_t>(i)] == '\n')
      {
        ++newlines;
        last_newline_end = pos;
      }
    }
  }

  if (last_newline_end != pos)
  {
    // Trailing bytes after the last newline are a torn row; drop them.
    std::error_code ec;
    fs::resize_file(path, last_newline_end, ec);
  }

  // newlines counts the header plus every complete data row.
  return newlines > 0 ? newlines - 1 : 0;
}

}  // namespace

CsvHandler::CsvHandler(TopicInfo topic, const std::string& output_dir, const UnbagOptions& options)
  : topic_(std::move(topic))
  , overwrite_(options.overwrite)
  , batch_size_(options.csv.batch_size == 0 ? 1 : options.csv.batch_size)
{
  csv_path_ = fs::path(output_dir) / (topic_to_basename(topic_.name) + ".csv");
  // bag_time_ns is always the first column.
  note_column("bag_time_ns");
}

void CsvHandler::ensure_type_support_loaded()
{
  if (members_ != nullptr)
  {
    return;
  }
  type_support_library_ = rclcpp::get_typesupport_library(topic_.type, introspection::typesupport_identifier);
  const rosidl_message_type_support_t* type_support = rclcpp::get_message_typesupport_handle(
      topic_.type, introspection::typesupport_identifier, *type_support_library_);
  members_ = static_cast<const introspection::MessageMembers*>(type_support->data);
}

void CsvHandler::prepare()
{
  // Load the introspection type-support library now, on the main thread, so the
  // per-topic consumer threads never trigger concurrent library loads.
  ensure_type_support_loaded();
}

void CsvHandler::note_column(const std::string& column)
{
  if (seen_columns_.insert(column).second)
  {
    columns_.push_back(column);
  }
}

std::string CsvHandler::header_line() const
{
  std::string header;
  for (std::size_t i = 0; i < columns_.size(); ++i)
  {
    if (i != 0)
    {
      header.push_back(',');
    }
    header += csv_quote(columns_[i]);
  }
  return header;
}

void CsvHandler::preprocess(const rcutils_uint8_array_t& data, int64_t bag_time_ns)
{
  (void)bag_time_ns;
  ensure_type_support_loaded();
  const FlatRow flat = flatten_message(members_, data);
  for (const auto& [column, value] : flat)
  {
    (void)value;
    note_column(column);
  }
}

void CsvHandler::begin_write()
{
  std::error_code ec;
  const bool exists = fs::exists(csv_path_, ec);

  if (overwrite_)
  {
    if (exists)
    {
      fs::remove(csv_path_, ec);
    }
    append_ = false;
    skip_rows_ = 0;
    return;
  }

  if (!exists || fs::file_size(csv_path_, ec) == 0 || ec)
  {
    append_ = false;
    skip_rows_ = 0;
    return;
  }

  const std::optional<std::size_t> rows = inspect_existing_csv(csv_path_, header_line());
  if (!rows)
  {
    throw std::runtime_error("Existing CSV " + csv_path_.string() +
                             " has a different header than this run would produce; pass --overwrite to replace it.");
  }
  append_ = true;
  skip_rows_ = *rows;
}

void CsvHandler::ensure_open()
{
  if (opened_)
  {
    return;
  }
  std::error_code ec;
  fs::create_directories(csv_path_.parent_path(), ec);
  if (append_)
  {
    out_.open(csv_path_, std::ios::out | std::ios::app | std::ios::binary);
  }
  else
  {
    out_.open(csv_path_, std::ios::out | std::ios::trunc | std::ios::binary);
  }
  if (!out_)
  {
    throw std::runtime_error("Failed to open for writing: " + csv_path_.string());
  }
  if (!append_)
  {
    out_ << header_line() << '\n';
  }
  opened_ = true;
}

void CsvHandler::buffer_line(std::string line)
{
  batch_.push_back(std::move(line));
  if (batch_.size() >= batch_size_)
  {
    flush_batch();
  }
}

void CsvHandler::flush_batch()
{
  if (batch_.empty())
  {
    return;
  }
  ensure_open();
  for (const auto& line : batch_)
  {
    out_ << line << '\n';
  }
  out_.flush();
  batch_.clear();
}

void CsvHandler::write(const rcutils_uint8_array_t& data, int64_t bag_time_ns, uint64_t write_index)
{
  // The CSV handler writes one row per message in bag order on its own consumer
  // thread, so it does not need the reader-assigned write index.
  (void)write_index;

  if (skip_rows_ > 0)
  {
    // Row already on disk from a previous run; skip to resume.
    --skip_rows_;
    ++succeeded_;
    return;
  }

  ensure_type_support_loaded();

  FlatRow flat;
  try
  {
    flat = flatten_message(members_, data);
  }
  catch (const std::exception& e)
  {
    // A single malformed message must not abort the whole topic; drop it,
    // count it, and warn once (the rest of the rows are still written).
    ++failed_;
    if (!flatten_warned_)
    {
      flatten_warned_ = true;
      std::cerr << "WARNING - Failed to flatten a message on " << topic_.name << " (" << e.what()
                << "); skipping further flatten errors on this topic.\n";
    }
    return;
  }

  std::unordered_map<std::string, std::string> values;
  values.reserve(flat.size() + 1);
  values["bag_time_ns"] = std::to_string(bag_time_ns);
  for (const auto& [column, value] : flat)
  {
    // Duplicate columns within a message: last value wins (matches Python's
    // dict(gen_msg_values(msg))).
    values[column] = value;
  }

  std::string line;
  for (std::size_t i = 0; i < columns_.size(); ++i)
  {
    if (i != 0)
    {
      line.push_back(',');
    }
    const auto it = values.find(columns_[i]);
    if (it != values.end())
    {
      // Values are already CSV-formatted (numbers) and quoted (strings); a
      // column missing from this message stays empty.
      line += it->second;
    }
  }
  buffer_line(std::move(line));
  ++succeeded_;
}

void CsvHandler::finish()
{
  flush_batch();
  if (opened_)
  {
    out_.close();
  }
}

}  // namespace tabletop_unbag
