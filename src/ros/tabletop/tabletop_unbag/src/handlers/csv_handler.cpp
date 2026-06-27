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

#include <array>
#include <charconv>
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>

namespace fs = std::filesystem;

namespace tabletop_unbag
{

namespace
{

// ---------------------------------------------------------------------------
// Value formatting (pandas `DataFrame.to_csv` compatible). Lives here because
// only the CSV handler formats values for CSV output; the flattening itself is
// shared (tabletop_unbag::MessageFlattener) and type-preserving.
// ---------------------------------------------------------------------------

/// Format a floating-point value the way pandas writes it: shortest decimal
/// that round-trips (std::to_chars, the algorithm behind Python's repr(float)),
/// a trailing ".0" kept on integral values ("1.0"), NaN as the empty string
/// (pandas' default na_rep) and +/-inf as "inf"/"-inf". Templated on float vs
/// double so each is rendered at its own precision (a float and the same bits
/// widened to double do not share a shortest round-tripping text).
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

/// Render one flattened typed value as pandas' `to_csv` would: booleans as
/// `True`/`False`, integers by value, floats by shortest round-trip (at their
/// own precision), strings RFC-4180 quoted.
std::string format_flat_value(const FlatScalar& value)
{
  return std::visit(
      [](const auto& v) -> std::string {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, bool>)
        {
          return v ? "True" : "False";
        }
        else if constexpr (std::is_same_v<T, int64_t> || std::is_same_v<T, uint64_t>)
        {
          return std::to_string(v);
        }
        else if constexpr (std::is_same_v<T, float> || std::is_same_v<T, double>)
        {
          return format_float(v);
        }
        else  // std::string (already UTF-8; wstrings were converted on flatten)
        {
          return csv_quote(v);
        }
      },
      value);
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
  , flattener_(topic_.type)
{
  csv_path_ = fs::path(output_dir) / (topic_to_basename(topic_.name) + ".csv");
  // bag_time_ns is always the first column.
  note_column("bag_time_ns");
}

void CsvHandler::prepare()
{
  // Load the introspection type-support library now, on the main thread, so the
  // per-topic consumer threads never trigger concurrent library loads.
  flattener_.prepare();
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
  const FlatRow flat = flattener_.flatten(data);
  for (const auto& column : flat)
  {
    note_column(column.name);
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

  FlatRow flat;
  try
  {
    flat = flattener_.flatten(data);
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
  for (const auto& column : flat)
  {
    // Duplicate columns within a message: last value wins (matches Python's
    // dict(gen_msg_values(msg))).
    values[column.name] = format_flat_value(column.value);
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
