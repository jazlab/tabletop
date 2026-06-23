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

#include "tabletop_unbag/bag_converter.hpp"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "tabletop_unbag/image_exporter.hpp"
#include "tabletop_unbag/message_flattener.hpp"
#include "tabletop_unbag/value_formatter.hpp"

namespace fs = std::filesystem;

namespace tabletop_unbag
{

namespace
{

/// Accumulates flattened rows for a single topic, tracking the union of all
/// columns ever seen in first-seen order (matching the Python implementation,
/// where a later short message simply leaves trailing columns empty).
class CsvTable
{
public:
  void add_row(int64_t bag_time_ns, const FlatRow& flat)
  {
    std::unordered_map<std::string, std::string> row;
    // bag_time_ns is always the first column.
    add_cell(row, "bag_time_ns", std::to_string(bag_time_ns));
    for (const auto& [column, value] : flat)
    {
      add_cell(row, column, value);
    }
    rows_.push_back(std::move(row));
  }

  void write_csv(const fs::path& path) const
  {
    std::ofstream out(path, std::ios::trunc);
    if (!out)
    {
      throw std::runtime_error("Failed to open for writing: " + path.string());
    }

    for (size_t i = 0; i < columns_.size(); ++i)
    {
      if (i != 0)
      {
        out << ',';
      }
      out << csv_quote(columns_[i]);
    }
    out << '\n';

    for (const auto& row : rows_)
    {
      for (size_t i = 0; i < columns_.size(); ++i)
      {
        if (i != 0)
        {
          out << ',';
        }
        const auto it = row.find(columns_[i]);
        if (it != row.end())
        {
          // Values are already CSV-formatted (numbers) and quoted (strings).
          out << it->second;
        }
      }
      out << '\n';
    }
  }

private:
  void add_cell(std::unordered_map<std::string, std::string>& row, const std::string& column, const std::string& value)
  {
    if (seen_.insert(column).second)
    {
      columns_.push_back(column);
    }
    row[column] = value;
  }

  std::vector<std::string> columns_;
  std::unordered_set<std::string> seen_;
  std::vector<std::unordered_map<std::string, std::string>> rows_;
};

}  // namespace

std::string topic_to_basename(const std::string& topic)
{
  const auto start = topic.find_first_not_of('/');
  std::string basename = start == std::string::npos ? "" : topic.substr(start);
  std::replace(basename.begin(), basename.end(), '/', '_');
  return basename;
}

std::vector<std::string> rosbag_to_csv(const std::string& bag_dir, const ConvertOptions& options,
                                       const std::optional<std::string>& save_dir)
{
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_dir;
  storage_options.storage_id = "mcap";
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  reader.open(storage_options, converter_options);

  // Map each topic to its message type.
  std::unordered_map<std::string, std::string> topic_types;
  for (const auto& topic : reader.get_all_topics_and_types())
  {
    topic_types[topic.name] = topic.type;
  }

  // Resolve the set of topics to process (whitelist minus excludes).
  std::set<std::string> selected;
  if (options.topics)
  {
    selected.insert(options.topics->begin(), options.topics->end());
  }
  else
  {
    for (const auto& [name, type] : topic_types)
    {
      selected.insert(name);
    }
  }
  for (const auto& excluded : options.exclude_topics)
  {
    selected.erase(excluded);
  }

  MessageFlattener flattener;
  ImageExporter image_exporter(options.force);
  std::map<std::string, CsvTable> tables;
  bool image_save_warned = false;

  while (reader.has_next())
  {
    const auto bag_msg = reader.read_next();
    const std::string& topic = bag_msg->topic_name;
    if (selected.find(topic) == selected.end())
    {
      continue;
    }

    const auto type_it = topic_types.find(topic);
    if (type_it == topic_types.end())
    {
      continue;
    }
    const std::string& ros_type = type_it->second;

    if (is_image_type(ros_type))
    {
      if (options.convert_images)
      {
        if (save_dir)
        {
          const fs::path topic_dir = fs::path(*save_dir) / topic_to_basename(topic);
          image_exporter.save(ros_type, *bag_msg->serialized_data, topic_dir.string());
        }
        else if (!image_save_warned)
        {
          std::cerr << "WARNING - Image messages found but no save directory; "
                    << "skipping.\n";
          image_save_warned = true;
        }
      }
      continue;
    }

    FlatRow row = flattener.flatten(ros_type, *bag_msg->serialized_data);
    tables[topic].add_row(bag_msg->recv_timestamp, row);
  }

  std::vector<std::string> processed_topics;
  processed_topics.reserve(tables.size());
  for (const auto& [topic, table] : tables)
  {
    processed_topics.push_back(topic);
    if (save_dir)
    {
      const fs::path path = fs::path(*save_dir) / (topic_to_basename(topic) + ".csv");
      table.write_csv(path);
      std::cout << "Saved " << path.string() << "\n";
    }
  }
  return processed_topics;
}

void rosbag_session_to_csv(const std::string& session_dir, const ConvertOptions& options)
{
  // Discover bags as "<session_dir>/*/*.mcap".
  std::vector<fs::path> mcap_files;
  std::error_code ec;
  for (const auto& sub : fs::directory_iterator(session_dir, ec))
  {
    if (!sub.is_directory())
    {
      continue;
    }
    for (const auto& entry : fs::directory_iterator(sub.path(), ec))
    {
      if (entry.path().extension() == ".mcap")
      {
        mcap_files.push_back(entry.path());
      }
    }
  }
  std::sort(mcap_files.begin(), mcap_files.end());

  if (mcap_files.empty())
  {
    throw std::runtime_error("No .mcap files found in " + session_dir);
  }

  std::set<std::string> all_topics;
  for (const auto& mcap_file : mcap_files)
  {
    std::cout << "Converting " << mcap_file.string() << "...\n";
    const std::string bag_dir = mcap_file.parent_path().string();

    const std::vector<std::string> new_topics = rosbag_to_csv(bag_dir, options, session_dir);

    for (const auto& topic : new_topics)
    {
      if (!all_topics.insert(topic).second)
      {
        throw std::runtime_error("Topic collision in " + session_dir + ": " + topic + " appears in more than one bag");
      }
    }
  }
}

}  // namespace tabletop_unbag
