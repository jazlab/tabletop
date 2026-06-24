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

#include "tabletop_unbag/unbagger.hpp"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "tabletop_unbag/handlers/csv_handler.hpp"
#include "tabletop_unbag/handlers/handler.hpp"
#include "tabletop_unbag/handlers/image_handler.hpp"
#include "tabletop_unbag/progress_bar.hpp"

namespace fs = std::filesystem;

namespace tabletop_unbag
{

namespace
{

using HandlerFactory =
    std::function<std::unique_ptr<MessageHandler>(TopicInfo, const std::string&, const UnbagOptions&)>;

/// One kind of handler the unbagger can dispatch to.
struct HandlerKind
{
  std::string name;
  bool (*claims)(const std::string&);
  HandlerFactory make;
};

/// The handler kinds in dispatch priority order. The image handler is checked
/// before the CSV handler, which is the catch-all (it claims every type), so
/// image topics never fall through to CSV.
const std::vector<HandlerKind>& handler_registry()
{
  static const std::vector<HandlerKind> registry = {
    { ImageHandler::handler_name(), &ImageHandler::handles,
      [](TopicInfo topic, const std::string& out, const UnbagOptions& opts) -> std::unique_ptr<MessageHandler> {
        return std::make_unique<ImageHandler>(std::move(topic), out, opts);
      } },
    { CsvHandler::handler_name(), &CsvHandler::handles,
      [](TopicInfo topic, const std::string& out, const UnbagOptions& opts) -> std::unique_ptr<MessageHandler> {
        return std::make_unique<CsvHandler>(std::move(topic), out, opts);
      } },
  };
  return registry;
}

/// Determine which handler should process `ros_type`, given the enabled set.
/// Returns nullptr if the type is unclaimed, or if its canonical handler (the
/// first in priority order that claims it) is not enabled -- in which case the
/// topic is skipped rather than falling through to a lower-priority handler.
const HandlerKind* choose_handler(const std::string& ros_type, const std::set<std::string>& enabled)
{
  for (const auto& kind : handler_registry())
  {
    if (kind.claims(ros_type))
    {
      return enabled.count(kind.name) != 0 ? &kind : nullptr;
    }
  }
  return nullptr;
}

void open_reader(rosbag2_cpp::Reader& reader, const std::string& bag_dir, const std::string& storage_id,
                 const std::string& serialization_format)
{
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_dir;
  storage_options.storage_id = storage_id;
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = serialization_format;
  converter_options.output_serialization_format = serialization_format;
  reader.open(storage_options, converter_options);
}

/// Read every message in the bag once, invoking `fn` for each.
template <typename Fn>
void for_each_message(const std::string& bag_dir, const std::string& storage_id,
                      const std::string& serialization_format, Fn&& fn)
{
  rosbag2_cpp::Reader reader;
  open_reader(reader, bag_dir, storage_id, serialization_format);
  while (reader.has_next())
  {
    const auto msg = reader.read_next();
    fn(*msg);
  }
}

}  // namespace

const std::vector<std::string>& handler_names()
{
  static const std::vector<std::string> names = [] {
    std::vector<std::string> result;
    for (const auto& kind : handler_registry())
    {
      result.push_back(kind.name);
    }
    return result;
  }();
  return names;
}

void unbag(const std::string& bag_dir, const std::string& output_dir, const UnbagOptions& options)
{
  // --- Infer storage / serialization / topics from the bag metadata. ---------
  std::string storage_id = "mcap";
  std::string serialization_format = "cdr";
  uint64_t total_messages = 0;
  std::unordered_map<std::string, std::string> topic_types;

  rosbag2_storage::MetadataIo metadata_io;
  if (metadata_io.metadata_file_exists(bag_dir))
  {
    const rosbag2_storage::BagMetadata metadata = metadata_io.read_metadata(bag_dir);
    if (!metadata.storage_identifier.empty())
    {
      storage_id = metadata.storage_identifier;
    }
    total_messages = metadata.message_count;
    for (const auto& topic : metadata.topics_with_message_count)
    {
      topic_types[topic.topic_metadata.name] = topic.topic_metadata.type;
      if (!topic.topic_metadata.serialization_format.empty())
      {
        serialization_format = topic.topic_metadata.serialization_format;
      }
    }
  }
  if (options.storage_id)
  {
    storage_id = *options.storage_id;
  }
  if (options.serialization_format)
  {
    serialization_format = *options.serialization_format;
  }

  // Fall back to interrogating the opened bag if metadata was unavailable.
  if (topic_types.empty() || total_messages == 0)
  {
    rosbag2_cpp::Reader reader;
    open_reader(reader, bag_dir, storage_id, serialization_format);
    if (topic_types.empty())
    {
      for (const auto& topic : reader.get_all_topics_and_types())
      {
        topic_types[topic.name] = topic.type;
      }
    }
    if (total_messages == 0)
    {
      total_messages = reader.get_metadata().message_count;
    }
  }

  // --- Resolve the topic selection (--topics XOR --exclude-topics). ----------
  const auto is_selected = [&](const std::string& topic) {
    if (options.topics)
    {
      return std::find(options.topics->begin(), options.topics->end(), topic) != options.topics->end();
    }
    if (options.exclude_topics)
    {
      return std::find(options.exclude_topics->begin(), options.exclude_topics->end(), topic) ==
             options.exclude_topics->end();
    }
    return true;
  };

  // --- Resolve the enabled handler set. --------------------------------------
  std::set<std::string> enabled;
  if (options.handlers.empty())
  {
    enabled.insert(handler_names().begin(), handler_names().end());
  }
  else
  {
    enabled.insert(options.handlers.begin(), options.handlers.end());
  }

  // --- Build one handler instance per selected, handled topic. ---------------
  std::error_code ec;
  fs::create_directories(output_dir, ec);

  std::map<std::string, std::unique_ptr<MessageHandler>> handlers;
  for (const auto& [topic, type] : topic_types)
  {
    if (!is_selected(topic))
    {
      continue;
    }
    const HandlerKind* kind = choose_handler(type, enabled);
    if (kind == nullptr)
    {
      if (options.verbose)
      {
        std::cerr << "INFO - Skipping " << topic << " (" << type << "): no enabled handler\n";
      }
      continue;
    }
    if (options.verbose)
    {
      std::cerr << "INFO - " << topic << " (" << type << ") -> " << kind->name << " handler\n";
    }
    handlers.emplace(topic, kind->make(TopicInfo{ topic, type }, output_dir, options));
  }

  if (handlers.empty())
  {
    std::cerr << "WARNING - No topics to unbag in " << bag_dir << "\n";
    return;
  }

  const bool any_preprocess =
      std::any_of(handlers.begin(), handlers.end(), [](const auto& kv) { return kv.second->needs_preprocess(); });

  // --- Phase 1: preprocess pass (only if some handler needs it). -------------
  if (any_preprocess)
  {
    ProgressBar bar(total_messages, "Preprocessing");
    for_each_message(bag_dir, storage_id, serialization_format, [&](const auto& msg) {
      bar.tick();
      const auto it = handlers.find(msg.topic_name);
      if (it == handlers.end() || !it->second->needs_preprocess())
      {
        return;
      }
      it->second->preprocess(*msg.serialized_data, msg.recv_timestamp);
    });
    bar.close();
  }

  for (auto& [topic, handler] : handlers)
  {
    handler->begin_write();
  }

  // --- Phase 2: write pass. --------------------------------------------------
  {
    ProgressBar bar(total_messages, "Unbagging");
    for_each_message(bag_dir, storage_id, serialization_format, [&](const auto& msg) {
      bar.tick();
      const auto it = handlers.find(msg.topic_name);
      if (it == handlers.end())
      {
        return;
      }
      it->second->write(*msg.serialized_data, msg.recv_timestamp);
    });
    bar.close();
  }

  for (auto& [topic, handler] : handlers)
  {
    handler->finish();
  }

  std::cout << "Unbagged " << handlers.size() << " topic(s) from " << bag_dir << " into " << output_dir << "\n";
}

}  // namespace tabletop_unbag
