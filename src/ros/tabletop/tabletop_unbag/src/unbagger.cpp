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
#include <atomic>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "tabletop_unbag/concurrent_queue.hpp"
#include "tabletop_unbag/handlers/csv_handler.hpp"
#include "tabletop_unbag/handlers/handler.hpp"
#include "tabletop_unbag/handlers/image_handler.hpp"
#include "tabletop_unbag/progress_bar.hpp"

namespace fs = std::filesystem;

namespace tabletop_unbag
{

namespace
{

/// Set by the SIGINT/SIGTERM handler. Polled by the reader loop so an
/// interrupted run stops reading, closes its queues, drains the work already
/// queued, flushes, and joins -- leaving valid, resumable output.
std::atomic<bool> g_stop{ false };

extern "C" void handle_stop_signal(int /*signum*/)
{
  g_stop.store(true);
}

using MsgPtr = std::shared_ptr<rosbag2_storage::SerializedBagMessage>;

/// Which lifecycle method a pass invokes on its handlers.
enum class PassKind
{
  Preprocess,
  Write,
};

void process_one(MessageHandler* handler, const MsgPtr& msg, PassKind kind)
{
  if (kind == PassKind::Preprocess)
  {
    handler->preprocess(*msg->serialized_data, msg->recv_timestamp);
  }
  else
  {
    handler->write(*msg->serialized_data, msg->recv_timestamp);
  }
}

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

/// Run one streaming pass over the bag.
///
/// A single reader thread (this thread) pulls messages in bag order and routes
/// each to a worker:
///   * `serial_handlers` -- handlers that must process their topic in order on
///     one thread (the CSV handlers: one file per topic, rows in bag order).
///     Each gets its own bounded queue and consumer thread, so different CSV
///     topics are written concurrently.
///   * `pool_handlers` -- handlers whose per-message work is independent (the
///     image handlers). Their messages go to a single shared queue drained by
///     `pool_workers` threads, giving per-image parallelism across all image
///     topics at once.
///
/// The queues are bounded, so when the (slow) image workers fall behind, the
/// reader blocks on push() instead of buffering the bag in RAM -- memory stays
/// bounded and the reader self-throttles to the bottleneck. When `filter_topics`
/// is non-null it is pushed into the storage reader so non-selected topics are
/// not even read (used to skip image payloads during the CSV preprocess pass).
void run_pass(const std::string& bag_dir, const std::string& storage_id, const std::string& serialization_format,
              const std::vector<std::string>* filter_topics,
              const std::unordered_map<std::string, MessageHandler*>& serial_handlers,
              const std::unordered_map<std::string, MessageHandler*>& pool_handlers, std::size_t pool_workers,
              uint64_t total, const std::string& label, PassKind kind)
{
  constexpr std::size_t kSerialQueueCap = 1024;

  // Per-topic serial queues and their consumer threads.
  std::unordered_map<std::string, std::unique_ptr<ConcurrentQueue<MsgPtr>>> serial_queues;
  std::vector<std::thread> serial_threads;
  serial_threads.reserve(serial_handlers.size());
  for (const auto& [topic, handler] : serial_handlers)
  {
    auto queue = std::make_unique<ConcurrentQueue<MsgPtr>>(kSerialQueueCap);
    ConcurrentQueue<MsgPtr>* q = queue.get();
    serial_queues.emplace(topic, std::move(queue));
    serial_threads.emplace_back([q, handler, kind] {
      while (auto item = q->pop())
      {
        process_one(handler, *item, kind);
      }
    });
  }

  // Shared pool for per-message-parallel handlers.
  using PoolTask = std::pair<MessageHandler*, MsgPtr>;
  const std::size_t pool_cap = std::max<std::size_t>(64, pool_workers * 4);
  ConcurrentQueue<PoolTask> pool_queue(pool_cap);
  std::vector<std::thread> pool_threads;
  if (!pool_handlers.empty())
  {
    pool_threads.reserve(pool_workers);
    for (std::size_t i = 0; i < pool_workers; ++i)
    {
      pool_threads.emplace_back([&pool_queue, kind] {
        while (auto item = pool_queue.pop())
        {
          process_one(item->first, item->second, kind);
        }
      });
    }
  }

  // Reader loop (this thread).
  rosbag2_cpp::Reader reader;
  open_reader(reader, bag_dir, storage_id, serialization_format);
  if (filter_topics != nullptr)
  {
    rosbag2_storage::StorageFilter filter;
    filter.topics = *filter_topics;
    reader.set_filter(filter);
  }

  ProgressBar bar(total, label);
  while (reader.has_next())
  {
    if (g_stop.load())
    {
      break;
    }
    MsgPtr msg = reader.read_next();
    bar.tick();
    const auto sit = serial_queues.find(msg->topic_name);
    if (sit != serial_queues.end())
    {
      sit->second->push(msg);
      continue;
    }
    const auto pit = pool_handlers.find(msg->topic_name);
    if (pit != pool_handlers.end())
    {
      pool_queue.push(PoolTask{ pit->second, std::move(msg) });
    }
  }
  bar.close();

  // No more input: close the queues so consumers drain their backlog and exit,
  // then join. This ordering is what makes interruption safe -- every message
  // already handed to a worker is fully processed before we return to finish().
  for (auto& [topic, queue] : serial_queues)
  {
    queue->close();
  }
  pool_queue.close();
  for (auto& thread : serial_threads)
  {
    thread.join();
  }
  for (auto& thread : pool_threads)
  {
    thread.join();
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
  // We parallelize across images ourselves, so keep OpenCV's own per-image
  // threading off to avoid oversubscribing the cores with our worker pool.
  cv::setNumThreads(1);

  // Stop gracefully on Ctrl-C / TERM (flush + join, see run_pass()).
  std::signal(SIGINT, handle_stop_signal);
  std::signal(SIGTERM, handle_stop_signal);

  // --- Infer storage / serialization / topics from the bag metadata. ---------
  std::string storage_id = "mcap";
  std::string serialization_format = "cdr";
  uint64_t total_messages = 0;
  std::unordered_map<std::string, std::string> topic_types;
  std::unordered_map<std::string, uint64_t> topic_counts;

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
      topic_counts[topic.topic_metadata.name] = topic.message_count;
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

  // Warm each handler's one-time setup (e.g. loading introspection type support)
  // on this thread, before any worker threads start, to avoid concurrent library
  // loads racing.
  for (auto& [topic, handler] : handlers)
  {
    handler->prepare();
  }

  // --- Partition handlers by how they should be driven. ----------------------
  // serial_*  -> one consumer thread per topic (ordered, single output file).
  // pool_*    -> shared worker pool (independent per-message output, e.g images).
  std::unordered_map<std::string, MessageHandler*> serial_write;
  std::unordered_map<std::string, MessageHandler*> pool_write;
  std::unordered_map<std::string, MessageHandler*> serial_preprocess;
  std::vector<std::string> preprocess_topics;
  uint64_t preprocess_total = 0;
  for (auto& [topic, handler] : handlers)
  {
    if (handler->parallelizable_per_message())
    {
      pool_write.emplace(topic, handler.get());
    }
    else
    {
      serial_write.emplace(topic, handler.get());
    }
    if (handler->needs_preprocess())
    {
      serial_preprocess.emplace(topic, handler.get());
      preprocess_topics.push_back(topic);
      preprocess_total += topic_counts.count(topic) != 0 ? topic_counts[topic] : 0;
    }
  }
  if (preprocess_total == 0)
  {
    preprocess_total = total_messages;
  }

  std::size_t workers = options.jobs;
  if (workers == 0)
  {
    const unsigned hw = std::thread::hardware_concurrency();
    workers = hw == 0 ? 4 : hw;
  }

  const std::unordered_map<std::string, MessageHandler*> no_pool;

  // --- Phase 1: preprocess pass (CSV column discovery). ----------------------
  // Only the topics whose handler needs it are read -- the storage filter skips
  // image payloads entirely, so this pass does not pay to read the bulk of the
  // bag (the images) just to learn CSV columns.
  if (!serial_preprocess.empty())
  {
    run_pass(bag_dir, storage_id, serialization_format, &preprocess_topics, serial_preprocess, no_pool, workers,
             preprocess_total, "Preprocessing", PassKind::Preprocess);
  }

  if (g_stop.load())
  {
    std::cerr << "\nInterrupted during preprocessing; no output written (re-run to start over).\n";
    return;
  }

  for (auto& [topic, handler] : handlers)
  {
    handler->begin_write();
  }

  // --- Phase 2: write pass (all selected topics). ----------------------------
  run_pass(bag_dir, storage_id, serialization_format, nullptr, serial_write, pool_write, workers, total_messages,
           "Unbagging", PassKind::Write);

  for (auto& [topic, handler] : handlers)
  {
    handler->finish();
  }

  if (g_stop.load())
  {
    std::cerr << "\nInterrupted; flushed partial output for " << handlers.size()
              << " topic(s). Re-run without --overwrite to resume.\n";
    return;
  }

  std::cout << "Unbagged " << handlers.size() << " topic(s) from " << bag_dir << " into " << output_dir << "\n";
}

}  // namespace tabletop_unbag
