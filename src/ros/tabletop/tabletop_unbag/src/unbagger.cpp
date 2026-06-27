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
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/reindexer.hpp"
#include "rosbag2_storage/default_storage_id.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "tabletop_unbag/concurrent_queue.hpp"
#include "tabletop_unbag/handlers/csv_handler.hpp"
#include "tabletop_unbag/handlers/handler.hpp"
#include "tabletop_unbag/handlers/hdf5_handlers.hpp"
#include "tabletop_unbag/handlers/image_handler.hpp"
#include "tabletop_unbag/hdf5_writer.hpp"
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

/// A message paired with the write index note_for_write() assigned it on the
/// reader thread (in bag order). It is 0 in the preprocess pass, which does not
/// call note_for_write().
struct WorkItem
{
  MsgPtr msg;
  uint64_t write_index = 0;
};

/// Which lifecycle method a pass invokes on its handlers.
enum class PassKind
{
  Preprocess,
  Write,
};

void process_one(MessageHandler* handler, const WorkItem& item, PassKind kind)
{
  if (kind == PassKind::Preprocess)
  {
    handler->preprocess(*item.msg->serialized_data, item.msg->recv_timestamp);
  }
  else
  {
    handler->write(*item.msg->serialized_data, item.msg->recv_timestamp, item.write_index);
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

void open_reader(rosbag2_cpp::Reader& reader, const std::string& bag_dir, const std::string& storage_id)
{
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_dir;
  storage_options.storage_id = storage_id;

  // The reader determines each message's *input* serialization from the
  // per-topic metadata, so we leave input_serialization_format empty.
  //
  // For the *output* format we request "cdr": our handlers parse the payload as
  // PLAIN_CDR directly (the CSV flattener reads it with Fast CDR, the image
  // handler with rclcpp::Serialization), so they are not serialization-agnostic.
  // rosbag2 only inserts a converter when the requested output format differs
  // from what is stored, so for the usual cdr-on-mcap bag this is a no-op; it is
  // a safety net that converts to cdr if a bag were ever stored in another
  // format. (It is not derived from the active RMW: the dependence is on our
  // CDR-based deserializers, not on the installed middleware.)
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.output_serialization_format = "cdr";
  reader.open(storage_options, converter_options);
}

/// Throw a std::system_error if `ec` is set, prefixing it with `what` and the
/// offending `path`. Used to surface filesystem errors that the std::error_code
/// overloads otherwise swallow.
void throw_if_ec(const std::error_code& ec, const std::string& what, const fs::path& path)
{
  if (ec)
  {
    throw std::system_error(ec, what + ": " + path.string());
  }
}

/// Rebuild a missing metadata.yaml from the bag's storage files using the
/// rosbag2 reindexer, and return the storage id the metadata was written with.
///
/// The reindexer needs a storage id up front (it opens the storage plugin to
/// read the files), which is the one thing it cannot infer. We use the caller's
/// `--storage-id` override if given, otherwise the installed default storage
/// plugin (mcap on a stock Jazzy install). After reindexing we re-read the
/// freshly written metadata.yaml and return its storage_identifier, so the rest
/// of unbag() proceeds exactly as it would for a bag that shipped with metadata.
std::string reindex_and_detect_storage_id(const std::string& bag_dir, const std::optional<std::string>& storage_override)
{
  const std::string seed_storage_id = storage_override ? *storage_override : rosbag2_storage::get_default_storage_id();

  std::cerr << "INFO - " << rosbag2_storage::MetadataIo::metadata_filename << " missing in " << bag_dir
            << "; reindexing with storage id '" << seed_storage_id << "'.\n";

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_dir;
  storage_options.storage_id = seed_storage_id;

  try
  {
    rosbag2_cpp::Reindexer reindexer;
    reindexer.reindex(storage_options);
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("Failed to reindex bag '" + bag_dir + "' with storage id '" + seed_storage_id +
                             "'. Pass --storage-id to specify the storage plugin. (" + e.what() + ")");
  }

  rosbag2_storage::MetadataIo metadata_io;
  if (!metadata_io.metadata_file_exists(bag_dir))
  {
    throw std::runtime_error("Reindexing did not produce a metadata file in '" + bag_dir +
                             "'. Pass --storage-id to specify the storage plugin.");
  }

  const rosbag2_storage::BagMetadata metadata = metadata_io.read_metadata(bag_dir);
  return metadata.storage_identifier.empty() ? seed_storage_id : metadata.storage_identifier;
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
void run_pass(const std::string& bag_dir, const std::string& storage_id, const std::vector<std::string>* filter_topics,
              const std::unordered_map<std::string, MessageHandler*>& serial_handlers,
              const std::unordered_map<std::string, MessageHandler*>& pool_handlers, std::size_t pool_workers,
              uint64_t total, const std::string& label, PassKind kind)
{
  constexpr std::size_t kSerialQueueCap = 1024;

  // Per-topic serial queues and their consumer threads.
  std::unordered_map<std::string, std::unique_ptr<ConcurrentQueue<WorkItem>>> serial_queues;
  std::vector<std::thread> serial_threads;
  serial_threads.reserve(serial_handlers.size());
  for (const auto& [topic, handler] : serial_handlers)
  {
    auto queue = std::make_unique<ConcurrentQueue<WorkItem>>(kSerialQueueCap);
    ConcurrentQueue<WorkItem>* q = queue.get();
    serial_queues.emplace(topic, std::move(queue));
    serial_threads.emplace_back([q, handler, kind] {
      while (auto item = q->pop())
      {
        process_one(handler, *item, kind);
      }
    });
  }

  // Shared pool for per-message-parallel handlers.
  using PoolTask = std::pair<MessageHandler*, WorkItem>;
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
  open_reader(reader, bag_dir, storage_id);
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
      // note_for_write() runs here, on the reader, in bag order; the write pass
      // hands its return value back to write(). Serial handlers process in order
      // and don't need it, but the call is uniform (and a cheap default).
      uint64_t write_index = 0;
      if (kind == PassKind::Write)
      {
        write_index = serial_handlers.at(msg->topic_name)->note_for_write(*msg->serialized_data, msg->recv_timestamp);
      }
      sit->second->push(WorkItem{ std::move(msg), write_index });
      continue;
    }
    const auto pit = pool_handlers.find(msg->topic_name);
    if (pit != pool_handlers.end())
    {
      // The pool processes images out of bag order, so the write index that
      // disambiguates same-stamp frames must be assigned here, in bag order.
      uint64_t write_index = 0;
      if (kind == PassKind::Write)
      {
        write_index = pit->second->note_for_write(*msg->serialized_data, msg->recv_timestamp);
      }
      pool_queue.push(PoolTask{ pit->second, WorkItem{ std::move(msg), write_index } });
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
  // OpenCV's internal per-image threading. The default (--opencv-threads 1)
  // keeps it single-threaded because we already parallelize across images via
  // the worker pool (--jobs); letting OpenCV spawn threads per decode on top of
  // that would oversubscribe the cores. The best split of --opencv-threads vs
  // --jobs is machine- and bag-dependent and is left for the user to tune. 0
  // (cv::setNumThreads(0)) tells OpenCV to choose for itself.
  cv::setNumThreads(options.opencv_threads);

  // Stop gracefully on Ctrl-C / TERM (flush + join, see run_pass()).
  std::signal(SIGINT, handle_stop_signal);
  std::signal(SIGTERM, handle_stop_signal);

  // --- Infer storage id + topics from the bag metadata. ----------------------
  // The serialization format is *not* tracked here: the reader picks each
  // message's input format from the per-topic metadata, and open_reader()
  // requests cdr output for our deserializers (see open_reader()).
  uint64_t total_messages = 0;
  std::unordered_map<std::string, std::string> topic_types;
  std::unordered_map<std::string, uint64_t> topic_counts;

  rosbag2_storage::MetadataIo metadata_io;
  if (!metadata_io.metadata_file_exists(bag_dir))
  {
    // No metadata.yaml: rebuild it with the reindexer so the bag is readable
    // again, then carry on with the storage id detected from the new metadata.
    reindex_and_detect_storage_id(bag_dir, options.storage_id);
  }

  std::string storage_id = rosbag2_storage::get_default_storage_id();
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
    }
  }
  // An explicit --storage-id always wins over the inferred value.
  if (options.storage_id)
  {
    storage_id = *options.storage_id;
  }

  // Fall back to interrogating the opened bag if metadata was unavailable.
  if (topic_types.empty() || total_messages == 0)
  {
    rosbag2_cpp::Reader reader;
    open_reader(reader, bag_dir, storage_id);
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
  // Fail loudly if the output directory cannot be created -- otherwise the
  // handlers would all fail later with confusing per-file errors.
  std::error_code ec;
  fs::create_directories(output_dir, ec);
  throw_if_ec(ec, "Failed to create output directory", output_dir);

  // HDF5 output goes into one file shared by every handler. The serial HDF5
  // library is not thread-safe, so the writer serializes its own calls; the
  // per-message decode/flatten still parallelizes in the handlers before they
  // call in. Unlike the CSV backend, HDF5 rewrites the whole file rather than
  // resuming, so an existing file requires --overwrite.
  std::unique_ptr<Hdf5Writer> hdf5_writer;
  if (options.format == OutputFormat::Hdf5)
  {
    const fs::path h5_path = fs::path(output_dir) / "unbag.h5";
    if (!options.overwrite && fs::exists(h5_path))
    {
      throw std::runtime_error("HDF5 output " + h5_path.string() + " already exists; pass --overwrite to replace it.");
    }
    hdf5_writer = std::make_unique<Hdf5Writer>(h5_path.string(), options.hdf5.gzip_level, options.csv.batch_size);
  }

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
    std::unique_ptr<MessageHandler> handler;
    if (options.format == OutputFormat::Hdf5)
    {
      if (kind->name == ImageHandler::handler_name())
      {
        handler = std::make_unique<Hdf5ImageHandler>(TopicInfo{ topic, type }, *hdf5_writer, options);
      }
      else
      {
        handler = std::make_unique<Hdf5CsvHandler>(TopicInfo{ topic, type }, *hdf5_writer, options);
      }
    }
    else
    {
      handler = kind->make(TopicInfo{ topic, type }, output_dir, options);
    }
    handlers.emplace(topic, std::move(handler));
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
    run_pass(bag_dir, storage_id, &preprocess_topics, serial_preprocess, no_pool, workers, preprocess_total,
             "Preprocessing", PassKind::Preprocess);
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
  run_pass(bag_dir, storage_id, nullptr, serial_write, pool_write, workers, total_messages, "Unbagging",
           PassKind::Write);

  for (auto& [topic, handler] : handlers)
  {
    handler->finish();
  }

  // Flush and close the single HDF5 file (no-op for the CSV backend).
  if (hdf5_writer)
  {
    hdf5_writer->close();
  }

  // --- End-of-run summary: successes vs failures, per topic and aggregate. ----
  // The workers have joined (run_pass() returned), so reading stats() here is
  // safe. A topic's failure is usually all-or-nothing -- a partial count is the
  // signal worth investigating, so per-topic failing counts are always shown,
  // with the fully-successful topics summarized in one line unless --verbose.
  HandlerStats totals;
  std::size_t topics_with_failures = 0;
  std::size_t total_duplicates = 0;
  for (const auto& [topic, handler] : handlers)
  {
    const HandlerStats s = handler->stats();
    const std::size_t dups = handler->duplicate_count();
    totals.succeeded += s.succeeded;
    totals.failed += s.failed;
    total_duplicates += dups;
    if (s.failed != 0)
    {
      ++topics_with_failures;
    }
    if (s.failed != 0 || dups != 0 || options.verbose)
    {
      std::cerr << "  " << topic << ": " << s.succeeded << " ok, " << s.failed << " failed";
      if (dups != 0)
      {
        std::cerr << ", " << dups << " duplicate-stamp frame(s) renamed";
      }
      std::cerr << "\n";
    }
  }
  std::cerr << "Messages: " << totals.succeeded << " unbagged, " << totals.failed << " failed across "
            << handlers.size() << " topic(s)";
  if (topics_with_failures != 0)
  {
    std::cerr << " (" << topics_with_failures << " topic(s) had failures)";
  }
  if (total_duplicates != 0)
  {
    std::cerr << "; " << total_duplicates << " duplicate-stamp frame(s) renamed";
  }
  std::cerr << ".\n";

  if (g_stop.load())
  {
    std::cerr << "\nInterrupted; flushed partial output for " << handlers.size()
              << " topic(s). Re-run without --overwrite to resume.\n";
    return;
  }

  std::cout << "Unbagged " << handlers.size() << " topic(s) from " << bag_dir << " into " << output_dir << "\n";
}

}  // namespace tabletop_unbag
