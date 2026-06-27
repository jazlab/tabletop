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

// rosbag2 storage plugin: records straight into the same analysis-ready HDF5
// layout that `unbag --format hdf5` produces, by flattening / decoding each
// message at write time and appending it to a shared Hdf5Writer.
//
// It is *write-only*. The analysis-ready layout (flattened columns + decoded
// image stacks) is lossy with respect to the original serialized message, so the
// bag is not round-trippable and playback/reading is not supported -- the read
// side of the interface throws. Use the mcap plugin when you need a faithful,
// replayable bag and run `unbag --format hdf5` on it afterwards.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include <rosbag2_storage/storage_interfaces/read_write_interface.hpp>

#include "tabletop_unbag/handlers/handler.hpp"
#include "tabletop_unbag/handlers/hdf5_handlers.hpp"
#include "tabletop_unbag/hdf5_writer.hpp"
#include "tabletop_unbag/options.hpp"

namespace tabletop_unbag
{

namespace fs = std::filesystem;
using rosbag2_storage::storage_interfaces::IOFlag;

/// rosbag2 storage plugin writing the analysis-ready HDF5 layout (see file
/// comment). Registered as storage id "hdf5".
class Hdf5Storage : public rosbag2_storage::storage_interfaces::ReadWriteInterface
{
public:
  Hdf5Storage() = default;
  ~Hdf5Storage() override;

  // --- BaseIOInterface -------------------------------------------------------
  void open(const rosbag2_storage::StorageOptions& storage_options, IOFlag io_flag) override;

  // --- BaseWriteInterface ----------------------------------------------------
  void write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg) override;
  void write(const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>& msgs) override;
  void create_topic(const rosbag2_storage::TopicMetadata& topic,
                    const rosbag2_storage::MessageDefinition& message_definition) override;
  void remove_topic(const rosbag2_storage::TopicMetadata& topic) override;
  void update_metadata(const rosbag2_storage::BagMetadata& bag_metadata) override;

  // --- BaseInfoInterface -----------------------------------------------------
  rosbag2_storage::BagMetadata get_metadata() override;
  std::string get_relative_file_path() const override;
  uint64_t get_bagfile_size() const override;
  std::string get_storage_identifier() const override;

  // --- BaseReadInterface (write-only: unsupported) ---------------------------
  bool set_read_order(const rosbag2_storage::ReadOrder& read_order) override;
  bool has_next() override;
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() override;
  std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types() override;
  void get_all_message_definitions(std::vector<rosbag2_storage::MessageDefinition>& definitions) override;

  // --- ReadOnlyInterface (write-only: unsupported) ---------------------------
  void set_filter(const rosbag2_storage::StorageFilter& storage_filter) override;
  void reset_filter() override;
  void seek(const rcutils_time_point_value_t& timestamp) override;

  // --- ReadWriteInterface ----------------------------------------------------
  uint64_t get_minimum_split_file_size() const override;

private:
  /// Everything we keep per recorded topic: its metadata, the handler that turns
  /// its messages into HDF5 (the same handler used by `unbag --format hdf5`), and
  /// a running message count for the metadata summary.
  struct TopicEntry
  {
    rosbag2_storage::TopicMetadata metadata;
    std::unique_ptr<MessageHandler> handler;  // null => topic skipped (non-CDR)
    uint64_t message_count = 0;
  };

  std::unique_ptr<Hdf5Writer> writer_;
  std::string absolute_path_;
  std::string relative_path_;
  UnbagOptions options_;
  std::unordered_map<std::string, TopicEntry> topics_;
  uint64_t total_count_ = 0;
  int64_t min_time_ns_ = 0;
  int64_t max_time_ns_ = 0;
  bool any_time_ = false;
  std::string ros_distro_;
  std::unordered_map<std::string, std::string> custom_data_;
};

Hdf5Storage::~Hdf5Storage()
{
  // rosbag2 closes a storage plugin by destroying it; flush every topic and the
  // file here. Never let an exception escape a destructor.
  try
  {
    for (auto& [name, entry] : topics_)
    {
      if (entry.handler)
      {
        entry.handler->finish();
      }
    }
    if (writer_)
    {
      writer_->close();
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "ERROR - hdf5 storage failed to close cleanly: " << e.what() << "\n";
  }
}

void Hdf5Storage::open(const rosbag2_storage::StorageOptions& storage_options, IOFlag io_flag)
{
  if (io_flag != IOFlag::READ_WRITE)
  {
    throw std::runtime_error("the 'hdf5' storage plugin is write-only: its analysis-ready layout is not "
                             "round-trippable, so reading/playback/append is not supported. Record with the "
                             "mcap plugin if you need a replayable bag.");
  }

  // On create, rosbag2 passes the bagfile base path and expects the plugin to
  // append its own extension. Tolerate a path that already ends in .h5.
  fs::path path(storage_options.uri);
  if (path.extension() != ".h5")
  {
    path += ".h5";
  }
  absolute_path_ = path.string();
  relative_path_ = path.filename().string();

  // v1 uses the converter's defaults (gzip 4, bgr8); per-recording overrides via
  // storage_config_uri can be added later.
  writer_ = std::make_unique<Hdf5Writer>(absolute_path_, options_.hdf5.gzip_level, options_.csv.batch_size);
}

void Hdf5Storage::create_topic(const rosbag2_storage::TopicMetadata& topic,
                               const rosbag2_storage::MessageDefinition& message_definition)
{
  (void)message_definition;
  if (topics_.find(topic.name) != topics_.end())
  {
    return;  // already created (rosbag2 may call once per topic; be idempotent)
  }

  TopicEntry entry;
  entry.metadata = topic;

  // The flatteners read the CDR payload directly; a non-CDR topic would be
  // misparsed, so skip it (a null handler makes write() a no-op for it).
  const std::string& fmt = topic.serialization_format;
  if (!fmt.empty() && fmt != "cdr")
  {
    std::cerr << "WARNING - hdf5 storage: skipping topic " << topic.name << " with serialization format '" << fmt
              << "' (only 'cdr' is supported).\n";
    topics_.emplace(topic.name, std::move(entry));
    return;
  }

  try
  {
    if (Hdf5ImageHandler::handles(topic.type))
    {
      entry.handler = std::make_unique<Hdf5ImageHandler>(TopicInfo{ topic.name, topic.type }, *writer_, options_);
    }
    else
    {
      entry.handler = std::make_unique<Hdf5CsvHandler>(TopicInfo{ topic.name, topic.type }, *writer_, options_);
    }
    entry.handler->prepare();
  }
  catch (const std::exception& e)
  {
    std::cerr << "WARNING - hdf5 storage: cannot handle topic " << topic.name << " (" << topic.type << "): " << e.what()
              << "; skipping.\n";
    entry.handler.reset();
  }
  topics_.emplace(topic.name, std::move(entry));
}

void Hdf5Storage::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  auto it = topics_.find(msg->topic_name);
  if (it == topics_.end())
  {
    // rosbag2 always create_topic()s before write(); a message for an unknown
    // topic is unexpected, but drop it rather than crash the recorder.
    return;
  }
  TopicEntry& entry = it->second;
  if (!entry.handler || msg->serialized_data == nullptr)
  {
    return;  // skipped topic (e.g. non-CDR)
  }

  const rcutils_uint8_array_t& data = *msg->serialized_data;
  const int64_t t = msg->recv_timestamp;

  // Drive the handler exactly as the offline pipeline does: note_for_write()
  // assigns the bag-order index (image handler uses it as the frame row), then
  // write() does the flatten/decode + append. write() is called sequentially by
  // rosbag2's writer thread, so this single-threaded ordering is correct.
  const uint64_t index = entry.handler->note_for_write(data, t);
  entry.handler->write(data, t, index);

  ++entry.message_count;
  ++total_count_;
  if (!any_time_)
  {
    min_time_ns_ = max_time_ns_ = t;
    any_time_ = true;
  }
  else
  {
    min_time_ns_ = std::min(min_time_ns_, t);
    max_time_ns_ = std::max(max_time_ns_, t);
  }
}

void Hdf5Storage::write(const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>& msgs)
{
  for (const auto& msg : msgs)
  {
    write(msg);
  }
}

void Hdf5Storage::remove_topic(const rosbag2_storage::TopicMetadata& topic)
{
  auto it = topics_.find(topic.name);
  if (it != topics_.end())
  {
    if (it->second.handler)
    {
      it->second.handler->finish();
    }
    topics_.erase(it);
  }
}

void Hdf5Storage::update_metadata(const rosbag2_storage::BagMetadata& bag_metadata)
{
  ros_distro_ = bag_metadata.ros_distro;
  custom_data_ = bag_metadata.custom_data;
}

rosbag2_storage::BagMetadata Hdf5Storage::get_metadata()
{
  rosbag2_storage::BagMetadata metadata;
  metadata.storage_identifier = get_storage_identifier();
  metadata.relative_file_paths = { relative_path_ };
  metadata.message_count = total_count_;
  metadata.bag_size = get_bagfile_size();
  metadata.ros_distro = ros_distro_;
  metadata.custom_data = custom_data_;

  for (const auto& [name, entry] : topics_)
  {
    rosbag2_storage::TopicInformation info;
    info.topic_metadata = entry.metadata;
    info.message_count = entry.message_count;
    metadata.topics_with_message_count.push_back(info);
  }

  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::nanoseconds;
  const auto start = high_resolution_clock::time_point(
      duration_cast<high_resolution_clock::duration>(nanoseconds(any_time_ ? min_time_ns_ : 0)));
  metadata.starting_time = start;
  metadata.duration = nanoseconds(any_time_ ? (max_time_ns_ - min_time_ns_) : 0);

  rosbag2_storage::FileInformation file_info;
  file_info.path = relative_path_;
  file_info.starting_time = start;
  file_info.duration = metadata.duration;
  file_info.message_count = total_count_;
  metadata.files = { file_info };
  return metadata;
}

std::string Hdf5Storage::get_relative_file_path() const
{
  return relative_path_;
}

uint64_t Hdf5Storage::get_bagfile_size() const
{
  std::error_code ec;
  const auto size = fs::file_size(absolute_path_, ec);
  return ec ? 0u : static_cast<uint64_t>(size);
}

std::string Hdf5Storage::get_storage_identifier() const
{
  return "hdf5";
}

uint64_t Hdf5Storage::get_minimum_split_file_size() const
{
  // Splitting into multiple HDF5 files mid-recording is not supported; report 0
  // so the default (no-split) configuration works. Setting --max-bag-size with
  // this plugin is not supported.
  return 0;
}

// --- Read side: unsupported for a write-only, non-round-trippable format. -----

namespace
{
[[noreturn]] void read_unsupported()
{
  throw std::runtime_error("the 'hdf5' storage plugin is write-only (analysis-ready, not round-trippable); "
                           "reading/playback is not supported.");
}
}  // namespace

bool Hdf5Storage::set_read_order(const rosbag2_storage::ReadOrder&)
{
  return false;
}

bool Hdf5Storage::has_next()
{
  return false;
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> Hdf5Storage::read_next()
{
  read_unsupported();
}

std::vector<rosbag2_storage::TopicMetadata> Hdf5Storage::get_all_topics_and_types()
{
  std::vector<rosbag2_storage::TopicMetadata> out;
  out.reserve(topics_.size());
  for (const auto& [name, entry] : topics_)
  {
    out.push_back(entry.metadata);
  }
  return out;
}

void Hdf5Storage::get_all_message_definitions(std::vector<rosbag2_storage::MessageDefinition>& definitions)
{
  definitions.clear();
}

void Hdf5Storage::set_filter(const rosbag2_storage::StorageFilter&)
{
}

void Hdf5Storage::reset_filter()
{
}

void Hdf5Storage::seek(const rcutils_time_point_value_t&)
{
  read_unsupported();
}

}  // namespace tabletop_unbag

PLUGINLIB_EXPORT_CLASS(tabletop_unbag::Hdf5Storage, rosbag2_storage::storage_interfaces::ReadWriteInterface)
