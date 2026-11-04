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

// A read-throughput benchmark that isolates the storage-read cost of unbagging.
//
// It times a full scan of a bag's messages two ways, with no decoding or
// writing, so the only thing measured is how fast each path pulls the bag off
// disk:
//   * "rosbag2" -- rosbag2_cpp::Reader, exactly as tabletop_unbag's unbagger
//     drives it: the MCAP plugin walks the file in *log-time* order using the
//     message index, which seeks between the bag's many small, time-overlapping
//     chunks.
//   * "mcap"    -- mcap::McapReader iterated in *file* order (by default), one
//     forward sequential scan of the file, served by either a buffered reader
//     (--io buffered, the default and the fast one) or an mmap (--io mmap, the
//     measured-slower baseline).
//
// To make the cold-cache cost visible (the case that matters for a first
// unbag), it can evict the bag's pages from the OS page cache before each run
// with posix_fadvise(POSIX_FADV_DONTNEED) -- which needs no root, unlike
// `echo 3 > /proc/sys/vm/drop_caches`. Pass --no-evict to measure the warm
// (page-cache-resident) case instead.

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "mcap/reader.hpp"

namespace fs = std::filesystem;

namespace
{

/// An mcap::IReadable that memory-maps the whole file and hands the reader
/// pointers straight into the mapping (zero copy). Included here only as the
/// baseline the benchmark measures against buffered reads: it turns out to be
/// *slower* than a buffered pread loop for a full linear scan (faulting a
/// 100+ GB file in 4 KB pages is tens of millions of minor faults), so the
/// converter does not use it -- see docs/performance.md.
class MmapReadable final : public mcap::IReadable
{
public:
  explicit MmapReadable(const std::string& path)
  {
    fd_ = ::open(path.c_str(), O_RDONLY);
    if (fd_ < 0)
    {
      throw std::runtime_error("MmapReadable: cannot open " + path);
    }
    struct stat st{};
    if (::fstat(fd_, &st) != 0)
    {
      ::close(fd_);
      throw std::runtime_error("MmapReadable: cannot stat " + path);
    }
    size_ = static_cast<uint64_t>(st.st_size);
    if (size_ != 0)
    {
      void* base = ::mmap(nullptr, size_, PROT_READ, MAP_PRIVATE, fd_, 0);
      if (base == MAP_FAILED)
      {
        ::close(fd_);
        throw std::runtime_error("MmapReadable: mmap failed for " + path);
      }
      base_ = static_cast<std::byte*>(base);
      ::madvise(base_, size_, MADV_SEQUENTIAL);
    }
  }

  ~MmapReadable() override
  {
    if (base_ != nullptr)
    {
      ::munmap(base_, size_);
    }
    if (fd_ >= 0)
    {
      ::close(fd_);
    }
  }

  MmapReadable(const MmapReadable&) = delete;
  MmapReadable& operator=(const MmapReadable&) = delete;

  uint64_t size() const override
  {
    return size_;
  }

  uint64_t read(std::byte** output, uint64_t offset, uint64_t size) override
  {
    if (output == nullptr || offset >= size_)
    {
      return 0;
    }
    const uint64_t available = std::min(size, size_ - offset);
    *output = base_ + offset;
    return available;
  }

private:
  int fd_ = -1;
  std::byte* base_ = nullptr;
  uint64_t size_ = 0;
};

struct ScanResult
{
  uint64_t messages = 0;
  uint64_t payload_bytes = 0;
  double seconds = 0.0;
};

/// Find the .mcap file inside a bag directory (or accept a path to one).
std::string resolve_mcap_path(const std::string& bag)
{
  if (fs::is_regular_file(bag) && fs::path(bag).extension() == ".mcap")
  {
    return bag;
  }
  if (fs::is_directory(bag))
  {
    std::string best;
    for (const auto& entry : fs::directory_iterator(bag))
    {
      if (entry.path().extension() == ".mcap")
      {
        // Prefer the first split (bag_0.mcap) if there are several.
        if (best.empty() || entry.path().filename().string() < fs::path(best).filename().string())
        {
          best = entry.path().string();
        }
      }
    }
    if (!best.empty())
    {
      return best;
    }
  }
  throw std::runtime_error("No .mcap file found at " + bag);
}

/// Best-effort eviction of a file's pages from the OS page cache, so the next
/// read is cold. Needs no privileges (unlike drop_caches).
void evict_from_cache(const std::string& path)
{
  const int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0)
  {
    return;
  }
  ::posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED);
  ::close(fd);
}

double human_throughput_mb_s(uint64_t bytes, double seconds)
{
  if (seconds <= 0.0)
  {
    return 0.0;
  }
  return (static_cast<double>(bytes) / (1024.0 * 1024.0)) / seconds;
}

/// Scan the bag with rosbag2_cpp::Reader (log-time order, indexed), stopping
/// after `max_messages` (0 = no limit).
ScanResult scan_rosbag2(const std::string& bag_dir, uint64_t max_messages)
{
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_dir;
  storage_options.storage_id = "mcap";

  rosbag2_cpp::Reader reader;
  reader.open(storage_options);

  ScanResult result;
  const auto t0 = std::chrono::steady_clock::now();
  while (reader.has_next())
  {
    const auto msg = reader.read_next();
    result.payload_bytes += msg->serialized_data->buffer_length;
    ++result.messages;
    if (max_messages != 0 && result.messages >= max_messages)
    {
      break;
    }
  }
  const auto t1 = std::chrono::steady_clock::now();
  result.seconds = std::chrono::duration<double>(t1 - t0).count();
  return result;
}

/// Scan the bag with mcap::McapReader, in the given order, using either an mmap
/// or a buffered (ifstream pread) IReadable, stopping after `max_messages`
/// (0 = no limit).
ScanResult scan_mcap(const std::string& mcap_path, mcap::ReadMessageOptions::ReadOrder order, bool use_mmap,
                     uint64_t max_messages)
{
  // Hold whichever IReadable we picked plus, for the buffered case, the
  // ifstream it wraps, for the lifetime of the scan.
  std::ifstream stream;
  std::unique_ptr<mcap::IReadable> readable;
  if (use_mmap)
  {
    readable = std::make_unique<MmapReadable>(mcap_path);
  }
  else
  {
    stream.open(mcap_path, std::ios::binary);
    if (!stream)
    {
      throw std::runtime_error("cannot open " + mcap_path);
    }
    readable = std::make_unique<mcap::FileStreamReader>(stream);
  }

  mcap::McapReader reader;
  const mcap::Status status = reader.open(*readable);
  if (!status.ok())
  {
    throw std::runtime_error("mcap open failed: " + status.message);
  }

  mcap::ReadMessageOptions options;
  options.readOrder = order;
  // Touch every byte of every payload so the read is not optimized away and the
  // pages are actually faulted in -- this is the work a real consumer forces.
  const mcap::ProblemCallback on_problem = [](const mcap::Status&) {};

  ScanResult result;
  volatile uint64_t checksum = 0;
  const auto t0 = std::chrono::steady_clock::now();
  for (const auto& view : reader.readMessages(on_problem, options))
  {
    const std::byte* data = view.message.data;
    const uint64_t n = view.message.dataSize;
    // Sample a few bytes so the mapping is genuinely paged in without paying a
    // full memcpy; the kernel still has to fetch the page either way.
    if (n > 0)
    {
      checksum += static_cast<uint64_t>(data[0]);
      checksum += static_cast<uint64_t>(data[n / 2]);
      checksum += static_cast<uint64_t>(data[n - 1]);
    }
    result.payload_bytes += n;
    ++result.messages;
    if (max_messages != 0 && result.messages >= max_messages)
    {
      break;
    }
  }
  const auto t1 = std::chrono::steady_clock::now();
  (void)checksum;
  reader.close();
  result.seconds = std::chrono::duration<double>(t1 - t0).count();
  return result;
}

void report(const std::string& label, const ScanResult& r)
{
  std::cout << "  " << label << ":\n";
  std::cout << "    messages: " << r.messages << "\n";
  std::cout << "    payload : " << (r.payload_bytes / (1024 * 1024)) << " MiB\n";
  std::cout << "    time    : " << r.seconds << " s\n";
  std::cout << "    payload throughput: " << human_throughput_mb_s(r.payload_bytes, r.seconds) << " MiB/s\n";
}

}  // namespace

int main(int argc, char** argv)
{
  std::string bag;
  bool evict = true;
  std::string which = "both";   // rosbag2 | mcap | both
  std::string order = "file";   // file | logtime (mcap only)
  std::string io = "buffered";  // buffered | mmap (mcap only)
  uint64_t max_messages = 0;    // 0 = whole bag

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--no-evict")
    {
      evict = false;
    }
    else if (arg == "--reader" && i + 1 < argc)
    {
      which = argv[++i];
    }
    else if (arg == "--order" && i + 1 < argc)
    {
      order = argv[++i];
    }
    else if (arg == "--io" && i + 1 < argc)
    {
      io = argv[++i];
    }
    else if (arg == "--max-messages" && i + 1 < argc)
    {
      max_messages = std::stoull(argv[++i]);
    }
    else if (arg == "-h" || arg == "--help")
    {
      std::cout << "Usage: " << argv[0]
                << " BAG_DIR_OR_MCAP [--no-evict] [--reader rosbag2|mcap|both]\n"
                   "         [--order file|logtime] [--io buffered|mmap] [--max-messages N]\n";
      return 0;
    }
    else if (!arg.empty() && arg[0] != '-')
    {
      bag = arg;
    }
  }

  if (bag.empty())
  {
    std::cerr << "ERROR - a bag directory or .mcap file is required\n";
    return 2;
  }

  const std::string mcap_path = resolve_mcap_path(bag);
  const std::string bag_dir = fs::is_directory(bag) ? bag : fs::path(bag).parent_path().string();
  const mcap::ReadMessageOptions::ReadOrder read_order = order == "logtime" ?
                                                             mcap::ReadMessageOptions::ReadOrder::LogTimeOrder :
                                                             mcap::ReadMessageOptions::ReadOrder::FileOrder;

  const bool use_mmap = io == "mmap";

  std::cout << "Benchmarking " << mcap_path << "\n";
  std::cout << "  cache: " << (evict ? "COLD (evicted via posix_fadvise before each run)" : "WARM (as-is)") << "\n";
  if (max_messages != 0)
  {
    std::cout << "  limit: first " << max_messages << " messages\n";
  }
  std::cout << "\n";

  try
  {
    if (which == "rosbag2" || which == "both")
    {
      if (evict)
      {
        evict_from_cache(mcap_path);
      }
      report("rosbag2_cpp::Reader (log-time order, indexed)", scan_rosbag2(bag_dir, max_messages));
    }
    if (which == "mcap" || which == "both")
    {
      if (evict)
      {
        evict_from_cache(mcap_path);
      }
      report("mcap " + io + " (" + order + " order)", scan_mcap(mcap_path, read_order, use_mmap, max_messages));
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "ERROR - " << e.what() << "\n";
    return 1;
  }
  return 0;
}
