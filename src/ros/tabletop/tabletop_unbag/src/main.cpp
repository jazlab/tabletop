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

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "tabletop_unbag/bag_converter.hpp"

namespace fs = std::filesystem;

namespace
{

void print_usage(const char* argv0)
{
  std::cout << "Usage: " << argv0
            << " [options]\n\n"
               "Convert ROS 2 bag files (MCAP) to per-topic CSV files. A session\n"
               "directory contains one or more bag subdirectories; one CSV is written per\n"
               "topic into the session directory.\n\n"
               "Options:\n"
               "  -d, --session-dir DIR   Session directory to convert.\n"
               "                          Default: $ROS_BAG_DIR/latest\n"
               "  -a, --all-sessions      Convert all session directories in $ROS_BAG_DIR.\n"
               "  --topics T [T ...]      Whitelist of topics to include (default: all).\n"
               "  --exclude-topics T ...  Topics to exclude.\n"
               "  --image                 Export image topics as image files.\n"
               "  -f, --force             Overwrite existing CSV/image files.\n"
               "  -v, --verbose           Increase logging verbosity.\n"
               "  -h, --help              Show this help message and exit.\n";
}

/// Consume the values following a list-valued flag, stopping at the next flag
/// (an argument starting with '-') or the end of the argument list.
std::vector<std::string> consume_list(int argc, char** argv, int& i)
{
  std::vector<std::string> values;
  while (i + 1 < argc)
  {
    const std::string next = argv[i + 1];
    if (!next.empty() && next[0] == '-')
    {
      break;
    }
    values.push_back(next);
    ++i;
  }
  return values;
}

}  // namespace

int main(int argc, char** argv)
{
  std::optional<std::string> session_dir;
  bool all_sessions = false;
  tabletop_unbag::ConvertOptions options;

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help")
    {
      print_usage(argv[0]);
      return 0;
    }
    else if (arg == "-d" || arg == "--session-dir")
    {
      if (i + 1 >= argc)
      {
        std::cerr << "ERROR - " << arg << " requires a value\n";
        return 2;
      }
      session_dir = argv[++i];
    }
    else if (arg == "-a" || arg == "--all-sessions")
    {
      all_sessions = true;
    }
    else if (arg == "--topics")
    {
      options.topics = consume_list(argc, argv, i);
    }
    else if (arg == "--exclude-topics")
    {
      options.exclude_topics = consume_list(argc, argv, i);
    }
    else if (arg == "--image")
    {
      options.convert_images = true;
    }
    else if (arg == "-f" || arg == "--force")
    {
      options.force = true;
    }
    else if (arg == "-v" || arg == "--verbose")
    {
      options.verbose = true;
    }
    else
    {
      std::cerr << "ERROR - Unknown argument: " << arg << "\n";
      print_usage(argv[0]);
      return 2;
    }
  }

  const char* ros_bag_dir = std::getenv("ROS_BAG_DIR");

  // Build the list of session directories to process.
  std::vector<std::string> session_dirs;
  if (all_sessions)
  {
    if (ros_bag_dir == nullptr)
    {
      std::cerr << "ERROR - --all-sessions requires the ROS_BAG_DIR "
                << "environment variable to be set\n";
      return 1;
    }
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(ros_bag_dir, ec))
    {
      session_dirs.push_back(entry.path().string());
    }
    if (session_dirs.empty())
    {
      std::cerr << "ERROR - No session directories found in ROS_BAG_DIR (" << ros_bag_dir << ")\n";
      return 1;
    }
  }
  else if (session_dir)
  {
    session_dirs.push_back(*session_dir);
  }
  else
  {
    if (ros_bag_dir == nullptr)
    {
      std::cerr << "ERROR - No --session-dir given and ROS_BAG_DIR is not set\n";
      return 1;
    }
    session_dirs.push_back((fs::path(ros_bag_dir) / "latest").string());
  }

  // Canonicalize, drop non-directories, and de-duplicate (a symlink such as
  // "latest" and its target must not be converted twice).
  std::set<std::string> resolved;
  for (const auto& dir : session_dirs)
  {
    std::error_code ec;
    if (!fs::is_directory(dir, ec))
    {
      continue;
    }
    resolved.insert(fs::canonical(dir, ec).string());
  }

  for (const auto& dir : resolved)
  {
    // Skip sessions that already have CSVs unless --force.
    bool has_csv = false;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(dir, ec))
    {
      if (entry.path().extension() == ".csv")
      {
        has_csv = true;
        break;
      }
    }
    if (has_csv && !options.force)
    {
      std::cerr << "WARNING - " << dir << " already converted, skipping...\n";
      continue;
    }

    try
    {
      tabletop_unbag::rosbag_session_to_csv(dir, options);
    }
    catch (const std::exception& e)
    {
      std::cerr << "ERROR - Error converting " << dir << ": " << e.what() << "\n";
      continue;
    }

    std::cout << std::string(80, '-') << "\n";
  }

  return 0;
}
