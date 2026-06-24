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

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "tabletop_unbag/unbagger.hpp"

namespace fs = std::filesystem;

namespace
{

void print_usage(const char* argv0)
{
  std::cout << "Usage: " << argv0
            << " BAG_DIR [options]\n\n"
               "Unbag a ROS 2 bag (MCAP) into per-topic outputs: CSV files for normal\n"
               "messages and image files for image topics. Each message type is routed to\n"
               "a handler that knows how to write it.\n\n"
               "Positional arguments:\n"
               "  BAG_DIR                 Directory containing the bag (its .mcap files and\n"
               "                          metadata.yaml).\n\n"
               "Options:\n"
               "  -o, --output-dir DIR    Where to write outputs.\n"
               "                          Default: <parent of BAG_DIR>/unbag_output\n"
               "  --handlers H [H ...]    Handlers to enable (e.g. csv image). Topics whose\n"
               "                          type is not claimed by an enabled handler are\n"
               "                          skipped. Default: all handlers.\n"
               "  --topics T [T ...]      Only unbag these topics.\n"
               "  --exclude-topics T ...  Unbag all topics except these.\n"
               "                          (--topics and --exclude-topics are mutually\n"
               "                          exclusive.)\n"
               "  --overwrite             Delete previously unbagged output for the selected\n"
               "                          topics before writing. Without it, an interrupted\n"
               "                          run resumes where it left off.\n"
               "  --batch-size N          Messages buffered in memory before flushing to\n"
               "                          disk (default 1000).\n"
               "  --image-encoding ENC    Target encoding for saved images (default bgr8).\n"
               "  --storage-id ID         Storage plugin override (default: inferred,\n"
               "                          fallback mcap).\n"
               "  --serialization-format F  Serialization override (default: inferred,\n"
               "                          fallback cdr).\n"
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
    if (next.size() > 1 && next[0] == '-' && !std::isdigit(static_cast<unsigned char>(next[1])))
    {
      break;
    }
    values.push_back(next);
    ++i;
  }
  return values;
}

/// Strip trailing path separators so parent_path() of "a/b/" yields "a", not
/// "a/b".
std::string strip_trailing_slash(std::string path)
{
  while (path.size() > 1 && path.back() == '/')
  {
    path.pop_back();
  }
  return path;
}

}  // namespace

int main(int argc, char** argv)
{
  tabletop_unbag::UnbagOptions options;
  std::vector<std::string> positionals;
  std::string output_dir;
  bool have_topics = false;
  bool have_exclude = false;

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help")
    {
      print_usage(argv[0]);
      return 0;
    }
    else if (arg == "-o" || arg == "--output-dir")
    {
      if (i + 1 >= argc)
      {
        std::cerr << "ERROR - " << arg << " requires a value\n";
        return 2;
      }
      output_dir = argv[++i];
    }
    else if (arg == "--handlers")
    {
      options.handlers = consume_list(argc, argv, i);
    }
    else if (arg == "--topics")
    {
      options.topics = consume_list(argc, argv, i);
      have_topics = true;
    }
    else if (arg == "--exclude-topics")
    {
      options.exclude_topics = consume_list(argc, argv, i);
      have_exclude = true;
    }
    else if (arg == "--overwrite")
    {
      options.overwrite = true;
    }
    else if (arg == "--batch-size")
    {
      if (i + 1 >= argc)
      {
        std::cerr << "ERROR - " << arg << " requires a value\n";
        return 2;
      }
      try
      {
        const long long value = std::stoll(argv[++i]);
        if (value <= 0)
        {
          throw std::out_of_range("batch size must be positive");
        }
        options.batch_size = static_cast<std::size_t>(value);
      }
      catch (const std::exception&)
      {
        std::cerr << "ERROR - --batch-size requires a positive integer\n";
        return 2;
      }
    }
    else if (arg == "--image-encoding")
    {
      if (i + 1 >= argc)
      {
        std::cerr << "ERROR - " << arg << " requires a value\n";
        return 2;
      }
      options.image_encoding = argv[++i];
    }
    else if (arg == "--storage-id")
    {
      if (i + 1 >= argc)
      {
        std::cerr << "ERROR - " << arg << " requires a value\n";
        return 2;
      }
      options.storage_id = argv[++i];
    }
    else if (arg == "--serialization-format")
    {
      if (i + 1 >= argc)
      {
        std::cerr << "ERROR - " << arg << " requires a value\n";
        return 2;
      }
      options.serialization_format = argv[++i];
    }
    else if (arg == "-v" || arg == "--verbose")
    {
      options.verbose = true;
    }
    else if (!arg.empty() && arg[0] == '-')
    {
      std::cerr << "ERROR - Unknown argument: " << arg << "\n";
      print_usage(argv[0]);
      return 2;
    }
    else
    {
      positionals.push_back(arg);
    }
  }

  // --- Validate. -------------------------------------------------------------
  if (positionals.empty())
  {
    std::cerr << "ERROR - A bag directory is required\n";
    print_usage(argv[0]);
    return 2;
  }
  if (positionals.size() > 1)
  {
    std::cerr << "ERROR - Expected a single bag directory, got " << positionals.size() << "\n";
    return 2;
  }
  const std::string bag_dir = positionals.front();

  if (have_topics && have_exclude)
  {
    std::cerr << "ERROR - --topics and --exclude-topics are mutually exclusive\n";
    return 2;
  }

  for (const auto& name : options.handlers)
  {
    const auto& valid = tabletop_unbag::handler_names();
    if (std::find(valid.begin(), valid.end(), name) == valid.end())
    {
      std::cerr << "ERROR - Unknown handler '" << name << "'. Valid handlers:";
      for (const auto& v : valid)
      {
        std::cerr << ' ' << v;
      }
      std::cerr << "\n";
      return 2;
    }
  }

  std::error_code ec;
  if (!fs::is_directory(bag_dir, ec))
  {
    std::cerr << "ERROR - Not a directory: " << bag_dir << "\n";
    return 1;
  }

  // --- Resolve the output directory. -----------------------------------------
  if (output_dir.empty())
  {
    const fs::path bag_path = strip_trailing_slash(bag_dir);
    fs::path parent = bag_path.parent_path();
    if (parent.empty())
    {
      parent = ".";
    }
    output_dir = (parent / "unbag_output").string();
  }

  try
  {
    tabletop_unbag::unbag(bag_dir, output_dir, options);
  }
  catch (const std::exception& e)
  {
    std::cerr << "ERROR - " << e.what() << "\n";
    return 1;
  }

  return 0;
}
