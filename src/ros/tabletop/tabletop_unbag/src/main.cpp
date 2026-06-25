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
               "                          Default: <parent of BAG_DIR>/unbag\n"
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
               "  --jobs N                Worker threads for the shared image-decoding pool\n"
               "                          (default: number of hardware threads). Each CSV\n"
               "                          topic also runs on its own consumer thread.\n"
               "  --opencv-threads N      Threads OpenCV uses internally per image decode\n"
               "                          (default 1). We already parallelize across images\n"
               "                          via --jobs, so 1 avoids oversubscribing cores; tune\n"
               "                          against --jobs on your machine. 0 lets OpenCV pick.\n"
               "  --image-encoding ENC    Target encoding for saved images (default bgr8).\n"
               "  --storage-id ID         Storage plugin override (default: inferred from the\n"
               "                          bag metadata, reindexing first if metadata.yaml is\n"
               "                          missing; fallback to the installed default plugin).\n"
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
    else if (arg == "--jobs")
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
          throw std::out_of_range("jobs must be positive");
        }
        options.jobs = static_cast<std::size_t>(value);
      }
      catch (const std::exception&)
      {
        std::cerr << "ERROR - --jobs requires a positive integer\n";
        return 2;
      }
    }
    else if (arg == "--opencv-threads")
    {
      if (i + 1 >= argc)
      {
        std::cerr << "ERROR - " << arg << " requires a value\n";
        return 2;
      }
      try
      {
        // 0 is allowed: it tells OpenCV to choose the thread count itself.
        const long long value = std::stoll(argv[++i]);
        if (value < 0)
        {
          throw std::out_of_range("opencv threads must be non-negative");
        }
        options.opencv_threads = static_cast<int>(value);
      }
      catch (const std::exception&)
      {
        std::cerr << "ERROR - --opencv-threads requires a non-negative integer\n";
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
    output_dir = (parent / "unbag").string();
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
