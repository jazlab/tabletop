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

#include "tabletop_unbag/value_formatter.hpp"

#include <array>
#include <charconv>
#include <cmath>

namespace tabletop_unbag
{

namespace
{

/// Shared implementation for float and double formatting.
template <typename T>
std::string float_to_string(T value)
{
  if (std::isnan(value))
  {
    // pandas writes NaN cells using na_rep, which defaults to the empty string.
    return "";
  }
  if (std::isinf(value))
  {
    return value < 0 ? "-inf" : "inf";
  }

  // std::to_chars (no format/precision) yields the shortest string that
  // round-trips back to the same value -- the same property Python's repr()
  // relies on, so for normal-magnitude values the two agree exactly.
  std::array<char, 64> buf;
  auto result = std::to_chars(buf.data(), buf.data() + buf.size(), value);
  std::string str(buf.data(), result.ptr);

  // pandas keeps a decimal point on integral floats ("1.0", "-3.0"). to_chars
  // drops it ("1", "-3"), so re-add it unless the value is in scientific form.
  if (str.find('.') == std::string::npos && str.find('e') == std::string::npos && str.find('E') == std::string::npos)
  {
    str += ".0";
  }
  return str;
}

}  // namespace

std::string format_double(double value)
{
  return float_to_string(value);
}

std::string format_float(float value)
{
  return float_to_string(value);
}

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

}  // namespace tabletop_unbag
