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

#ifndef TABLETOP_UNBAG__VALUE_FORMATTER_HPP_
#define TABLETOP_UNBAG__VALUE_FORMATTER_HPP_

#include <string>

namespace tabletop_unbag
{

/// Format a double as a CSV cell, matching pandas' `DataFrame.to_csv` output.
///
/// Uses the shortest round-tripping decimal representation (`std::to_chars`,
/// the same algorithm behind Python's `repr(float)`), keeps a trailing ".0"
/// for integral values (pandas writes `1.0`, not `1`), renders NaN as the
/// empty string (pandas' default `na_rep`) and +/-inf as "inf"/"-inf".
std::string format_double(double value);

/// Format a float as a CSV cell. See format_double for the formatting rules.
std::string format_float(float value);

/// Quote a CSV field per RFC 4180, matching Python's csv.QUOTE_MINIMAL: the
/// field is wrapped in double quotes only if it contains a comma, a double
/// quote, or a newline/carriage return, and embedded double quotes are doubled.
std::string csv_quote(const std::string& field);

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__VALUE_FORMATTER_HPP_
