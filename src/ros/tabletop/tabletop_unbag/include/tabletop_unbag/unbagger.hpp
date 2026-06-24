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

#ifndef TABLETOP_UNBAG__UNBAGGER_HPP_
#define TABLETOP_UNBAG__UNBAGGER_HPP_

#include <string>
#include <vector>

#include "tabletop_unbag/options.hpp"

namespace tabletop_unbag
{

/// The registry names of every handler the unbagger knows about, in dispatch
/// priority order (the first one that claims a message type wins). Used by the
/// CLI to validate --handlers.
const std::vector<std::string>& handler_names();

/// Unbag a single bag directory into `output_dir`.
///
/// Reads the bag's metadata to infer the storage plugin and serialization
/// format (overridable via options), routes each selected topic to the handler
/// that claims its message type, and runs the preprocess pass (if any handler
/// needs it) followed by the write pass.
///
/// \param bag_dir Directory that directly contains the bag's .mcap files and
///   metadata.yaml.
/// \param output_dir Directory to write CSVs and image subdirectories into
///   (created if necessary).
/// \throws std::runtime_error on failure to open the bag, or when an existing
///   output would be corrupted by a resume (use options.overwrite instead).
void unbag(const std::string& bag_dir, const std::string& output_dir, const UnbagOptions& options);

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__UNBAGGER_HPP_
