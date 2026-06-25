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
/// Reads the bag's metadata to infer the storage plugin (overridable via
/// options.storage_id); if metadata.yaml is missing it is first rebuilt with
/// the rosbag2 reindexer and the storage id read back from the result. Routes
/// each selected topic to the handler that claims its message type, and runs
/// the preprocess pass (if any handler needs it) followed by the write pass.
/// The serialization format is not inferred or overridable: the reader takes
/// each message's input format from the per-topic metadata, and the converter
/// always targets CDR for the handlers' deserializers.
///
/// \param bag_dir Directory that directly contains the bag's .mcap files and
///   metadata.yaml.
/// \param output_dir Directory to write CSVs and image subdirectories into
///   (created if necessary).
/// \throws std::runtime_error on failure to open or reindex the bag, or when an
///   existing output would be corrupted by a resume (use options.overwrite
///   instead).
/// \throws std::system_error if the output directory cannot be created.
void unbag(const std::string& bag_dir, const std::string& output_dir, const UnbagOptions& options);

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__UNBAGGER_HPP_
