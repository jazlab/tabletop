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

#ifndef TABLETOP_UNBAG__FLATTEN_HPP_
#define TABLETOP_UNBAG__FLATTEN_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "rcpputils/shared_library.hpp"
#include "rcutils/types/uint8_array.h"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace tabletop_unbag
{

/// One typed scalar leaf produced by flattening a message.
///
/// The alternatives are deliberately fine-grained so that **both** consumers
/// get what they need from one flatten pass:
///   * the CSV backend needs `float` and `double` kept distinct because their
///     shortest round-tripping text differs (`0.1f` -> "0.1", but the same bits
///     widened to double -> "0.10000000149011612"), and it formats signed and
///     unsigned integers from their own values;
///   * the HDF5 backend maps each alternative to a native HDF5 type, so a
///     `float32` field lands in an H5T_NATIVE_FLOAT dataset, an unsigned field
///     in an unsigned dataset, and so on.
///
/// Integers are widened to int64/uint64 (preserving signedness and value); the
/// exact 8/16/32-bit width is not retained, which is lossless for text and a
/// negligible, gzip-friendly storage cost in HDF5. wstrings are converted to
/// UTF-8 and stored as String.
using FlatScalar = std::variant<bool, int64_t, uint64_t, float, double, std::string>;

/// A flattened column: its dotted/bracketed name and typed value.
struct FlatColumn
{
  std::string name;
  FlatScalar value;
};

/// A flattened message: ordered (column, value) pairs, in first-seen field
/// order (the same order the CSV header uses).
using FlatRow = std::vector<FlatColumn>;

/// Loads the runtime introspection type support for one ROS message type and
/// flattens serialized CDR messages of that type into ordered, typed
/// (column, value) pairs.
///
/// Nested submessages become dot-separated columns (`pose.position.x`); fixed
/// arrays and (bounded/unbounded) sequences become bracket-indexed columns
/// (`name[0]`, `position[2]`). This is the shared core behind both the CSV and
/// HDF5 backends: the CSV handler formats the values pandas-style, the HDF5
/// handler stores them natively, but the field walk and CDR decode are here, in
/// one place.
///
/// Thread model: call prepare() once on the main thread to load the type-support
/// shared library (loading a library concurrently from several threads is
/// unsafe); after that, flatten() is const and may be called concurrently from
/// different threads on the same instance.
class MessageFlattener
{
public:
  explicit MessageFlattener(std::string ros_type);

  /// Load the introspection type support now (on the calling thread). Idempotent.
  void prepare();

  /// Flatten one serialized (PLAIN_CDR / XCDRv1) message. prepare() must have
  /// been called first. Throws std::runtime_error on an unsupported field type
  /// or a truncated/garbled buffer.
  FlatRow flatten(const rcutils_uint8_array_t& data) const;

  const std::string& ros_type() const
  {
    return ros_type_;
  }

private:
  std::string ros_type_;
  std::shared_ptr<rcpputils::SharedLibrary> type_support_library_;
  const rosidl_typesupport_introspection_cpp::MessageMembers* members_ = nullptr;
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__FLATTEN_HPP_
