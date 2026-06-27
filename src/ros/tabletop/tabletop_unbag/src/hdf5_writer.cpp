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

#include "tabletop_unbag/hdf5_writer.hpp"

#include <hdf5.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "tabletop_unbag/handlers/handler.hpp"  // topic_to_basename

namespace tabletop_unbag
{

namespace
{

constexpr hsize_t kColumnChunk = 4096;  // rows per chunk for 1-D column datasets

/// Throw if an HDF5 id/status is negative (the library's error convention).
void check(long long id_or_status, const std::string& what)
{
  if (id_or_status < 0)
  {
    throw std::runtime_error("HDF5 error: " + what);
  }
}

/// The HDF5 native numeric type for a FlatScalar alternative. Strings (index 5)
/// are handled separately with the variable-length string type.
hid_t native_type_for(std::size_t variant_index)
{
  switch (variant_index)
  {
    case 0:
      return H5T_NATIVE_UINT8;  // bool stored as 0/1
    case 1:
      return H5T_NATIVE_INT64;
    case 2:
      return H5T_NATIVE_UINT64;
    case 3:
      return H5T_NATIVE_FLOAT;
    case 4:
      return H5T_NATIVE_DOUBLE;
    default:
      return -1;
  }
}

}  // namespace

struct Hdf5Writer::Impl
{
  hid_t file = -1;
  hid_t str_type = -1;  // variable-length UTF-8 string
  int gzip_level = 4;
  std::size_t batch_size = 1000;
  std::mutex mutex;

  /// Per-column dataset state for a table topic.
  struct ColumnState
  {
    hid_t dataset = -1;
    std::size_t variant_index = 0;  // fixes the HDF5 type (set on first value)
  };

  /// A non-image topic: one group, a bag_time_ns dataset, and one dataset per
  /// flattened column, plus a buffer of rows awaiting a block write.
  struct TableState
  {
    hid_t group = -1;
    hid_t bag_time = -1;
    hsize_t rows_written = 0;
    std::vector<int64_t> bag_time_buffer;
    std::vector<FlatRow> row_buffer;
    std::vector<std::string> column_order;
    std::unordered_map<std::string, ColumnState> columns;
  };

  /// An image topic: one group, a stacked (N,H,W,C) frame dataset, and stamp
  /// datasets. length is the current extent (max frame index + 1 seen so far).
  struct ImageState
  {
    hid_t group = -1;
    hid_t images = -1;
    hid_t stamp_sec = -1;
    hid_t stamp_nanosec = -1;
    hsize_t height = 0, width = 0, channels = 0;
    hsize_t length = 0;
  };

  std::unordered_map<std::string, TableState> tables;
  std::unordered_map<std::string, ImageState> images;

  // -- group / dataset creation -------------------------------------------------

  /// Create the topic's group (named like the CSV files) and stamp it with the
  /// ros_type / topic as string attributes.
  hid_t create_group(const std::string& topic, const std::string& ros_type)
  {
    const std::string name = "/" + topic_to_basename(topic);
    hid_t group = H5Gcreate2(file, name.c_str(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    check(group, "create group " + name);
    write_string_attr(group, "ros_type", ros_type);
    write_string_attr(group, "topic", topic);
    return group;
  }

  void write_string_attr(hid_t loc, const char* attr_name, const std::string& value)
  {
    hid_t space = H5Screate(H5S_SCALAR);
    hid_t attr = H5Acreate2(loc, attr_name, str_type, space, H5P_DEFAULT, H5P_DEFAULT);
    if (attr >= 0)
    {
      const char* v = value.c_str();
      H5Awrite(attr, str_type, &v);
      H5Aclose(attr);
    }
    H5Sclose(space);
  }

  /// Create a 1-D extendable, chunked, gzip-compressed dataset of the given
  /// type, with a type-appropriate fill value (NaN for floats, 0 otherwise, ""
  /// for strings).
  hid_t create_1d_dataset(hid_t group, const std::string& name, hid_t type)
  {
    const hsize_t dims[1] = { 0 };
    const hsize_t maxdims[1] = { H5S_UNLIMITED };
    const hsize_t chunk[1] = { kColumnChunk };
    hid_t space = H5Screate_simple(1, dims, maxdims);
    hid_t dcpl = H5Pcreate(H5P_DATASET_CREATE);
    H5Pset_chunk(dcpl, 1, chunk);
    if (gzip_level > 0)
    {
      H5Pset_deflate(dcpl, static_cast<unsigned>(gzip_level));
    }
    set_fill_value(dcpl, type);
    hid_t dset = H5Dcreate2(group, name.c_str(), type, space, H5P_DEFAULT, dcpl, H5P_DEFAULT);
    check(dset, "create dataset " + name);
    H5Pclose(dcpl);
    H5Sclose(space);
    return dset;
  }

  /// Set a back-fill value appropriate to the dataset type, derived from the
  /// type itself (not a column index) so it is correct for any width: NaN for
  /// floats, "" for variable-length strings, zero for every integer width.
  void set_fill_value(hid_t dcpl, hid_t type)
  {
    const H5T_class_t cls = H5Tget_class(type);
    if (cls == H5T_STRING)
    {
      const char* empty = "";
      H5Pset_fill_value(dcpl, type, &empty);
      return;
    }
    if (cls == H5T_FLOAT)
    {
      if (H5Tget_size(type) == sizeof(float))
      {
        float f = std::numeric_limits<float>::quiet_NaN();
        H5Pset_fill_value(dcpl, type, &f);
      }
      else
      {
        double f = std::numeric_limits<double>::quiet_NaN();
        H5Pset_fill_value(dcpl, type, &f);
      }
      return;
    }
    // Integer (any width/signedness): zero fill.
    unsigned char zero[sizeof(uint64_t)] = { 0 };
    H5Pset_fill_value(dcpl, type, zero);
  }

  // -- table writing ------------------------------------------------------------

  /// Extend a 1-D dataset to `new_len` and write `count` elements at `start`.
  void write_1d_block(hid_t dset, hid_t mem_type, hsize_t start, hsize_t count, const void* data)
  {
    const hsize_t new_len = start + count;
    check(H5Dset_extent(dset, &new_len), "extend dataset");
    hid_t fspace = H5Dget_space(dset);
    H5Sselect_hyperslab(fspace, H5S_SELECT_SET, &start, nullptr, &count, nullptr);
    hid_t mspace = H5Screate_simple(1, &count, nullptr);
    check(H5Dwrite(dset, mem_type, mspace, fspace, H5P_DEFAULT, data), "write dataset block");
    H5Sclose(mspace);
    H5Sclose(fspace);
  }

  template <typename T, typename Getter>
  void write_numeric_column(hid_t dset, hid_t mem_type, hsize_t start, const std::vector<const FlatScalar*>& ptrs,
                            T fill, Getter getter)
  {
    std::vector<T> buf(ptrs.size());
    for (std::size_t i = 0; i < ptrs.size(); ++i)
    {
      buf[i] = ptrs[i] != nullptr ? getter(*ptrs[i]) : fill;
    }
    write_1d_block(dset, mem_type, start, buf.size(), buf.data());
  }

  void write_string_column(hid_t dset, hsize_t start, const std::vector<const FlatScalar*>& ptrs)
  {
    std::vector<const char*> buf(ptrs.size());
    for (std::size_t i = 0; i < ptrs.size(); ++i)
    {
      buf[i] = ptrs[i] != nullptr ? std::get<std::string>(*ptrs[i]).c_str() : "";
    }
    write_1d_block(dset, str_type, start, buf.size(), buf.data());
  }

  void write_column(hid_t dset, std::size_t variant_index, hsize_t start, const std::vector<const FlatScalar*>& ptrs)
  {
    switch (variant_index)
    {
      case 0:
        write_numeric_column<uint8_t>(dset, H5T_NATIVE_UINT8, start, ptrs, 0, [](const FlatScalar& v) {
          return std::get<bool>(v) ? uint8_t{ 1 } : uint8_t{ 0 };
        });
        break;
      case 1:
        write_numeric_column<int64_t>(dset, H5T_NATIVE_INT64, start, ptrs, 0,
                                      [](const FlatScalar& v) { return std::get<int64_t>(v); });
        break;
      case 2:
        write_numeric_column<uint64_t>(dset, H5T_NATIVE_UINT64, start, ptrs, 0,
                                       [](const FlatScalar& v) { return std::get<uint64_t>(v); });
        break;
      case 3:
        write_numeric_column<float>(dset, H5T_NATIVE_FLOAT, start, ptrs, std::numeric_limits<float>::quiet_NaN(),
                                    [](const FlatScalar& v) { return std::get<float>(v); });
        break;
      case 4:
        write_numeric_column<double>(dset, H5T_NATIVE_DOUBLE, start, ptrs, std::numeric_limits<double>::quiet_NaN(),
                                     [](const FlatScalar& v) { return std::get<double>(v); });
        break;
      case 5:
        write_string_column(dset, start, ptrs);
        break;
      default:
        break;
    }
  }

  void flush_table_locked(TableState& st)
  {
    const hsize_t batch = st.row_buffer.size();
    if (batch == 0)
    {
      return;
    }
    const hsize_t start = st.rows_written;

    // bag_time_ns: always present, int64, written first.
    if (st.bag_time < 0)
    {
      st.bag_time = create_1d_dataset(st.group, "bag_time_ns", H5T_NATIVE_INT64);
    }
    write_1d_block(st.bag_time, H5T_NATIVE_INT64, start, batch, st.bag_time_buffer.data());

    // Build per-column value pointers (nullptr where a row lacks the column),
    // registering any newly-seen columns in first-seen order.
    std::unordered_map<std::string, std::vector<const FlatScalar*>> values;
    for (const auto& name : st.column_order)
    {
      values.emplace(name, std::vector<const FlatScalar*>(batch, nullptr));
    }
    for (hsize_t r = 0; r < batch; ++r)
    {
      for (const FlatColumn& col : st.row_buffer[r])
      {
        auto it = values.find(col.name);
        if (it == values.end())
        {
          st.column_order.push_back(col.name);
          st.columns[col.name].variant_index = col.value.index();
          it = values.emplace(col.name, std::vector<const FlatScalar*>(batch, nullptr)).first;
        }
        it->second[r] = &col.value;  // last value wins within a row
      }
    }

    for (const auto& name : st.column_order)
    {
      ColumnState& cs = st.columns[name];
      if (cs.dataset < 0)
      {
        const hid_t type = cs.variant_index == 5 ? str_type : native_type_for(cs.variant_index);
        cs.dataset = create_1d_dataset(st.group, name, type);
      }
      write_column(cs.dataset, cs.variant_index, start, values[name]);
    }

    st.rows_written += batch;
    st.bag_time_buffer.clear();
    st.row_buffer.clear();
  }

  // -- image writing ------------------------------------------------------------

  hid_t create_image_dataset(hid_t group, hsize_t h, hsize_t w, hsize_t c)
  {
    const hsize_t dims[4] = { 0, h, w, c };
    const hsize_t maxdims[4] = { H5S_UNLIMITED, h, w, c };
    const hsize_t chunk[4] = { 1, h, w, c };  // one frame per chunk
    hid_t space = H5Screate_simple(4, dims, maxdims);
    hid_t dcpl = H5Pcreate(H5P_DATASET_CREATE);
    H5Pset_chunk(dcpl, 4, chunk);
    if (gzip_level > 0)
    {
      H5Pset_deflate(dcpl, static_cast<unsigned>(gzip_level));
    }
    hid_t dset = H5Dcreate2(group, "images", H5T_NATIVE_UINT8, space, H5P_DEFAULT, dcpl, H5P_DEFAULT);
    check(dset, "create images dataset");
    H5Pclose(dcpl);
    H5Sclose(space);
    return dset;
  }
};

Hdf5Writer::Hdf5Writer(const std::string& path, int gzip_level, std::size_t batch_size)
  : impl_(std::make_unique<Impl>())
{
  // Don't dump HDF5's own error stack to stderr; we translate failures into
  // exceptions with our own messages.
  H5Eset_auto2(H5E_DEFAULT, nullptr, nullptr);

  impl_->gzip_level = gzip_level;
  impl_->batch_size = batch_size == 0 ? 1 : batch_size;

  impl_->str_type = H5Tcopy(H5T_C_S1);
  H5Tset_size(impl_->str_type, H5T_VARIABLE);
  H5Tset_cset(impl_->str_type, H5T_CSET_UTF8);

  impl_->file = H5Fcreate(path.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
  if (impl_->file < 0)
  {
    throw std::runtime_error("Failed to create HDF5 file: " + path);
  }
}

Hdf5Writer::~Hdf5Writer()
{
  close();
  if (impl_ && impl_->str_type >= 0)
  {
    H5Tclose(impl_->str_type);
    impl_->str_type = -1;
  }
}

void Hdf5Writer::append_row(const std::string& topic, const std::string& ros_type, int64_t bag_time_ns,
                            const FlatRow& row)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  Impl::TableState& st = impl_->tables[topic];
  if (st.group < 0)
  {
    st.group = impl_->create_group(topic, ros_type);
  }
  st.bag_time_buffer.push_back(bag_time_ns);
  st.row_buffer.push_back(row);
  if (st.row_buffer.size() >= impl_->batch_size)
  {
    impl_->flush_table_locked(st);
  }
}

void Hdf5Writer::append_image(const std::string& topic, const std::string& ros_type, uint64_t frame_index, int32_t sec,
                              uint32_t nanosec, const cv::Mat& image)
{
  // Make the pixels contiguous and 8-bit *before* taking the lock (the copy is
  // the only potentially expensive bit and needs no serialization).
  cv::Mat frame = image;
  if (frame.depth() != CV_8U)
  {
    frame.convertTo(frame, CV_8U);
  }
  if (!frame.isContinuous())
  {
    frame = frame.clone();
  }
  const hsize_t h = static_cast<hsize_t>(frame.rows);
  const hsize_t w = static_cast<hsize_t>(frame.cols);
  const hsize_t c = static_cast<hsize_t>(frame.channels());

  std::lock_guard<std::mutex> lock(impl_->mutex);
  Impl::ImageState& st = impl_->images[topic];
  if (st.group < 0)
  {
    st.group = impl_->create_group(topic, ros_type);
  }
  if (st.images < 0)
  {
    st.height = h;
    st.width = w;
    st.channels = c;
    st.images = impl_->create_image_dataset(st.group, h, w, c);
    st.stamp_sec = impl_->create_1d_dataset(st.group, "stamp_sec", H5T_NATIVE_INT32);
    st.stamp_nanosec = impl_->create_1d_dataset(st.group, "stamp_nanosec", H5T_NATIVE_UINT32);
  }
  if (h != st.height || w != st.width || c != st.channels)
  {
    throw std::runtime_error("Image size changed within topic " + topic + " (HDF5 stores one fixed (H,W,C) per topic)");
  }

  // Grow all three datasets to cover this frame index if needed (gaps fill 0).
  if (frame_index + 1 > st.length)
  {
    const hsize_t new_len = frame_index + 1;
    const hsize_t img_dims[4] = { new_len, st.height, st.width, st.channels };
    check(H5Dset_extent(st.images, img_dims), "extend images");
    check(H5Dset_extent(st.stamp_sec, &new_len), "extend stamp_sec");
    check(H5Dset_extent(st.stamp_nanosec, &new_len), "extend stamp_nanosec");
    st.length = new_len;
  }

  // Write the frame at [frame_index, :, :, :].
  hid_t fspace = H5Dget_space(st.images);
  const hsize_t start[4] = { frame_index, 0, 0, 0 };
  const hsize_t count[4] = { 1, st.height, st.width, st.channels };
  H5Sselect_hyperslab(fspace, H5S_SELECT_SET, start, nullptr, count, nullptr);
  hid_t mspace = H5Screate_simple(4, count, nullptr);
  check(H5Dwrite(st.images, H5T_NATIVE_UINT8, mspace, fspace, H5P_DEFAULT, frame.data), "write image frame");
  H5Sclose(mspace);
  H5Sclose(fspace);

  // Write the stamp scalars at [frame_index]. hsize_t is a distinct type from
  // uint64_t (unsigned long long vs unsigned long on LP64), so copy into an
  // hsize_t before taking its address for the hyperslab start.
  const hsize_t one = 1;
  const hsize_t offset = frame_index;
  const auto write_scalar = [&](hid_t dset, hid_t type, const void* val) {
    hid_t fs = H5Dget_space(dset);
    H5Sselect_hyperslab(fs, H5S_SELECT_SET, &offset, nullptr, &one, nullptr);
    hid_t ms = H5Screate_simple(1, &one, nullptr);
    check(H5Dwrite(dset, type, ms, fs, H5P_DEFAULT, val), "write stamp");
    H5Sclose(ms);
    H5Sclose(fs);
  };
  write_scalar(st.stamp_sec, H5T_NATIVE_INT32, &sec);
  write_scalar(st.stamp_nanosec, H5T_NATIVE_UINT32, &nanosec);
}

void Hdf5Writer::finish_topic(const std::string& topic)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto it = impl_->tables.find(topic);
  if (it != impl_->tables.end())
  {
    impl_->flush_table_locked(it->second);
  }
}

void Hdf5Writer::close()
{
  if (!impl_ || impl_->file < 0)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(impl_->mutex);

  for (auto& [topic, st] : impl_->tables)
  {
    impl_->flush_table_locked(st);  // any stragglers (finish_topic should have run)
    for (auto& [name, cs] : st.columns)
    {
      if (cs.dataset >= 0)
      {
        H5Dclose(cs.dataset);
      }
    }
    if (st.bag_time >= 0)
    {
      H5Dclose(st.bag_time);
    }
    if (st.group >= 0)
    {
      H5Gclose(st.group);
    }
  }
  for (auto& [topic, st] : impl_->images)
  {
    if (st.images >= 0)
    {
      H5Dclose(st.images);
    }
    if (st.stamp_sec >= 0)
    {
      H5Dclose(st.stamp_sec);
    }
    if (st.stamp_nanosec >= 0)
    {
      H5Dclose(st.stamp_nanosec);
    }
    if (st.group >= 0)
    {
      H5Gclose(st.group);
    }
  }
  impl_->tables.clear();
  impl_->images.clear();

  H5Fclose(impl_->file);
  impl_->file = -1;
}

}  // namespace tabletop_unbag
