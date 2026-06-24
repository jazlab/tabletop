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

#ifndef TABLETOP_UNBAG__CONCURRENT_QUEUE_HPP_
#define TABLETOP_UNBAG__CONCURRENT_QUEUE_HPP_

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace tabletop_unbag
{

/// A bounded, blocking multi-producer / multi-consumer queue.
///
/// This is the coupling between the single reader thread and the worker
/// threads. The bound is what keeps memory in check and what throttles the
/// reader to the slowest consumer: push() blocks while the queue is full, so a
/// reader that outruns the (slow) image workers simply waits instead of
/// buffering the whole bag in RAM.
///
/// Lifecycle: producers call close() when no more items will be pushed. After
/// close(), pop() returns the remaining items and then std::nullopt, so each
/// consumer drains its backlog and exits cleanly -- which is exactly what makes
/// an interrupted run leave valid, resumable output (the reader stops, closes
/// the queues, and every already-enqueued item is still flushed before join).
template <typename T>
class ConcurrentQueue
{
public:
  explicit ConcurrentQueue(std::size_t capacity) : capacity_(capacity == 0 ? 1 : capacity)
  {
  }

  /// Push an item, blocking while the queue is full. Returns false if the queue
  /// was closed before the item could be accepted (the item is dropped).
  bool push(T item)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_full_.wait(lock, [&] { return queue_.size() < capacity_ || closed_; });
    if (closed_)
    {
      return false;
    }
    queue_.push_back(std::move(item));
    lock.unlock();
    not_empty_.notify_one();
    return true;
  }

  /// Pop an item, blocking while the queue is empty. Returns std::nullopt once
  /// the queue is both closed and drained.
  std::optional<T> pop()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [&] { return !queue_.empty() || closed_; });
    if (queue_.empty())
    {
      return std::nullopt;  // closed and fully drained
    }
    T item = std::move(queue_.front());
    queue_.pop_front();
    lock.unlock();
    not_full_.notify_one();
    return item;
  }

  /// Signal that no more items will be pushed. Wakes all blocked threads;
  /// pending pop()s drain the remainder, then return std::nullopt.
  void close()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    not_empty_.notify_all();
    not_full_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable not_full_;
  std::condition_variable not_empty_;
  std::deque<T> queue_;
  std::size_t capacity_;
  bool closed_ = false;
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__CONCURRENT_QUEUE_HPP_
