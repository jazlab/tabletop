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

#ifndef TABLETOP_UNBAG__PROGRESS_BAR_HPP_
#define TABLETOP_UNBAG__PROGRESS_BAR_HPP_

#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>

namespace tabletop_unbag
{

/// A minimal, dependency-free, tqdm-style progress bar rendered to stderr.
///
/// Inspired by tqdm (https://github.com/tqdm/tqdm); reimplemented here rather
/// than vendoring a third-party header. When stderr is a TTY it draws a live,
/// throttled (~10 Hz) carriage-return bar with a rate and ETA; otherwise (e.g.
/// when logs are redirected to a file) it degrades to a one-line update roughly
/// every 10 % so it does not flood the log.
class ProgressBar
{
public:
  ProgressBar(uint64_t total, std::string label, bool enabled = true)
    : total_(total), label_(std::move(label)), enabled_(enabled)
  {
    is_tty_ = isatty(STDERR_FILENO) != 0;
    start_ = std::chrono::steady_clock::now();
    last_render_ = start_;
    if (enabled_)
    {
      render();
    }
  }

  ProgressBar(const ProgressBar&) = delete;
  ProgressBar& operator=(const ProgressBar&) = delete;

  ~ProgressBar()
  {
    close();
  }

  /// Advance the counter by `n` and re-render (throttled).
  void tick(uint64_t n = 1)
  {
    current_ += n;
    if (!enabled_)
    {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    const auto since_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_render_).count();
    if (since_ms >= 100 || current_ >= total_)
    {
      last_render_ = now;
      render();
    }
  }

  /// Render the final state and move to a new line. Idempotent.
  void close()
  {
    if (!enabled_ || closed_)
    {
      return;
    }
    closed_ = true;
    render();
    if (is_tty_)
    {
      std::cerr << '\n';
    }
  }

private:
  void render()
  {
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
    const double rate = elapsed > 0.0 ? static_cast<double>(current_) / elapsed : 0.0;
    double frac = total_ > 0 ? static_cast<double>(current_) / static_cast<double>(total_) : 1.0;
    if (frac > 1.0)
    {
      frac = 1.0;
    }
    const int percent = static_cast<int>(frac * 100.0);

    if (!is_tty_)
    {
      if (closed_ || percent >= last_percent_ + 10)
      {
        last_percent_ = percent;
        std::cerr << label_ << ": " << percent << "% (" << current_ << "/" << total_ << ")\n";
      }
      return;
    }

    constexpr int kWidth = 30;
    const int filled = static_cast<int>(frac * kWidth);
    std::ostringstream line;
    line << '\r' << label_ << ' ' << percent << "%|";
    for (int i = 0; i < kWidth; ++i)
    {
      line << (i < filled ? '#' : ' ');
    }
    const double eta = rate > 0.0 ? static_cast<double>(total_ - current_) / rate : 0.0;
    line << "| " << current_ << '/' << total_ << " [" << format_duration(elapsed) << '<' << format_duration(eta) << ", "
         << format_rate(rate) << ']';
    // Pad with spaces to overwrite any leftover characters from a longer line.
    line << "      ";
    std::cerr << line.str() << std::flush;
  }

  static std::string format_duration(double seconds)
  {
    if (seconds < 0.0 || seconds > 359999.0)
    {
      return "--:--";
    }
    const auto total = static_cast<int64_t>(seconds + 0.5);
    const int64_t h = total / 3600;
    const int64_t m = (total % 3600) / 60;
    const int64_t s = total % 60;
    char buf[32];
    if (h > 0)
    {
      std::snprintf(buf, sizeof(buf), "%lld:%02lld:%02lld", static_cast<long long>(h), static_cast<long long>(m),
                    static_cast<long long>(s));
    }
    else
    {
      std::snprintf(buf, sizeof(buf), "%02lld:%02lld", static_cast<long long>(m), static_cast<long long>(s));
    }
    return buf;
  }

  static std::string format_rate(double rate)
  {
    char buf[32];
    if (rate >= 1000.0)
    {
      std::snprintf(buf, sizeof(buf), "%.0f it/s", rate);
    }
    else
    {
      std::snprintf(buf, sizeof(buf), "%.1f it/s", rate);
    }
    return buf;
  }

  uint64_t total_;
  std::string label_;
  bool enabled_;
  bool is_tty_ = false;
  bool closed_ = false;
  uint64_t current_ = 0;
  int last_percent_ = -100;
  std::chrono::steady_clock::time_point start_;
  std::chrono::steady_clock::time_point last_render_;
};

}  // namespace tabletop_unbag

#endif  // TABLETOP_UNBAG__PROGRESS_BAR_HPP_
