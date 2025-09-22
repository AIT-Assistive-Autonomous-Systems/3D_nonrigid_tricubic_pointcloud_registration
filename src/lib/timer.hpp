#pragma once

#include <fmt/format.h>

#include <chrono>

class Timer {
 public:
  Timer() : start_time_(std::chrono::high_resolution_clock::now()) {}

  void reset() { start_time_ = std::chrono::high_resolution_clock::now(); }

  double elapsed() const {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_);
    return duration.count();
  }

  friend std::ostream& operator<<(std::ostream& os, const Timer& timer) {
    os << fmt::format("{:.3f}s", timer.elapsed());
    return os;
  }

  // Helper function to get formatted elapsed time as string
  std::string str() const { return fmt::format("{:.3f}s", elapsed()); }

 private:
  std::chrono::high_resolution_clock::time_point start_time_;
};

// Custom formatter for Timer class to work with fmt::format
template <>
struct fmt::formatter<Timer> : formatter<std::string> {
  auto format(const Timer& timer, format_context& ctx) const {
    return formatter<std::string>::format(timer.str(), ctx);
  }
};