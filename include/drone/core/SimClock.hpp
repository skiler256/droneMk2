#pragma once
#include <atomic>
#include <chrono>

namespace TYPES {

// TrivialClock interchangeable avec steady_clock — cf. sitl/SITL.md §10-11.
// Relais vers steady_clock::now() décalable ; le rejeu accéléré n'est pas
// encore construit, seul le point d'extension est posé.
struct SimClock {
  using rep = std::chrono::steady_clock::rep;
  using period = std::chrono::steady_clock::period;
  using duration = std::chrono::steady_clock::duration;
  using time_point = std::chrono::time_point<SimClock, duration>;
  static constexpr bool is_steady = true;

  [[nodiscard]] static time_point now() noexcept {
    auto real = std::chrono::steady_clock::now().time_since_epoch();
    return time_point(real + offset_.load(std::memory_order_relaxed));
  }

  // Décale TOUT le process, aucune coordination inter-process.
  static void setOffset(duration offset) noexcept {
    offset_.store(offset, std::memory_order_relaxed);
  }

private:
  static inline std::atomic<duration> offset_{duration::zero()};
};

} // namespace TYPES
