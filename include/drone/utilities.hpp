#pragma once
#include "drone/types.hpp"
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <thread>

#if defined(__aarch64__)
#include <arm_acle.h>
#endif

namespace UTILITIES {

template <typename Predicate>
bool waitUntil(Predicate &&pred, TYPES::Us timeout, bool invert = false,
               TYPES::Us period = TYPES::Us(10)) {
  // TYPES::Clock et non steady_clock en dur : chemin chaud du verrou CAS,
  // doit suivre la politique d'horloge du process (DRONE_USE_SIM_CLOCK).
  TYPES::TimePoint deadline = TYPES::Clock::now() + timeout;

  while (TYPES::Clock::now() < deadline) {
    if (std::forward<Predicate>(pred)() != invert)
      return true;

    std::this_thread::sleep_for(period);
  }

  return false;
}

[[nodiscard]] inline TYPES::Ms msBetween(const TYPES::TimePoint &start,
                                         const TYPES::TimePoint &end) {
  return std::chrono::duration_cast<TYPES::Ms>(end - start);
}

[[nodiscard]] inline TYPES::Us usBetween(const TYPES::TimePoint &start,
                                         const TYPES::TimePoint &end) {
  return std::chrono::duration_cast<TYPES::Us>(end - start);
}

// CRC-32C (Castagnoli) : instruction ARMv8 sur cible, table de lookup en
// dev x86, resultat identique des deux cotes. Integrite shm uniquement,
// pas de la cryptographie -- cf. ARCHITECTURE_IPC.md §2.4.
namespace Crc32Detail {

#if !defined(__aarch64__)
// Polynome 0x1EDC6F41 (reflechi 0x82F63B78), celui de l'instruction ARMv8.
inline constexpr auto makeTable() {
  std::array<uint32_t, 256> table{};
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t crc = i;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1) ? (crc >> 1) ^ 0x82F63B78u : (crc >> 1);
    }
    table[i] = crc;
  }
  return table;
}

inline constexpr std::array<uint32_t, 256> kCrc32Table = makeTable();
#endif

} // namespace Crc32Detail

inline uint32_t crc32(const void *data, std::size_t len) {
  const auto *bytes = static_cast<const uint8_t *>(data);
  uint32_t crc = 0xFFFFFFFFu;

#if defined(__aarch64__)
  std::size_t i = 0;
  for (; i + 8 <= len; i += 8) {
    uint64_t chunk;
    std::memcpy(&chunk, bytes + i, 8);
    crc = __crc32d(crc, chunk);
  }
  for (; i < len; ++i) {
    crc = __crc32b(crc, bytes[i]);
  }
#else
  for (std::size_t i = 0; i < len; ++i) {
    crc = Crc32Detail::kCrc32Table[(crc ^ bytes[i]) & 0xFFu] ^ (crc >> 8);
  }
#endif

  return crc ^ 0xFFFFFFFFu;
}

// Surcharge pour tout type trivially copyable (payloads shm).
template <typename T>
  requires std::is_trivially_copyable_v<T>
uint32_t crc32(const T &value) {
  return crc32(&value, sizeof(T));
}

template <typename F>
[[nodiscard]] std::expected<pthread_t, int>
launchRTThread(F &&f, int rt_priority, int core) {

  pthread_attr_t attr;
  pthread_attr_init(&attr);

  pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

  struct sched_param param;
  param.sched_priority = rt_priority;
  pthread_attr_setschedparam(&attr, &param);
  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core, &cpuset);
  pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);

  auto *callable = new std::decay_t<F>(std::forward<F>(f));

  pthread_t thread;
  int err = pthread_create(
      &thread, &attr,
      [](void *arg) -> void * {
        auto *fn = static_cast<std::decay_t<F> *>(arg);
        (*fn)();
        delete fn;
        return nullptr;
      },
      callable);

  pthread_attr_destroy(&attr);

  if (err != 0) {
    delete callable;
    return std::unexpected(err);
  }

  return thread;
}

}; // namespace UTILITIES