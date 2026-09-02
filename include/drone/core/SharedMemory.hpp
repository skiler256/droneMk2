#pragma once
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <fcntl.h>
#include <optional>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

// FIFO bornée POD — safe en shm (pas de mutex, manipulée uniquement via
// pushFifo()/popFifo() sous le CAS-lock de SharedMemoryHandler). Échoue au
// lieu d'écraser (cf. pushFifo) : perdre un élément silencieusement (ex: un
// ordre de MissionControl) est pire qu'un push refusé explicitement.
template <typename T, size_t N> struct RingBuffer {
  std::array<T, N> buf{};
  size_t head{0};
  size_t tail{0};
  size_t count{0};
};

// RAII : release uniquement, l'acquisition (CAS) est faite par l'appelant.
class WriterGuard {
public:
  explicit WriterGuard(std::atomic<TYPES::ComponentID> &owner) : owner_(owner) {}

  ~WriterGuard() {
    owner_.store(TYPES::ComponentID::Count, std::memory_order_release);
  }

private:
  std::atomic<TYPES::ComponentID> &owner_;
};

struct SharedMemory {
  std::atomic<bool> initialized{false};
  // Verrou CAS + registre d'identité, cf. ARCHITECTURE_IPC.md §2.2.
  std::atomic<TYPES::ComponentID> owner{TYPES::ComponentID::Count};
  uint32_t checksum;
};

class SharedMemoryHandler {
public:
  explicit SharedMemoryHandler(SharedMemory &mem) : mem_(mem) {};
  virtual ~SharedMemoryHandler() = default;

  // CAS conditionnel, cf. ARCHITECTURE_IPC.md §2.5.
  void resetWriterFlag(TYPES::ComponentID id) {
    TYPES::ComponentID expected = id;
    mem_.owner.compare_exchange_strong(expected, TYPES::ComponentID::Count,
                                       std::memory_order_acq_rel);
  };

  // Sticky : un succès ne l'efface pas, seule la consommation le fait.
  [[nodiscard]] TYPES::shmError consumeError() noexcept {
    return lastError_.exchange(TYPES::shmError::None,
                               std::memory_order_acq_rel);
  };

protected:
  SharedMemory &mem_;

  // Checksum par composant sur les segments multi-écrivains, cf.
  // ARCHITECTURE_IPC.md §2.4.
  virtual uint32_t computeChecksum(TYPES::ComponentID id) = 0;
  virtual void reset(TYPES::ComponentID id) = 0;

  // Défaut : un seul checksum (segment mono-propriétaire). Override pour
  // multi-écrivains, cf. SharedSysStateMem.
  virtual bool checksumValid(TYPES::ComponentID id) {
    return mem_.checksum == computeChecksum(id);
  }
  virtual void updateChecksum(TYPES::ComponentID id) {
    mem_.checksum = computeChecksum(id);
  }

  // `id` = jeton de verrou, pas forcément le sujet lu/écrit (cf.
  // ARCHITECTURE_IPC.md §2.3). `repairOnCorrupt` : true seulement pour un
  // read-modify-write interne à une écriture, jamais pour une lecture pure.
  template <typename T>
  [[nodiscard]] std::optional<T> getData(TYPES::ComponentID id, TYPES::Us timeout,
                                         T &t, bool repairOnCorrupt = false) {
    T data;
    auto fetch = lockedAccess(id, timeout, repairOnCorrupt,
                              [&data, &t] { data = t; });
    if (!fetch) {
      recordError(fetch.error());
      return {};
    }
    return data;
  };

  template <typename T>
  void setData(TYPES::ComponentID id, TYPES::Us timeout, T &t, T &value) {

    auto fetch = lockedAccess(id, timeout, /*repairOnCorrupt=*/true,
                              [&value, &t] { t = value; });
    if (!fetch) {
      recordError(fetch.error());
    }
  };

  template <typename T, size_t N>
  [[nodiscard]] bool pushFifo(TYPES::ComponentID id, TYPES::Us timeout,
                              RingBuffer<T, N> &fifo, const T &value) {
    bool pushed = false;
    auto fetch = lockedAccess(id, timeout, /*repairOnCorrupt=*/true, [&] {
      if (fifo.count < N) {
        fifo.buf[fifo.head] = value;
        fifo.head = (fifo.head + 1) % N;
        ++fifo.count;
        pushed = true;
      }
    });
    if (!fetch)
      recordError(fetch.error());
    return pushed;
  };

  template <typename T, size_t N>
  std::optional<T> popFifo(TYPES::ComponentID id, TYPES::Us timeout,
                           RingBuffer<T, N> &fifo) {
    std::optional<T> result;
    auto fetch = lockedAccess(id, timeout, /*repairOnCorrupt=*/true, [&] {
      if (fifo.count > 0) {
        result = fifo.buf[fifo.tail];
        fifo.tail = (fifo.tail + 1) % N;
        --fifo.count;
      }
    });
    if (!fetch)
      recordError(fetch.error());
    return result;
  };

  template <typename T, typename A>
  std::optional<T> getArrayElement(TYPES::ComponentID id, TYPES::Us timeout,
                                   size_t i, A &array) {
    std::optional<T> data;
    auto fetch = getData(id, timeout, array);
    if (!fetch)
      return data;
    return data.emplace(fetch->at(i));
  };

  // Lit-modifie-écrit sous un seul verrou : évite le lost-update d'un
  // getData()+setData() séparé.
  template <typename T, typename A>
  void setArrayElement(TYPES::ComponentID id, TYPES::Us timeout, size_t i, T &t,
                       A &array) {
    auto fetch = lockedAccess(id, timeout, /*repairOnCorrupt=*/true,
                              [&] { array[i] = t; });
    if (!fetch) {
      recordError(fetch.error());
    }
  };

private:
  std::atomic<TYPES::shmError> lastError_{TYPES::shmError::None};

  void recordError(TYPES::shmError e) noexcept {
    lastError_.store(e, std::memory_order_release);
  };

  // Verrou CAS unique, lecture ET écriture, cf. ARCHITECTURE_IPC.md §2.2-2.3.
  template <typename F>
  std::expected<void, TYPES::shmError>
  lockedAccess(TYPES::ComponentID id, TYPES::Us timeout, bool repairOnCorrupt,
              F &&f) {
    auto tryAcquire = [&] {
      TYPES::ComponentID expected = TYPES::ComponentID::Count;
      return mem_.owner.compare_exchange_strong(expected, id,
                                                std::memory_order_acq_rel);
    };

    if (UTILITIES::waitUntil(tryAcquire, timeout))

    {

      WriterGuard guard(mem_.owner);

      if (!mem_.initialized.load(std::memory_order_acquire)) {
        updateChecksum(id);
        mem_.initialized.store(true, std::memory_order_release);
      }

      if (!checksumValid(id)) {
        if (repairOnCorrupt)
          reset(id);
        return std::unexpected(TYPES::shmError::corrupt);
      }

      std::forward<F>(f)();
      updateChecksum(id);
      return {};
    } else
      return std::unexpected(TYPES::shmError::timeout);
  };
};