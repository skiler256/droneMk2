#pragma once
#include "drone/types.hpp"
#include "drone/utilities.hpp"
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
  // Free == ComponentID::Count. Sert à la fois de verrou (CAS) et de
  // registre d'identité du détenteur.
  std::atomic<TYPES::ComponentID> owner{TYPES::ComponentID::Count};
  uint32_t checksum;
};

class SharedMemoryHandler {
public:
  explicit SharedMemoryHandler(SharedMemory &mem) : mem_(mem) {};
  virtual ~SharedMemoryHandler() = default;

  // Appelé par GlobalWatchdog à la mort d'un composant : ne libère que si
  // `id` détenait vraiment le verrou (sinon no-op, pas de vol possible).
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

  // `id` transite jusqu'ici pour permettre un checksum PAR COMPOSANT sur
  // les segments multi-écrivains (override checksumValid/updateChecksum ET
  // computeChecksum ci-dessous) — sans ça, reset(id) d'un writer revalide
  // silencieusement la corruption d'un slot qu'il n'a pas touché, en
  // recalculant un checksum global sur des données jamais vérifiées.
  virtual uint32_t computeChecksum(TYPES::ComponentID id) = 0;
  virtual void reset(TYPES::ComponentID id) = 0;

  // Défaut : un seul checksum pour tout le payload (id ignoré) — correct
  // pour un segment mono-propriétaire (Nav/SF/CompMem). À override pour un
  // segment multi-écrivains (cf. SharedSysStateMem).
  virtual bool checksumValid(TYPES::ComponentID id) {
    return mem_.checksum == computeChecksum(id);
  }
  virtual void updateChecksum(TYPES::ComponentID id) {
    mem_.checksum = computeChecksum(id);
  }

  // `id` : identité de l'appelant (jeton de verrou, pas forcément le sujet
  // lu/écrit). `repairOnCorrupt` : à true seulement pour un read-modify-write
  // interne à une écriture (ex: recordStart), jamais pour une lecture pure.
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

  // Voie d'accès unique (lecture ET écriture) : même CAS `owner`, donc pas
  // de fenêtre check-then-set et pas de lecture concurrente à une écriture.
  // `id` doit être l'identité réelle de l'appelant, pas le sujet lu/écrit
  // (sinon GlobalWatchdog ne peut pas libérer le verrou au crash du bon
  // process — deadlock permanent).
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