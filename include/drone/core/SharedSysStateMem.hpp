#pragma once
#include "drone/core/SharedMemory.hpp"
#include "drone/generated/codes.hpp"
#include "drone/types.hpp"
#include "drone/utilities.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>

inline constexpr std::size_t kMinorCodeRingSize = 16;
inline constexpr std::size_t kMajorCodeSlots = 8;
// Marge sur le composant le plus fourni (MavlinkInterface, 5 Task).
inline constexpr std::size_t kMaxTasksPerComponent = 8;

// code == 0 signifie "slot vide" (aucun CODES::Code réel n'a component=0).
struct DiagEntry {
  uint32_t code{0};
  TYPES::TimePoint ts{};
};

struct SharedSysStateMem {
  SharedMemory mem;
  std::array<TYPES::ComponentHealth,
             static_cast<std::size_t>(TYPES::ComponentID::Count)>
      compHealth;

  // Mineurs/majeurs, checksum par composant : cf. ARCHITECTURE_IPC.md §2.4/§5.
  std::array<std::array<DiagEntry, kMinorCodeRingSize>,
             static_cast<std::size_t>(TYPES::ComponentID::Count)>
      minorCodes;

  std::array<std::array<DiagEntry, kMajorCodeSlots>,
             static_cast<std::size_t>(TYPES::ComponentID::Count)>
      majorCodes;

  // Fréquence réelle mesurée par Task, cf. SYSMONITORING.md §13. 0 = pas
  // encore mesurée.
  std::array<std::array<float, kMaxTasksPerComponent>,
             static_cast<std::size_t>(TYPES::ComponentID::Count)>
      taskHz{};

  // Fréquence configurée, publiée une fois par Task. Permet au GCS d'afficher
  // l'écart sans dupliquer les loopFrequency, cf. SYSMONITORING.md §13.
  std::array<std::array<float, kMaxTasksPerComponent>,
             static_cast<std::size_t>(TYPES::ComponentID::Count)>
      taskTargetHz{};

  std::array<uint32_t, static_cast<std::size_t>(TYPES::ComponentID::Count)>
      checksums{};
  std::array<bool, static_cast<std::size_t>(TYPES::ComponentID::Count)>
      checksumSet{};
};

class SharedSysStateMemHandler : public SharedMemoryHandler {
public:
  // ownId = jeton de verrou, pas le sujet lu — cf. ARCHITECTURE_IPC.md §2.3.
  explicit SharedSysStateMemHandler(SharedSysStateMem &state,
                                    TYPES::ComponentID ownId, TYPES::Us timeout)
      : SharedMemoryHandler(state.mem), state_(state), ownId_(ownId),
        timeout_(timeout) {};

  std::optional<TYPES::ComponentHealth> getHealth(TYPES::ComponentID id) {
    return getArrayElement<TYPES::ComponentHealth>(
        ownId_, timeout_, static_cast<std::size_t>(id), state_.compHealth);
  };

  // Toujours self-report (pas de paramètre `id`), cf. ARCHITECTURE_IPC.md §2.3.
  void setHealth(TYPES::ComponentHealth health) {
    setArrayElement(ownId_, timeout_, static_cast<std::size_t>(ownId_), health,
                    state_.compHealth);
  };

  // Code inconnu du dictionnaire -> mineur, cf. ARCHITECTURE_IPC.md §5.
  void raiseCode(CODES::Code code) {
    const auto *entry = CODES::find(code);
    if (entry && CODES::isMajor(entry->severity))
      raiseMajor(code);
    else
      raiseMinor(code);
  };

  // Majeurs seulement : le ring des mineurs s'auto-purge par écrasement.
  void clearCode(CODES::Code code) {
    clearMajor(code);
  };

  // Self-report, comme setHealth : un composant ne publie que ses Task.
  void setTaskHz(TYPES::TaskID task, float hz) {
    if (task >= kMaxTasksPerComponent)
      return;
    auto idx = static_cast<std::size_t>(ownId_);
    modifyData(ownId_, timeout_, state_.taskHz[idx],
               [&](auto &rates) { rates[task] = hz; });
  };

  std::optional<std::array<float, kMaxTasksPerComponent>>
  getTaskHz(TYPES::ComponentID id) {
    return getArrayElement<std::array<float, kMaxTasksPerComponent>>(
        ownId_, timeout_, static_cast<std::size_t>(id), state_.taskHz);
  };

  void setTaskTargetHz(TYPES::TaskID task, float hz) {
    if (task >= kMaxTasksPerComponent)
      return;
    auto idx = static_cast<std::size_t>(ownId_);
    modifyData(ownId_, timeout_, state_.taskTargetHz[idx],
               [&](auto &rates) { rates[task] = hz; });
  };

  std::optional<std::array<float, kMaxTasksPerComponent>>
  getTaskTargetHz(TYPES::ComponentID id) {
    return getArrayElement<std::array<float, kMaxTasksPerComponent>>(
        ownId_, timeout_, static_cast<std::size_t>(id), state_.taskTargetHz);
  };

  std::optional<std::array<DiagEntry, kMinorCodeRingSize>>
  getMinorCodes(TYPES::ComponentID id) {
    return getArrayElement<std::array<DiagEntry, kMinorCodeRingSize>>(
        ownId_, timeout_, static_cast<std::size_t>(id), state_.minorCodes);
  };

  std::optional<std::array<DiagEntry, kMajorCodeSlots>>
  getMajorCodes(TYPES::ComponentID id) {
    return getArrayElement<std::array<DiagEntry, kMajorCodeSlots>>(
        ownId_, timeout_, static_cast<std::size_t>(id), state_.majorCodes);
  };

private:
  SharedSysStateMem &state_;
  TYPES::ComponentID ownId_;
  TYPES::Us timeout_;

  // Décalage plutôt qu'un index de tête, qui échapperait au verrou.
  void raiseMinor(CODES::Code code) {
    auto idx = static_cast<std::size_t>(ownId_);
    modifyData(ownId_, timeout_, state_.minorCodes[idx], [&](auto &ring) {
      for (std::size_t i = kMinorCodeRingSize - 1; i > 0; --i)
        ring[i] = ring[i - 1];
      ring[0] = DiagEntry{code.pack(), TYPES::Clock::now()};
    });
  };

  void raiseMajor(CODES::Code code) {
    auto idx = static_cast<std::size_t>(ownId_);
    const auto packed = code.pack();
    modifyData(ownId_, timeout_, state_.majorCodes[idx], [&](auto &slots) {
      for (auto &s : slots) {
        if (s.code == packed) { // déjà actif : rafraîchit l'horodatage
          s.ts = TYPES::Clock::now();
          return;
        }
      }
      for (auto &s : slots) {
        if (s.code == 0) {
          s = DiagEntry{packed, TYPES::Clock::now()};
          return;
        }
      }
      // Saturation : code perdu, limite assumée (ARCHITECTURE_IPC.md §6).
    });
  };

  void clearMajor(CODES::Code code) {
    auto idx = static_cast<std::size_t>(ownId_);
    const auto packed = code.pack();
    modifyData(ownId_, timeout_, state_.majorCodes[idx], [&](auto &slots) {
      for (auto &s : slots) {
        if (s.code == packed) {
          s = DiagEntry{};
          return;
        }
      }
    });
  };

  // Uniquement le slot `id`, cf. ARCHITECTURE_IPC.md §2.4.
  uint32_t computeChecksum(TYPES::ComponentID id) {
    auto idx = static_cast<std::size_t>(id);
    return UTILITIES::crc32(state_.compHealth[idx]) ^
           UTILITIES::crc32(state_.minorCodes[idx]) ^
           UTILITIES::crc32(state_.majorCodes[idx]) ^
           UTILITIES::crc32(state_.taskHz[idx]) ^
           UTILITIES::crc32(state_.taskTargetHz[idx]);
  };

  // checksumSet[id] = équivalent par-slot de mem_.initialized (§2.4).
  bool checksumValid(TYPES::ComponentID id) {
    auto idx = static_cast<std::size_t>(id);
    if (!state_.checksumSet[idx])
      return true;
    return state_.checksums[idx] == computeChecksum(id);
  }

  void updateChecksum(TYPES::ComponentID id) {
    auto idx = static_cast<std::size_t>(id);
    state_.checksums[idx] = computeChecksum(id);
    state_.checksumSet[idx] = true;
  }

  // Reset segmenté par composant, cf. ARCHITECTURE_IPC.md §2.4.
  void reset(TYPES::ComponentID id) {
    if (id == TYPES::ComponentID::Count)
      return; // sentinel "Free", jamais un vrai appelant

    auto idx = static_cast<std::size_t>(id);
    state_.compHealth[idx] = TYPES::ComponentHealth::IDLE;
    state_.minorCodes[idx] = {};
    state_.majorCodes[idx] = {};
    state_.taskHz[idx] = {};
    state_.taskTargetHz[idx] = {};
    updateChecksum(id); // ne revalide QUE ce slot, jamais les autres
  };
};