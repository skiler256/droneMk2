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

  std::array<uint32_t, static_cast<std::size_t>(TYPES::ComponentID::Count)>
      checksums{};
  std::array<bool, static_cast<std::size_t>(TYPES::ComponentID::Count)>
      checksumSet{};
};

class SharedSysStateMemHandler : public SharedMemoryHandler {
public:
  // ownId = identité de verrouillage, distincte du sujet getHealth/setHealth
  // -- cf. ARCHITECTURE_IPC.md §2.3.
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

  // Route selon CODES::kTable, cf. ARCHITECTURE_IPC.md §5. Code inconnu du
  // dictionnaire -> traité comme mineur (jamais silencieusement perdu).
  void raiseCode(CODES::Code code) {
    const auto *entry = CODES::find(code);
    if (entry && CODES::isMajor(entry->severity))
      raiseMajor(code);
    else
      raiseMinor(code);
  };

  // Retire un code de l'ensemble actif majeur. No-op si le code n'y était
  // pas (jamais levé, déjà effacé, ou mineur — le ring n'a pas de clear,
  // il s'auto-purge par écrasement FIFO).
  void clearCode(CODES::Code code) {
    clearMajor(code);
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

  // Décalage façon SharedCompMem::recordStart — même pattern déjà éprouvé,
  // pas besoin d'un index de tête séparé (qui échapperait au verrou).
  // Toujours sur le slot ownId_ (self-report, cf. raiseCode()).
  void raiseMinor(CODES::Code code) {
    auto idx = static_cast<std::size_t>(ownId_);
    auto fetch = getData(ownId_, timeout_, state_.minorCodes[idx]);
    if (!fetch)
      return;
    auto ring = fetch.value();
    for (std::size_t i = kMinorCodeRingSize - 1; i > 0; --i)
      ring[i] = ring[i - 1];
    ring[0] = DiagEntry{code.pack(), TYPES::Clock::now()};
    setData(ownId_, timeout_, state_.minorCodes[idx], ring);
  };

  void raiseMajor(CODES::Code code) {
    auto idx = static_cast<std::size_t>(ownId_);
    auto fetch = getData(ownId_, timeout_, state_.majorCodes[idx]);
    if (!fetch)
      return;
    auto slots = fetch.value();
    auto packed = code.pack();

    for (auto &s : slots) {
      if (s.code == packed) { // déjà actif : rafraîchit juste l'horodatage
        s.ts = TYPES::Clock::now();
        setData(ownId_, timeout_, state_.majorCodes[idx], slots);
        return;
      }
    }
    for (auto &s : slots) {
      if (s.code == 0) {
        s = DiagEntry{packed, TYPES::Clock::now()};
        setData(ownId_, timeout_, state_.majorCodes[idx], slots);
        return;
      }
    }
    // Les kMajorCodeSlots sont tous pris : limite connue, pas de repli
    // silencieux élégant ici (cf. discussion — un composant avec plus de
    // kMajorCodeSlots pannes critiques simultanées a un problème plus grave
    // que ce compteur).
  };

  void clearMajor(CODES::Code code) {
    auto idx = static_cast<std::size_t>(ownId_);
    auto fetch = getData(ownId_, timeout_, state_.majorCodes[idx]);
    if (!fetch)
      return;
    auto slots = fetch.value();
    auto packed = code.pack();
    for (auto &s : slots) {
      if (s.code == packed) {
        s = DiagEntry{};
        setData(ownId_, timeout_, state_.majorCodes[idx], slots);
        return;
      }
    }
  };

  // Uniquement le slot `id`, cf. ARCHITECTURE_IPC.md §2.4.
  uint32_t computeChecksum(TYPES::ComponentID id) {
    auto idx = static_cast<std::size_t>(id);
    return UTILITIES::crc32(state_.compHealth[idx]) ^
           UTILITIES::crc32(state_.minorCodes[idx]) ^
           UTILITIES::crc32(state_.majorCodes[idx]);
  };

  // checksums[id] par composant + checksumSet[id] (équivalent par-slot de
  // mem_.initialized) : cf. ARCHITECTURE_IPC.md §2.4.
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
    updateChecksum(id); // ne revalide QUE ce slot, jamais les autres
  };
};