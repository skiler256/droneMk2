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

  // Mineurs : ring buffer FIFO, écrasement normal des plus anciens.
  std::array<std::array<DiagEntry, kMinorCodeRingSize>,
             static_cast<std::size_t>(TYPES::ComponentID::Count)>
      minorCodes;

  // Majeurs (Critical/Emergency) : ensemble actif, jamais écrasé
  // silencieusement — un code y reste jusqu'à clearCode() explicite.
  std::array<std::array<DiagEntry, kMajorCodeSlots>,
             static_cast<std::size_t>(TYPES::ComponentID::Count)>
      majorCodes;

  // Checksum PAR composant (pas un seul global) : segment multi-écrivains,
  // donc reset(id) d'un writer ne doit jamais revalider silencieusement un
  // autre slot qu'il n'a pas touché et jamais vérifié. Voir
  // SharedSysStateMemHandler::checksumValid/updateChecksum.
  std::array<uint32_t, static_cast<std::size_t>(TYPES::ComponentID::Count)>
      checksums{};
  std::array<bool, static_cast<std::size_t>(TYPES::ComponentID::Count)>
      checksumSet{};
};

class SharedSysStateMemHandler : public SharedMemoryHandler {
public:
  // `ownId` : identité de CE process (verrouillage), distincte du `id` de
  // getHealth/setHealth qui désigne le sujet consulté/modifié.
  explicit SharedSysStateMemHandler(SharedSysStateMem &state,
                                    TYPES::ComponentID ownId, TYPES::Us timeout)
      : SharedMemoryHandler(state.mem), state_(state), ownId_(ownId),
        timeout_(timeout) {};

  std::optional<TYPES::ComponentHealth> getHealth(TYPES::ComponentID id) {
    return getArrayElement<TYPES::ComponentHealth>(
        ownId_, timeout_, static_cast<std::size_t>(id), state_.compHealth);
  };

  // Toujours un self-report : le handler ne peut écrire QUE son propre
  // slot (ownId_), il n'y a pas de raison légitime aujourd'hui d'écrire la
  // santé d'un autre composant. Pas de paramètre `id` — impossible de se
  // tromper de cible. (resetWriterFlag(id), lui, garde un `id` explicite :
  // c'est GlobalWatchdog qui l'appelle pour libérer le verrou d'un AUTRE
  // process mort, pas un self-report.)
  void setHealth(TYPES::ComponentHealth health) {
    setArrayElement(ownId_, timeout_, static_cast<std::size_t>(ownId_), health,
                    state_.compHealth);
  };

  // Route vers le ring mineur ou l'ensemble actif majeur selon la sévérité
  // du code dans le dictionnaire (CODES::kTable). Code inconnu du
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

  // Uniquement les données du slot `id` — jamais celles des autres
  // composants, sinon un accès de A ferait échouer/masquerait une
  // corruption chez B qu'il n'a ni causée ni les moyens de réparer.
  uint32_t computeChecksum(TYPES::ComponentID id) {
    auto idx = static_cast<std::size_t>(id);
    return UTILITIES::crc32(state_.compHealth[idx]) ^
           UTILITIES::crc32(state_.minorCodes[idx]) ^
           UTILITIES::crc32(state_.majorCodes[idx]);
  };

  // Un checksum par composant (state_.checksums[id]), pas un seul global :
  // sinon reset(id) recalculerait un hash sur TOUT le payload — y compris
  // les slots des autres composants jamais vérifiés — et revaliderait
  // silencieusement leur corruption éventuelle. `checksumSet[id]` remplace
  // le mem_.initialized générique (global, un seul id le déclenche) par un
  // équivalent par slot : jamais vérifié -> pas invalide, juste inconnu.
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

  // Segmenté par composant (contrairement à SharedNavMem/SharedSFMem) :
  // ces champs sont écrits par tout le monde, un reset ne doit toucher QUE
  // le slot de `id`, jamais effacer les données des autres.
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