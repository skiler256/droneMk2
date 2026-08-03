#pragma once
#include "drone/types.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <string_view>

// Segment minuscule, PUBLIÉ par GlobalWatchdog (comme les autres shm) et
// lu à chaque mort d'enfant dans monitorLoop() — permet à un outil de test
// externe (ex: le futur SITL_MASTER) de suspendre sélectivement le respawn
// d'UN composant tué volontairement, pour observer la réaction du reste du
// système à une absence prolongée plutôt que de voir le process revenir
// avant que quiconque ait eu le temps de réagir.
//
// cf. INTEGRATION_TEST.md §7.2 (pannes système) / §10.1 (décision — "à
// trancher dans le code du watchdog, pas dans l'orchestrateur").
//
// Toujours à `false` par défaut (mémoire shm fraîchement mappée = zéro) :
// en vol normal, personne n'écrit jamais dans ce segment, le comportement
// de GlobalWatchdog est donc identique à avant ce changement.
//
// Pas de verrou CAS/checksum ici (contrairement à SharedSysStateMem etc.) :
// chaque case est un bool indépendant, pas d'invariant multi-champs à
// protéger — un simple atomic<bool> par composant suffit.
struct SharedWatchdogControl {
  std::array<std::atomic<bool>, static_cast<std::size_t>(TYPES::ComponentID::Count)>
      suppressRespawn{};
};

inline constexpr std::string_view kWatchdogControlShmPath = "/drone_watchdog_control";
