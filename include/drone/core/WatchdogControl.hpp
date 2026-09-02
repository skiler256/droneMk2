#pragma once
#include "drone/types.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <string_view>

// Permet à un outil de test de suspendre sélectivement le respawn d'un
// composant tué volontairement, pour observer la réaction du système à une
// absence prolongée -- cf. sitl/SITL.md §5, INTEGRATION_TEST.md §10.1.
// Toujours `false` par défaut (shm fraîchement mappée = zéro) : en vol
// normal, comportement de GlobalWatchdog inchangé. Pas de verrou/checksum :
// bools indépendants, pas d'invariant multi-champs à protéger.
struct SharedWatchdogControl {
  std::array<std::atomic<bool>, static_cast<std::size_t>(TYPES::ComponentID::Count)>
      suppressRespawn{};
};

inline constexpr std::string_view kWatchdogControlShmPath = "/drone_watchdog_control";
