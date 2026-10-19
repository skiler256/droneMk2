#pragma once
#include "drone/types.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <string_view>

// Suspend le respawn d'un composant tué par un outil de test — cf.
// sitl/SITL.md §5. `false` par défaut (shm mappée = zéro), donc sans effet
// en vol. Pas de verrou : bools indépendants, aucun invariant à protéger.
struct SharedWatchdogControl {
  std::array<std::atomic<bool>, static_cast<std::size_t>(TYPES::ComponentID::Count)>
      suppressRespawn{};
};

inline constexpr std::string_view kWatchdogControlShmPath = "/drone_watchdog_control";
