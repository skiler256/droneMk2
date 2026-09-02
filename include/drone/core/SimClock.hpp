#pragma once
#include <atomic>
#include <chrono>

namespace TYPES {

// Horloge simulée — satisfait TrivialClock, interchangeable avec
// std::chrono::steady_clock via TYPES::Clock (types.hpp, option CMake
// DRONE_USE_SIM_CLOCK). Résolue à la compilation, pas de dispatch virtuel
// (contrainte JPL -fno-rtti) -- cf. sitl/SITL.md §11.
//
// Pour l'instant : relais vers steady_clock::now(), décalable via
// setOffset(). Pilotage explicite du temps pour le rejeu accéléré pas
// encore construit (cf. sitl/SITL.md §10) -- ce point d'extension est posé
// maintenant plutôt qu'après, pendant que le rétrofit ne touche que les 5
// composants existants.
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

  // Décale le temps observé par TOUT le process (tous les composants d'un
  // même binaire partagent ce décalage, pas de coordination inter-process
  // ici) — no-op tant qu'aucun outil de rejeu ne l'appelle.
  static void setOffset(duration offset) noexcept {
    offset_.store(offset, std::memory_order_relaxed);
  }

private:
  static inline std::atomic<duration> offset_{duration::zero()};
};

} // namespace TYPES
