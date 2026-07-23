// include/drone/generated/codes.hpp
// ────────────────────────────────────────────────────────
// GÉNÉRÉ depuis INDEX.csv par tools/gen_codes.py — NE PAS ÉDITER À LA MAIN.
// Régénérer : python3 tools/gen_codes.py (ou : cmake --build .)
#pragma once
#include "drone/types.hpp"
#include <array>
#include <cstdint>

namespace CODES {

enum class Component : uint8_t {
  FC = 1, // Flight Controller
  SF = 2, // Sensor Fusion
  NV = 3, // Navigation
  MC = 4, // Mission Control
  SM = 5, // Sys Monitoring
  MV = 6, // Mavlink Interface
  GW = 7, // Global Watchdog
};

enum class Category : uint8_t {
  P = 1, // Power
  S = 2, // Sensor
  A = 3, // Action
  C = 4, // Communication
};

enum class Severity : uint8_t { Info = 0, Warning = 1, Critical = 2, Emergency = 3 };

// true si Severity doit être stockée dans l'ensemble actif "majeur"
// (jamais écrasé silencieusement) plutôt que le ring buffer "mineur".
[[nodiscard]] constexpr bool isMajor(Severity s) noexcept {
  return s == Severity::Critical || s == Severity::Emergency;
}

struct Code {
  Component component;
  Category category;
  uint16_t number;

  constexpr bool operator==(const Code &) const noexcept = default;

  // [component:8][category:8][number:16] — 4 octets.
  [[nodiscard]] constexpr uint32_t pack() const noexcept {
    return (static_cast<uint32_t>(component) << 24) |
           (static_cast<uint32_t>(category) << 16) |
           static_cast<uint32_t>(number);
  }

  [[nodiscard]] static constexpr Code unpack(uint32_t packed) noexcept {
    return Code{
        static_cast<Component>((packed >> 24) & 0xFF),
        static_cast<Category>((packed >> 16) & 0xFF),
        static_cast<uint16_t>(packed & 0xFFFF),
    };
  }
};

// Une entrée par code : sert au lookup runtime de severity()/name()/description().
struct Entry {
  Code code;
  Severity severity;
  const char *name;
  const char *description;
};

inline constexpr Code FCP001{Component::FC, Category::P, 1};
inline constexpr Code MVC001{Component::MV, Category::C, 1};
inline constexpr Code NVA001{Component::NV, Category::A, 1};
inline constexpr Code NVA002{Component::NV, Category::A, 2};
inline constexpr Code NVA003{Component::NV, Category::A, 3};
inline constexpr Code SFS001{Component::SF, Category::S, 1};
inline constexpr Code SFS002{Component::SF, Category::S, 2};

inline constexpr std::array<Entry, 7> kTable{{
    Entry{FCP001, Severity::Critical, "batt_low", "tension batterie faible"},
    Entry{MVC001, Severity::Warning, "unexpected_ack", "ACK reçu (COMMAND_ACK/PARAM_VALUE) ne correspond à aucune commande TTxACK en attente"},
    Entry{NVA001, Severity::Critical, "landing_cat_1", "atterissage d'urgence, FC autonome"},
    Entry{NVA002, Severity::Info, "landing_cat_2", "atterissage assité par LIDAR"},
    Entry{NVA003, Severity::Info, "landing_cat_3", "atterissage guidé"},
    Entry{SFS001, Severity::Info, "GPS_lock", "signal gps vérouillé"},
    Entry{SFS002, Severity::Critical, "GPS_lost", "signal gps perdu"},
}};

// Recherche linéaire : table petite, appelée hors boucle chaude uniquement
// (raiseCode/clearCode, pas depuis loop()).
[[nodiscard]] constexpr const Entry *find(Code code) noexcept {
  for (const auto &e : kTable)
    if (e.code == code)
      return &e;
  return nullptr;
}

} // namespace CODES
