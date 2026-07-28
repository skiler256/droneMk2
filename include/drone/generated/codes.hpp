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
  O = 5, // Command
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
inline constexpr Code FCP002{Component::FC, Category::P, 2};
inline constexpr Code MCO001{Component::MC, Category::O, 1};
inline constexpr Code MCO002{Component::MC, Category::O, 2};
inline constexpr Code MCO003{Component::MC, Category::O, 3};
inline constexpr Code MCO004{Component::MC, Category::O, 4};
inline constexpr Code MCO005{Component::MC, Category::O, 5};
inline constexpr Code MVC001{Component::MV, Category::C, 1};
inline constexpr Code MVC002{Component::MV, Category::C, 2};
inline constexpr Code MVC003{Component::MV, Category::C, 3};
inline constexpr Code MVC004{Component::MV, Category::C, 4};
inline constexpr Code NVA001{Component::NV, Category::A, 1};
inline constexpr Code NVA002{Component::NV, Category::A, 2};
inline constexpr Code NVA003{Component::NV, Category::A, 3};
inline constexpr Code NVA004{Component::NV, Category::A, 4};
inline constexpr Code NVA005{Component::NV, Category::A, 5};
inline constexpr Code NVA006{Component::NV, Category::A, 6};
inline constexpr Code NVA007{Component::NV, Category::A, 7};
inline constexpr Code NVA008{Component::NV, Category::A, 8};
inline constexpr Code NVA009{Component::NV, Category::A, 9};
inline constexpr Code NVA010{Component::NV, Category::A, 10};
inline constexpr Code SFS001{Component::SF, Category::S, 1};
inline constexpr Code SFS002{Component::SF, Category::S, 2};
inline constexpr Code SFS003{Component::SF, Category::S, 3};
inline constexpr Code SFS004{Component::SF, Category::S, 4};
inline constexpr Code SFS005{Component::SF, Category::S, 5};

inline constexpr std::array<Entry, 26> kTable{{
    Entry{FCP001, Severity::Critical, "batt_low", "tension batterie faible"},
    Entry{FCP002, Severity::Emergency, "Eng_failure", "Perte moteur"},
    Entry{MCO001, Severity::Info, "takeoff", "ordre de décollage vers MavlinkInterface (payload: altitude cible en cm)"},
    Entry{MCO002, Severity::Info, "set_mode_guided", "ordre de passage en mode GUIDED vers MavlinkInterface"},
    Entry{MCO003, Severity::Info, "arm", "ordre d'armement vers MavlinkInterface (payload ignoré)"},
    Entry{MCO004, Severity::Info, "disarm", "ordre de désarmement vers MavlinkInterface (payload ignoré)"},
    Entry{MCO005, Severity::Info, "start_segment", "ordre d'amorçage vers Nav (payload ignoré) - promeut armedSegment en engagedSegment si sécurité OK"},
    Entry{MVC001, Severity::Warning, "unexpected_ack", "ACK reçu (COMMAND_ACK/PARAM_VALUE) ne correspond à aucune commande TTxACK en attente"},
    Entry{MVC002, Severity::Critical, "command_rejected", "COMMAND_ACK != ACCEPTED ou timeout épuisé sur une commande interne sans origine MissionControl (ex: requestInitStreams) - pour un ordre MC voir SharedCmdMem::CommandReport - séquence abandonnée (ackQueue_ vidée)"},
    Entry{MVC003, Severity::Warning, "queue_full", "push échoué sur ackQueue_, ackFifo_ ou mavlinkReports (pleine), commande/ACK/report perdu"},
    Entry{MVC004, Severity::Critical, "fc_link_lost", "plus de HEARTBEAT du FC depuis le seuil FSTask"},
    Entry{NVA001, Severity::Critical, "landing_cat_1", "atterissage d'urgence, FC autonome"},
    Entry{NVA002, Severity::Info, "landing_cat_2", "atterissage assité par LIDAR"},
    Entry{NVA003, Severity::Info, "landing_cat_3", "atterissage guidé"},
    Entry{NVA004, Severity::Critical, "hold_obstacle", "TFS - obstacle détecté dans la zone de sécurité devant le drone - hold déclenché"},
    Entry{NVA005, Severity::Critical, "hold_ekf_stale", "TFS - DynamicState SF périmé (redondant au check EKFTask de SF) - hold déclenché"},
    Entry{NVA006, Severity::Info, "lateral_waypoint", "FMA axe latéral - segment de type Waypoint armé ou engagé"},
    Entry{NVA007, Severity::Info, "vertical_hold", "FMA axe vertical - maintien d'altitude"},
    Entry{NVA008, Severity::Info, "vertical_climb", "FMA axe vertical - montée"},
    Entry{NVA009, Severity::Info, "vertical_descend", "FMA axe vertical - descente"},
    Entry{NVA010, Severity::Info, "speed_proportional", "FMA axe vitesse - loi P saturée (cf. computeWaypointGuidance)"},
    Entry{SFS001, Severity::Info, "GPS_lock", "signal gps vérouillé"},
    Entry{SFS002, Severity::Emergency, "GPS_lost", "signal gps perdu ou aberrant"},
    Entry{SFS003, Severity::Emergency, "Mag_lost", "lecture magnétomètre en échec ou aberrante"},
    Entry{SFS004, Severity::Critical, "Lidar_lost", "lecture LIDAR en échec (driver)"},
    Entry{SFS005, Severity::Emergency, "EKF_dead", "Plus de maj. de l'EKF"},
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
