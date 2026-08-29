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
  // AlarmId (ALARMS.csv) pour l'alarme GCS ("" = pas d'alarme dédiée, retombe
  // sur l'alarme générique de catégorie MW/MC — cf. GCS/public/js/alarm.js).
  const char *alarm;
  // Voyants du boîtier ESP32 à allumer, séparés par virgule ("" = aucun) —
  // labels de GCS/ESP32/mapingESP.csv, cf. GCS/serial-bridge.js.
  const char *panel;
};

inline constexpr Code FCP001{Component::FC, Category::P, 1};
inline constexpr Code FCP002{Component::FC, Category::P, 2};
inline constexpr Code FCP003{Component::FC, Category::P, 3};
inline constexpr Code MCO001{Component::MC, Category::O, 1};
inline constexpr Code MCO002{Component::MC, Category::O, 2};
inline constexpr Code MCO003{Component::MC, Category::O, 3};
inline constexpr Code MCO004{Component::MC, Category::O, 4};
inline constexpr Code MCO005{Component::MC, Category::O, 5};
inline constexpr Code MCO006{Component::MC, Category::O, 6};
inline constexpr Code MVC001{Component::MV, Category::C, 1};
inline constexpr Code MVC002{Component::MV, Category::C, 2};
inline constexpr Code MVC003{Component::MV, Category::C, 3};
inline constexpr Code MVC004{Component::MV, Category::C, 4};
inline constexpr Code MVC005{Component::MV, Category::C, 5};
inline constexpr Code MVC006{Component::MV, Category::C, 6};
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
inline constexpr Code SMO001{Component::SM, Category::O, 1};
inline constexpr Code SMC001{Component::SM, Category::C, 1};
inline constexpr Code SMC002{Component::SM, Category::C, 2};
inline constexpr Code SMC003{Component::SM, Category::C, 3};
inline constexpr Code MCA001{Component::MC, Category::A, 1};
inline constexpr Code MCA002{Component::MC, Category::A, 2};
inline constexpr Code MCA003{Component::MC, Category::A, 3};
inline constexpr Code MCA004{Component::MC, Category::A, 4};
inline constexpr Code MCA005{Component::MC, Category::A, 5};
inline constexpr Code MCA007{Component::MC, Category::A, 7};
inline constexpr Code MCA008{Component::MC, Category::A, 8};
inline constexpr Code MCA009{Component::MC, Category::A, 9};
inline constexpr Code MVP001{Component::MV, Category::P, 1};
inline constexpr Code MVP002{Component::MV, Category::P, 2};

inline constexpr std::array<Entry, 44> kTable{{
    Entry{FCP001, Severity::Critical, "batt_low", "tension batterie faible", "", ""},
    Entry{FCP002, Severity::Emergency, "Eng_failure", "Perte moteur", "fire_bell", "ENG"},
    Entry{FCP003, Severity::Emergency, "batt_critical", "tension batterie proche du seuil de coupure moteur ESC - atterrissage immédiat, pas d'attente de récupération", "", ""},
    Entry{MCO001, Severity::Info, "takeoff", "ordre de décollage vers MavlinkInterface (payload: altitude cible en cm)", "", ""},
    Entry{MCO002, Severity::Info, "set_mode_guided", "ordre de passage en mode GUIDED vers MavlinkInterface", "", ""},
    Entry{MCO003, Severity::Info, "arm", "ordre d'armement vers MavlinkInterface (payload ignoré)", "", ""},
    Entry{MCO004, Severity::Info, "disarm", "ordre de désarmement vers MavlinkInterface (payload ignoré)", "", ""},
    Entry{MCO005, Severity::Info, "start_segment", "ordre d'amorçage vers Nav (payload ignoré) - promeut armedSegment en engagedSegment si sécurité OK", "", ""},
    Entry{MCO006, Severity::Info, "land", "ordre d'atterrissage vers MavlinkInterface (payload ignoré)", "", ""},
    Entry{MVC001, Severity::Warning, "unexpected_ack", "ACK reçu (COMMAND_ACK/PARAM_VALUE) ne correspond à aucune commande TTxACK en attente", "", ""},
    Entry{MVC002, Severity::Critical, "command_rejected", "COMMAND_ACK != ACCEPTED ou timeout épuisé sur une commande interne sans origine MissionControl (ex: requestInitStreams) - pour un ordre MC voir SharedCmdMem::CommandReport - séquence abandonnée (ackQueue_ vidée)", "", ""},
    Entry{MVC003, Severity::Warning, "queue_full", "push échoué sur ackQueue_, ackFifo_ ou mavlinkReports (pleine), commande/ACK/report perdu", "", ""},
    Entry{MVC004, Severity::Critical, "fc_link_lost", "plus de HEARTBEAT du FC depuis le seuil FSTask", "", ""},
    Entry{MVC005, Severity::Emergency, "mc_heartbeat_lost", "plus de heartbeat MissionControl depuis le seuil FSTask - LAND autonome déclenché (MC ne fait jamais de cold-start, aucun autre filet)", "", ""},
    Entry{MVC006, Severity::Emergency, "gcs_double_link_lost_failsafe", "perte simultanée wfb-ng+433MHz vue indépendamment par MavlinkInterface (défense en profondeur, pas juste SysMonitoring/MissionControl) - LAND autonome déclenché", "", ""},
    Entry{NVA001, Severity::Critical, "landing_cat_1", "atterissage d'urgence, FC autonome", "", ""},
    Entry{NVA002, Severity::Info, "landing_cat_2", "atterissage assité par LIDAR", "", ""},
    Entry{NVA003, Severity::Info, "landing_cat_3", "atterissage guidé", "", ""},
    Entry{NVA004, Severity::Critical, "hold_obstacle", "TFS - obstacle détecté dans la zone de sécurité devant le drone - hold déclenché", "", "OBS"},
    Entry{NVA005, Severity::Critical, "hold_ekf_stale", "TFS - DynamicState SF périmé (redondant au check EKFTask de SF) - hold déclenché", "", ""},
    Entry{NVA006, Severity::Info, "lateral_waypoint", "FMA axe latéral - segment de type Waypoint armé ou engagé", "", ""},
    Entry{NVA007, Severity::Info, "vertical_hold", "FMA axe vertical - maintien d'altitude", "", ""},
    Entry{NVA008, Severity::Info, "vertical_climb", "FMA axe vertical - montée", "", ""},
    Entry{NVA009, Severity::Info, "vertical_descend", "FMA axe vertical - descente", "", ""},
    Entry{NVA010, Severity::Info, "speed_proportional", "FMA axe vitesse - loi P saturée (cf. computeWaypointGuidance)", "", ""},
    Entry{SFS001, Severity::Info, "GPS_lock", "signal gps vérouillé", "", ""},
    Entry{SFS002, Severity::Emergency, "GPS_lost", "signal gps perdu ou aberrant", "airbus_caution", "GPS,EKF,EMG"},
    Entry{SFS003, Severity::Emergency, "Mag_lost", "lecture magnétomètre en échec ou aberrante", "airbus_caution", "MAG,EMG"},
    Entry{SFS004, Severity::Critical, "Lidar_lost", "lecture LIDAR en échec (driver)", "", ""},
    Entry{SFS005, Severity::Emergency, "EKF_dead", "Plus de maj. de l'EKF", "airbus_caution", "EKF,EMG"},
    Entry{SMO001, Severity::Emergency, "emergency_stop_requested", "E-stop reçu du GCS (wfb-ng ou 433MHz) - signalé directement, sans passer par le FIFO gcsCmds (pas de latence de dépilement)", "", ""},
    Entry{SMC001, Severity::Warning, "gcs_cmd_queue_full", "push échoué sur gcsCmds (pleine) - commande GCS perdue", "", ""},
    Entry{SMC002, Severity::Critical, "gcs_main_link_lost", "plus de HEARTBEAT_GCS sur le lien principal (wfb-ng) depuis le seuil GCSRxTask", "", ""},
    Entry{SMC003, Severity::Critical, "gcs_sec_link_lost", "plus de HEARTBEAT_GCS sur le lien secondaire (433MHz) depuis le seuil GCSRxTask", "", ""},
    Entry{MCA001, Severity::Warning, "segment_type_unsupported", "type de segment du plan de mission pas encore exécutable côté Nav (Corridor/Curve) ou pas encore routé (PayloadAction) - segment sauté par TMiSequencer", "", ""},
    Entry{MCA002, Severity::Warning, "gcs_command_unrouted", "commande GCS reçue sans destination implémentée (CMD_WAYPOINT - pilotage manuel Nav pas encore construit) - commande abandonnée par TGCSCmdRouter", "", ""},
    Entry{MCA003, Severity::Warning, "rtb_not_implemented_v1", "CMD_RTL reçu - pas de RTB en V1 (décidé explicitement) - commande abandonnée par TGCSCmdRouter", "", ""},
    Entry{MCA004, Severity::Warning, "mission_plan_load_failed", "échec chargement/parsing du plan de mission au PRE_FLIGHT (fichier absent, illisible ou corrompu) - retour à IDLE", "", ""},
    Entry{MCA005, Severity::Critical, "nav_stale_while_sequencing", "Nav ne publie plus de NavCommand frais (ou santé dégradée) alors qu'un segment est en cours d'exécution - pas un simple timeout sur la durée du segment", "", ""},
    Entry{MCA007, Severity::Critical, "nav_volume_breached", "drone hors du volume de vol autorisé (rayon/plafond du plan de mission) - HOLD déclenché", "", ""},
    Entry{MCA008, Severity::Emergency, "crash_suspected", "désarmé pendant LANDING mais vitesse encore significative - pas un atterrissage normal", "", ""},
    Entry{MCA009, Severity::Warning, "prearm_health_check_failed", "GPS/EKF perdu ou tension batterie sous le seuil bas au PRE_FLIGHT - retour à IDLE, distinct d'un plan de mission illisible (MCA004)", "", ""},
    Entry{MVP001, Severity::Critical, "rpm_too_low", "RPM moteur sous le seuil ou donnée ESC_STATUS absente/aberrante alors que le FC est armé - probable panne mécanique/électrique", "", ""},
    Entry{MVP002, Severity::Warning, "rpm_too_high", "RPM moteur au-dessus du seuil attendu alors que le FC est armé - voyant GCS, pas une panne bloquante en soi", "", ""},
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
