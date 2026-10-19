#pragma once
#include "drone/generated/codes.hpp"
#include "drone/types.hpp"
#include <cstdint>

// Ordre de MissionControl vers un subordonné — cf. MAVLINKINTERFACE.md.
// `payload` = un scalaire libre (ex: altitude en cm) ; `seq` est assigné
// par pushMavlinkCommand(), pas par l'appelant.
struct PendingCommand {
  CODES::Code code{};
  uint32_t payload{0};
  uint32_t seq{0};
  TYPES::TimePoint ts{};
};

// Succès/échec seulement : le diagnostic fin passe par les CODES::.
struct CommandReport {
  uint32_t seq{0};
  CODES::Code code{}; // redondant avec seq, gardé pour l'inspection
  bool success{false};
  TYPES::TimePoint ts{};
};
